// Copyright 2026 Héctor Azpúrua
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file
/// @brief `point_sim_3d`: a kinematic point robot for the 3D follower's tests and demo
/// (experimental).
///
/// It integrates the last `cmd_vel` (TwistStamped: body heading frame when `frame_id` is
/// `base_frame`, else `global_frame`) with vectorfield_core::d3::sim::integrate (optional
/// velocity lag), and publishes `odom` (pose in `global_frame`, twist in `base_frame`), TF
/// `global_frame → base_frame` and, with `publish_clock`, `/clock` at `real_time_factor`. A
/// command older than `cmd_timeout` counts as zero (an autopilot's failsafe). Optional extras: a
/// frame moving with a constant body twist (`moving_frame`: TF and Odometry), and a PointCloud2
/// of the surfaces of `obstacle_spheres` (dynamic) on `cloud_topic`.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <exception>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/point_field.hpp"
#include "tf2_msgs/msg/tf_message.hpp"
#include "tf2_ros/qos.hpp"
#include "vectorfield_core/d3/frame.hpp"
#include "vectorfield_core/d3/simulator.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_follower_3d/conversions.hpp"

namespace vectorfield_follower_3d
{

namespace
{

namespace d3 = vectorfield_core::d3;

/// Simulated time at start-up (s): non-zero, so that clients see a valid time at once.
constexpr double kStartTime = 1.0;
/// Golden angle of the Fibonacci sphere (rad).
const double kGoldenAngle = vectorfield_core::kPi * (3.0 - std::sqrt(5.0));

d3::Vec3 vec3Parameter(const std::vector<double> & values)
{
  return values.size() == 3 ? d3::Vec3(values[0], values[1], values[2]) : d3::Vec3::Zero();
}

/// Points on the surfaces of spheres (x y z r ...), about `spacing` apart.
std::vector<d3::Vec3> spherePoints(const std::vector<double> & spheres, double spacing)
{
  std::vector<d3::Vec3> points;
  for (std::size_t i = 0; i + 3 < spheres.size(); i += 4) {
    const d3::Vec3 c(spheres[i], spheres[i + 1], spheres[i + 2]);
    const double r = spheres[i + 3];
    if (!(r > 0.0) || !d3::isFinite(c)) {
      continue;
    }
    const auto n =
      static_cast<int>(std::ceil(4.0 * vectorfield_core::kPi * r * r / (spacing * spacing))) + 1;
    for (int k = 0; k < n; ++k) {
      const double z = 1.0 - 2.0 * (k + 0.5) / n;
      const double rho = std::sqrt(std::max(0.0, 1.0 - z * z));
      const double theta = kGoldenAngle * k;
      points.push_back(c + r * d3::Vec3(rho * std::cos(theta), rho * std::sin(theta), z));
    }
  }
  return points;
}

class PointSim3D : public rclcpp::Node
{
public:
  PointSim3D() : rclcpp::Node("point_sim_3d")
  {
    step_ = declare_parameter<double>("step_size", 0.01);
    const double rate = declare_parameter<double>("real_time_factor", 1.0);
    publish_clock_ = declare_parameter<bool>("publish_clock", true);
    odom_period_ = 1.0 / declare_parameter<double>("odom_rate", 50.0);
    plant_.velocity_time_constant = declare_parameter<double>("velocity_time_constant", 0.0);
    cmd_timeout_ = declare_parameter<double>("cmd_timeout", 0.5);
    state_.pose.p = d3::Vec3(
      declare_parameter<double>("initial_x", 0.0), declare_parameter<double>("initial_y", 0.0),
      declare_parameter<double>("initial_z", 1.0));
    state_.pose.yaw = declare_parameter<double>("initial_yaw", 0.0);
    global_frame_ = declare_parameter<std::string>("global_frame", "map");
    base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
    const auto cmd_topic = declare_parameter<std::string>("cmd_vel_topic", "cmd_vel");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "odom");
    moving_frame_ = declare_parameter<std::string>("moving_frame", "");
    const auto frame_topic =
      declare_parameter<std::string>("moving_frame_topic", "moving_frame/odom");
    const auto origin =
      declare_parameter<std::vector<double>>("moving_frame_origin", std::vector<double>{0, 0, 0});
    frame_twist_.v = vec3Parameter(
      declare_parameter<std::vector<double>>("moving_frame_linear", std::vector<double>{0, 0, 0}));
    frame_twist_.w = vec3Parameter(
      declare_parameter<std::vector<double>>("moving_frame_angular", std::vector<double>{0, 0, 0}));
    frame_origin_ = d3::Isometry3::Identity();
    frame_origin_.translation() = vec3Parameter(origin);
    declare_parameter<std::vector<double>>("obstacle_spheres", std::vector<double>{});
    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "obstacle_cloud");
    cloud_period_ = 1.0 / declare_parameter<double>("cloud_rate", 2.0);
    cloud_spacing_ = declare_parameter<double>("cloud_spacing", 0.05);
    if (
      !(step_ > 0.0) || !(rate >= 0.0) || !(odom_period_ > 0.0) || !(cloud_period_ > 0.0) ||
      !(cloud_spacing_ > 0.0) || plant_.velocity_time_constant < 0.0) {
      throw std::invalid_argument("point_sim_3d: invalid step, rate or period");
    }

    tf_pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/tf", tf2_ros::DynamicBroadcasterQoS());
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(10).reliable());
    cloud_pub_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(cloud_topic, rclcpp::QoS(1).reliable());
    if (!moving_frame_.empty()) {
      frame_pub_ =
        create_publisher<nav_msgs::msg::Odometry>(frame_topic, rclcpp::QoS(10).reliable());
    }
    if (publish_clock_) {
      clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
        "/clock", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
    }
    cmd_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_topic, rclcpp::QoS(10).reliable(),
      [this](geometry_msgs::msg::TwistStamped::ConstSharedPtr msg) {
        const std::lock_guard<std::mutex> lock(mutex_);
        cmd_ = *msg;
        cmd_time_ = t_;
        has_cmd_ = true;
      });
    t_ = publish_clock_ ? kStartTime : now().seconds();
    if (rate > 0.0) {
      const auto period =
        std::chrono::nanoseconds(static_cast<std::int64_t>(std::llround(step_ * 1e9 / rate)));
      timer_ = create_wall_timer(period, [this]() { step(); });
    }
    RCLCPP_INFO(
      get_logger(), "point_sim_3d: step %.3f s at %.2fx, start (%.2f, %.2f, %.2f)", step_, rate,
      state_.pose.p.x(), state_.pose.p.y(), state_.pose.p.z());
  }

private:
  void step()
  {
    if (publish_clock_) {
      t_ += step_;
    } else {
      t_ = now().seconds();
    }
    // The command, in the world frame.
    d3::Twist3 cmd;
    {
      const std::lock_guard<std::mutex> lock(mutex_);
      if (has_cmd_ && t_ - cmd_time_ <= cmd_timeout_) {
        const d3::Vec3 v = toVec3(cmd_.twist.linear);
        cmd.v =
          cmd_.header.frame_id == base_frame_ ? d3::Vec3(yawRotation(state_.pose.yaw) * v) : v;
        cmd.wz = cmd_.twist.angular.z;
      }
    }
    if (d3::isFinite(cmd)) {
      state_ = d3::sim::integrate(state_, cmd, step_, plant_);
    }
    const rclcpp::Time stamp(static_cast<std::int64_t>(std::llround(t_ * 1e9)), RCL_ROS_TIME);
    if (publish_clock_) {
      rosgraph_msgs::msg::Clock clock;
      clock.clock = stamp;
      clock_pub_->publish(clock);
    }
    if (t_ - last_odom_ >= odom_period_ - 1e-9) {
      last_odom_ = t_;
      publishOdometry(stamp);
    }
    if (t_ - last_cloud_ >= cloud_period_ - 1e-9) {
      last_cloud_ = t_;
      publishCloud(stamp);
    }
  }

  void publishOdometry(const rclcpp::Time & stamp)
  {
    const d3::Mat3 R = yawRotation(state_.pose.yaw);
    geometry_msgs::msg::TransformStamped base;
    base.header.frame_id = global_frame_;
    base.header.stamp = stamp;
    base.child_frame_id = base_frame_;
    base.transform.translation = toVector3Msg(state_.pose.p);
    base.transform.rotation = toQuaternionMsg(Eigen::Quaterniond(R));
    tf2_msgs::msg::TFMessage tf;
    tf.transforms.push_back(base);

    nav_msgs::msg::Odometry odom;
    odom.header = base.header;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position = toPointMsg(state_.pose.p);
    odom.pose.pose.orientation = base.transform.rotation;
    odom.twist.twist.linear = toVector3Msg(R.transpose() * state_.twist.v);
    odom.twist.twist.angular.z = state_.twist.wz;
    odom_pub_->publish(odom);

    if (frame_pub_) {
      const d3::Isometry3 T = frame_origin_ * d3::expSE3(frame_twist_, t_ - kStartTime);
      geometry_msgs::msg::TransformStamped frame;
      frame.header = base.header;
      frame.child_frame_id = moving_frame_;
      frame.transform.translation = toVector3Msg(T.translation());
      frame.transform.rotation = toQuaternionMsg(Eigen::Quaterniond(T.rotation()));
      tf.transforms.push_back(frame);
      nav_msgs::msg::Odometry frame_odom;
      frame_odom.header = base.header;
      frame_odom.child_frame_id = moving_frame_;
      frame_odom.pose.pose = toPoseMsg(T);
      frame_odom.twist.twist.linear = toVector3Msg(frame_twist_.v);
      frame_odom.twist.twist.angular = toVector3Msg(frame_twist_.w);
      frame_pub_->publish(frame_odom);
    }
    tf_pub_->publish(tf);
  }

  void publishCloud(const rclcpp::Time & stamp)
  {
    const std::vector<d3::Vec3> points =
      spherePoints(get_parameter("obstacle_spheres").as_double_array(), cloud_spacing_);
    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header.frame_id = global_frame_;
    cloud.header.stamp = stamp;
    cloud.height = 1;
    cloud.width = static_cast<std::uint32_t>(points.size());
    const char * names[3] = {"x", "y", "z"};
    for (std::uint32_t i = 0; i < 3; ++i) {
      sensor_msgs::msg::PointField field;
      field.name = names[i];
      field.offset = 4U * i;
      field.datatype = sensor_msgs::msg::PointField::FLOAT32;
      field.count = 1;
      cloud.fields.push_back(field);
    }
    cloud.is_bigendian = false;
    cloud.point_step = 12;
    cloud.row_step = cloud.point_step * cloud.width;
    cloud.is_dense = true;
    cloud.data.resize(cloud.row_step);
    for (std::size_t k = 0; k < points.size(); ++k) {
      const float xyz[3] = {
        static_cast<float>(points[k].x()), static_cast<float>(points[k].y()),
        static_cast<float>(points[k].z())};
      std::memcpy(&cloud.data[k * 12U], xyz, sizeof(xyz));
    }
    cloud_pub_->publish(cloud);
  }

  double step_{0.01};
  bool publish_clock_{true};
  double odom_period_{0.02};
  double cmd_timeout_{0.5};
  double cloud_period_{0.5};
  double cloud_spacing_{0.05};
  d3::sim::PlantParams3 plant_;
  d3::sim::PlantState3 state_;
  std::string global_frame_;
  std::string base_frame_;
  std::string moving_frame_;
  d3::Isometry3 frame_origin_{d3::Isometry3::Identity()};
  d3::FrameTwist3 frame_twist_;
  double t_{0.0};
  double last_odom_{-1e300};
  double last_cloud_{-1e300};
  std::mutex mutex_;
  geometry_msgs::msg::TwistStamped cmd_;
  double cmd_time_{0.0};
  bool has_cmd_{false};
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr frame_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_pub_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr cmd_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace

}  // namespace vectorfield_follower_3d

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int status = 0;
  try {
    rclcpp::spin(std::make_shared<vectorfield_follower_3d::PointSim3D>());
  } catch (const std::exception & error) {
    RCLCPP_FATAL(rclcpp::get_logger("point_sim_3d"), "%s", error.what());
    status = 1;
  }
  rclcpp::shutdown();
  return status;
}
