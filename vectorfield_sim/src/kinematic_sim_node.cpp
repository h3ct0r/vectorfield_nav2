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

#include "vectorfield_sim/kinematic_sim_node.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rcl_interfaces/msg/integer_range.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_ros/compat.hpp"
#include "vectorfield_sim/obstacles.hpp"

namespace vectorfield_sim
{

namespace
{

namespace vf = vectorfield_core;
namespace compat = vectorfield_ros::compat;
using rcl_interfaces::msg::ParameterDescriptor;

constexpr const char * kCircleParameter = "circle_obstacles";
constexpr const char * kBoxParameter = "box_obstacles";
/// Throttle period of the warnings in callbacks (ms).
constexpr int kThrottleMs = 1000;

ParameterDescriptor readOnly(const std::string & description)
{
  ParameterDescriptor descriptor;
  descriptor.description = description;
  descriptor.read_only = true;
  return descriptor;
}

ParameterDescriptor readOnly(const std::string & description, double from, double to)
{
  ParameterDescriptor descriptor = readOnly(description);
  rcl_interfaces::msg::FloatingPointRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = 0.0;
  descriptor.floating_point_range.push_back(range);
  return descriptor;
}

ParameterDescriptor readOnlyInteger(
  const std::string & description, std::int64_t from, std::int64_t to)
{
  ParameterDescriptor descriptor = readOnly(description);
  rcl_interfaces::msg::IntegerRange range;
  range.from_value = from;
  range.to_value = to;
  range.step = 1;
  descriptor.integer_range.push_back(range);
  return descriptor;
}

builtin_interfaces::msg::Time toStamp(std::int64_t time_ns)
{
  builtin_interfaces::msg::Time stamp;
  stamp.sec = static_cast<std::int32_t>(time_ns / 1'000'000'000);
  stamp.nanosec = static_cast<std::uint32_t>(time_ns % 1'000'000'000);
  return stamp;
}

geometry_msgs::msg::Quaternion toQuaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(0.5 * yaw);
  q.w = std::cos(0.5 * yaw);
  return q;
}

double yawOf(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

vf::Pose2 toPose2(const geometry_msgs::msg::Pose & pose)
{
  return vf::Pose2{vf::Vec2(pose.position.x, pose.position.y), yawOf(pose.orientation)};
}

vf::Twist2 toTwist2(const geometry_msgs::msg::Twist & twist)
{
  return vf::Twist2{twist.linear.x, twist.linear.y, twist.angular.z};
}

geometry_msgs::msg::TransformStamped toTransform(
  const vf::Pose2 & pose, const builtin_interfaces::msg::Time & stamp, const std::string & parent,
  const std::string & child)
{
  geometry_msgs::msg::TransformStamped transform;
  transform.header.stamp = stamp;
  transform.header.frame_id = parent;
  transform.child_frame_id = child;
  transform.transform.translation.x = pose.p.x();
  transform.transform.translation.y = pose.p.y();
  transform.transform.translation.z = 0.0;
  transform.transform.rotation = toQuaternion(pose.yaw);
  return transform;
}

nav_msgs::msg::Odometry toOdometry(
  const vf::Pose2 & pose, const vf::Twist2 & twist, const builtin_interfaces::msg::Time & stamp,
  const std::string & parent, const std::string & child)
{
  nav_msgs::msg::Odometry odometry;
  odometry.header.stamp = stamp;
  odometry.header.frame_id = parent;
  odometry.child_frame_id = child;
  odometry.pose.pose.position.x = pose.p.x();
  odometry.pose.pose.position.y = pose.p.y();
  odometry.pose.pose.orientation = toQuaternion(pose.yaw);
  odometry.twist.twist.linear.x = twist.vx;
  odometry.twist.twist.linear.y = twist.vy;
  odometry.twist.twist.angular.z = twist.wz;
  return odometry;
}

/// A numeric list from an obstacle parameter: double or integer arrays, or any empty array
/// (`ros2 param set ... "[]"` sends an empty bool array).
bool toNumberList(const rclcpp::ParameterValue & value, std::vector<double> * out)
{
  using rclcpp::ParameterType;
  switch (value.get_type()) {
    case ParameterType::PARAMETER_DOUBLE_ARRAY:
      *out = value.get<std::vector<double>>();
      return true;
    case ParameterType::PARAMETER_INTEGER_ARRAY: {
      const auto & integers = value.get<std::vector<std::int64_t>>();
      out->assign(integers.begin(), integers.end());
      return true;
    }
    case ParameterType::PARAMETER_BOOL_ARRAY:
      out->clear();
      return value.get<std::vector<bool>>().empty();
    case ParameterType::PARAMETER_STRING_ARRAY:
      out->clear();
      return value.get<std::vector<std::string>>().empty();
    case ParameterType::PARAMETER_BYTE_ARRAY:
      out->clear();
      return value.get<std::vector<std::uint8_t>>().empty();
    default:
      return false;
  }
}

GridMap toGridMap(const nav_msgs::msg::OccupancyGrid & message, std::int8_t occupied_threshold)
{
  GridMap map;
  map.width = static_cast<int>(message.info.width);
  map.height = static_cast<int>(message.info.height);
  map.resolution = message.info.resolution;
  map.origin = toPose2(message.info.origin);
  map.data = message.data;
  map.occupied_threshold = occupied_threshold;
  return map;
}

}  // namespace

KinematicSimNode::KinematicSimNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("vectorfield_sim", options)
{
  // Motion model and plant.
  const auto model_name = declare_parameter<std::string>(
    "motion_model", "differential",
    readOnly("differential, skid_steer, omnidirectional or ackermann"));
  vf::MotionModel model = vf::MotionModel::kDifferential;
  if (!vf::fromString(model_name, &model)) {
    throw std::invalid_argument(
      "motion_model '" + model_name +
      "' is not one of differential, skid_steer, omnidirectional, ackermann");
  }
  PlantConfig config;
  config.kinematics.type = model;
  config.kinematics.min_turning_radius = declare_parameter<double>(
    "min_turning_radius", 0.5,
    readOnly("Ackermann minimum turning radius R_min (m); > 0 for ackermann", 0.0, 1e3));
  config.kinematics.reference_point_offset = declare_parameter<double>(
    "reference_point_offset", 0.0,
    readOnly("Ackermann rear-axle offset x_r along body x from base_footprint (m)", -1e3, 1e3));
  if (model == vf::MotionModel::kAckermann && !(config.kinematics.min_turning_radius > 0.0)) {
    throw std::invalid_argument("min_turning_radius must be > 0 for motion_model ackermann");
  }
  const double eta_psi = declare_parameter<double>(
    "slip_eta_psi", kSkidSteerSlip.eta_psi,
    readOnly("skid_steer only: yaw-rate efficiency eta_psi", 0.0, 1.0));
  const double x_icr = declare_parameter<double>(
    "slip_x_icr", kSkidSteerSlip.x_icr,
    readOnly("skid_steer only: ICR offset x_ICR along body x (m)", -1e3, 1e3));
  if (model == vf::MotionModel::kSkidSteer) {
    config.slip = vf::sim::SlipParams{eta_psi, x_icr};
  }
  const double step_size =
    declare_parameter<double>("step_size", 0.01, readOnly("Fixed simulation step (s)", 1e-4, 1.0));
  config.step_ns = toNanoseconds(step_size);
  config.command_timeout_ns = toNanoseconds(
    declare_parameter<double>(
      "cmd_vel_timeout", 0.5,
      readOnly(
        "A command older than this (simulated s) is replaced by zero; 0 disables", 0.0, 1e6)));
  const double real_time_factor = declare_parameter<double>(
    "real_time_factor", 1.0,
    readOnly("Simulated seconds per wall second; 0 stops the step timer", 0.0, 1e3));
  const vf::Pose2 initial_pose{
    vf::Vec2(
      declare_parameter<double>("initial_pose_x", 0.0, readOnly("Initial map x (m)")),
      declare_parameter<double>("initial_pose_y", 0.0, readOnly("Initial map y (m)"))),
    declare_parameter<double>("initial_pose_yaw", 0.0, readOnly("Initial map yaw (rad)"))};
  if (!vf::isFinite(initial_pose)) {
    throw std::invalid_argument(
      "initial_pose_x, initial_pose_y and initial_pose_yaw must be finite");
  }
  plant_ = std::make_unique<KinematicPlant>(config, initial_pose);

  // Frames and topics.
  map_frame_ = declare_parameter<std::string>("map_frame", "map", readOnly("Map frame"));
  odom_frame_ = declare_parameter<std::string>("odom_frame", "odom", readOnly("Odometry frame"));
  base_frame_ =
    declare_parameter<std::string>("base_frame", "base_footprint", readOnly("Robot base frame"));
  scan_frame_ =
    declare_parameter<std::string>("scan_frame", "base_scan", readOnly("Frame of the scan"));
  const auto cmd_vel_topic =
    declare_parameter<std::string>("cmd_vel_topic", "cmd_vel", readOnly("Command topic"));
  const bool stamped = declare_parameter<bool>(
    "enable_stamped_cmd_vel", compat::kStampedCmdVelDefault,
    readOnly("Subscribe to TwistStamped instead of Twist (default: the distro's Nav2 default)"));
  const auto map_topic =
    declare_parameter<std::string>("map_topic", "map", readOnly("OccupancyGrid topic"));
  const auto scan_topic =
    declare_parameter<std::string>("scan_topic", "scan", readOnly("LaserScan topic"));

  // Laser.
  const auto beams = declare_parameter<std::int64_t>(
    "scan_beams", 360, readOnlyInteger("Beams over a full turn from -pi", 1, 100000));
  const double scan_rate = declare_parameter<double>(
    "scan_rate", 10.0, readOnly("Scan rate (Hz of simulated time)", 0.01, 1e4));
  scan_range_min_ = declare_parameter<double>(
    "scan_range_min", 0.05, readOnly("LaserScan range_min (m)", 0.0, 1e3));
  const double range_max = declare_parameter<double>(
    "scan_range_max", 30.0, readOnly("LaserScan range_max (m)", 0.01, 1e4));
  scan_use_inf_ = declare_parameter<bool>(
    "scan_use_inf", true, readOnly("No return is +inf (else range_max - 0.1, like loopback)"));
  const auto occupied_threshold = declare_parameter<std::int64_t>(
    "occupied_threshold", 65, readOnlyInteger("Grid cells >= this value are walls", 1, 100));
  scan_geometry_ = ScanGeometry::fullCircle(static_cast<int>(beams), range_max);
  scan_every_steps_ = stepsPerPeriod(toNanoseconds(1.0 / scan_rate), config.step_ns);

  // Unmapped obstacles: the callback validates the declarations below too.
  parameter_callback_ =
    add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter> & parameters) {
      return onSetParameters(parameters);
    });
  ParameterDescriptor obstacles;
  obstacles.dynamic_typing = true;
  obstacles.description = "Unmapped circles, flat [x, y, radius] per circle (map frame)";
  declare_parameter(kCircleParameter, rclcpp::ParameterValue(std::vector<double>{}), obstacles);
  obstacles.description = "Unmapped boxes, flat [x, y, size_x, size_y, yaw] per box (map frame)";
  declare_parameter(kBoxParameter, rclcpp::ParameterValue(std::vector<double>{}), obstacles);

  // Interface.
  tf_broadcaster_ = compat::makeTfBroadcaster(this);
  static_tf_broadcaster_ = compat::makeStaticTfBroadcaster(this);
  clock_pub_ = create_publisher<rosgraph_msgs::msg::Clock>(
    "/clock", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());
  odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
  ground_truth_pub_ = create_publisher<nav_msgs::msg::Odometry>("~/ground_truth", rclcpp::QoS(10));
  scan_pub_ = create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::SensorDataQoS());
  infeasible_pub_ = create_publisher<std_msgs::msg::UInt64>(
    "~/infeasible_command_count", rclcpp::QoS(1).transient_local());

  if (stamped) {
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      cmd_vel_topic, rclcpp::QoS(10), [this](const geometry_msgs::msg::TwistStamped & message) {
        onCommand(toTwist2(message.twist));
      });
  } else {
    cmd_vel_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, rclcpp::QoS(10),
      [this](const geometry_msgs::msg::Twist & message) { onCommand(toTwist2(message)); });
  }
  initial_pose_sub_ = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", rclcpp::QoS(10),
    [this](const geometry_msgs::msg::PoseWithCovarianceStamped & message) {
      onInitialPose(message);
    });
  map_sub_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic, rclcpp::QoS(1).transient_local().reliable(),
    [this, threshold = static_cast<std::int8_t>(occupied_threshold)](
      const nav_msgs::msg::OccupancyGrid & message) {
      if (!message.header.frame_id.empty() && message.header.frame_id != map_frame_) {
        RCLCPP_WARN(
          get_logger(), "Ignoring a map in frame '%s' (expected '%s')",
          message.header.frame_id.c_str(), map_frame_.c_str());
        return;
      }
      const GridMap map = toGridMap(message, threshold);
      if (!map.valid()) {
        RCLCPP_WARN(get_logger(), "Ignoring a malformed map (size and data do not match)");
        return;
      }
      RCLCPP_INFO(
        get_logger(), "Map %dx%d at %.3f m received", map.width, map.height, map.resolution);
      const std::lock_guard<std::mutex> lock(mutex_);
      caster_.setMap(map);
    });
  step_map_to_odom_srv_ = create_service<nav2_msgs::srv::SetInitialPose>(
    "~/step_map_to_odom",
    [this](
      const std::shared_ptr<nav2_msgs::srv::SetInitialPose::Request> request,
      std::shared_ptr<nav2_msgs::srv::SetInitialPose::Response> /*response*/) {
      onStepMapToOdom(*request);
    });

  publishMapToOdom(plant_->mapToOdom(), plant_->timeNs());
  publishInfeasibleCount(0);
  if (real_time_factor > 0.0) {
    const auto period = std::chrono::nanoseconds(
      std::llround(static_cast<double>(config.step_ns) / real_time_factor));
    step_timer_ = create_wall_timer(period, [this]() { step(); });
  }
  RCLCPP_INFO(
    get_logger(), "%s base, step %.4f s at %.2fx real time, %s on '%s'", model_name.c_str(),
    step_size, real_time_factor, stamped ? "TwistStamped" : "Twist", cmd_vel_topic.c_str());
}

KinematicSimNode::~KinematicSimNode() = default;

void KinematicSimNode::step()
{
  const std::lock_guard<std::mutex> lock(mutex_);
  plant_->step();
  const auto stamp = toStamp(plant_->timeNs());

  rosgraph_msgs::msg::Clock clock;
  clock.clock = stamp;
  clock_pub_->publish(clock);

  const vf::Pose2 & pose = plant_->odomPose();
  const vf::Twist2 & twist = plant_->twist();
  odom_pub_->publish(toOdometry(pose, twist, stamp, odom_frame_, base_frame_));
  tf_broadcaster_->sendTransform(toTransform(pose, stamp, odom_frame_, base_frame_));
  ground_truth_pub_->publish(
    toOdometry(plant_->groundTruth(), twist, stamp, map_frame_, base_frame_));

  if (plant_->steps() % static_cast<std::uint64_t>(scan_every_steps_) == 0) {
    publishScan(stamp);
  }
}

rcl_interfaces::msg::SetParametersResult KinematicSimNode::onSetParameters(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  std::optional<std::vector<Circle>> circles;
  std::optional<std::vector<Box>> boxes;
  for (const rclcpp::Parameter & parameter : parameters) {
    const std::string & name = parameter.get_name();
    const bool is_circles = name == kCircleParameter;
    if (!is_circles && name != kBoxParameter) {
      continue;
    }
    std::vector<double> values;
    std::string error;
    bool valid = toNumberList(parameter.get_parameter_value(), &values);
    if (!valid) {
      error = name + ": expected a list of numbers, not " + parameter.get_type_name();
    } else if (is_circles) {
      std::vector<Circle> parsed;
      valid = parseCircleObstacles(values, &parsed, &error);
      circles = std::move(parsed);
    } else {
      std::vector<Box> parsed;
      valid = parseBoxObstacles(values, &parsed, &error);
      boxes = std::move(parsed);
    }
    if (!valid) {
      RCLCPP_ERROR(get_logger(), "Rejected obstacles: %s", error.c_str());
      result.successful = false;
      result.reason = error;
      return result;
    }
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  if (circles) {
    caster_.setCircles(std::move(*circles));
  }
  if (boxes) {
    caster_.setBoxes(std::move(*boxes));
  }
  return result;
}

void KinematicSimNode::onCommand(const vf::Twist2 & command)
{
  if (!std::isfinite(command.vx) || !std::isfinite(command.vy) || !std::isfinite(command.wz)) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kThrottleMs, "Ignoring a non-finite command");
    return;
  }
  bool infeasible = false;
  std::uint64_t count = 0;
  {
    const std::lock_guard<std::mutex> lock(mutex_);
    infeasible = plant_->setCommand(command);
    count = plant_->infeasibleCount();
  }
  if (infeasible) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kThrottleMs,
      "Infeasible Ackermann command v = 0, omega = %.3f rad/s (%llu so far)", command.wz,
      static_cast<unsigned long long>(count));
    publishInfeasibleCount(count);
  }
}

void KinematicSimNode::onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & message)
{
  if (!message.header.frame_id.empty() && message.header.frame_id != map_frame_) {
    RCLCPP_WARN(
      get_logger(), "Ignoring an initial pose in frame '%s' (expected '%s')",
      message.header.frame_id.c_str(), map_frame_.c_str());
    return;
  }
  const vf::Pose2 pose = toPose2(message.pose.pose);
  if (!vf::isFinite(pose)) {
    RCLCPP_WARN(get_logger(), "Ignoring a non-finite initial pose");
    return;
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  plant_->setInitialPose(pose);
  publishMapToOdom(plant_->mapToOdom(), plant_->timeNs());
  RCLCPP_INFO(
    get_logger(), "Initial pose (%.3f, %.3f, %.3f rad)", pose.p.x(), pose.p.y(), pose.yaw);
}

void KinematicSimNode::onStepMapToOdom(const nav2_msgs::srv::SetInitialPose::Request & request)
{
  const vf::Pose2 delta = toPose2(request.pose.pose.pose);
  if (!vf::isFinite(delta)) {
    RCLCPP_ERROR(get_logger(), "Ignoring a non-finite map -> odom step");
    return;
  }
  const std::lock_guard<std::mutex> lock(mutex_);
  plant_->stepMapToOdom(delta.p.x(), delta.p.y(), delta.yaw);
  publishMapToOdom(plant_->mapToOdom(), plant_->timeNs());
  RCLCPP_INFO(
    get_logger(), "Stepped map -> odom by (%.3f m, %.3f m, %.4f rad)", delta.p.x(), delta.p.y(),
    delta.yaw);
}

void KinematicSimNode::publishMapToOdom(const vf::Pose2 & map_to_odom, std::int64_t time_ns)
{
  static_tf_broadcaster_->sendTransform(
    toTransform(map_to_odom, toStamp(time_ns), map_frame_, odom_frame_));
}

void KinematicSimNode::publishInfeasibleCount(std::uint64_t count)
{
  std_msgs::msg::UInt64 message;
  message.data = count;
  infeasible_pub_->publish(message);
}

void KinematicSimNode::publishScan(const builtin_interfaces::msg::Time & stamp)
{
  const std::vector<double> ranges = caster_.castScan(plant_->groundTruth(), scan_geometry_);
  sensor_msgs::msg::LaserScan scan;
  scan.header.stamp = stamp;
  scan.header.frame_id = scan_frame_;
  scan.angle_min = static_cast<float>(scan_geometry_.angle_min);
  scan.angle_max = static_cast<float>(scan_geometry_.angleMax());
  scan.angle_increment = static_cast<float>(scan_geometry_.angle_increment);
  scan.time_increment = 0.0F;
  scan.scan_time =
    static_cast<float>(static_cast<double>(scan_every_steps_ * plant_->stepNs()) * 1e-9);
  scan.range_min = static_cast<float>(scan_range_min_);
  scan.range_max = static_cast<float>(scan_geometry_.range_max);
  const auto no_return = static_cast<float>(scan_geometry_.range_max - 0.1);
  scan.ranges.reserve(ranges.size());
  for (const double range : ranges) {
    scan.ranges.push_back(
      std::isfinite(range) || scan_use_inf_ ? static_cast<float>(range) : no_return);
  }
  scan_pub_->publish(scan);
}

}  // namespace vectorfield_sim

RCLCPP_COMPONENTS_REGISTER_NODE(vectorfield_sim::KinematicSimNode)
