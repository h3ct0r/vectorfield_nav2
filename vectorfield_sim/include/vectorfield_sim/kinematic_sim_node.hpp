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
/// @brief The kinematic simulator node (rclcpp component `vectorfield_sim::KinematicSimNode`,
/// executable `kinematic_sim`; plan §8).
///
/// **Interface** (relative names resolve in the node's namespace):
///
/// | Kind | Name | Type | Notes |
/// |---|---|---|---|
/// | sub | `cmd_vel_topic` (`cmd_vel`) | `Twist` or `TwistStamped` | per `enable_stamped_cmd_vel` |
/// | sub | `initialpose` | `PoseWithCovarianceStamped` | map frame; teleports the robot |
/// | sub | `map_topic` (`map`) | `OccupancyGrid` | transient_local; the laser's walls |
/// | pub | `odom` | `Odometry` | odom → base_footprint, with the executed body twist |
/// | pub | `/tf` | odom → base_footprint | every step |
/// | pub | `/tf_static` | map → odom | the localization estimate; re-sent on change |
/// | pub | `/clock` | `rosgraph_msgs/Clock` | the time source: one message per step |
/// | pub | `scan` | `LaserScan` | `scan_frame`, `scan_beams` beams, `scan_rate` Hz, SensorDataQoS |
/// | pub | `~/ground_truth` | `Odometry` | true map-frame pose of base_footprint |
/// | pub | `~/infeasible_command_count` | `std_msgs/UInt64` | transient_local; Ackermann only |
/// | srv | `~/step_map_to_odom` | `nav2_msgs/SetInitialPose` | adds (x, y, yaw) to map → odom |
///
/// **Time.** The node is the time source: every step advances simulated time by `step_size` and
/// publishes `/clock`; every stamp is simulated time, whatever `use_sim_time` says. A wall timer
/// runs the steps at `real_time_factor` × real time; 0 disables the timer, and the node then
/// advances only through step() (the tests do this).
///
/// **Frames.** The laser sits at base_footprint (bringup publishes identity base_footprint →
/// base_link → base_scan). See KinematicPlant for the true and published map → odom.
///
/// Every parameter is read-only except `circle_obstacles` and `box_obstacles` (obstacles.hpp),
/// which are validated as a whole and may be set at run time; a malformed list is rejected with
/// an error, at construction (the constructor throws) as at run time.

#ifndef VECTORFIELD_SIM__KINEMATIC_SIM_NODE_HPP_
#define VECTORFIELD_SIM__KINEMATIC_SIM_NODE_HPP_

#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/srv/set_initial_pose.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/u_int64.hpp"
#include "vectorfield_core/types.hpp"
#include "vectorfield_sim/plant.hpp"
#include "vectorfield_sim/ray_caster.hpp"

namespace tf2_ros
{
class StaticTransformBroadcaster;
class TransformBroadcaster;
}  // namespace tf2_ros

namespace vectorfield_sim
{

/// @brief The simulator node. Its callbacks and step() serialize on one mutex.
class KinematicSimNode : public rclcpp::Node
{
public:
  /// @brief Declares the parameters, creates the interface and, unless `real_time_factor` is 0,
  /// starts stepping.
  /// @throws rclcpp::exceptions::InvalidParameterValueException for an out-of-range parameter or
  /// a malformed obstacle list; std::invalid_argument for an unknown `motion_model`.
  explicit KinematicSimNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~KinematicSimNode() override;

  KinematicSimNode(const KinematicSimNode &) = delete;
  KinematicSimNode & operator=(const KinematicSimNode &) = delete;
  KinematicSimNode(KinematicSimNode &&) = delete;
  KinematicSimNode & operator=(KinematicSimNode &&) = delete;

  /// @brief Advances one fixed step and publishes `/clock`, `odom`, the TF, `~/ground_truth` and,
  /// when due, `scan` (the step timer's callback).
  void step();

private:
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    const std::vector<rclcpp::Parameter> & parameters);
  void onCommand(const vectorfield_core::Twist2 & command);
  void onInitialPose(const geometry_msgs::msg::PoseWithCovarianceStamped & message);
  void onMap(const nav_msgs::msg::OccupancyGrid & message);
  void onStepMapToOdom(const nav2_msgs::srv::SetInitialPose::Request & request);
  void publishMapToOdom(const vectorfield_core::Pose2 & map_to_odom, std::int64_t time_ns);
  void publishInfeasibleCount(std::uint64_t count);
  void publishScan(const builtin_interfaces::msg::Time & stamp);

  std::mutex mutex_;
  std::unique_ptr<KinematicPlant> plant_;
  RayCaster caster_;
  ScanGeometry scan_geometry_;
  std::int64_t scan_every_steps_{1};
  double scan_range_min_{0.05};
  bool scan_use_inf_{true};
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_frame_;
  std::string scan_frame_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr clock_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr ground_truth_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Publisher<std_msgs::msg::UInt64>::SharedPtr infeasible_pub_;
  rclcpp::SubscriptionBase::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Service<nav2_msgs::srv::SetInitialPose>::SharedPtr step_map_to_odom_srv_;
  rclcpp::TimerBase::SharedPtr step_timer_;
};

}  // namespace vectorfield_sim

#endif  // VECTORFIELD_SIM__KINEMATIC_SIM_NODE_HPP_
