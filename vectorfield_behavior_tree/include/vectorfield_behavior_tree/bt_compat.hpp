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
/// @brief BT.CPP and nav2_behavior_tree compatibility: the only distro switch of this package
/// (docs/design/follow_curve_bt.md, "Distro differences").
///
/// | | Humble | Jazzy | Lyrical, Rolling |
/// |---|---|---|---|
/// | BT.CPP | v3, `behaviortree_cpp_v3/` | v4, `behaviortree_cpp/` | v4 |
/// | blackboard `node` (`BtNode`) | `rclcpp::Node` | `rclcpp::Node` | `nav2::LifecycleNode` |
///
/// `VF_BTCPP_VERSION` (3 or 4) is defined by CMake from the BT.CPP package that
/// nav2_behavior_tree pulls in, so a machine with both BT.CPP versions installed still compiles
/// against the one Nav2 uses. `VF_BT_HAS_ROS_COMMON` probes nav2_ros_common like
/// vectorfield_ros's compat.hpp.
///
/// The header also includes Nav2's `BT::convertFromString` specializations (`bt_conversions.hpp`
/// on Humble, `bt_utils.hpp` later). A translation unit that puts `std::chrono::milliseconds` or
/// a `PoseStamped` on a blackboard without them instantiates BT.CPP's generic, throwing
/// conversion under the same symbol, which can then replace Nav2's in the plugins: a literal
/// `server_timeout="1000"` fails to parse.

#ifndef VECTORFIELD_BEHAVIOR_TREE__BT_COMPAT_HPP_
#define VECTORFIELD_BEHAVIOR_TREE__BT_COMPAT_HPP_

#ifndef VF_BTCPP_VERSION
#error "VF_BTCPP_VERSION is not defined; link vectorfield_behavior_tree's CMake targets"
#endif

#if VF_BTCPP_VERSION >= 4
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/bt_cout_logger.h"
#include "behaviortree_cpp/utils/shared_library.h"
#else
#include "behaviortree_cpp_v3/bt_factory.h"
#include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
#include "behaviortree_cpp_v3/utils/shared_library.h"
#endif

#if __has_include(<nav2_behavior_tree/bt_utils.hpp>)
#include "nav2_behavior_tree/bt_utils.hpp"
#else
#include "nav2_behavior_tree/bt_conversions.hpp"
#endif

/// @def VF_BT_HAS_ROS_COMMON
/// @brief 1 when nav2_ros_common exists (Lyrical, Rolling): the blackboard `node` is then a
/// `nav2::LifecycleNode`.
#if __has_include(<nav2_ros_common/lifecycle_node.hpp>)
#define VF_BT_HAS_ROS_COMMON 1
#include "nav2_ros_common/lifecycle_node.hpp"
#else
#define VF_BT_HAS_ROS_COMMON 0
#include "rclcpp/rclcpp.hpp"
#endif

namespace vectorfield_behavior_tree::compat
{

#if VF_BT_HAS_ROS_COMMON
/// The node type that nav2_behavior_tree's action nodes read from the blackboard key `node`.
using BtNode = nav2::LifecycleNode;
#else
/// The node type that nav2_behavior_tree's action nodes read from the blackboard key `node`.
using BtNode = rclcpp::Node;
#endif

/// @brief Whether BtActionNode::tick() starts a new goal in node status `status` (v3, Humble:
/// IDLE; v4: not active, `BT::isStatusActive`).
inline bool startsGoal(BT::NodeStatus status)
{
#if VF_BTCPP_VERSION >= 4
  return !BT::isStatusActive(status);
#else
  return status == BT::NodeStatus::IDLE;
#endif
}

/// @brief Ticks the root of `tree` once and returns its status (v3 `tickRoot`, v4 `tickOnce`).
inline BT::NodeStatus tickOnce(BT::Tree & tree)
{
#if VF_BTCPP_VERSION >= 4
  return tree.tickOnce();
#else
  return tree.tickRoot();
#endif
}

}  // namespace vectorfield_behavior_tree::compat

#endif  // VECTORFIELD_BEHAVIOR_TREE__BT_COMPAT_HPP_
