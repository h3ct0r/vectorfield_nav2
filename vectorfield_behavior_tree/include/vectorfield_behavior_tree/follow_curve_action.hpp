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
/// @brief FollowCurveAction: the Nav2 behaviour-tree node `FollowCurve`, a client of the curve
/// follower's `follow_curve` action (docs/design/m7_follow_curve_bt.md, docs/behavior_tree.md).
///
/// The goal comes either from the `curve` port (a `vectorfield_msgs/msg/Curve`, usually a
/// blackboard entry) or from the `shape`, `parameters` and `frame_id` ports (a built-in closed
/// shape); the other ports fill the remaining goal fields. Ports that cannot make a goal fail the
/// node with `INVALID_GOAL` before anything is sent; every other check is the follower's.

#ifndef VECTORFIELD_BEHAVIOR_TREE__FOLLOW_CURVE_ACTION_HPP_
#define VECTORFIELD_BEHAVIOR_TREE__FOLLOW_CURVE_ACTION_HPP_

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "nav2_behavior_tree/bt_action_node.hpp"
#include "vectorfield_msgs/action/follow_curve.hpp"
#include "vectorfield_msgs/msg/curve.hpp"

namespace vectorfield_behavior_tree
{

/// @brief Parses built-in parameters written as `name=value` pairs separated by `;` or `,`
/// (`"a=2.0;b=1.0"`; whitespace around names and values is ignored, an empty text has none).
///
/// @param text The `parameters` port value.
/// @param names Receives the names, in order.
/// @param values Receives the values, in order.
/// @return An empty optional on success, otherwise why `text` is malformed (an empty name, a
/// missing `=`, a value that is not a finite number, a repeated name).
std::optional<std::string> parseShapeParameters(
  const std::string & text, std::vector<std::string> & names, std::vector<double> & values);

/// @brief The `FollowCurve` behaviour-tree node: sends a FollowCurve goal and returns SUCCESS
/// when the follower succeeds, FAILURE when it aborts.
///
/// - Outputs (`error_code_id`, `error_msg`, `laps_completed`, `distance_travelled`) are reset
///   when a goal starts and set from every result. An abort without an error code (the follower
///   aborts a goal preempted by another client with an empty result) reads `UNKNOWN`.
/// - A goal canceled by the server or another client returns SUCCESS, Nav2's convention.
/// - While the goal runs, each tick rebuilds the goal from the ports and sends it again when it
///   changed (for example a new `{curve}`), so the follower preempts the running goal.
/// - Halting the node cancels the goal (nav2_behavior_tree::BtActionNode::halt()).
class FollowCurveAction
: public nav2_behavior_tree::BtActionNode<vectorfield_msgs::action::FollowCurve>
{
public:
  using Action = vectorfield_msgs::action::FollowCurve;

  /// @param xml_tag_name The node name in the tree.
  /// @param action_name The default action name (`follow_curve`); the `server_name` port
  /// overrides it.
  /// @param conf The node configuration (ports and blackboard).
  FollowCurveAction(
    const std::string & xml_tag_name, const std::string & action_name,
    const BT::NodeConfiguration & conf);

  /// @brief Builds the goal; ports that make no goal fail the node without sending one.
  void on_tick() override;

  /// @brief Sends the goal again when the ports now build a different one (preemption).
  void on_wait_for_result(std::shared_ptr<const Action::Feedback> feedback) override;

  /// @brief Sets the outputs from the result; SUCCESS.
  BT::NodeStatus on_success() override;

  /// @brief Sets the outputs from the result (`UNKNOWN` if it carries no code); FAILURE.
  BT::NodeStatus on_aborted() override;

  /// @brief Sets the outputs from the result, if any; SUCCESS.
  BT::NodeStatus on_cancelled() override;

  /// @brief The ports of the node, including BtActionNode's basic ports.
  static BT::PortsList providedPorts();

private:
  /// Builds a goal from the input ports into `goal`; returns why it cannot, if it cannot.
  std::optional<std::string> makeGoal(Action::Goal & goal) const;

  /// Writes `result` (or the reset values when null) to the output ports.
  void writeOutputs(const Action::Result * result);
};

}  // namespace vectorfield_behavior_tree

#endif  // VECTORFIELD_BEHAVIOR_TREE__FOLLOW_CURVE_ACTION_HPP_
