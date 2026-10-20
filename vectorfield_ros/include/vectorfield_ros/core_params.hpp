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
/// @brief Parameter ↔ core binding: the binding-table row, `kCommonCoreParams`, toCoreConfig(),
/// the node-derived fields, the cross-parameter validator and validateFromNode() (plan §4
/// "Parameters").
///
/// **The tables are the only name ↔ field mapping** (plan §4):
/// - toCoreConfig() / applyRows() apply every table a node owns (the plugin: kCommonCoreParams;
///   the follower: kCommonCoreParams and its own kFollowerCoreParams);
/// - the CrossValidator applies each incoming name through the `set` of the row that holds it;
/// - default parity and coverage run per node over the union of its tables: the names under the
///   node's prefix must equal the tables plus an explicit ROS-only allow-list
///   (kCommonRosOnlyParams, plus the legacy path keys on Gen A/B and the follower's ROS-only
///   keys), and every row must round-trip a non-default value.
///
/// **Node-derived fields** (`CoreConfig::node`: the control frequency and the footprint) are not
/// parameters of these tables; setNodeDerivedFields() sets them after the tables are applied.
///
/// Split: the table (src/core_params_table.cpp), toCoreConfig() and toVizConfig() are
/// implemented in M2 wave 1; the CrossValidator, setNodeDerivedFields(), controllerFrequencyOf(),
/// validateParams() and validateFromNode() in src/core_params.cpp (M2 wave 2, ros-io).

#ifndef VECTORFIELD_ROS__CORE_PARAMS_HPP_
#define VECTORFIELD_ROS__CORE_PARAMS_HPP_

#include <array>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "geometry_msgs/msg/point.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/node_interfaces/node_parameters_interface.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/parameter_value.hpp"
#include "vectorfield_core/config.hpp"
#include "vectorfield_core/span.hpp"
#include "vectorfield_ros/vectorfield_common_params.hpp"
#include "vectorfield_ros/viz_config.hpp"

namespace vectorfield_ros
{

/// @brief What the binding tables write: the Tracker configuration and the follower's
/// curve-intake options. Rows of kCommonCoreParams write only `config`; the follower's
/// kFollowerCoreParams writes `curve_options` and `config.field.assumption3_margin`.
struct CoreParamTarget
{
  /// Tracker configuration (node-derived fields are set separately).
  vectorfield_core::CoreConfig config;
  /// Follower curve-intake options (validated by vectorfield_core::makeCurve).
  vectorfield_core::CurveOptions curve_options;
};

/// @brief One row of a binding table (plan §4): a parameter name and the three conversions that
/// connect the gpl `Params` struct, a `rclcpp::ParameterValue` and the core configuration.
///
/// Rows are written as stateless lambdas converted to function pointers, so a table is a
/// constant array with no dynamic initialisation and no allocation. Every row of a table must
/// satisfy, for every valid value `v` of the parameter (checked by the coverage tests):
/// `set(t, v)` succeeds and `get(t) == v`, and `from_params(p) == get(toTarget(p))`.
/// @tparam Params The gpl-generated `Params` struct of the table's YAML.
template <typename Params>
struct CoreParamRow
{
  /// Parameter name without the node prefix, e.g. "desired_speed".
  const char * name;
  /// The parameter's value in a gpl `Params` struct (as declared: double, bool or string).
  rclcpp::ParameterValue (*from_params)(const Params & params);
  /// Writes a value into the target: copies doubles and bools; parses enum strings with
  /// vectorfield_core::fromString(). Negative auto sentinels are copied unchanged (the core
  /// treats any negative value as auto). Returns false, leaving the target unchanged, for a value
  /// of the wrong type or an unknown enum string.
  bool (*set)(CoreParamTarget & target, const rclcpp::ParameterValue & value);
  /// Reads the value back from the target, in the parameter's type (enums as their parameter
  /// string, vectorfield_core::toString()).
  rclcpp::ParameterValue (*get)(const CoreParamTarget & target);
};

/// @brief The gpl-generated parameters of vectorfield_common_params.yaml.
using CommonParams = vectorfield_common_params::Params;
/// @brief The gpl-generated listener of vectorfield_common_params.yaml.
using CommonParamListener = vectorfield_common_params::ParamListener;
/// @brief A row of kCommonCoreParams.
using CommonParamRow = CoreParamRow<CommonParams>;

/// @brief Number of rows of kCommonCoreParams: every key of vectorfield_common_params.yaml except
/// the Visualization group (Field 3, Motion 19, Phases 10, Obstacles 16, Safety 3).
inline constexpr std::size_t kCommonCoreParamCount = 51;

/// @brief The binding table of vectorfield_common_params (plan §4), in YAML order.
extern const std::array<CommonParamRow, kCommonCoreParamCount> kCommonCoreParams;

/// @brief The keys of vectorfield_common_params with no core field (the Visualization group,
/// read by toVizConfig()): the common part of every node's ROS-only allow-list.
inline constexpr std::array<std::string_view, 6> kCommonRosOnlyParams = {
  "visualize",       "field_grid_size",  "field_grid_resolution",
  "field_grid_rate", "field_grid_style", "publish_obstacle_distance",
};

/// @brief The row of `rows` named `name`, or null.
/// @tparam Rows A range of CoreParamRow (a table such as kCommonCoreParams, or a Span of one).
template <typename Rows>
const typename Rows::value_type * findRow(const Rows & rows, std::string_view name) noexcept
{
  for (const auto & row : rows) {
    if (name == row.name) {
      return &row;
    }
  }
  return nullptr;
}

/// @brief Applies every row of a table: `row.set(*target, row.from_params(params))`.
/// @tparam Rows A range of `CoreParamRow<Params>`.
/// @param rows The table.
/// @param params Values from the table's listener.
/// @param target In/out; rows are applied in table order.
/// @param error If not null, receives the name of the first row that failed.
/// @return False if a row's set() failed (a value the gpl validators let through but the core
/// cannot parse: a bug); the remaining rows are still applied.
template <typename Rows, typename Params>
bool applyRows(
  const Rows & rows, const Params & params, CoreParamTarget * target, std::string * error = nullptr)
{
  bool ok = true;
  for (const auto & row : rows) {
    if (!row.set(*target, row.from_params(params)) && ok) {
      ok = false;
      if (error != nullptr) {
        *error = row.name;
      }
    }
  }
  return ok;
}

/// @brief The CoreConfig of the common parameters: kCommonCoreParams applied to a default
/// CoreConfig. Node-derived fields keep their defaults (setNodeDerivedFields()).
/// @throws vectorfield_core::ConfigError naming the parameter if a row cannot be applied.
vectorfield_core::CoreConfig toCoreConfig(const CommonParams & params);

/// @brief The VizConfig of the common parameters (the Visualization group).
/// @throws vectorfield_core::ConfigError for an unknown `field_grid_style` (gpl rejects it first).
VizConfig toVizConfig(const CommonParams & params);

/// @brief A row's name and `set`, independent of the table's Params type: what the
/// CrossValidator needs to apply incoming parameters of several tables.
struct ParamSetter
{
  /// Parameter name without the node prefix.
  std::string_view name;
  /// The row's set().
  bool (*set)(CoreParamTarget & target, const rclcpp::ParameterValue & value);
};

/// @brief Appends the setters of a table (any range of CoreParamRow).
template <typename Rows>
void appendSetters(const Rows & rows, std::vector<ParamSetter> * out)
{
  for (const auto & row : rows) {
    out->push_back(ParamSetter{row.name, row.set});
  }
}

/// @brief Sets the node-derived fields after the tables (plan §4): the control frequency and the
/// robot-frame footprint (toFootprintSpec() of the polygon). The footprint radii \f$w_{\mathrm{lat}},
/// r_{\mathrm{insc}}, r_{\mathrm{circ}}\f$ and the auto \f$\lambda, D_{\mathrm{in}},
/// D'_{\mathrm{in}}\f$ follow from them inside the core.
/// @param controller_frequency The node's control frequency (Hz): the controller_server's
/// `controller_frequency` in the plugin, the follower's own.
/// @param robot_footprint Robot-frame polygon (plugin: `Costmap2DROS::getRobotFootprint()`).
/// @param config In/out.
void setNodeDerivedFields(
  double controller_frequency, const std::vector<geometry_msgs::msg::Point> & robot_footprint,
  vectorfield_core::CoreConfig * config);

/// @brief The node's `controller_frequency`: the declared value if declared, else a parameter
/// override (e.g. from `--params-file`), else `fallback`. Non-numeric values give `fallback`.
double controllerFrequencyOf(
  const rclcpp::node_interfaces::NodeParametersInterface & parameters, double fallback = 20.0);

/// @brief Builds the configuration of a node and validates it (pure; no node needed).
///
/// toCoreConfig(params), then setNodeDerivedFields(), then
/// `vectorfield_core::validateConfig(config, costmap)`. A row that cannot be applied is reported
/// as an `invalid_value` error instead of throwing.
/// @param params The node's common parameters.
/// @param controller_frequency Control frequency (Hz).
/// @param costmap Costmap geometry for the costmap rules (nullopt skips them).
/// @param robot_footprint Robot-frame footprint polygon.
/// @param config If not null, receives the configuration that was validated.
vectorfield_core::ValidationReport validateParams(
  const CommonParams & params, double controller_frequency,
  const std::optional<vectorfield_core::CostmapInfo> & costmap,
  const std::vector<geometry_msgs::msg::Point> & robot_footprint,
  vectorfield_core::CoreConfig * config = nullptr);

/// @brief Shared validation of a node's configuration (plan §4): used by the plugin's
/// configure() and by the system_tests config check (M3).
///
/// Constructs a temporary `CommonParamListener` on `parameters` with the prefix `plugin_name`
/// (declaring the parameters if they are not declared yet, which adopts any override from a
/// params file), reads controllerFrequencyOf(), and returns validateParams() with `costmap`.
/// The listener and its callback are gone when the function returns.
/// @param parameters The node's parameters interface.
/// @param logger Logger for the listener.
/// @param plugin_name Prefix of the keys (the plugin name; "" for the follower).
/// @param costmap Costmap geometry captured at configure.
/// @param robot_footprint configure(): `getRobotFootprint()`; system_tests: the TB3 footprint.
/// @param config If not null, receives the validated configuration.
/// @throws rclcpp::exceptions::InvalidParameterValueException if a value fails a gpl validator at
/// declaration (as the plugin's own listener would).
vectorfield_core::ValidationReport validateFromNode(
  const rclcpp::node_interfaces::NodeParametersInterface::SharedPtr & parameters,
  const rclcpp::Logger & logger, const std::string & plugin_name,
  const vectorfield_core::CostmapInfo & costmap,
  const std::vector<geometry_msgs::msg::Point> & robot_footprint,
  vectorfield_core::CoreConfig * config = nullptr);

/// @brief validateFromNode() for any node-like pointer (`get_node_parameters_interface()`,
/// `get_logger()`): an rclcpp::Node, a lifecycle node or compat::LifecycleNode.
template <typename NodeT>
vectorfield_core::ValidationReport validateFromNode(
  const NodeT & node, const std::string & plugin_name,
  const vectorfield_core::CostmapInfo & costmap,
  const std::vector<geometry_msgs::msg::Point> & robot_footprint,
  vectorfield_core::CoreConfig * config = nullptr)
{
  return validateFromNode(
    node->get_node_parameters_interface(), node->get_logger(), plugin_name, costmap,
    robot_footprint, config);
}

/// @brief The cross-parameter validator (plan §4): one `add_on_set_parameters_callback`,
/// registered **after** the gpl listeners.
///
/// rclcpp runs the newest on-set callback first and stops at the first rejection, so this
/// callback sees every batch before gpl validates and commits it. For each batch it:
/// 1. ignores names outside the prefix and names under the prefix that are not in its setters
///    (`<plugin>.plugin`, the rotation shim's keys on Humble and Jazzy, the Visualization and
///    legacy keys, which gpl validates alone);
/// 2. if any of its names is present, builds the candidate `base()` and applies each of them
///    through its setter (an unknown enum string rejects the batch with the parameter's name);
/// 3. runs `vectorfield_core::validateConfig(candidate.config, costmap)` and `extra`, and rejects
///    the batch with the errors' summary if there is any error (warnings never reject).
///
/// **Threading.** validate() runs on the thread that serves parameter requests (the node's
/// executor), concurrently with the control thread; `base` and `extra` must be thread-safe.
/// `base` should build from the listener's committed values (`get_params()`), which are current
/// for every earlier accepted request, plus the node-derived fields under the owner's mutex.
///
/// **Lifetime.** The callback is registered by the constructor and removed by the destructor;
/// the owner resets it in `cleanup()` (the gpl listener's callback outlives deactivate()).
class CrossValidator
{
public:
  /// @brief Returns the configuration currently in force: the tables applied to the node's
  /// committed parameter values, plus the node-derived fields.
  using BaseProvider = std::function<CoreParamTarget()>;
  /// @brief Extra checks of a candidate (follower: validateCurveOptions); may be empty.
  using ExtraCheck =
    std::function<void(const CoreParamTarget & candidate, vectorfield_core::ValidationReport *)>;

  /// @brief Registers the callback on `parameters`.
  /// @param parameters The node's parameters interface.
  /// @param prefix Key prefix: the plugin name (a trailing '.' is added), or "" (follower).
  /// @param setters The setters of every table the node owns (appendSetters()).
  /// @param costmap CostmapInfo captured at configure (nullopt: no costmap rules; follower).
  /// @param base Provider of the committed configuration.
  /// @param logger Logger for rejections (one ERROR line per rejected batch).
  /// @param extra Optional extra checks.
  CrossValidator(
    rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters,
    const std::string & prefix, std::vector<ParamSetter> setters,
    std::optional<vectorfield_core::CostmapInfo> costmap, BaseProvider base, rclcpp::Logger logger,
    ExtraCheck extra = nullptr);

  /// @brief Removes the callback.
  ~CrossValidator();

  CrossValidator(const CrossValidator &) = delete;
  CrossValidator & operator=(const CrossValidator &) = delete;
  CrossValidator(CrossValidator &&) = delete;
  CrossValidator & operator=(CrossValidator &&) = delete;

  /// @brief The callback body (public for unit tests; registration calls it).
  /// @return `successful = false` with a reason naming the parameters and the violated rules, or
  /// `successful = true`.
  rcl_interfaces::msg::SetParametersResult validate(
    const std::vector<rclcpp::Parameter> & parameters) const;

  /// @brief The prefix including the trailing '.', or "" (follower).
  const std::string & prefix() const noexcept;

private:
  rclcpp::node_interfaces::NodeParametersInterface::SharedPtr parameters_;
  std::string prefix_;
  std::vector<ParamSetter> setters_;
  std::optional<vectorfield_core::CostmapInfo> costmap_;
  BaseProvider base_;
  ExtraCheck extra_;
  rclcpp::Logger logger_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr handle_;
};

}  // namespace vectorfield_ros

#endif  // VECTORFIELD_ROS__CORE_PARAMS_HPP_
