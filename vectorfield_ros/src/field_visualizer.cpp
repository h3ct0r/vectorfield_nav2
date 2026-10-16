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

#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <string_view>

#include "rclcpp/logging.hpp"
#include "rclcpp/qos.hpp"
#include "vectorfield_ros/field_state.hpp"
#include "vectorfield_ros/field_visualizer.hpp"
#include "vectorfield_ros/topics.hpp"

namespace vectorfield_ros
{

namespace
{

/// Throttle period of the error log (ms).
constexpr int kLogThrottleMs = 1000;

/// @brief True iff the publisher is active and has at least one subscriber.
template <typename PublisherT>
bool wanted(const PublisherT & publisher)
{
  return publisher && publisher->is_activated() && publisher->get_subscription_count() > 0;
}

/// @brief Seconds of a stamp.
double toSeconds(const builtin_interfaces::msg::Time & stamp) noexcept
{
  return static_cast<double>(stamp.sec) + 1.0e-9 * static_cast<double>(stamp.nanosec);
}

}  // namespace

FieldVisualizer::FieldVisualizer(
  const rclcpp_lifecycle::LifecycleNode::SharedPtr & node, const std::string & ns,
  const VizConfig & config)
: logger_(node->get_logger()), clock_(node->get_clock()), config_(config)
{
  const rclcpp::QoS latest = rclcpp::QoS(1).reliable().durability_volatile();
  const rclcpp::QoS history = rclcpp::QoS(10).reliable().durability_volatile();
  // Created through the rclcpp_lifecycle base on purpose: these publishers are activated and
  // deactivated explicitly by on_activate() / on_deactivate().
  rclcpp_lifecycle::LifecycleNode & base = *node;
  markers_pub_ = base.create_publisher<visualization_msgs::msg::MarkerArray>(
    topics::topicName(ns, topics::kMarkers), latest);
  grid_pub_ = base.create_publisher<visualization_msgs::msg::MarkerArray>(
    topics::topicName(ns, topics::kFieldGrid), latest);
  predicted_pub_ = base.create_publisher<nav_msgs::msg::Path>(
    topics::topicName(ns, topics::kPredictedPath), latest);
  window_pub_ =
    base.create_publisher<nav_msgs::msg::Path>(topics::topicName(ns, topics::kWindow), latest);
  state_pub_ = base.create_publisher<vectorfield_msgs::msg::FieldState>(
    topics::topicName(ns, topics::kFieldState), history);
  distance_pub_ = base.create_publisher<nav_msgs::msg::OccupancyGrid>(
    topics::topicName(ns, topics::kObstacleDistance), latest);
}

FieldVisualizer::~FieldVisualizer() = default;

void FieldVisualizer::on_activate()
{
  markers_pub_->on_activate();
  grid_pub_->on_activate();
  predicted_pub_->on_activate();
  window_pub_->on_activate();
  state_pub_->on_activate();
  distance_pub_->on_activate();
  last_grid_time_ = -1.0;
}

void FieldVisualizer::on_deactivate(
  const builtin_interfaces::msg::Time & stamp, std::string_view frame_id)
{
  clear(stamp, frame_id);
  markers_pub_->on_deactivate();
  grid_pub_->on_deactivate();
  predicted_pub_->on_deactivate();
  window_pub_->on_deactivate();
  state_pub_->on_deactivate();
  distance_pub_->on_deactivate();
}

void FieldVisualizer::setConfig(const VizConfig & config)
{
  config_ = config;
  last_grid_time_ = -1.0;  // a new rate takes effect at the next cycle
}

const VizConfig & FieldVisualizer::config() const noexcept { return config_; }

void FieldVisualizer::publish(const MarkerContext & context) noexcept
{
  if (!config_.visualize) {
    return;
  }
  try {
    if (wanted(markers_pub_)) {
      ++builder_calls_;
      markers_pub_->publish(builder_.markers(context));
    }
    // field_grid: at most field_grid_rate, decimated with the cycle stamps. The due time
    // advances by whole periods so jitter in the control period does not lower the rate; a gap
    // of more than one period (or time going backwards) restarts the schedule.
    const double now = toSeconds(context.stamp);
    const double period = 1.0 / config_.field_grid_rate;
    if (std::isfinite(period) && period > 0.0 && wanted(grid_pub_)) {
      const bool restart =
        last_grid_time_ < 0.0 || now < last_grid_time_ || now - last_grid_time_ >= 2.0 * period;
      if (restart || now - last_grid_time_ >= period - 1.0e-9) {
        last_grid_time_ = restart ? now : last_grid_time_ + period;
        ++builder_calls_;
        grid_pub_->publish(builder_.fieldGrid(context, config_));
      }
    }
    if (wanted(predicted_pub_)) {
      ++builder_calls_;
      predicted_pub_->publish(builder_.predictedPath(context));
    }
    if (wanted(window_pub_)) {
      ++builder_calls_;
      window_pub_->publish(builder_.window(context));
    }
    if (wanted(state_pub_) && context.output != nullptr) {
      ++builder_calls_;
      toFieldState(*context.output, context.compute_time_ms, &field_state_);
      field_state_.header.stamp = context.stamp;
      field_state_.header.frame_id.assign(context.frame_id.data(), context.frame_id.size());
      state_pub_->publish(field_state_);
    }
    if (config_.publish_obstacle_distance && wanted(distance_pub_)) {
      ++builder_calls_;
      distance_pub_->publish(builder_.obstacleDistance(context));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, kLogThrottleMs, "Visualization publish failed: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, kLogThrottleMs, "Visualization publish failed: unknown exception");
  }
}

void FieldVisualizer::clear(
  const builtin_interfaces::msg::Time & stamp, std::string_view frame_id) noexcept
{
  try {
    const bool markers = wanted(markers_pub_);
    const bool grid = wanted(grid_pub_);
    if (markers || grid) {
      ++builder_calls_;
      const visualization_msgs::msg::MarkerArray & message = builder_.deleteAll(stamp, frame_id);
      if (markers) {
        markers_pub_->publish(message);
      }
      if (grid) {
        grid_pub_->publish(message);
      }
    }
    last_grid_time_ = -1.0;
  } catch (const std::exception & e) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, kLogThrottleMs, "Visualization clear failed: %s", e.what());
  } catch (...) {
    RCLCPP_ERROR_THROTTLE(
      logger_, *clock_, kLogThrottleMs, "Visualization clear failed: unknown exception");
  }
}

std::size_t FieldVisualizer::builderCalls() const noexcept { return builder_calls_; }

}  // namespace vectorfield_ros
