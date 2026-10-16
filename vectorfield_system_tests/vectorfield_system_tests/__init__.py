# Copyright 2026 Héctor Azpúrua
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Shared helpers of the vectorfield_system_tests launch tests.

- :mod:`.launch`: the launch descriptions under test, bringup's demos run headless with the test
  overlays of ``test/config``;
- :mod:`.harness`: :class:`~.harness.NavHarness`, an rclpy client that waits for Nav2, sends
  NavigateToPose goals, drives vectorfield_sim and records the controller's field_state, the true
  pose, Nav2's plans and FollowPath failures;
- :mod:`.geometry`: clearance to boxes and to the walls of a map, and the controller's auto
  contour distance;
- :mod:`.checks`: the assertions shared by several launch tests (the route, the final-command
  publishers).
"""
