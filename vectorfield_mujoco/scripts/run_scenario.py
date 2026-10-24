#!/usr/bin/env python3
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

"""Drive a MuJoCo example's scenario: its waypoints with Nav2, or its curve with the follower.

``ros2 run vectorfield_mujoco run_scenario SCENARIO`` reads a scenario
(``config/scenarios/<name>.yaml`` or a path; ``vectorfield_mujoco.examples.load_scenario``),
waits until Nav2 is active (and, for a curve, the ``follow_curve`` action server), then runs
vectorfield_bringup's clients:

- ``waypoints``: ``navigate_waypoints.py --poses ...`` (Nav2's simple commander, one
  NavigateToPose goal per pose);
- ``curve`` and ``laps``: ``send_curve.py --shape ... --laps ...`` (NavigateToPose to the curve,
  then the FollowCurve goal).

It prints the outcome with the wall and simulated durations and exits with 0 when every goal
succeeded, 1 otherwise (or on timeout), 2 for an invalid scenario. The example launches start it
with their scenario; ``exit_on_done:=true`` ends the launch with it.
"""

import argparse
import os
import subprocess
import sys
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from vectorfield_mujoco import examples

#: The waits for Nav2 and the follower (wall s): software rendering slows the first start.
READY_TIMEOUT = 300.0


def bringup_script(name):
    """Return the path of an installed vectorfield_bringup script."""
    from ament_index_python.packages import get_package_prefix

    prefix = get_package_prefix('vectorfield_bringup')
    return os.path.join(prefix, 'lib', 'vectorfield_bringup', name)


def client_command(scenario):
    """Return the vectorfield_bringup client command line that drives a scenario's route.

    Raises:
        ValueError: The scenario has no route.

    """
    if 'waypoints' in scenario:
        poses = ' '.join(','.join(f'{v:g}' for v in pose) for pose in scenario['waypoints'])
        return [bringup_script('navigate_waypoints.py'), '--poses', poses]
    if 'curve' in scenario:
        command = [bringup_script('send_curve.py'), '--shape', scenario['curve']]
        command += ['--laps', str(scenario['laps'])]
        if scenario.get('speed'):
            command += ['--speed', f'{scenario["speed"]:g}']
        return command
    raise ValueError('the scenario has no waypoints and no curve')


def parse_arguments(argv):
    """Parse the command line (ROS arguments removed)."""
    parser = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    parser.add_argument('scenario', help='a name in config/scenarios or a scenario YAML path')
    parser.add_argument(
        '--timeout', type=float, default=0.0, help='wall-time limit of the route (s); 0: none'
    )
    return parser.parse_args(argv)


class Waiter:
    """A node on simulated time that waits for Nav2 and reads the clock."""

    def __init__(self):
        self.node = rclpy.create_node(
            'run_scenario', parameter_overrides=[Parameter('use_sim_time', value=True)]
        )

    def sim_time(self, timeout=2.0):
        """Return the latest simulated time (s), spinning up to ``timeout`` wall s for /clock."""
        deadline = time.monotonic() + timeout
        before = self.node.get_clock().now().nanoseconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.05)
            if self.node.get_clock().now().nanoseconds != before:
                break
        return self.node.get_clock().now().nanoseconds * 1e-9

    def wait_for_nav2(self):
        """Wait until bt_navigator (and the controller) are active."""
        from nav2_simple_commander.robot_navigator import BasicNavigator

        navigator = BasicNavigator('run_scenario_navigator')
        try:
            # controller_server stands in for the localizer, which these setups do not have.
            navigator.waitUntilNav2Active(localizer='controller_server')
        finally:
            navigator.destroyNode()

    def wait_for_follower(self, timeout=READY_TIMEOUT):
        """Return whether the follow_curve action server appeared within ``timeout`` wall s."""
        from rclpy.action import ActionClient

        from vectorfield_msgs.action import FollowCurve

        client = ActionClient(self.node, FollowCurve, 'follow_curve')
        try:
            return client.wait_for_server(timeout_sec=timeout)
        finally:
            client.destroy()

    def close(self):
        """Destroy the node."""
        self.node.destroy_node()


def main():
    """Run the scenario; return the exit status."""
    args = parse_arguments(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    try:
        path = examples.resolve_scenario(args.scenario)
        scenario = examples.load_scenario(path)
        command = client_command(scenario)
    except (OSError, ValueError) as e:
        print(f'run_scenario: {e}', file=sys.stderr)
        return 2
    name = os.path.splitext(os.path.basename(path))[0]
    # Without rclpy's signal handlers Ctrl-C raises KeyboardInterrupt here, and the clients
    # (which get it too) cancel their goals.
    rclpy.init(args=sys.argv, signal_handler_options=SignalHandlerOptions.NO)
    waiter = Waiter()
    logger = waiter.node.get_logger()
    code = 1
    try:
        logger.info(f'scenario {name}: {scenario.get("description", "")}; waiting for Nav2')
        waiter.wait_for_nav2()
        if 'curve' in scenario and not waiter.wait_for_follower():
            logger.error('no follow_curve action server (is the curve follower launch running?)')
            return 1
        start_wall, start_sim = time.monotonic(), waiter.sim_time()
        logger.info(f'scenario {name}: running {" ".join(os.path.basename(c) for c in command)}')
        try:
            code = subprocess.run(command, timeout=args.timeout or None, check=False).returncode
        except subprocess.TimeoutExpired:
            logger.error(f'scenario {name}: no result after {args.timeout:g} s')
            code = 1
        wall, sim = time.monotonic() - start_wall, waiter.sim_time() - start_sim
        outcome = 'SUCCEEDED' if code == 0 else 'FAILED'
        logger.info(f'scenario {name}: {outcome} in {sim:.1f} s simulated ({wall:.1f} s wall)')
    except KeyboardInterrupt:
        code = 1
    finally:
        waiter.close()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if code == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
