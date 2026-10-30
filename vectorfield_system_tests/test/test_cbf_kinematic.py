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

"""CBF obstacle mode (math_spec §6.7) on vectorfield_sim for one motion model.

kinematic_sim_demo on ``open_10x10`` with ``obstacle_mode: cbf`` and the global costmap's obstacle
layer off, so the plan runs straight through the unmapped boxes. In order:

1. a 0.4 m box beside the path, its near side 0.1 m from it (closer than the footprint radius)
   and 0.8 m ahead: the filter steers around its corner, NavigateToPose SUCCEEDED, the footprint
   keeps at least cbf_margin − res from the box, no NoValidControl, and the robot rejoins the
   path and reaches the goal; Ackermann (math_spec §6.7 [Design, 0.3.0]): the same, with the
   filter at the rear axle and the turning margin, and no infeasible command reached the
   simulator;
2. the same box centred on the path (the accepted deadlock): the filter stops the robot in front
   of the face, the CBF hold raises kBlocked (NoValidControl), FollowPath fails and Nav2 asks its
   planner again; the footprint keeps at least cbf_margin − res and stops within 0.3 m of where
   the barrier holds it. Not Ackermann: the case drives back along −x, a U-turn for a car (the
   core oracle O59 covers the car's hold).
"""

import math
import unittest

import launch_testing.markers
import pytest
from action_msgs.msg import GoalStatus
from vectorfield_system_tests.harness import NO_VALID_CONTROL, TRACKING_PHASES, NavHarness
from vectorfield_system_tests.shutdown import TestCleanShutdown  # noqa: F401

from vectorfield_system_tests import checks, geometry, launch

MOTION_MODEL = launch.motion_model()
ACKERMANN = MOTION_MODEL == 'ackermann'

#: The controller's cbf_margin default (m).
CBF_MARGIN = 0.05
BOX_SIZE = 0.4
BOX_AHEAD = 0.8
BOX_GOAL = 4.0
BLOCKED_TIMEOUT = 45.0


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Start kinematic_sim_demo on open_10x10 with obstacle_mode cbf, no global obstacle layer."""
    return launch.demo(
        'kinematic_sim_demo.launch.py',
        MOTION_MODEL,
        'open_10x10',
        [launch.overlay('global_obstacle_layer_off'), launch.overlay('obstacle_mode_cbf')],
    )


class TestCbf(unittest.TestCase):
    """The box beside the path, then across it; the methods run in name order on one robot."""

    @classmethod
    def setUpClass(cls):
        """Start the client and wait for Nav2."""
        cls.nav = NavHarness()
        cls.nav.wait_until_active()

    @classmethod
    def tearDownClass(cls):
        """Stop the client."""
        cls.nav.close()

    def _box(self, lateral, heading):
        """Return (box, goal): a box BOX_AHEAD m along ``heading``, ``lateral`` m left of the path.

        The goal is BOX_GOAL m along ``heading`` (the map is 10 m wide: the cases alternate).
        """
        start = self.nav.pose()
        ux, uy = math.cos(heading), math.sin(heading)
        centre = BOX_AHEAD + 0.5 * BOX_SIZE
        box = [
            start.x + centre * ux - lateral * uy,
            start.y + centre * uy + lateral * ux,
            BOX_SIZE,
            BOX_SIZE,
            heading,
        ]
        goal = (start.x + BOX_GOAL * ux, start.y + BOX_GOAL * uy, heading)
        return box, goal

    def _clearance(self, episode, box):
        distance = episode.min_distance(lambda x, y: geometry.box_distance(x, y, box))
        return distance - geometry.ROBOT_RADIUS

    def test_1_box_beside_the_path(self):
        """A box 0.1 m beside the path: passed with the margin, the goal succeeds."""
        nav = self.nav
        start = nav.pose()
        box, goal = self._box(0.1 + 0.5 * BOX_SIZE, 0.0)
        nav.set_boxes([box])
        nav.sleep(1.0)
        episode = nav.navigate(*goal, timeout=checks.GOAL_TIMEOUT)
        nav.set_boxes([])
        nav.clear_costmaps()

        self.assertEqual(episode.status, GoalStatus.STATUS_SUCCEEDED, episode.status_name)
        self.assertEqual(episode.no_valid_control(), [], checks.describe(episode.samples, -3))
        self.assertEqual(episode.contour_activations(), 0)
        clearance = self._clearance(episode, box)
        self.assertGreaterEqual(clearance, CBF_MARGIN - geometry.RESOLUTION)
        # It went around: the path itself passes 0.1 m from the box, inside the footprint.
        self.assertLess(clearance, 0.5)
        tracking = [s for s in episode.samples if s.phase in TRACKING_PHASES]
        # The true lateral offset from the straight line of the request (heading 0): a replan
        # starts at the robot, so D against the latest plan understates a gradual detour (a car's).
        deviation = max((abs(p.y - start.y) for p in episode.poses), default=0.0)
        # One cell of tolerance, as for the clearance above: 0.119 m has been sampled against the
        # 0.12 m geometric minimum.
        self.assertGreater(
            deviation,
            geometry.ROBOT_RADIUS - 0.1 - geometry.RESOLUTION,
            'the robot never left the path',
        )
        after = [s for s in tracking if s.t > episode.end - 2.0]
        closest = min((abs(s.distance) for s in after), default=math.inf)
        print(
            f'[cbf {MOTION_MODEL}] box beside the path: clearance {clearance:.3f} m, '
            f'max lateral offset {deviation:.3f} m, final |D| {closest:.3f} m'
        )
        self.assertLess(closest, checks.tracking_limit(MOTION_MODEL), 'no rejoin')
        final = nav.settled_pose(checks.SETTLE_TIME)
        miss = math.hypot(final.x - goal[0], final.y - goal[1])
        self.assertLessEqual(miss, checks.XY_GOAL_TOLERANCE + checks.STOP_SLACK)
        if ACKERMANN:
            self.assertEqual(nav.infeasible_command_count(), 0)

    @unittest.skipIf(ACKERMANN, 'a U-turn for a car; the core oracle O59 covers its hold')
    def test_2_box_across_the_path(self):
        """A box centred on the path: hold, kBlocked, FollowPath failure, replanning."""
        nav = self.nav
        box, goal = self._box(0.0, math.pi)
        nav.set_boxes([box])
        nav.sleep(1.0)
        handle = nav.send_goal(*goal)

        def stopped():
            return next(
                (
                    s.t
                    for s in nav.field_samples_since(handle.start)
                    if s.status & NO_VALID_CONTROL
                ),
                None,
            )

        def failed():
            stop = stopped()
            return None if stop is None else next(iter(nav.follow_path_failures_since(stop)), None)

        def replanned():
            failure = failed()
            return failure is not None and bool(
                nav.plans_since(failure + 1e-6) or nav.planner_requests_since(failure + 1e-6)
            )

        nav.wait_until(replanned, BLOCKED_TIMEOUT)
        result = {'stopped': stopped(), 'failed': failed(), 'replanned': replanned()}
        nav.cancel(handle)
        episode = nav.episode(handle.start, None)
        nav.set_boxes([])
        nav.clear_costmaps()

        context = checks.describe(episode.samples, -3)
        clearance = self._clearance(episode, box)
        self.assertGreaterEqual(clearance, CBF_MARGIN - geometry.RESOLUTION, context)
        self.assertIsNotNone(result['stopped'], f'no NoValidControl status; {context}')
        self.assertIsNotNone(result['failed'], f'no FollowPath failure after it; {context}')
        self.assertTrue(result['replanned'], f'no replanning after the failure; {context}')
        # The barrier, not the collision cap, stopped it: the evaluation point e ahead of the base
        # holds δ_safe = r + e + m from the face, so the footprint stops 2 e + m short of it.
        barrier = 2.0 * geometry.evaluation_offset(MOTION_MODEL) + CBF_MARGIN
        self.assertLess(clearance, barrier + 0.3, f'stopped early; {context}')
        self.assertEqual(episode.contour_activations(), 0)
