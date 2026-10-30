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

"""The check that a demo shuts down cleanly, shared by the launch tests.

A launch test module imports :class:`TestCleanShutdown` into its namespace
(``from vectorfield_system_tests.shutdown import TestCleanShutdown  # noqa: F401``), and
launch_testing runs it after the shutdown signal.
"""

import unittest

import launch_testing
import launch_testing.asserts


@launch_testing.post_shutdown_test()
class TestCleanShutdown(unittest.TestCase):
    """Every process of the launch exited 0 at the shutdown signal.

    A crash at exit (-6, -11) or a process launch had to kill (-9, -15) fails: a node waiting
    on simulated time after its simulator stopped, a lifecycle manager's library unloaded under
    its own callback, a container that never returns.
    """

    def test_every_process_exits_cleanly(self, proc_info):
        """Exit code 0 for every process, the ones that ended early included."""
        launch_testing.asserts.assertExitCodes(proc_info)
