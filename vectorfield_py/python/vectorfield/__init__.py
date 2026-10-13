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

"""Python bindings for vectorfield_core, the ROS-free core of vectorfield_nav2.

At M0 the package exposes only ``__version__``, the version of the compiled core. Curves,
fields, tracking laws, the Tracker, the occupancy grid and the simulator arrive in M4.
"""

from ._vectorfield import __version__

__all__ = ['__version__']
