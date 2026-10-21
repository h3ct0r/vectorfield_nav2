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

"""Reading rosbag2 recordings (sqlite3 and mcap) with ``rosbag2_py``, Humble and later.

The storage plugin is taken from the bag's ``metadata.yaml`` (``storage_identifier``), else from
the file extension, and passed explicitly: Humble's reader does not detect it. The ROS imports
are inside the functions, so the module imports without a ROS environment.
"""

import math
from pathlib import Path

import numpy as np
import yaml

#: File extension -> rosbag2 storage plugin.
EXTENSIONS = {'.db3': 'sqlite3', '.mcap': 'mcap'}


def storage_id(uri):
    """Return the storage plugin of a bag directory or file ('' if unknown)."""
    path = Path(uri)
    if path.is_dir():
        metadata = path / 'metadata.yaml'
        if metadata.is_file():
            data = yaml.safe_load(metadata.read_text()) or {}
            info = data.get('rosbag2_bagfile_information', {})
            if info.get('storage_identifier'):
                return str(info['storage_identifier'])
        for extension, plugin in EXTENSIONS.items():
            if any(path.glob(f'*{extension}')):
                return plugin
        return ''
    return EXTENSIONS.get(path.suffix, '')


def open_reader(uri):
    """Open a bag for sequential reading; raise RuntimeError with a hint on failure."""
    import rosbag2_py

    plugin = storage_id(uri)
    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(uri), storage_id=plugin),
            rosbag2_py.ConverterOptions(
                input_serialization_format='cdr', output_serialization_format='cdr'
            ),
        )
    except RuntimeError as error:
        hint = ''
        if plugin == 'mcap':
            hint = ' (install rosbag2_storage_mcap to read mcap bags)'
        elif not (Path(uri) / 'metadata.yaml').exists():
            hint = ' (no metadata.yaml: run `ros2 bag reindex` on the directory first)'
        raise RuntimeError(f'cannot open {uri} with storage "{plugin}": {error}{hint}') from error
    return reader


def topic_types(reader):
    """Return {topic: type} of an open reader."""
    return {meta.name: meta.type for meta in reader.get_all_topics_and_types()}


def summary(reader):
    """Return (duration s, {topic: message count}) from an open reader's metadata.

    Returns (0.0, {}) when the metadata is not available.
    """
    try:
        metadata = reader.get_metadata()
    except AttributeError:
        return 0.0, {}
    duration = metadata.duration  # datetime.timedelta (Humble) or rclpy Duration (Jazzy)
    if hasattr(duration, 'total_seconds'):
        seconds = duration.total_seconds()
    elif hasattr(duration, 'nanoseconds'):
        seconds = duration.nanoseconds * 1e-9
    else:
        seconds = 0.0
    counts = {
        info.topic_metadata.name: info.message_count for info in metadata.topics_with_message_count
    }
    return float(seconds), counts


def read_messages(uri, topics, warn=print):
    """Yield (topic, message, receive time s) for the given topics, in bag order.

    Messages whose type cannot be imported are skipped, with one warning per topic.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = open_reader(uri)
    types = topic_types(reader)
    wanted = [t for t in topics if t in types]
    if not wanted:
        return
    reader.set_filter(rosbag2_py.StorageFilter(topics=wanted))
    classes = {}
    for topic in wanted:
        try:
            classes[topic] = get_message(types[topic])
        except (AttributeError, ModuleNotFoundError, ValueError) as error:
            warn(f'skipping {topic}: cannot import {types[topic]} ({error})')
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        message_class = classes.get(topic)
        if message_class is not None:
            yield topic, deserialize_message(data, message_class), stamp * 1e-9


def stamp_seconds(stamp):
    """Return a builtin_interfaces/Time as seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_of(quaternion):
    """Return the yaw of a quaternion."""
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y**2 + quaternion.z**2),
    )


class TimeMap:
    """Maps bag receive times to the stamp clock of the recorded messages.

    In simulation the stamps are simulated time and the receive times wall time; on a robot both
    are wall time, up to transport latency. The offset ``stamp - receive`` of header-stamped
    messages is smoothed (running median) and interpolated, so a real-time factor other than 1
    is followed too. Without pairs the map is the identity.
    """

    def __init__(self, receive, stamp, window=21):
        receive = np.asarray(receive, dtype=float)
        stamp = np.asarray(stamp, dtype=float)
        valid = stamp > 0.0
        receive, stamp = receive[valid], stamp[valid]
        order = np.argsort(receive)
        self._receive = receive[order]
        offset = stamp[order] - self._receive
        if offset.size > window:
            half = window // 2
            padded = np.pad(offset, half, mode='edge')
            windows = np.lib.stride_tricks.sliding_window_view(padded, window)
            offset = np.median(windows, axis=1)
        self._offset = offset

    def __call__(self, receive):
        """Return the stamp-clock time of receive time(s)."""
        if self._offset.size == 0:
            return receive
        return receive + np.interp(receive, self._receive, self._offset)
