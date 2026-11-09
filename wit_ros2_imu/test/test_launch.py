# Copyright 2026 alianlbj23
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

"""Check that launch/imu.launch.py loads and resolves every included file."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node

LAUNCH_FILE = Path(__file__).resolve().parent.parent / 'launch' / 'imu.launch.py'


def _load_description():
    spec = importlib.util.spec_from_file_location('imu_launch', LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.generate_launch_description()


def test_launch_arguments_and_defaults():
    description = _load_description()
    args = {a.name: a.default_value for a in description.entities
            if isinstance(a, DeclareLaunchArgument)}
    assert set(args) == {'port', 'baud', 'rviz', 'rosbridge', 'foxglove', 'calibration_file'}
    context = LaunchContext()
    defaults = {name: ''.join(s.perform(context) for s in value)
                for name, value in args.items()}
    assert defaults == {
        'port': '/dev/ttyUSB0',
        'baud': '9600',
        'rviz': 'false',
        'rosbridge': 'true',
        'foxglove': 'true',
        'calibration_file': '/config/imu_calibration.yaml',
    }
    nodes = [e for e in description.entities if isinstance(e, Node)]
    assert len(nodes) == 2  # driver + rviz2 (conditional)


def test_bridge_includes_resolve_to_existing_files():
    description = _load_description()
    context = LaunchContext()
    includes = [e for e in description.entities if isinstance(e, IncludeLaunchDescription)]
    assert len(includes) == 2
    for include in includes:
        source = include.launch_description_source
        # Performs the PathJoinSubstitution and parses the upstream XML launch file.
        assert source.get_launch_description(context) is not None
        assert Path(source.location).is_file(), source.location
