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

"""Launch the CMP10A IMU driver with optional RViz2, rosbridge and foxglove_bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

ROSBRIDGE_PORT = '9090'
FOXGLOVE_PORT = '8765'
# Inside the container; the host-side binding is decided by `docker run -p`.
BRIDGE_ADDRESS = '0.0.0.0'


def generate_launch_description():
    """Build the launch description."""
    args = [
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='Serial device or pyserial URL '
                                          '(e.g. socket://host.docker.internal:5760)'),
        DeclareLaunchArgument('baud', default_value='9600', description='Baud rate'),
        DeclareLaunchArgument('rviz', default_value='false', description='Start RViz2'),
        DeclareLaunchArgument('rosbridge', default_value='true',
                              description=f'Start rosbridge_websocket on port {ROSBRIDGE_PORT}'),
        DeclareLaunchArgument('foxglove', default_value='true',
                              description=f'Start foxglove_bridge on port {FOXGLOVE_PORT}'),
        DeclareLaunchArgument('calibration_file',
                              default_value='/config/imu_calibration.yaml',
                              description='Software calibration YAML (missing file = none)'),
    ]

    imu_node = Node(
        package='wit_ros2_imu',
        executable='wit_ros2_imu',
        name='wit_ros2_imu',
        output='screen',
        parameters=[{
            'port': ParameterValue(LaunchConfiguration('port'), value_type=str),
            'baud': ParameterValue(LaunchConfiguration('baud'), value_type=int),
            'calibration_file': ParameterValue(LaunchConfiguration('calibration_file'),
                                               value_type=str),
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution(
            [FindPackageShare('wit_ros2_imu'), 'rviz', 'imu.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # Official Jazzy launch files: rosbridge_server/launch/rosbridge_websocket_launch.xml
    # (rosbridge_websocket + rosapi_node) and foxglove_bridge/launch/foxglove_bridge_launch.xml.
    rosbridge = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('rosbridge_server'), 'launch',
             'rosbridge_websocket_launch.xml'])),
        launch_arguments={'port': ROSBRIDGE_PORT, 'address': BRIDGE_ADDRESS}.items(),
        condition=IfCondition(LaunchConfiguration('rosbridge')),
    )

    foxglove = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml'])),
        launch_arguments={'port': FOXGLOVE_PORT, 'address': BRIDGE_ADDRESS}.items(),
        condition=IfCondition(LaunchConfiguration('foxglove')),
    )

    return LaunchDescription(args + [imu_node, rviz_node, rosbridge, foxglove])
