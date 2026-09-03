# Copyright 2026 Brash Robotics
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

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackagePrefix, FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare('brash_quadruped_bringup')
    description_renderer = PathJoinSubstitution(
        [
            FindPackagePrefix('brash_quadruped_bringup'),
            'lib',
            'brash_quadruped_bringup',
            'render_b2w_sim_description.py',
        ]
    )
    vendor_xacro = PathJoinSubstitution(
        [FindPackageShare('b2_description'), 'xacro', 'robot.xacro']
    )
    robot_description_content = Command(
        [
            description_renderer,
            ' --xacro ',
            vendor_xacro,
            ' --controller-config ',
            PathJoinSubstitution([package_share, 'config', 'b2w_sim_controllers.yaml']),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    gz_sim_launch = PathJoinSubstitution(
        [FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py']
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'world',
                default_value=PathJoinSubstitution([package_share, 'worlds', 'empty.sdf']),
            ),
            DeclareLaunchArgument('gui', default_value='true'),
            DeclareLaunchArgument('teleop_bridge', default_value='false'),
            DeclareLaunchArgument('teleop_edge_url', default_value='http://127.0.0.1:4311/'),
            DeclareLaunchArgument('entity_name', default_value='brash_b2w'),
            DeclareLaunchArgument('spawn_x', default_value='0.0'),
            DeclareLaunchArgument('spawn_y', default_value='0.0'),
            DeclareLaunchArgument('spawn_z', default_value='0.8'),
            DeclareLaunchArgument('spawn_yaw', default_value='0.0'),
            # The vendor Xacro selects the wheeled B2W through environment flags.
            SetEnvironmentVariable('B2_DESC', '0'),
            SetEnvironmentVariable('B2_W_DESC', '1'),
            SetEnvironmentVariable('B2_ACCESSORIES', '0'),
            # Let Gazebo resolve package://b2_description mesh URIs.
            AppendEnvironmentVariable(
                'GZ_SIM_RESOURCE_PATH',
                PathJoinSubstitution([FindPackagePrefix('b2_description'), 'share']),
            ),
            # Run the simulator server in an empty ground-plane world.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={
                    'gz_args': ['-r -s -v 4 ', LaunchConfiguration('world')],
                    'on_exit_shutdown': 'true',
                }.items(),
            ),
            # Start the Gazebo GUI separately so gui:=false works on headless hosts.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(gz_sim_launch),
                launch_arguments={
                    'gz_args': '-g -v 4',
                    'on_exit_shutdown': 'true',
                }.items(),
                condition=IfCondition(LaunchConfiguration('gui')),
            ),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    robot_description,
                    {'use_sim_time': True, 'publish_frequency': 30.0},
                ],
            ),
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                name='brash_gz_bridge',
                output='screen',
                parameters=[
                    {
                        'config_file': PathJoinSubstitution(
                            [package_share, 'config', 'gz_bridge.yaml']
                        )
                    }
                ],
            ),
            # Give the world server time to expose its create service.
            TimerAction(
                period=3.0,
                actions=[
                    Node(
                        package='ros_gz_sim',
                        executable='create',
                        name='spawn_brash_b2w',
                        output='screen',
                        arguments=[
                            '-string',
                            robot_description_content,
                            '-name',
                            LaunchConfiguration('entity_name'),
                            '-x',
                            LaunchConfiguration('spawn_x'),
                            '-y',
                            LaunchConfiguration('spawn_y'),
                            '-z',
                            LaunchConfiguration('spawn_z'),
                            '-Y',
                            LaunchConfiguration('spawn_yaw'),
                            '-allow_renaming',
                            'false',
                        ],
                    )
                ],
            ),
            # These controllers claim only Gazebo interfaces. No Unitree SDK
            # process or physical command topic is started by this launch file.
            TimerAction(
                period=5.0,
                actions=[
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        name='spawn_b2w_leg_position_controller',
                        output='screen',
                        arguments=[
                            'joint_position_controller',
                            '--controller-manager',
                            '/controller_manager',
                        ],
                    ),
                    Node(
                        package='controller_manager',
                        executable='spawner',
                        name='spawn_b2w_wheel_velocity_controller',
                        output='screen',
                        arguments=[
                            'velocity_controller',
                            '--controller-manager',
                            '/controller_manager',
                        ],
                    ),
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package='brash_quadruped_driver',
                        executable='b2w_teleop_sim_bridge',
                        name='b2w_teleop_sim_bridge',
                        output='screen',
                        parameters=[
                            {
                                'use_sim_time': False,
                                'edge_url': LaunchConfiguration('teleop_edge_url'),
                            }
                        ],
                        condition=IfCondition(LaunchConfiguration('teleop_bridge')),
                    )
                ],
            ),
        ]
    )
