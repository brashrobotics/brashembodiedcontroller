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
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    b2_params = PathJoinSubstitution(
        [FindPackageShare('b2_platform'), 'config', 'b2_platform.yaml']
    )
    robot_description = {
        'robot_description': Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution(
                    [FindPackageShare('b2_description'), 'xacro', 'robot.xacro']
                ),
            ]
        )
    }
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    actions = [
        DeclareLaunchArgument('namespace', default_value='b2_unit_001'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_domain_bridge', default_value='true'),
        DeclareLaunchArgument('start_b2_driver', default_value='true'),
        DeclareLaunchArgument('start_b2_state', default_value='true'),
        DeclareLaunchArgument('start_b2_description', default_value='true'),
        DeclareLaunchArgument('start_builtin_cameras', default_value='false'),
        DeclareLaunchArgument('start_command_guard', default_value='true'),
        DeclareLaunchArgument('start_z1', default_value='false'),
        DeclareLaunchArgument('start_z1_controller', default_value='false'),
        DeclareLaunchArgument(
            'domain_bridge_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('brash_quadruped_bringup'), 'config', 'b2_domain_bridge.yaml']
            ),
        ),
        SetEnvironmentVariable('B2_NS', namespace),
        SetEnvironmentVariable('B2_DESC', '0'),
        SetEnvironmentVariable('B2_W_DESC', '1'),
        Node(
            package='domain_bridge',
            executable='domain_bridge',
            name='b2_domain_bridge',
            arguments=[LaunchConfiguration('domain_bridge_config')],
            output='screen',
            condition=IfCondition(LaunchConfiguration('start_domain_bridge')),
        ),
        Node(
            package='b2_platform',
            executable='b2_highroscontrol',
            namespace=namespace,
            name='b2_hardware',
            output='screen',
            parameters=[b2_params],
            remappings=tf_remaps
            + [('hardware/cmd_vel', 'hardware/cmd_vel_guarded')],
            condition=IfCondition(LaunchConfiguration('start_b2_driver')),
        ),
        Node(
            package='b2_platform',
            executable='b2_statepublisher',
            namespace=namespace,
            name='b2_statepublisher',
            output='screen',
            parameters=[
                b2_params,
                {
                    'robot_odom_pub': False,
                    'robot_odom_tf': 'odom',
                    'robot_baselink_tf': 'base_link',
                },
            ],
            remappings=tf_remaps,
            condition=IfCondition(LaunchConfiguration('start_b2_state')),
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            name='b2_robot_state_publisher',
            output='screen',
            parameters=[
                robot_description,
                {'use_sim_time': use_sim_time, 'publish_frequency': 30.0},
            ],
            remappings=tf_remaps,
            condition=IfCondition(LaunchConfiguration('start_b2_description')),
        ),
        Node(
            package='b2_platform',
            executable='b2_video',
            namespace=namespace,
            name='b2_front_video',
            output='screen',
            parameters=[b2_params],
            remappings=tf_remaps,
            condition=IfCondition(LaunchConfiguration('start_builtin_cameras')),
        ),
        Node(
            package='b2_platform',
            executable='b2_video',
            namespace=namespace,
            name='b2_rear_video',
            output='screen',
            parameters=[b2_params],
            remappings=tf_remaps,
            condition=IfCondition(LaunchConfiguration('start_builtin_cameras')),
        ),
        Node(
            package='brash_quadruped_driver',
            executable='velocity_guard',
            namespace=namespace,
            name='velocity_guard',
            output='screen',
            parameters=[
                PathJoinSubstitution(
                    [FindPackageShare('brash_quadruped_driver'), 'config', 'velocity_guard.yaml']
                )
            ],
            condition=IfCondition(LaunchConfiguration('start_command_guard')),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution(
                    [FindPackageShare('brash_quadruped_bringup'), 'launch', 'z1.launch.py']
                )
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': use_sim_time,
                'start_controller': LaunchConfiguration('start_z1_controller'),
            }.items(),
            condition=IfCondition(LaunchConfiguration('start_z1')),
        ),
    ]
    return LaunchDescription(actions)
