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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    controller_config = PathJoinSubstitution(
        [FindPackageShare('z1_bringup'), 'config', 'z1_controllers.yaml']
    )
    robot_description = {
        'robot_description': Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                PathJoinSubstitution(
                    [FindPackageShare('z1_description'), 'urdf', 'z1.urdf.xacro']
                ),
                ' name:=z1',
                ' with_gripper:=true',
                ' sim_ignition:=false',
                ' controllers:=',
                controller_config,
            ]
        )
    }
    manager = ['/', namespace, '/z1/controller_manager']
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='b2_unit_001'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('start_controller', default_value='false'),
            DeclareLaunchArgument('mount_x', default_value='0.0'),
            DeclareLaunchArgument('mount_y', default_value='0.0'),
            DeclareLaunchArgument('mount_z', default_value='0.0'),
            DeclareLaunchArgument('mount_roll', default_value='0.0'),
            DeclareLaunchArgument('mount_pitch', default_value='0.0'),
            DeclareLaunchArgument('mount_yaw', default_value='0.0'),
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=[namespace, '/z1'],
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    robot_description,
                    {'frame_prefix': 'z1_', 'use_sim_time': use_sim_time},
                ],
                remappings=tf_remaps,
            ),
            Node(
                package='controller_manager',
                executable='ros2_control_node',
                namespace=[namespace, '/z1'],
                name='controller_manager',
                output='screen',
                parameters=[robot_description, controller_config, {'use_sim_time': use_sim_time}],
            ),
            ExecuteProcess(
                cmd=[
                    'python3',
                    PathJoinSubstitution(
                        [
                            FindPackageShare('z1_hardware_interface'),
                            'scripts',
                            'z1_controller_process.py',
                        ]
                    ),
                ],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_state_broadcaster', '--controller-manager', manager],
                output='screen',
            ),
            Node(
                package='controller_manager',
                executable='spawner',
                arguments=['joint_trajectory_controller', '--controller-manager', manager],
                output='screen',
                condition=IfCondition(LaunchConfiguration('start_controller')),
            ),
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                namespace=namespace,
                name='z1_mount_tf',
                arguments=[
                    '--x', LaunchConfiguration('mount_x'),
                    '--y', LaunchConfiguration('mount_y'),
                    '--z', LaunchConfiguration('mount_z'),
                    '--roll', LaunchConfiguration('mount_roll'),
                    '--pitch', LaunchConfiguration('mount_pitch'),
                    '--yaw', LaunchConfiguration('mount_yaw'),
                    '--frame-id', 'base_link',
                    '--child-frame-id', 'z1_world',
                ],
                remappings=tf_remaps,
            ),
        ]
    )
