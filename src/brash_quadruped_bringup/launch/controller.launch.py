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
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch(filename, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare('brash_quadruped_bringup'), 'launch', filename]
            )
        ),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    common = {'namespace': namespace, 'use_sim_time': use_sim_time}

    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='b2_unit_001'),
            DeclareLaunchArgument('ros_domain_id', default_value='10'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('start_domain_bridge', default_value='true'),
            DeclareLaunchArgument('start_b2_driver', default_value='true'),
            DeclareLaunchArgument('start_b2_state', default_value='true'),
            DeclareLaunchArgument('start_b2_description', default_value='true'),
            DeclareLaunchArgument('start_builtin_cameras', default_value='false'),
            DeclareLaunchArgument('start_command_guard', default_value='true'),
            DeclareLaunchArgument('start_z1', default_value='false'),
            DeclareLaunchArgument('start_z1_controller', default_value='false'),
            DeclareLaunchArgument('start_imu_bridge', default_value='true'),
            DeclareLaunchArgument('start_hesai', default_value='true'),
            DeclareLaunchArgument('start_realsense', default_value='true'),
            DeclareLaunchArgument('start_lio', default_value='true'),
            DeclareLaunchArgument('start_fusion', default_value='true'),
            DeclareLaunchArgument('start_rgbd_odometry', default_value='true'),
            DeclareLaunchArgument('start_rgbd_slam', default_value='false'),
            SetEnvironmentVariable('ROS_DOMAIN_ID', LaunchConfiguration('ros_domain_id')),
            include_launch(
                'hardware.launch.py',
                common
                | {
                    'start_domain_bridge': LaunchConfiguration('start_domain_bridge'),
                    'start_b2_driver': LaunchConfiguration('start_b2_driver'),
                    'start_b2_state': LaunchConfiguration('start_b2_state'),
                    'start_b2_description': LaunchConfiguration('start_b2_description'),
                    'start_builtin_cameras': LaunchConfiguration('start_builtin_cameras'),
                    'start_command_guard': LaunchConfiguration('start_command_guard'),
                    'start_z1': LaunchConfiguration('start_z1'),
                    'start_z1_controller': LaunchConfiguration('start_z1_controller'),
                },
            ),
            include_launch(
                'sensors.launch.py',
                {'namespace': namespace}
                | {
                    'start_imu_bridge': LaunchConfiguration('start_imu_bridge'),
                    'start_hesai': LaunchConfiguration('start_hesai'),
                    'start_realsense': LaunchConfiguration('start_realsense'),
                },
            ),
            include_launch(
                'localization.launch.py',
                common
                | {
                    'start_lio': LaunchConfiguration('start_lio'),
                    'start_fusion': LaunchConfiguration('start_fusion'),
                    'start_rgbd_odometry': LaunchConfiguration('start_rgbd_odometry'),
                    'start_rgbd_slam': LaunchConfiguration('start_rgbd_slam'),
                },
            ),
        ]
    )
