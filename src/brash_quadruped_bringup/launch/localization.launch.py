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
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    return LaunchDescription(
        [
            DeclareLaunchArgument('namespace', default_value='b2_unit_001'),
            DeclareLaunchArgument('use_sim_time', default_value='false'),
            DeclareLaunchArgument('start_lio', default_value='true'),
            DeclareLaunchArgument('start_fusion', default_value='true'),
            DeclareLaunchArgument('start_rgbd_odometry', default_value='true'),
            DeclareLaunchArgument('start_rgbd_slam', default_value='false'),
            DeclareLaunchArgument(
                'kiss_icp_config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('brash_quadruped_bringup'), 'config', 'kiss_icp_xt32.yaml']
                ),
            ),
            DeclareLaunchArgument(
                'ekf_config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('brash_quadruped_bringup'), 'config', 'ekf_lio.yaml']
                ),
            ),
            DeclareLaunchArgument(
                'rgbd_odometry_config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('brash_quadruped_bringup'), 'config', 'rgbd_odometry.yaml']
                ),
            ),
            DeclareLaunchArgument(
                'rtabmap_slam_config',
                default_value=PathJoinSubstitution(
                    [FindPackageShare('brash_quadruped_bringup'), 'config', 'rtabmap_slam.yaml']
                ),
            ),
            Node(
                package='kiss_icp',
                executable='kiss_icp_node',
                namespace=namespace,
                name='kiss_icp',
                output='screen',
                parameters=[
                    LaunchConfiguration('kiss_icp_config'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=tf_remaps
                + [
                    ('pointcloud_topic', 'sensor/lidar/points'),
                    ('/kiss/odometry', 'localization/lidar_odom'),
                    ('kiss/odometry', 'localization/lidar_odom'),
                ],
                condition=IfCondition(LaunchConfiguration('start_lio')),
            ),
            Node(
                package='rtabmap_odom',
                executable='rgbd_odometry',
                namespace=namespace,
                name='rgbd_odometry',
                output='screen',
                parameters=[
                    LaunchConfiguration('rgbd_odometry_config'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=tf_remaps
                + [
                    ('rgb/image', 'camera/d435i/color/image_raw'),
                    ('depth/image', 'camera/d435i/aligned_depth_to_color/image_raw'),
                    ('rgb/camera_info', 'camera/d435i/color/camera_info'),
                    ('imu', 'sensor/imu/data'),
                    ('odom', 'localization/visual_odom'),
                ],
                condition=IfCondition(LaunchConfiguration('start_rgbd_odometry')),
            ),
            Node(
                package='robot_localization',
                executable='ekf_node',
                namespace=namespace,
                name='lio_ekf',
                output='screen',
                parameters=[LaunchConfiguration('ekf_config'), {'use_sim_time': use_sim_time}],
                remappings=tf_remaps + [('odometry/filtered', 'localization/odometry')],
                condition=IfCondition(LaunchConfiguration('start_fusion')),
            ),
            Node(
                package='rtabmap_slam',
                executable='rtabmap',
                namespace=namespace,
                name='rtabmap',
                output='screen',
                parameters=[
                    LaunchConfiguration('rtabmap_slam_config'),
                    {'use_sim_time': use_sim_time},
                ],
                remappings=tf_remaps
                + [
                    ('rgb/image', 'camera/d435i/color/image_raw'),
                    ('depth/image', 'camera/d435i/aligned_depth_to_color/image_raw'),
                    ('rgb/camera_info', 'camera/d435i/color/camera_info'),
                    ('odom', 'localization/odometry'),
                ],
                arguments=['--ros-args', '--log-level', 'info'],
                condition=IfCondition(LaunchConfiguration('start_rgbd_slam')),
            ),
        ]
    )
