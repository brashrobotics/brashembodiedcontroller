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
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare


def static_transform(name, child_frame, xyz_prefix, rpy_prefix, namespace, condition):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        namespace=namespace,
        name=name,
        arguments=[
            '--x', LaunchConfiguration(f'{xyz_prefix}_x'),
            '--y', LaunchConfiguration(f'{xyz_prefix}_y'),
            '--z', LaunchConfiguration(f'{xyz_prefix}_z'),
            '--roll', LaunchConfiguration(f'{rpy_prefix}_roll'),
            '--pitch', LaunchConfiguration(f'{rpy_prefix}_pitch'),
            '--yaw', LaunchConfiguration(f'{rpy_prefix}_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', child_frame,
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        condition=condition,
    )


def transform_arguments(prefix):
    return [
        DeclareLaunchArgument(f'{prefix}_x', default_value='0.0'),
        DeclareLaunchArgument(f'{prefix}_y', default_value='0.0'),
        DeclareLaunchArgument(f'{prefix}_z', default_value='0.0'),
        DeclareLaunchArgument(f'{prefix}_roll', default_value='0.0'),
        DeclareLaunchArgument(f'{prefix}_pitch', default_value='0.0'),
        DeclareLaunchArgument(f'{prefix}_yaw', default_value='0.0'),
    ]


def generate_launch_description():
    namespace = LaunchConfiguration('namespace')
    start_imu = IfCondition(LaunchConfiguration('start_imu_bridge'))
    start_hesai = IfCondition(LaunchConfiguration('start_hesai'))
    start_realsense = IfCondition(LaunchConfiguration('start_realsense'))

    actions = [
        DeclareLaunchArgument('namespace', default_value='b2_unit_001'),
        DeclareLaunchArgument('start_imu_bridge', default_value='true'),
        DeclareLaunchArgument('start_hesai', default_value='true'),
        DeclareLaunchArgument('start_realsense', default_value='true'),
        DeclareLaunchArgument('realsense_serial', default_value=chr(39) * 2),
        DeclareLaunchArgument(
            'hesai_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('brash_quadruped_bringup'), 'config', 'hesai_xt32.yaml']
            ),
        ),
        DeclareLaunchArgument(
            'imu_config',
            default_value=PathJoinSubstitution(
                [FindPackageShare('brash_quadruped_driver'), 'config', 'imu_bridge.yaml']
            ),
        ),
    ]
    actions += transform_arguments('imu')
    actions += transform_arguments('lidar')
    actions += transform_arguments('camera')
    actions += [
        LogInfo(
            msg=(
                'Brash sensor extrinsics default to zero; replace them with '
                'measured base_link transforms before mapping.'
            )
        ),
        Node(
            package='brash_quadruped_driver',
            executable='unitree_imu_bridge',
            namespace=namespace,
            name='unitree_imu_bridge',
            output='screen',
            parameters=[LaunchConfiguration('imu_config')],
            condition=start_imu,
        ),
        Node(
            package='hesai_ros_driver',
            executable='hesai_ros_driver_node',
            namespace=namespace,
            name='hesai_ros_driver_node',
            output='screen',
            parameters=[{'config_path': LaunchConfiguration('hesai_config')}],
            condition=start_hesai,
        ),
        GroupAction(
            actions=[
                SetRemap(src='/tf', dst='tf'),
                SetRemap(src='/tf_static', dst='tf_static'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution(
                            [
                                FindPackageShare('realsense2_camera'),
                                'launch',
                                'rs_launch.py',
                            ]
                        )
                    ),
                    launch_arguments={
                        'camera_namespace': [namespace, '/camera'],
                        'camera_name': 'd435i',
                        'serial_no': LaunchConfiguration('realsense_serial'),
                        'enable_color': 'true',
                        'enable_depth': 'true',
                        'enable_sync': 'true',
                        'align_depth.enable': 'true',
                        'pointcloud.enable': 'true',
                        'enable_gyro': 'true',
                        'enable_accel': 'true',
                        'unite_imu_method': '2',
                        'publish_tf': 'true',
                    }.items(),
                ),
            ],
            condition=start_realsense,
        ),
        static_transform('imu_mount_tf', 'imu_link', 'imu', 'imu', namespace, start_imu),
        static_transform(
            'lidar_mount_tf', 'hesai_lidar', 'lidar', 'lidar', namespace, start_hesai
        ),
        static_transform(
            'camera_mount_tf',
            'd435i_link',
            'camera',
            'camera',
            namespace,
            start_realsense,
        ),
    ]
    return LaunchDescription(actions)
