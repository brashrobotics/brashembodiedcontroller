#!/usr/bin/env python3
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

"""Render the vendor B2W Xacro with its Gazebo wheel-joint names corrected."""

import argparse
import os
import sys

import xacro


WHEEL_JOINT_ALIASES = {
    'FL_foot_fixed': 'FL_wheel_joint',
    'FR_foot_fixed': 'FR_wheel_joint',
    'RL_foot_fixed': 'RL_wheel_joint',
    'RR_foot_fixed': 'RR_wheel_joint',
}


def _element(document, parent, tag, text=None, **attributes):
    child = document.createElement(tag)
    for name, value in attributes.items():
        child.setAttribute(name, str(value))
    if text is not None:
        child.appendChild(document.createTextNode(str(text)))
    parent.appendChild(child)
    return child


def _add_simulation_camera(document, robot):
    """Attach a small forward RGB camera used only by the Gazebo launch."""
    if any(link.getAttribute('name') == 'brash_sim_camera_link'
           for link in document.getElementsByTagName('link')):
        raise RuntimeError('B2W description already contains brash_sim_camera_link')

    link = _element(document, robot, 'link', name='brash_sim_camera_link')
    inertial = _element(document, link, 'inertial')
    _element(document, inertial, 'origin', xyz='0 0 0', rpy='0 0 0')
    _element(document, inertial, 'mass', value='0.01')
    _element(
        document,
        inertial,
        'inertia',
        ixx='1e-6',
        ixy='0',
        ixz='0',
        iyy='1e-6',
        iyz='0',
        izz='1e-6',
    )
    visual = _element(document, link, 'visual')
    _element(document, visual, 'origin', xyz='0 0 0', rpy='0 0 0')
    geometry = _element(document, visual, 'geometry')
    _element(document, geometry, 'box', size='0.035 0.08 0.035')
    material = _element(document, visual, 'material', name='brash_camera_dark')
    _element(document, material, 'color', rgba='0.04 0.06 0.05 1')

    joint = _element(document, robot, 'joint', name='brash_sim_camera_joint', type='fixed')
    _element(document, joint, 'parent', link='trunk')
    _element(document, joint, 'child', link='brash_sim_camera_link')
    _element(document, joint, 'origin', xyz='0.49 0 0.10', rpy='0 0 0')

    gazebo = _element(document, robot, 'gazebo', reference='brash_sim_camera_link')
    sensor = _element(document, gazebo, 'sensor', name='brash_sim_front_camera', type='camera')
    _element(document, sensor, 'always_on', 'true')
    _element(document, sensor, 'update_rate', '10')
    _element(document, sensor, 'visualize', 'true')
    _element(document, sensor, 'topic', '/camera/b2w/front')
    camera = _element(document, sensor, 'camera')
    _element(document, camera, 'horizontal_fov', '1.0471975512')
    image = _element(document, camera, 'image')
    _element(document, image, 'width', '640')
    _element(document, image, 'height', '360')
    _element(document, image, 'format', 'R8G8B8')
    clip = _element(document, camera, 'clip')
    _element(document, clip, 'near', '0.05')
    _element(document, clip, 'far', '30.0')


def render(xacro_path, controller_config=None):
    """Return a B2W URDF after applying the vendor simulation compatibility fix."""
    os.environ['B2_DESC'] = '0'
    os.environ['B2_W_DESC'] = '1'
    os.environ['B2_ACCESSORIES'] = '0'

    document = xacro.process_file(xacro_path)
    robot = document.documentElement
    urdf_joint_names = {
        joint.getAttribute('name')
        for joint in robot.childNodes
        if getattr(joint, 'tagName', None) == 'joint'
    }

    for control in document.getElementsByTagName('ros2_control'):
        for joint in control.getElementsByTagName('joint'):
            old_name = joint.getAttribute('name')
            if old_name in WHEEL_JOINT_ALIASES:
                joint.setAttribute('name', WHEEL_JOINT_ALIASES[old_name])

    for element in document.getElementsByTagName('joint_name'):
        if element.firstChild is None:
            continue
        old_name = element.firstChild.nodeValue.strip()
        if old_name in WHEEL_JOINT_ALIASES:
            element.firstChild.nodeValue = WHEEL_JOINT_ALIASES[old_name]

    if controller_config is not None:
        controller_config = os.path.abspath(controller_config)
        if not os.path.isfile(controller_config):
            raise RuntimeError(f'simulation controller config does not exist: {controller_config}')
        updated_parameters = 0
        for plugin in document.getElementsByTagName('plugin'):
            if plugin.getAttribute('name') != 'gz_ros2_control::GazeboSimROS2ControlPlugin':
                continue
            for element in plugin.getElementsByTagName('parameters'):
                if element.firstChild is None:
                    element.appendChild(document.createTextNode(controller_config))
                else:
                    element.firstChild.nodeValue = controller_config
                updated_parameters += 1
        if updated_parameters != 1:
            raise RuntimeError(
                'expected exactly one gz_ros2_control controller parameter path, '
                f'found {updated_parameters}'
            )

    _add_simulation_camera(document, robot)

    control_joint_names = {
        joint.getAttribute('name')
        for control in document.getElementsByTagName('ros2_control')
        for joint in control.getElementsByTagName('joint')
    }
    missing = sorted(control_joint_names - urdf_joint_names)
    if missing:
        raise RuntimeError(
            'ros2_control references joints missing from the B2W URDF: '
            + ', '.join(missing)
        )

    return document.toxml()


def main():
    """Render the corrected description to standard output for ROS launch."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--xacro', required=True, help='Vendor B2W robot.xacro path')
    parser.add_argument(
        '--controller-config',
        help='Optional replacement gz_ros2_control YAML used by the rendered simulation URDF',
    )
    args = parser.parse_args()

    try:
        sys.stdout.write(render(args.xacro, args.controller_config))
    except Exception as error:  # launch needs a concise renderer failure on stderr
        print(f'Failed to render B2W simulation description: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
