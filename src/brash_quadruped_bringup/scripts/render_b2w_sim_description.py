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


def render(xacro_path):
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
    args = parser.parse_args()

    try:
        sys.stdout.write(render(args.xacro))
    except Exception as error:  # launch needs a concise renderer failure on stderr
        print(f'Failed to render B2W simulation description: {error}', file=sys.stderr)
        return 1
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
