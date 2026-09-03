# Brash Quadruped Controller

Robot-side ROS 2 integration for the Brash B2W platform, Unitree Z1 arm,
Hesai XT32 LiDAR, and RGB-D cameras.

The controller is deliberately an integration layer. It keeps the equipment
drivers in `/home/aras/avs_ws` unchanged and owns the topic contracts, safety
gate, sensor extrinsics, odometry fusion, and top-level bringup.

## Implemented pipeline

| Equipment / algorithm | Input | Brash output |
| --- | --- | --- |
| Unitree B2W base | Unitree DDS through `domain_bridge` | base odometry, joints, battery, guarded velocity control |
| B2 onboard IMU bridge | `unitree_go/msg/LowState` | `sensor_msgs/msg/Imu` and temperature |
| Hesai XT32 | UDP packets through the official Hesai ROS 2 driver | `sensor/lidar/points` |
| LiDAR + IMU odometry | KISS-ICP plus `robot_localization` | `localization/odometry`, `odom -> base_link` |
| RealSense D435i | color, aligned depth, camera info | namespaced RGB-D streams |
| Camera odometry / SLAM | RTAB-Map | `localization/visual_odom`, optional `map -> odom` |
| Unitree Z1 | `z1_ros2` hardware interface | namespaced `ros2_control`; command controller is opt-in |

The default LiDAR pipeline is loosely coupled LIO: KISS-ICP estimates LiDAR
motion, then an EKF fuses it with the 100 Hz B2 IMU and base motion. This is the
reliable choice for the current Hesai driver, whose default `XYZI` point type
does not contain the per-point timestamps required by tightly coupled
FAST-LIO2 deskewing.

## Source workspace layout

The equipment inspection found these driver roots:

- B2W: `/home/aras/avs_ws/unitree_b2w/src/mybotshop`
- Unitree ROS messages: `/home/aras/avs_ws/unitree_b2w/src/third_party/unitree/18Mar2025_unitree_ros2/cyclonedds_ws/src/unitree`
- Z1: `/home/aras/avs_ws/b2_robot_ros2_ws/src/z1_ros2`

No files under `/home/aras/avs_ws` are modified by this project.

## Dependencies

Use ROS 2 Jazzy (the ROS distribution installed on this machine). Install the
binary algorithm dependencies where available:

```bash
sudo apt install \
  libboost-all-dev \
  libgstreamer-plugins-base1.0-dev \
  libgstreamer1.0-dev \
  libyaml-cpp-dev \
  python3-vcstool \
  ros-jazzy-domain-bridge \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-realsense2-camera \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-robot-localization \
  ros-jazzy-ros-gz \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-rosidl-generator-dds-idl \
  ros-jazzy-rtabmap-ros \
  ros-jazzy-tf2-tools \
  ros-jazzy-xacro
```

If KISS-ICP or the Hesai driver is not available as a binary, import the source
dependencies:

```bash
cd /home/aras/brashrobotics/brashquadrupedcontroller
vcs import src < brash_controller.repos
git -C src/vendor/hesai_ros_driver submodule update --init --recursive
```

Build Brash together with the local B2W, Unitree message, and Z1 source trees:

```bash
cd /home/aras/brashrobotics/brashquadrupedcontroller
source /opt/ros/jazzy/setup.bash

# The B2 vendor package still includes cv_bridge/cv_bridge.h. Jazzy provides
# cv_bridge/cv_bridge.hpp, so expose this repository's compatibility shim.
export CPLUS_INCLUDE_PATH="$PWD/compat${CPLUS_INCLUDE_PATH:+:$CPLUS_INCLUDE_PATH}"

colcon build --symlink-install --base-paths \
  src \
  /home/aras/avs_ws/unitree_b2w/src/mybotshop \
  /home/aras/avs_ws/unitree_b2w/src/third_party/unitree/18Mar2025_unitree_ros2/cyclonedds_ws/src/unitree \
  /home/aras/avs_ws/b2_robot_ros2_ws/src/z1_ros2 \
  --packages-up-to \
  brash_quadruped_bringup \
  b2_description \
  b2_platform \
  z1_bringup

source install/setup.bash
```

Do not build every package under `unitree_b2w` unless all optional Livox, GPS,
Gazebo, webserver, and camera dependencies are installed. The equipment notes
record several known failures in that all-package build. The minimum package
selection above keeps unrelated optional hardware out of the controller build.

The local B2 source bundle also expects Unitree SDK2 libraries in `/usr/local`.
Follow the SDK installation recorded in the equipment workspace's
`INSTALLATION_NOTES.md` before building `b2_platform`. See the Brash
[`equipment audit`](docs/EQUIPMENT_AUDIT.md) for the exact gaps found on this
machine.

## Hardware setup before launch

1. Put the XT32 and the LiDAR NIC on the same subnet. The supplied profile
   expects LiDAR `192.168.1.201`, host `192.168.1.100`, UDP `2368`, and PTC
   `9347`. Change `config/hesai_xt32.yaml` if the hardware differs.
2. Measure the transforms from `base_link` to `imu_link`, `hesai_lidar`, and
   `d435i_link`. The launch defaults are zero only so the graph can start; they
   are not calibration values.
3. Keep the robot supported and the Z1 workspace clear during first bringup.
   Z1 is disabled by default, and its motion controller requires a second
   explicit opt-in.
4. Synchronize the robot computer and LiDAR clock with PTP before collecting
   mapping bags. The B2 message has a microsecond controller tick but no ROS
   header; the bridge can use receipt time or tick-aligned time after the
   offset is calibrated.

## Launch

Run a B2W-only smoke test in an empty Gazebo world without starting any real
hardware drivers:

```bash
ros2 launch brash_quadruped_bringup simulation.launch.py

# Server-only mode for CI or a machine without a display
ros2 launch brash_quadruped_bringup simulation.launch.py gui:=false
```

This launch uses the vendor `b2_description` Xacro and its Gazebo control
plugin. The Brash renderer corrects the vendor simulation's stale
`*_foot_fixed` ros2_control names to the B2W URDF's actual `*_wheel_joint`
names without modifying `/home/aras/avs_ws`. It starts Gazebo-only position and
wheel velocity controllers and holds a conservative standing pose. The physical Hesai,
RealSense, and Z1 processes are intentionally not launched, and their separate
TF attachments are not Gazebo geometry.

To connect the B2W simulation to the BRASH teleoperation edge, export the
simulation edge credential without putting it on the ROS command line and opt
in to the bridge:

```bash
export BRASH_TELEOP_EDGE_TOKEN='<same value used by brashteleoperation/.env>'
ros2 launch brash_quadruped_bringup simulation.launch.py teleop_bridge:=true
```

The bridge accepts only a plain-HTTP loopback edge URL. It converts the edge's
accepted X/yaw command to wheel angular velocities, rejects lateral motion,
reports controller and joint-state health, and acknowledges ownership only
after measured wheel velocity is stopped. Any edge error, invalid state, lost
ownership, disabled safety state, or command watchdog timeout publishes zero.
Its publishers are restricted to `/joint_position_controller/commands` and
`/velocity_controller/commands`, which exist only in this Gazebo launch.

Bring up the base, sensor drivers, LIO fusion, and RGB-D odometry:

```bash
ros2 launch brash_quadruped_bringup controller.launch.py
```

Useful overrides:

```bash
# Sensors and algorithms only, suitable for a stationary bench test
ros2 launch brash_quadruped_bringup controller.launch.py \
  start_b2_driver:=false start_z1:=false

# Enable Z1 hardware state, but still do not start a command controller
ros2 launch brash_quadruped_bringup controller.launch.py start_z1:=true

# Enable Z1 trajectory control only after the real-arm safety check
ros2 launch brash_quadruped_bringup controller.launch.py \
  start_z1:=true start_z1_controller:=true

# Enable RTAB-Map graph SLAM; it publishes map -> odom
ros2 launch brash_quadruped_bringup controller.launch.py start_rgbd_slam:=true
```

The B2 driver is remapped to accept commands only from the Brash velocity
guard. The guard requires a live enable heartbeat and fresh velocity commands:

```bash
ros2 topic pub -r 5 /b2_unit_001/safety/enable std_msgs/msg/Bool '{data: true}'
ros2 topic pub -r 10 /b2_unit_001/command/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.1}, angular: {z: 0.0}}'
```

Stopping either publisher commands zero velocity within the configured
timeout. This gate does not replace the physical emergency stop.

## Validate the sensor contract

```bash
ros2 topic hz /b2_unit_001/sensor/imu/data
ros2 topic echo --once /b2_unit_001/sensor/imu/data
ros2 topic echo --once /b2_unit_001/sensor/lidar/points
ros2 topic hz /b2_unit_001/camera/d435i/color/image_raw
ros2 topic echo --once /b2_unit_001/localization/odometry
ros2 run tf2_tools view_frames
```

Expected TF ownership is documented in
[`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md).
