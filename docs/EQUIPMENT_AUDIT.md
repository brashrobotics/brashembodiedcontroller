# `/home/aras/avs_ws` equipment audit

Inspected on 2026-08-23. The source workspace was treated as read-only.

## Inventory

### B2W workspace

`unitree_b2w/src/mybotshop` contains packages for B2 bringup, control, platform
DDS integration, B2/B2W descriptions, RealSense, LiDARs, Nav2, GPS, Gazebo,
teleoperation, visualization, and web control. Its third-party tree contains
Unitree ROS messages/SDK snapshots and Livox sources.

The platform contract used by Brash is:

- namespace default: `b2_unit_001`
- ROS domain: 10
- raw Unitree topics in domain 0: `/lf/lowstate`, `/odommodestate`
- B2 command input: `hardware/cmd_vel`
- base odometry output: `base/odom`
- built-in video nodes: `b2_front_video`, `b2_rear_video`
- base frames: `odom`, `base_link`

### Z1 workspace

`b2_robot_ros2_ws/src/z1_ros2` contains `z1_description`, `z1_bringup`,
`z1_hardware_interface`, MoveIt configuration, and examples. The real-hardware
path uses `ros2_control` plus a separate `z1_controller_process.py` helper.

### LiDAR and cameras

The B2 source has a Livox MID360 profile and KISS-ICP/MOLA launch files. It has
no Hesai or XT32 driver. The camera material covers the B2 built-in RGB cameras
and RealSense D435i/D405 accessories.

## Integration defects handled by Brash

1. `b2_statepublisher` receives `LowState.imu_state`, declares an IMU publisher,
   but never copies the received fields into its message or schedules its IMU
   update method. Brash supplies `unitree_imu_bridge` with quaternion
   normalization, axis mapping, covariance, temperature, controller-tick
   unwrapping, and stale-source detection.
2. The vendor state publisher normally owns `odom -> base_link`. Brash disables
   that TF while preserving `base/odom`, so the fusion EKF is the single TF
   owner.
3. The vendor `system.launch.py` references launch/package names absent from
   this checkout (`description.launch.py`, `platform.launch.py`, and
   `b2_joystick`). Brash launches the available driver executables and B2W
   xacro directly.
4. The D435i launch file contains duplicate `camera_namespace` declarations and
   tracks an older wrapper interface. Brash includes the installed RealSense
   `rs_launch.py` and gives the camera one deterministic namespace.
5. The stock Z1 URDF roots the arm at `world`, which would create a second TF
   tree beside the mobile base. Brash uses the `z1_` frame prefix and attaches
   `z1_world` below `base_link`. Arm command control remains disabled by
   default.
6. The B2 hardware command topic is directly writable in the vendor launch.
   Brash remaps it behind a heartbeat, timeout, finite-value, and per-axis
   velocity gate.

## Build state found on this machine

The equipment directories are source-only; there are no `build`, `install`, or
`log` overlays. ROS 2 Jazzy is installed, while the B2 notes describe a prior
Humble deployment.

The Brash-only packages build and test successfully on Jazzy. A compatibility
build against the equipment sources stops before compilation because the host
is missing:

- `xacro` for `b2_description`
- `ros2_control` / `hardware_interface` for Z1
- `rosidl_generator_dds_idl` for the Unitree message packages
- the Unitree SDK2 shared libraries expected by `b2_platform`
- GStreamer app development files expected by the built-in camera driver

The existing `INSTALLATION_NOTES.md` independently records the incomplete
Unitree SDK snapshot, a B2/Go2 namespace bug in the low-level example, missing
Livox installation, and hard-coded CycloneDDS interface names. Brash does not
edit or conceal those vendor issues.

## Algorithm decision

The official Hesai ROS 2 driver is not present locally. Its current upstream
CMake configuration defaults to an `XYZI` PointCloud2, which omits the ring and
per-point timestamp fields needed for reliable tightly coupled FAST-LIO2
deskewing. The first Brash implementation therefore uses KISS-ICP for LiDAR
motion and an EKF for IMU/base/RGB-D fusion. A future tightly coupled profile
should only be enabled after an `XYZIRT` driver build, time-offset calibration,
and LiDAR-to-IMU extrinsic calibration are verified from a recorded bag.
