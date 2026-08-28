# Controller architecture

## Runtime data flow

```text
Unitree DDS domain 0
  -> domain_bridge -> /b2_unit_001/lf/lowstate
      -> unitree_imu_bridge -> sensor/imu/data -----------+
  -> domain_bridge -> /b2_unit_001/odommodestate          |
      -> b2_statepublisher -> base/odom ----------------+ |
                                                          v v
XT32 UDP -> hesai_ros_driver -> sensor/lidar/points -> KISS-ICP
                                                   -> localization/lidar_odom
                                                             |
D435i -> color + aligned depth -> RTAB-Map RGB-D odometry ----+
                                      -> localization/visual_odom
                                                             |
                                                             v
                                                    robot_localization EKF
                                                             |
                                      localization/odometry + odom -> base_link
```

## TF ownership

Each transform has exactly one owner:

| Transform | Publisher |
| --- | --- |
| `odom -> base_link` | `robot_localization/ekf_node` |
| `map -> odom` | RTAB-Map only when `start_rgbd_slam:=true` |
| B2W kinematic links | B2 `robot_state_publisher` |
| `base_link -> imu_link` | Brash static transform publisher |
| `base_link -> hesai_lidar` | Brash static transform publisher |
| `base_link -> d435i_link` | Brash static transform publisher |
| D435i internal optical frames | `realsense2_camera` |
| `base_link -> z1_world` | Brash static transform publisher |
| Z1 links under the `z1_` frame prefix | Z1 `robot_state_publisher` |

The vendor B2 odometry TF is explicitly disabled. KISS-ICP and RGB-D odometry
also publish messages without TF so they can be fused without multiple
publishers fighting over `odom -> base_link`.

## Namespaces and domains

The default robot namespace is `b2_unit_001`, matching the equipment files.
ROS 2 traffic uses domain 10. The Unitree low-level topics originate in domain
0 and are bridged into domain 10 by `config/b2_domain_bridge.yaml`.

That bridge profile contains the default namespace literally because
`domain_bridge` YAML has no launch substitution support. For another robot,
copy the file, update both `remap` values, and pass it as
`domain_bridge_config:=/absolute/path/to/file.yaml`.

## Time policy

`LowState.tick` is a wrapping 32-bit microsecond controller clock. The IMU
bridge provides two modes:

- `receipt` (default): stamp at ROS receipt time. Robust for the loosely coupled
  EKF and tolerant of controller resets.
- `tick_aligned`: unwrap the controller tick and align it to ROS time at the
  first sample. This removes inter-message transport jitter but still needs a
  calibrated constant offset to the Hesai/PTP time base.

Use `tick_aligned` only after bagging both streams and estimating
`timestamp_offset_sec`. A wrong time offset damages LIO more than receipt-time
jitter.

## Camera choices

The B2 built-in front/rear cameras are exposed by `b2_platform` and can be
enabled with `start_builtin_cameras:=true`. They provide RGB and camera info.
The D435i path is the default algorithm input because aligned metric depth makes
RGB-D odometry and local obstacle generation observable at low platform speed.

RTAB-Map odometry is an independent measurement source. RTAB-Map graph SLAM is
off by default because it owns `map -> odom`; enable it only when mapping or
loop closure is wanted.

## Tightly coupled FAST-LIO2

The supplied pipeline intentionally does not pretend that an `XYZI` cloud is a
deskewable cloud. Tightly coupled FAST-LIO2 needs a point cloud containing at
least `x`, `y`, `z`, `intensity`, `ring`, and a per-point `timestamp`, plus a
LiDAR/IMU time offset and calibrated extrinsic. The official Hesai ROS 2 driver
currently defaults to `POINT_TYPE XYZI` in its CMake file.

Before adding FAST-LIO2, build the Hesai driver with its `XYZIRT` point type,
verify the actual PointCloud2 fields, calibrate the B2 IMU time offset, and use
a ROS 2 FAST-LIO2 implementation whose output frames are configurable. Until
those checks pass, KISS-ICP plus EKF is the honest and reproducible LIO path.
