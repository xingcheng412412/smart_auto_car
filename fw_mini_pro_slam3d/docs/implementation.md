# FAIRY FAST-LIO2 integration

User approved FAST-LIO2 on 2026-09-24. Work in the requested directory; workspace root has no valid Git repository, so no worktree or root commit is possible.

## Scope and design
ROS 2 Humble, real RSFAIRY pointcloud and internal IMU. Pin official hku-mars/FAST_LIO ROS2 revision a4743b095409588842a5b30ddfa27e29d2f99164, preserve license. FAST-LIO2 odometry and incremental map, no loop closure. No chassis commands.

Keep upstream algorithm in fast_lio/. Add package fw_mini_pro_slam3d in bringup/, install the group's config/, launch/, rviz/, scripts/ into its share directory. Adapter converts driver absolute timestamp seconds into Velodyne-compatible float time (relative seconds), finite points only, preserving rings and header clock. Keep same device clock on cloud/IMU. Label IMU frame fairy_imu; no invented map-to-base TF. FAST-LIO frame camera_init -> body; publish static body -> rslidar using verified inverse LiDAR-to-IMU calibration when available.

Calibration: read raw DIFOP once before driver starts, verify packet layout/header, quaternion and SI units from manufacturer's documentation; record source and values. Do not claim calibration quality from static test. Invalid/missing calibration must be explicit.

## Tasks and validation
- [x] Bring in pinned official sources and ikd-Tree; make Livox-only dependency optional without changing LIO math. Build with Livox disabled.
- [x] Test adapter before implementation: relative timestamps at epoch-scale, sorting, NaN removal, rings, organized clouds with padding, malformed fields/truncated payload, endian handling, no valid points, invalid scan duration. Implement converter plus node; reject backward cloud/IMU timestamps instead of passing reset clocks to estimator.
- [x] Configure FAIRY, launch driver/adapter/FAST-LIO/RViz with use_sim_time false and optional driver/RViz switches. Set online extrinsic estimation false with documented hardware calibration.
- [x] Provide map save with actual error reporting (empty map, invalid path) and explicit writable destination. Preserve full session voxel map in a separate bounded voxel accumulator if upstream service only saves local map.
- [x] Build selected dependencies/packages with limited parallelism. Run unit tests and launch argument validation.
- [x] Run short real-data smoke check; confirm adapter, Odometry, registered pointcloud, nonempty saved PCD and clean shutdown of new nodes. Do not claim motion accuracy without real motion data.
- [x] Review changed files, document exact commands, upstream provenance, limitations, and verification results.

## Outcome
Built and tested; live smoke and PCD save passed. Motion accuracy and manufacturer translation-unit confirmation remain acceptance work. New nodes shut down cleanly; existing driver abort on SIGINT is documented. Root workspace is not a Git repository.
