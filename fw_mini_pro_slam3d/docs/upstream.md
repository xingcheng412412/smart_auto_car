# Upstream provenance and local changes

Algorithm: https://github.com/hku-mars/FAST_LIO/tree/ROS2
Pinned commit: a4743b095409588842a5b30ddfa27e29d2f99164
ikd-Tree: https://github.com/hku-mars/ikd-Tree
Pinned submodule: e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4

Upstream algorithm LICENSE is GPL-2.0 and is retained verbatim. The original package.xml describes its license as BSD, while repository LICENSE is GPL-2.0; use the repository license and inspect upstream before redistributing. The separately written integration package is Apache-2.0 and communicates with FAST-LIO through ROS messages.

Local changes in fast_lio/:

1. `FASTLIO_WITH_LIVOX` CMake option defaults OFF. Guard only the Livox CustomMsg include, overload, callback and subscription. PointCloud2 FAST-LIO2 estimator math is unchanged. Enabling it requires an installed livox_ros_driver2 package supplied separately.
2. Declare tf2_ros and previously implicit ROS dependencies for direct ROS 2 compilation.
3. Preserve rclcpp SIGINT handling; avoid closing a null optional debug log file.
4. Bound upstream debug-array writes and check the log handle. This avoids out-of-bounds writes on long runs.

The upstream cumulative map timer and `/map_save` are disabled in this integration. The separate `fairy_map` receives each registered scan, accumulates a bounded voxel map, publishes it and writes binary PCD with failure reporting. Its map save service is `/slam3d/save_map`.

Reproducibility: the downloaded fast_lio folder is a Git clone with its submodule and local patches. `git -C fast_lio diff` shows local modifications. Do not reset or update upstream without preserving these changes. A matching patch is stored as `fast_lio_local.patch` next to this file.
