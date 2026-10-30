# Verification on the connected RSFAIRY

2026-09-24, ROS 2 Humble. No chassis commands were issued.

- Official FAST-LIO ROS2 plus integration package built successfully.
- 13 gtest cases passed (9 point conversion, 4 voxel map). CTest/colcon summary includes 2 test-suite wrappers, yielding 15 reported tests with zero errors/failures.
- `ros2 launch fw_mini_pro_slam3d mapping.launch.py --show-args` passed.
- A 25-second real sensor run produced 5001 IMU samples (~200 Hz), 213 adapted clouds (~8.7 Hz), 211 odometry/registered frames (~8.8 Hz), and 25 map messages (~1 Hz).
- Odometry values were finite, no observed backward timestamps. Start-to-end reported translation was 0.01853 m. No controlled motion or ground truth was supplied, so this is not an accuracy or drift guarantee.
- Save service returned success. PCD header reports 4949 points; file length 79339 bytes matches header plus 4949 x 16-byte binary records. File: `maps/validation_smoke.pcd`.
- Raw statistics: `live_validation.json`.
- New adapter, FAST-LIO, map accumulator and TF nodes exited cleanly. Existing rslidar_sdk node still throws `std::system_error: Invalid argument` on shutdown (exit -6); its runtime reception works. This driver issue is outside the new algorithm changes.
- The driver logs a duplicate rosout node-name warning because launch renames its internal ROS nodes together. This does not prevent pointcloud/IMU delivery; it should be removed in launch by avoiding driver name override.
- FAST-LIO initially logs one `No point, skip this scan!` during IMU initialization, then initializes its map and publishes normally.

The independent reviewer began checking TF/time semantics but could not complete due to tool usage limits. The primary agent completed the remaining source review; do not describe the review as independently approved.
