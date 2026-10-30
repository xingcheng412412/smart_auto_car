# RSFAIRY calibration captured 2026-09-24

Read-only UDP capture from 192.168.86.200:7788. DIFOP length 1248, header `a5ff005a11115555`, tail `0ff0`. Offset 1092 contains seven network-order IEEE754 float32 values:

- q_IMU_in_LiDAR (x,y,z,w): (-0.0017696995,-0.0049832235,-0.0045967186,0.9999754429)
- t_IMU_in_LiDAR: (0.0136500001,0.00781999994,0.00492000021)
- raw bytes: `bae7f543bba34a4fbb96a0123f7ffe643c5fa4403c001f753ba137f4`

The quaternion norm is 0.9999999903. The official manuals describe these as IMU calibration relative to the LiDAR coordinate system. FAST-LIO needs the opposite transform: p_I = R_IL p_L + t_IL. Use R_IL = R_LI transpose; t_IL = -R_IL t_LI. The configured transform is this inverse. IMU accelerations and gyro values are passed through, in m/s² and rad/s according to the packet specification.

Sources:
- https://robosense-robotics.github.io/product-manual/en/Fairy_48TX/ (4.4.4, C.15)
- https://robosense-robotics.github.io/product-manual/en/Fairy_48T1/ (C.15)
- FAST-LIO src/IMU_Processing.hpp, Lidar_R_wrt_IMU / Lidar_T_wrt_IMU

Limitations: C.15 does not explicitly state the translation unit. We assume metres, consistent with the physical millimetre-scale offset and the manual example; verify with the manufacturer before accuracy acceptance. Captured MSOP model byte 0x01 maps to 48 channels in the installed decoder; the manuals' model-byte table differs, so exact 48TX/48T1 subtype is not established. This calibration is specific to this connected unit: recapture when replacing the sensor. Static operation is not a motion-based calibration validation.
