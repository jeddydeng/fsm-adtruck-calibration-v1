# LiDAR intrinsic/configuration files

RoboSense per-laser factory corrections are supplied by each unit's DIFOP stream (`wait_for_difop: true`); they cannot be estimated from the saved XYZ PCD frames. The files here preserve model, port, distance and topic configuration. Extrinsics are in `../extrinsics/lidar_relative_to_roof_lidar.yaml`.
