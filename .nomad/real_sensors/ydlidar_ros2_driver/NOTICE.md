# YDLIDAR live input

Runtime-only subset of the locally validated ydlidar_ros2_driver (package 1.0.2),
derived from https://github.com/YDLIDAR/ydlidar_ros2_driver/tree/humble .
The C++ source retains its original EAI copyright header with only trailing whitespace removed.
The upstream package declares MIT; its supplied LICENSE.txt is empty and is retained.
Build packaging omits the client, viewers, images, old launch files and device setup.
The selected node publishes matching LaserScan and PointCloud, including per-ray
`stamps` and `intensities`, and supports `auto_intensity=false`.
YDLidar SDK is an external dependency; this source was built with SDK 1.2.20.
Import hashes are in nomad_perception/config/real/import_manifest.json.
