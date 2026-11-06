Jetson traversability source delivery
Layout revision: traversability-jetson-20261106-3
Costmap code: nomad-costmap-cpp-20261105-2 (unchanged)
Temporary converter: ade20k-to-field12-temporary-20261106-2

Location: /home/nvidia/nomad_ws/src/nomad_perception/traversability

Build entrypoint: ./build.sh [--check]
Original C++-2 source/ remains byte-for-byte frozen (113 files).
The parent nomad_perception package and existing shared workspace install are unchanged.
COLCON_IGNORE is inside traversability only. This is a private CMake build.
Build-time Python/ROS generation is allowed. Runtime shell and native C++ do not call Python.
No GPU is used by the converter. This does not replace the external segmentation inference.

Folders:
source/                 frozen C++-2 source, interfaces, vendor SDK and driver
semantic_adapter/       separate C++ converter, explicit mapping and tests
build/                  compiler/test output (jobs default 2)
install/                private intermediate installation
runtime/                previous revision 2 runtime, preserved on this Jetson
runtime-semantic-test/  NEW default output for this revision

Build:
./build.sh --check
./build.sh

The build refuses a nonempty output runtime. For another build:
NOMAD_RUNTIME_DIR="$PWD/runtime-semantic-test-2" ./build.sh

NOMAD_BUILD_DIR, NOMAD_INSTALL_PREFIX and NOMAD_RUNTIME_DIR must stay inside this
folder, separate from one another and source/. NOMAD_BUILD_JOBS changes parallelism.
NOMAD_ROS_PREFIX defaults /opt/ros/humble/install.
NOMAD_WORKSPACE_ROOT defaults /home/nvidia/nomad_ws.
The native-only TF2 dependency closure is packaged from that workspace.
LiDAR SDK is built static (BUILD_SHARED_LIBS=OFF) as required by the driver.
The build runs the original 3 costmap CTests plus 2 converter CTests.
The ROS converter test uses isolated local domain 216 with mock images, no sensors.

Run ONLY the converter (foreground, Ctrl+C to stop):
./run_semantic_adapter.sh

It defaults to runtime-semantic-test, ROS_DOMAIN_ID=59, ROS_LOCALHOST_ONLY=0,
rmw_fastrtps_cpp. Match the domain used by the existing publishers and costmap.
The launcher sets native library paths directly without sourcing Python ROS setup.
It does not start the semantic model, ZED, Pi5, LiDAR or costmap.
The converter is a separate process; the existing costmap manager does not launch it.
NOMAD_RUNTIME_DIR can select another built native runtime for the launcher.
NOMAD_SEMANTIC_MODEL_CONFIG defaults /home/nvidia/models/segformer/ade20k_b2/config.json.
NOMAD_SEMANTIC_MAPPING can select another compatible mapping file.
The source model id2label must match every source class name/number in the mapping.
Bad mappings fail startup. Malformed image messages are dropped with throttled warnings.

Input:  /perception/semantic/ade20k/class_ids  sensor_msgs/msg/Image
Output: /perception/semantic/class_ids         sensor_msgs/msg/Image
Both:   640x360 mono8, frame zed2_23472639_left_rectified_optical_frame
QoS:    KEEP_LAST 5, BEST_EFFORT, VOLATILE
Original acquisition stamp and optical frame are preserved. Row padding is removed.
No resizing, frame transform, inference, forced publish timer or timestamp refresh.
Downstream costmap freshness/projection/calibration/expiry/math remain unchanged.
Input classes are ZERO-based 0..149, NOT the one-based ADE150 dataset list.
Raw 150..255 become project ignore 0. ADE wall 0 becomes artificial 10, not ignore.

Temporary mapping examples:
person -> 1; car/bus/truck/van/bicycle and other vehicles -> 2
whole tree/grass/plant/flower/palm -> vegetation 9
rock -> 7; sky -> 11; wall/fence/pole and manufactured objects -> artificial 10
earth/dirt track/land -> soil 4 (test assumption)
road/sidewalk -> paved 6 (test assumption, not measured surface material)
Ambiguous path/sand/field/mountain/hill, bridge/pier/runway, water/animal -> ignore 0.
ADE zero-based floor 3 -> project paved 6, semantic cost 0 (user-selected indoor policy).
Full source-to-target table: semantic_adapter/ade20k_to_field12.json

LIMITATION: output uses the project's 0..11 domain, but this is NOT a 12-class model.
Wood/trunk/roots (3), gravel (5), fallen leaves (8) have no dedicated ADE source label
and are never produced. Whole tree is vegetation, not a separated trunk mask.
Ignore 0 means unknown. It must not be interpreted as free space.
Temporary grouping is suitable for interface testing; terrain recognition accuracy
requires a suitable segmentation model and independently annotated field validation.

Build/test evidence is recorded separately in semantic_adapter_result.json after
actual Jetson validation. Do not infer physical READY or driving performance from it.

Primary references:
ADE150 source taxonomy (one-based):
https://raw.githubusercontent.com/CSAILVision/sceneparsing/master/objectInfo150.txt
ROS Humble Image acquisition header and row stride contract:
https://raw.githubusercontent.com/ros2/common_interfaces/humble/sensor_msgs/msg/Image.msg
SegFormer original research: https://arxiv.org/abs/2105.15203
Original implementation: https://github.com/NVlabs/SegFormer

2026-11-06 domain alignment with Pi5 localization:
Use ./run_costmap.sh for the native manager; it selects runtime-semantic-test
and ROS_DOMAIN_ID=59 by default, like Pi5 run_fusion.sh and this adapter launcher.
Example: ROS_DOMAIN_ID=59 ./run_costmap.sh --action start --run-id YOUR_NEW_RUN_ID
The adapter remains separate: ROS_DOMAIN_ID=59 ./run_semantic_adapter.sh
Explicit ROS_DOMAIN_ID overrides are supported; use the same value on both boards.
The frozen C++ source and packaged runtimes are unchanged. Direct manager binary
invocation still has its old fallback domain42; the launcher supplies the domain.
No modules were started by this configuration update.

2026-11-06 source reconciliation: jetson-source-sync-20261106-1
Base layout remains traversability-jetson-20261106-3 with the deployed mapping-only
update. This is not a claim that every file from the PC layout 4 was installed.
The PC floor pixel test fixture is now installed; ADE source 3 must map to 6.
semantic_adapter/ade20k.preview.floor.json is an exact source copy of the deployed
trial config/ade20k.preview.json. The normal and preview tables use different input
topics. Both tables retain acquisition stamp, frame and 640x360 mono8 output.

Current trial configuration is under:
/home/nvidia/nomad_ws/tools/visual_trial_20261106
Its runtime-map-preview contains the selected measured 2026-10-30 OAK and ZED
calibrations, revision measured-20261030-restored-20261106-1. Relative sensor mounts
were confirmed unchanged by the user. CameraInfo uses zero-D[8]
rational_polynomial at the selected K. zed_measured_20261030.yaml and the trial
launch retain self_calib=false, serial 23472639 and HD720 acquisition.
The frozen source and base runtime still retain the original zero-D[5] plumb_bob
contract. Use the trial configuration to reproduce the current sensor pipeline;
do not assume a fresh base build alone reproduces all trial settings.

At reconciliation time, the active external semantic provider is the separate
FCN ResNet18 SUNRGBD 21-label indoor trial under tools/indoor_fcn_20261106.
Its original floor source ID is 2, mapped to project 6. This differs from the
ADE150 source floor ID 3. Do not launch the ADE converter concurrently on the
same output topic. Both native launchers retain default ROS_DOMAIN_ID=59.

Frozen calculation source 113 files, live runtime configurations and executables
are preserved. Only source test coverage, a mapping copy and delivery metadata
are reconciled. Sensor alignment, terrain accuracy and the final costmap issue
are not declared fixed by this source update.
