"""Build/test inside a private ARM64 namespace. Never contacts the Jetson."""
from pathlib import Path
import json,os,subprocess,sys
root=Path(__file__).resolve().parents[1]
base=Path('/home/hyeonsu/jetson-restore/install/JetPack_5.1.2_Linux_JETSON_XAVIER_NX_TARGETS/Linux_for_Tegra/rootfs')
toolchain=Path('/home/hyeonsu/Documents/ChatGPT/Lidar + Camera/semantic_ground_validation/jetson/deployment_oak_native_20261104/work/emulation/upper')
reference=Path(json.loads((root/'reference/baseline.json').read_text())['artifact'])
emulation=Path(os.environ.get('NOMAD_ARM_EMULATION_DIR',root/'work/arm64'))
for name in ('root','upper','overlay-work'):(emulation/name).mkdir(parents=True,exist_ok=True)
script=r'''set -e
mount --make-rprivate /
ip link set lo up
ip link add nomad-cpp-dds type dummy
ip addr add 192.0.2.1/24 dev nomad-cpp-dds
ip link set nomad-cpp-dds up
ip route add 224.0.0.0/4 dev nomad-cpp-dds
base="$1"; toolchain="$2"; upper="$3"; overlay_work="$4"; merged="$5"; project="$6"; reference="$7"; shift 7
mount -t overlay overlay -o "lowerdir=$toolchain:$base,upperdir=$upper,workdir=$overlay_work" "$merged"
mkdir -p "$merged/work/cpp" "$merged/proc" "$merged/dev"
mount --bind "$project" "$merged/work/cpp"
mkdir -p "$merged/work/reference"
mount --bind "$reference" "$merged/work/reference"
mount -o remount,bind,ro "$merged/work/reference"
mount -t proc proc "$merged/proc"
mount -t tmpfs -o nosuid,size=16m,mode=0755 tmpfs "$merged/dev"
for name in null zero random urandom; do touch "$merged/dev/$name"; mount --bind "/dev/$name" "$merged/dev/$name"; done
mkdir -p "$merged/dev/shm" "$merged/tmp"
mount -t tmpfs -o nosuid,size=256m tmpfs "$merged/dev/shm"
chroot "$merged" /bin/bash --noprofile --norc -c 'cd /work/cpp; export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin; export LANG=C.UTF-8 LC_ALL=C.UTF-8 ROS_LOG_DIR=/tmp/cpp-ros-logs NOMAD_ROS_PREFIX=/opt/ros/humble/install NOMAD_REFERENCE_ARTIFACT=/work/reference; mkdir -p "$ROS_LOG_DIR"; if [[ -f /work/cpp/verification/libqemu_socket_compat.so ]]; then export LD_PRELOAD=/work/cpp/verification/libqemu_socket_compat.so NOMAD_PC_QEMU_SOCKET_COMPAT=1; fi; exec "$@"' nomad-cpp-arm "$@"
'''
command=['unshare','--user','--map-root-user','--mount','--pid','--fork','--net','bash','-c',script,
 'nomad-cpp-namespace',str(base),str(toolchain),str(emulation/'upper'),str(emulation/'overlay-work'),str(emulation/'root'),str(root),str(reference)]
raise SystemExit(subprocess.call(command+(sys.argv[1:] or ['/usr/bin/g++','--version'])))
