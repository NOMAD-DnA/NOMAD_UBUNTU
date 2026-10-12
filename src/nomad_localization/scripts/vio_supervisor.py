#!/usr/bin/env python3
"""Own one estimator process group; normalize poses and restart only that group."""
import copy, json, os, signal, subprocess, time
from pathlib import Path
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import UInt32, Bool
from std_srvs.srv import Trigger
from ament_index_python.packages import get_package_share_directory


class Supervisor(Node):
    def __init__(self):
        super().__init__("vio_supervisor")
        self.method = self.declare_parameter("method", "basalt").value
        self.manage = self.declare_parameter("manage_vio", True).value
        self.root = Path(
            self.declare_parameter(
                "vio_ws", str(Path.home() / "ws" / f"{self.method}_ws")
            ).value
        )
        self.logdir = Path(
            self.declare_parameter(
                "log_directory", str(Path.home() / ".ros/nomad_vio")
            ).value
        )
        self.sim_time = self.declare_parameter("use_estimator_sim_time", True).value
        self.vio_config = Path(
            self.declare_parameter(
                "vio_config_directory", str(self.root / "config")
            ).value
        ).expanduser()
        self.stop_before_restart = self.declare_parameter(
            "stop_before_restart", False
        ).value
        self.output_warmup = self.declare_parameter("output_warmup_s", 0.0).value
        if self.output_warmup < 0:
            raise ValueError("output_warmup_s must be nonnegative")
        self.pending_restart = False
        self.stop_pub = self.create_publisher(
            Bool,
            "/fusion/recovery_stop",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        self.create_subscription(Bool, "/fusion/restart_ready", self.ready, 10)
        self.min_stamp = -1.0
        self.epoch = 0
        self.proc = None
        self.logfile = None
        self.first = None
        self.block_until = 0
        self.closed = False
        if self.method not in ("basalt", "openvins"):
            raise ValueError("method must be basalt or openvins")
        self.pub = self.create_publisher(Odometry, "/fusion/vio_input", 100)
        self.epoch_pub = self.create_publisher(
            UInt32,
            "/fusion/vio_epoch",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )
        topic, typ = (
            ("/basalt/odom", Odometry)
            if self.method == "basalt"
            else ("/ov_msckf/poseimu", PoseWithCovarianceStamped)
        )
        self.sub = self.create_subscription(
            typ, topic, self.pose, qos_profile_sensor_data
        )
        self.service = self.create_service(Trigger, "/fusion/restart_vio", self.restart)
        self.timer = self.create_timer(
            1.0,
            self.watch,
            clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME),
        )
        if self.manage:
            self.start()
        self.announce()

    def event(self, name, **kw):
        self.logdir.mkdir(parents=True, exist_ok=True)
        with (self.logdir / "events.jsonl").open("a") as f:
            f.write(
                json.dumps(
                    {
                        "event": name,
                        "wall_time": time.time(),
                        "sim_time": self.get_clock().now().nanoseconds / 1e9,
                        "epoch": self.epoch,
                        **kw,
                    }
                )
                + "\n"
            )

    def announce(self):
        m = UInt32()
        m.data = self.epoch
        self.epoch_pub.publish(m)

    def start(self):
        share = Path(get_package_share_directory("nomad_localization"))
        if self.method == "basalt":
            command = ["ros2", "launch", str(share / "launch/basalt.launch.py")]
        else:
            command = [
                "ros2",
                "run",
                "ov_msckf",
                "run_subscribe_msckf",
                str(self.vio_config / "nomad_gazebo/estimator_config.yaml"),
                "--ros-args",
                "-r",
                "__ns:=/ov_msckf",
                "-p",
                f"use_sim_time:={str(self.sim_time).lower()}",
                "-p",
                "publish_global_to_imu_tf:=false",
                "-p",
                "publish_calibration_tf:=false",
            ]
        env = os.environ.copy()
        env["NOMAD_VIO_CONFIG"] = str(self.vio_config)
        env["VIO_USE_SIM_TIME"] = str(self.sim_time).lower()
        env["ROS_LOG_DIR"] = str(self.logdir / f"ros_epoch_{self.epoch}")
        self.logdir.mkdir(parents=True, exist_ok=True)
        self.logfile = (self.logdir / f"estimator_epoch_{self.epoch}.log").open("w")
        self.proc = subprocess.Popen(
            command,
            env=env,
            stdout=self.logfile,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        self.event("start", pid=self.proc.pid, command=command)
        self.get_logger().info(
            f"{self.method} started: owned pid {self.proc.pid}, epoch {self.epoch}"
        )

    def stop(self):
        if self.proc is not None:
            if self.proc.poll() is None:
                for sig, timeout in [
                    (signal.SIGINT, 8),
                    (signal.SIGTERM, 3),
                    (signal.SIGKILL, 2),
                ]:
                    try:
                        os.killpg(self.proc.pid, sig)
                    except ProcessLookupError:
                        break
                    try:
                        self.proc.wait(timeout=timeout)
                        break
                    except subprocess.TimeoutExpired:
                        continue
            self.event("stop", pid=self.proc.pid, returncode=self.proc.poll())
            self.proc = None
        if self.logfile:
            self.logfile.close()
            self.logfile = None

    def ready(self, m):
        if m.data and self.pending_restart:
            self.pending_restart = False
            self.event("vehicle_stationary_ack")
            self.perform_restart()

    def perform_restart(self):
        self.block_until = time.monotonic() + 2
        self.stop()
        self.epoch += 1
        self.first = None
        self.min_stamp = self.get_clock().now().nanoseconds / 1e9
        self.start()
        self.block_until = time.monotonic() + 0.2
        self.announce()
        self.event("restart")

    def restart(self, req, res):
        if not self.manage:
            res.success = False
            res.message = "unmanaged estimator: restart denied"
            return res
        if self.stop_before_restart:
            if not self.pending_restart:
                self.pending_restart = True
                self.stop_pub.publish(Bool(data=True))
                self.event("restart_queued_waiting_stationary")
            res.success = True
            res.message = (
                "restart queued: waiting for vehicle stationary acknowledgement"
            )
            return res
        self.perform_restart()
        res.success = True
        res.message = f"owned estimator restarted; epoch={self.epoch}"
        return res

    def pose(self, m):
        if time.monotonic() < self.block_until:
            return
        out = copy.deepcopy(m) if self.method == "basalt" else Odometry()
        if self.method == "openvins":
            out.header = copy.deepcopy(m.header)
            out.pose = copy.deepcopy(m.pose)
        out.child_frame_id = "oak_imu_frame"
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if t < self.min_stamp:
            return
        if self.first is None:
            self.first = t
        if t - self.first < self.output_warmup:
            return
        self.pub.publish(out)

    def watch(self):
        if self.proc is not None and self.proc.poll() is not None:
            self.get_logger().error(f"Owned estimator exited: {self.proc.returncode}")
            self.event("unexpected_exit", returncode=self.proc.returncode)
            self.proc = None

    def close(self):
        if self.closed:
            return
        self.closed = True
        self.stop()


def main():
    rclpy.init()
    n = Supervisor()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        n.close()
        n.destroy_node()
        rclpy.shutdown() if rclpy.ok() else None


if __name__ == "__main__":
    main()
