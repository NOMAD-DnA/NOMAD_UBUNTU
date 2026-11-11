#!/usr/bin/env python3
"""Record existing ROS traffic and cached configuration; never control the vehicle."""

import argparse
import csv
import datetime as dt
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import socket
import time


def utc_now():
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")


def redact(value):
    if isinstance(value, dict):
        return {k: ("[redacted]" if any(word in k.lower() for word in
                ("token", "password", "secret", "session")) else redact(v))
                for k, v in value.items()}
    if isinstance(value, list):
        return [redact(v) for v in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    return value


class TrialLog:
    """Buffered append-only files, with independent events and configuration snapshots."""
    columns = ("utc", "monotonic_s", "source", "run", "motion_segment", "state",
               "target_erpm", "command_erpm", "raw_erpm", "motor_current_a",
               "input_current_a", "voltage_v", "duty_cycle", "fault_code",
               "servo_position", "telemetry_fresh", "telemetry_receipt_age_s", "status_receipt_age_s",
               "reason", "planning_reason")

    def __init__(self, folder, workspace, max_bytes=50 * 1024 * 1024):
        self.folder, self.workspace = Path(folder), Path(workspace)
        self.folder.mkdir(parents=True, exist_ok=True)
        self.max_bytes, self.part, self.part_bytes = max_bytes, 0, 0
        self.part_started = time.monotonic()
        self.samples = self.csv_file = None
        self.events = (self.folder / "events.jsonl").open("a", encoding="utf-8", buffering=65536)
        self.status, self.telemetry, self.commands = {}, {}, {}
        self.status_received, self.telemetry_received = None, None
        self.state = None
        self.run, self.motion_segment, self.motion_active = 0, 0, False
        self.command_previous, self.snapshot_signature = {}, None
        self.snapshot_number = 0
        self.disk_ok = True
        metadata = {"started_utc": utc_now(), "host": socket.gethostname(), "pid": os.getpid(),
                    "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "0"),
                    "ros_localhost_only": os.environ.get("ROS_LOCALHOST_ONLY", ""),
                    "recording_kind": "passive_ros_observation", "actuation_publishers": 0,
                    "physical_motion_verified": False, "vesc_raw_max_hz": 20,
                    "configuration_note": "Snapshots are existing files, not fresh serial reads; YAML files may have runtime overrides."}
        (self.folder / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")
        self._rotate()
        self.event("recorder_started", metadata)
        self.snapshot()

    def _rotate(self):
        for handle in (self.samples, self.csv_file):
            if handle:
                handle.flush()
                handle.close()
        self.part += 1
        self.part_bytes = 0
        self.part_started = time.monotonic()
        self.samples = (self.folder / f"samples-{self.part:04d}.jsonl").open("a", encoding="utf-8", buffering=65536)
        self.csv_file = (self.folder / f"timeline-{self.part:04d}.csv").open("a", encoding="utf-8", newline="", buffering=65536)
        self.csv = csv.DictWriter(self.csv_file, fieldnames=self.columns)
        self.csv.writeheader()

    def event(self, kind, data):
        if not self.disk_ok:
            return
        self.events.write(json.dumps({"utc": utc_now(), "monotonic_s": time.monotonic(),
                                     "event": kind, "data": redact(data)}, ensure_ascii=False) + "\n")
        self.events.flush()

    def sample(self, source, data):
        if not self.disk_ok:
            return
        now = time.monotonic()
        if self.part_bytes >= self.max_bytes or now - self.part_started >= 3600:
            self._rotate()
        record = {"utc": utc_now(), "monotonic_s": now, "source": source,
                  "run": self.run, "motion_segment": self.motion_segment, "data": redact(data)}
        line = json.dumps(record, ensure_ascii=False) + "\n"
        self.samples.write(line)
        self.part_bytes += len(line.encode("utf-8"))
        s, t, c = self.status, self.telemetry, self.commands
        row = {"utc": record["utc"], "monotonic_s": now, "source": source, "run": self.run,
               "motion_segment": self.motion_segment, "state": s.get("state"),
               "target_erpm": s.get("target_motor_erpm", s.get("planning_running_target_erpm")),
               "command_erpm": s.get("commanded_motor_erpm", c.get("speed")),
               "raw_erpm": t.get("speed", s.get("raw_erpm")),
               "motor_current_a": t.get("current_motor", s.get("current_a")),
               "input_current_a": t.get("current_input"),
               "voltage_v": t.get("voltage_input", s.get("voltage_v")),
               "duty_cycle": t.get("duty_cycle"), "fault_code": t.get("fault_code", s.get("fault_code")),
               "servo_position": s.get("commanded_servo_position", c.get("servo")),
               "telemetry_fresh": s.get("telemetry_fresh"),
               "telemetry_receipt_age_s": now - self.telemetry_received if self.telemetry_received is not None else None,
               "status_receipt_age_s": now - self.status_received if self.status_received is not None else None,
               "reason": s.get("reason"),
               "planning_reason": s.get("planning_reason")}
        self.csv.writerow(row)

    def on_status(self, data):
        self.status_received = time.monotonic()
        state = data.get("state")
        if state != self.state:
            if state == "RUNNING":
                self.run += 1
            self.event("state_change", {"previous": self.state, "state": state,
                                       "reason": data.get("reason"), "run": self.run})
            self.state = state
        commanded = data.get("commanded_motor_erpm", 0)
        active = isinstance(commanded, (int, float)) and abs(commanded) > 0.001 and state == "RUNNING"
        if active != self.motion_active:
            if active:
                self.motion_segment += 1
            self.event("motor_command_start" if active else "motor_command_end",
                       {"run": self.run, "segment": self.motion_segment, "command_erpm": commanded,
                        "reason": data.get("reason"), "physical_motion_verified": False})
            self.motion_active = active
        for key in ("reason", "fault_code", "telemetry_fresh", "driver_ready", "planning_reset_required"):
            if data.get(key) != self.status.get(key):
                self.event("status_change", {"field": key, "previous": self.status.get(key), "value": data.get(key)})
        self.status = data
        self.sample("controller_status", data)

    def on_command(self, kind, value):
        now = time.monotonic()
        old, last = self.command_previous.get(kind, (None, -1e9))
        self.commands[kind] = value
        self.command_previous[kind] = (value, now)
        changed = value != old
        # Keep changes and receipt gaps; stable command heartbeats are coalesced to 1 Hz.
        last_written = self.commands.get("_written_" + kind, -1e9)
        if changed or now - last_written >= 1 or now - last > .15:
            self.sample("command_" + kind, {"value": value, "receipt_gap_ms": (now - last) * 1000 if last > 0 else None})
            self.commands["_written_" + kind] = now
        if changed:
            self.event("command_change", {"kind": kind, "previous": old, "value": value})

    def snapshot(self):
        source = self.workspace / "src/nomad_control_cpp/config"
        candidates = [source / name for name in ("commissioning.yaml", "planning.yaml", "planning_headless.yaml",
                     "nomad_vehicle_ready.yaml", "vesc_steering_test.yaml", "calibration.yaml")]
        readback = self.workspace / "commission_session/firmware_readback.json"
        candidates.append(readback)
        signatures = []
        for path in candidates:
            try:
                st = path.stat()
                signatures.append((str(path), st.st_mtime_ns, st.st_size))
            except OSError:
                pass
        signature = tuple(signatures)
        if signature == self.snapshot_signature:
            return
        self.snapshot_signature = signature
        self.snapshot_number += 1
        dest = self.folder / "snapshots" / f"{self.snapshot_number:04d}"
        dest.mkdir(parents=True, exist_ok=True)
        manifest = {"captured_utc": utc_now(), "fresh_serial_read_performed": False, "files": []}
        for name, _, _ in signatures:
            path = Path(name)
            try:
                content = path.read_bytes()
                (dest / path.name).write_bytes(content)
                manifest["files"].append({"source": str(path), "file": path.name,
                                          "mtime_utc": dt.datetime.fromtimestamp(path.stat().st_mtime, dt.timezone.utc).isoformat(),
                                          "sha256": hashlib.sha256(content).hexdigest()})
            except OSError as error:
                manifest.setdefault("errors", []).append(str(error))
        try:
            cached = json.loads(readback.read_text(encoding="utf-8"))
            backup = Path(cached["backup"]).resolve()
            allowed = (self.workspace.parent / "vesc_ws/backups").resolve()
            if allowed in backup.parents:
                raw_dest = dest / "vesc_cached_backup"
                raw_dest.mkdir(exist_ok=True)
                for name in ("motor_payload.bin", "app_payload.bin", "firmware_payload.bin", "summary.json"):
                    path = backup / name
                    if path.is_file() and path.stat().st_size < 1024 * 1024:
                        shutil.copy2(path, raw_dest / name)
                manifest["vesc_cached_backup_source"] = str(backup)
                manifest["vesc_readback_warning"] = "File timestamp identifies earlier hardware read; settings may have changed afterwards."
        except (OSError, ValueError, KeyError, TypeError) as error:
            manifest["vesc_cached_backup_warning"] = str(error)
        (dest / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
        self.event("configuration_snapshot", {"path": str(dest), "fresh_serial_read_performed": False})

    def maintenance(self):
        self.flush()
        if shutil.disk_usage(self.folder).free < 256 * 1024 * 1024:
            self.event("recording_stopped_low_disk", {"minimum_free_bytes": 256 * 1024 * 1024})
            self.disk_ok = False
            raise RuntimeError("Recording stopped: less than 256 MiB free. Existing logs retained.")
        self.snapshot()

    def flush(self):
        for handle in (self.samples, self.csv_file, self.events):
            handle.flush()

    def close(self):
        self.event("recorder_stopped", {"last_state": self.state})
        self.flush()
        for handle in (self.samples, self.csv_file, self.events):
            handle.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workspace", default=str(Path.home() / "nomad_cpp_ws"))
    parser.add_argument("--output-root")
    parser.add_argument("--status-topic", default="/nomad_vesc_trial/status_json")
    parser.add_argument("--telemetry-topic", default="/nomad_bench/sensors/core")
    parser.add_argument("--speed-topic", default="/nomad_bench/commands/motor/speed")
    parser.add_argument("--current-topic", default="/nomad_bench/commands/motor/current")
    parser.add_argument("--servo-topic", default="/nomad_bench/commands/servo/position")
    parser.add_argument("--duration-s", type=float, default=0, help="Optional recording-only test duration; 0 runs continuously.")
    args = parser.parse_args()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String, Float64
    from vesc_msgs.msg import VescStateStamped

    root = Path(args.output_root or Path(args.workspace) / "log/diagnostics")
    root.mkdir(parents=True, exist_ok=True)
    folder = root / (dt.datetime.now().strftime("%Y%m%d_%H%M%S") + f"_domain{os.environ.get('ROS_DOMAIN_ID', '0')}_{os.getpid()}")
    log = TrialLog(folder, args.workspace)
    (root / "latest.txt").write_text(str(folder) + "\n", encoding="utf-8")
    log.event("subscription_topics", vars(args))
    rclpy.init(args=[])
    node = Node("nomad_trial_recorder_" + str(os.getpid()))
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
    last_telemetry = [-1e9]
    telemetry_fields = ("speed", "current_motor", "current_input", "avg_id", "avg_iq", "voltage_input",
                        "duty_cycle", "fault_code", "temp_fet", "temp_motor", "displacement",
                        "distance_traveled", "avg_vd", "avg_vq")

    def on_status(message):
        try:
            data = json.loads(message.data)
            if not isinstance(data, dict):
                raise ValueError("Status payload is not an object")
            log.on_status(data)
        except (ValueError, TypeError) as error:
            log.event("invalid_status_json", {"error": str(error)})

    def on_telemetry(message):
        data = {key: getattr(message.state, key, None) for key in telemetry_fields}
        data["header_stamp_s"] = message.header.stamp.sec + message.header.stamp.nanosec / 1e9
        log.telemetry = data
        now = time.monotonic()
        log.telemetry_received = now
        fault_changed = data.get("fault_code") != getattr(on_telemetry, "fault", None)
        if now - last_telemetry[0] >= .05 or fault_changed:
            log.sample("vesc_telemetry", data)
            last_telemetry[0] = now
        if fault_changed:
            log.event("vesc_fault_change", {"previous": getattr(on_telemetry, "fault", None), "value": data.get("fault_code")})
            on_telemetry.fault = data.get("fault_code")

    subscriptions = [node.create_subscription(String, args.status_topic, on_status, qos),
                     node.create_subscription(VescStateStamped, args.telemetry_topic, on_telemetry, qos)]
    for kind, topic in (("speed", args.speed_topic), ("current", args.current_topic), ("servo", args.servo_topic)):
        subscriptions.append(node.create_subscription(Float64, topic, lambda message, k=kind: log.on_command(k, message.data), qos))
    timer = node.create_timer(1.0, log.maintenance)
    start = time.monotonic()
    print(str(folder), flush=True)
    try:
        while rclpy.ok() and (args.duration_s <= 0 or time.monotonic() - start < args.duration_s):
            rclpy.spin_once(node, timeout_sec=.2)
    except KeyboardInterrupt:
        pass
    finally:
        log.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
