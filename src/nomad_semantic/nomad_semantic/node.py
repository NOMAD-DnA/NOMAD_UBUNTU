import json, time, os
from pathlib import Path
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
from .runtime import Engine, make_preview
from nomad_logging.diagnostics import Diagnostics


def capture_ns(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
from ament_index_python.packages import get_package_share_directory


class Semantic(Node):
    def __init__(self):
        super().__init__("nomad_semantic")
        for k, v in [
            (
                "model",
                str(
                    Path(get_package_share_directory("nomad_semantic"))
                    / "models/nomad_b0_map04/model.onnx"
                ),
            ),
            ("backend", "cuda"),
            ("image_topic", "/oak/rgb/image_raw"),
            ("rate", 5.0),
            ("view", False),
            ("max_age", 0.6),
            ("diagnostics_directory", os.environ.get("NOMAD_MAP_DIAGNOSTICS_DIR", "")),
        ]:
            self.declare_parameter(k, v)
        model = self.get_parameter("model").value
        if not model:
            raise RuntimeError("model path is required")
        self.engine = Engine(model, self.get_parameter("backend").value)
        self.bridge = CvBridge()
        self.diagnostics = Diagnostics(self.get_parameter("diagnostics_directory").value, prefix="semantic")
        if self.diagnostics.path:
            self.get_logger().info(f"Semantic diagnostics: {self.diagnostics.path}")
        self.latest = None
        self.received = 0.0
        self.count = 0
        self.last_log = time.monotonic()
        self.show = self.get_parameter("view").value
        self.sub = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self.receive,
            qos_profile_sensor_data,
        )
        self.pubs = {
            name: self.create_publisher(Image, "/nomad/semantic/" + name, 1)
            for name in ["labels", "confidence", "colors", "overlay"]
        }
        self.info = self.create_publisher(String, "/nomad/semantic/info", 1)
        rate = float(self.get_parameter("rate").value)
        if rate <= 0:
            raise ValueError("rate must be positive")
        self.timer = self.create_timer(1 / rate, self.process)
        self.get_logger().info(
            "Ready: %s, %s, %d classes; capture stamps preserved."
            % (
                self.engine.backend,
                self.engine.meta["architecture"],
                len(self.engine.meta["classes"]),
            )
        )

    def trace(self, event, **fields):
        recorder = getattr(self, "diagnostics", None)
        if recorder is not None and recorder.path is not None:
            recorder.emit(event, sim_ns=self.get_clock().now().nanoseconds, **fields)

    def destroy_node(self):
        recorder = getattr(self, "diagnostics", None)
        if recorder is not None:
            if self.latest is not None:
                self.trace("input_excluded", stamp_ns=capture_ns(self.latest), reason="shutdown_pending")
            recorder.close()
            if recorder.error or recorder.dropped:
                self.get_logger().warning(f"Diagnostics incomplete: dropped={recorder.dropped}, error={recorder.error}")
        return super().destroy_node()

    def receive(self, msg):
        self.trace("input_received", stream="rgb", stamp_ns=capture_ns(msg), size_bytes=len(msg.data))
        if self.latest is not None:
            self.trace("input_excluded", stamp_ns=capture_ns(self.latest), reason="latest_overwritten",
                       replacement_stamp_ns=capture_ns(msg))
        self.latest = msg
        self.received = time.monotonic()

    def process(self):
        if self.latest is None:
            return
        msg = self.latest
        self.latest = None
        max_age = self.get_parameter("max_age").value
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        ros_age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        wall_age = time.monotonic() - self.received
        if wall_age > max_age or ros_age > max_age:
            self.trace("input_excluded", stamp_ns=capture_ns(msg), reason="input_expired",
                       wall_age_s=wall_age, capture_age_s=ros_age, max_age_s=max_age)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            t = time.perf_counter()
            self.trace("inference_start", stamp_ns=capture_ns(msg))
            labels, confidence, colors, overlay = self.engine.predict(image)
            ms = (time.perf_counter() - t) * 1000
            self.trace("inference_complete", stamp_ns=capture_ns(msg), elapsed_ms=ms)
            for name, array, encoding in [
                ("labels", labels, "mono8"),
                ("confidence", confidence, "32FC1"),
                ("colors", colors, "rgb8"),
                ("overlay", overlay, "bgr8"),
            ]:
                out = self.bridge.cv2_to_imgmsg(array, encoding=encoding)
                out.header = msg.header
                self.trace("publish_start", stream=name, stamp_ns=capture_ns(msg),
                           size_bytes=len(out.data), width=out.width, height=out.height, encoding=encoding)
                publish_start = time.perf_counter()
                self.pubs[name].publish(out)
                self.trace("publish_complete", stream=name, stamp_ns=capture_ns(msg),
                           elapsed_ms=(time.perf_counter()-publish_start)*1000, size_bytes=len(out.data))
            self.count += 1
            meta = {
                "backend": self.engine.backend,
                "inference_ms": ms,
                "frames": self.count,
                "class_names_verified": self.engine.meta["class_names_verified"],
                "classes": self.engine.meta["classes"],
            }
            self.info.publish(String(data=json.dumps(meta)))
            if time.monotonic() - self.last_log > 5:
                self.get_logger().info(
                    "frames=%d, inference=%.1f ms" % (self.count, ms)
                )
                self.last_log = time.monotonic()
            if self.show:
                cv2.imshow(
                    "NOMAD RGB | semantic overlay",
                    make_preview(
                        image, labels, overlay, self.engine.meta, self.engine.palette
                    ),
                )
                if cv2.waitKey(1) & 255 == ord("q"):
                    rclpy.shutdown()
        except Exception as e:
            self.trace("process_error", stamp_ns=capture_ns(msg), reason=str(e))
            self.get_logger().error(str(e))
            raise


def main(args=None):
    rclpy.init(args=args)
    n = None
    try:
        n = Semantic()
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        if n is not None:
            n.destroy_node()
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
