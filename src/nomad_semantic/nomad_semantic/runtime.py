"""Python 3.8 compatible inference, independent from ROS and training libraries."""

import json
from pathlib import Path
import cv2
import numpy as np

PALETTE = np.array(
    [
        [0, 0, 0],
        [255, 140, 0],
        [80, 170, 70],
        [130, 80, 180],
        [50, 150, 240],
        [190, 130, 65],
        [230, 65, 65],
        [220, 210, 80],
    ],
    dtype=np.uint8,
)


class Engine:
    def __init__(self, model, backend="cpu", threads=4):
        self.backend = backend
        self.meta = json.loads((Path(model).parent / "metadata.json").read_text())
        self.palette = np.asarray(
            self.meta.get("palette", PALETTE.tolist()), dtype=np.uint8
        )
        self.input_size = tuple(
            self.meta.get("input_shape", [1, 3, 480, 640])[-2:][::-1]
        )
        self.output_name = self.meta.get("output_name", "logits")
        if backend == "trt":
            import tensorrt as trt
            import pycuda.driver as cuda
            import pycuda.autoinit

            self.cuda = cuda
            self.trt_runtime = trt.Runtime(trt.Logger(trt.Logger.WARNING))
            self.engine = self.trt_runtime.deserialize_cuda_engine(
                Path(model).read_bytes()
            )
            if self.engine is None:
                raise RuntimeError(
                    "TensorRT engine could not be loaded; build on this Jetson"
                )
            if self.engine.num_bindings != 2:
                raise RuntimeError("Expected exactly one input and one output")
            self.context = self.engine.create_execution_context()
            self.input_index = next(
                i for i in range(2) if self.engine.binding_is_input(i)
            )
            self.output_index = 1 - self.input_index
            self.host_in = np.empty(
                self.engine.get_binding_shape(self.input_index),
                dtype=trt.nptype(self.engine.get_binding_dtype(self.input_index)),
            )
            self.host_out = np.empty(
                self.engine.get_binding_shape(self.output_index),
                dtype=trt.nptype(self.engine.get_binding_dtype(self.output_index)),
            )
            self.dev_in = cuda.mem_alloc(self.host_in.nbytes)
            self.dev_out = cuda.mem_alloc(self.host_out.nbytes)
            self.bindings = [0, 0]
            self.bindings[self.input_index] = int(self.dev_in)
            self.bindings[self.output_index] = int(self.dev_out)
        else:
            import onnxruntime as ort

            if backend == "cuda" and hasattr(ort, "preload_dlls"):
                ort.preload_dlls(directory="")
            available = ort.get_available_providers()
            if backend == "cuda" and "CUDAExecutionProvider" not in available:
                raise RuntimeError("CUDA ONNXRuntime provider unavailable")
            opts = ort.SessionOptions()
            opts.intra_op_num_threads = threads
            opts.inter_op_num_threads = 1
            opts.log_severity_level = 3
            providers = (
                [("CUDAExecutionProvider", {"use_tf32": False}), "CPUExecutionProvider"]
                if backend == "cuda"
                else ["CPUExecutionProvider"]
            )
            self.session = ort.InferenceSession(
                str(model), sess_options=opts, providers=providers
            )
            if (
                backend == "cuda"
                and self.session.get_providers()[0] != "CUDAExecutionProvider"
            ):
                raise RuntimeError(
                    "CUDA initialization failed; refusing silent CPU fallback"
                )

    def logits(self, x):
        if self.backend == "trt":
            np.copyto(self.host_in, x)
            self.cuda.memcpy_htod(self.dev_in, self.host_in)
            if not self.context.execute_v2(self.bindings):
                raise RuntimeError("TensorRT execution failed")
            self.cuda.memcpy_dtoh(self.host_out, self.dev_out)
            return self.host_out.copy()
        return self.session.run([self.output_name], {"bgr": x})[0]

    def predict(self, bgr):
        h, w = bgr.shape[:2]
        if self.meta.get("resize_backend") == "pil":
            from PIL import Image

            image = np.asarray(
                Image.fromarray(bgr[:, :, ::-1]).resize(
                    self.input_size, Image.Resampling.BILINEAR
                )
            )[:, :, ::-1]
        else:
            image = cv2.resize(bgr, self.input_size, interpolation=cv2.INTER_LINEAR)
        x = np.ascontiguousarray(image.transpose(2, 0, 1)[None], dtype=np.float32)
        result = self.logits(x)[0].astype(np.float32)
        if self.meta.get("output_kind") == "labels_confidence":
            labels = result[0].astype(np.uint8)
            confidence = result[1]
            if labels.shape != (h, w):
                labels = cv2.resize(labels, (w, h), interpolation=cv2.INTER_NEAREST)
                confidence = cv2.resize(
                    confidence, (w, h), interpolation=cv2.INTER_LINEAR
                )
            rgb = self.palette[labels]
            return (
                labels,
                confidence,
                rgb,
                cv2.addWeighted(bgr, 0.6, rgb[:, :, ::-1], 0.4, 0),
            )
        logits = result
        if logits.shape[1:] != (h, w):
            logits = np.stack(
                [cv2.resize(c, (w, h), interpolation=cv2.INTER_LINEAR) for c in logits]
            )
        shifted = logits - logits.max(axis=0, keepdims=True)
        prob = np.exp(shifted)
        prob /= prob.sum(axis=0, keepdims=True)
        labels = logits.argmax(axis=0).astype(np.uint8)
        confidence = prob.max(axis=0).astype(np.float32)
        rgb = self.palette[labels]
        overlay = cv2.addWeighted(bgr, 0.6, rgb[:, :, ::-1], 0.4, 0)
        return labels, confidence, rgb, overlay


def make_preview(image, labels, overlay, meta, palette):
    legend = np.full((480, 220, 3), 245, dtype=np.uint8)
    ids = sorted(
        np.unique(labels), key=lambda i: int(np.sum(labels == i)), reverse=True
    )
    for row, i in enumerate(ids[:12]):
        color = tuple(int(v) for v in palette[i, ::-1])
        cv2.rectangle(legend, (10, 10 + 35 * row), (32, 30 + 35 * row), color, -1)
        cv2.putText(
            legend,
            meta["classes"][int(i)],
            (42, 28 + 35 * row),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (25, 25, 25),
            1,
        )
    return np.hstack(
        [cv2.resize(image, (640, 480)), cv2.resize(overlay, (640, 480)), legend]
    )
