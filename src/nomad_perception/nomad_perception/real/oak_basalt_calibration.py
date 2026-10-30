"""Build a provisional *physical OAK-D-Lite BMI270* Basalt calibration.

Input images must be the SDK's undistorted, unrectified camera outputs.  Their
final ImgTransformation intrinsics, not EEPROM intrinsics, are supplied here.
Raw accelerometer and gyro vectors remain in their native BMI270 axes.
"""

from __future__ import annotations

import math

import numpy as np


BOARD_SOURCE = (
    "https://raw.githubusercontent.com/luxonis/depthai-boards/main/boards/OAK-D-LITE.json"
)
BOARD_SOURCE_REVISION = "R1M1E3"
BASALT_SOURCE = (
    "https://github.com/VladyslavUsenko/basalt/blob/master/thirdparty/"
    "basalt-headers/include/basalt/calibration/calibration.hpp"
)


def checked_transform(matrix, name="transform"):
    """Reject unit, reflection and corrupt-transform mistakes early."""
    transform = np.array(matrix, dtype=float, copy=True)
    if transform.shape != (4, 4) or not np.isfinite(transform).all():
        raise ValueError(f"{name} must be a finite 4x4 matrix")
    if not np.allclose(transform[3], [0, 0, 0, 1], atol=1e-7):
        raise ValueError(f"{name} has an invalid homogeneous row")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-3):
        raise ValueError(f"{name} rotation is not orthonormal")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1e-3):
        raise ValueError(f"{name} rotation must have determinant +1")
    return transform


def invert_transform(matrix):
    transform = checked_transform(matrix)
    result = np.eye(4)
    result[:3, :3] = transform[:3, :3].T
    result[:3, 3] = -result[:3, :3] @ transform[:3, 3]
    return result


def _quaternion_xyzw(rotation):
    # Branching avoids a zero divisor for the BMI270's nominal 180 degree X turn.
    r = np.asarray(rotation, dtype=float)
    trace = float(np.trace(r))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = np.array([(r[2, 1] - r[1, 2]) / s,
                      (r[0, 2] - r[2, 0]) / s,
                      (r[1, 0] - r[0, 1]) / s, 0.25 * s])
    elif r[0, 0] > r[1, 1] and r[0, 0] > r[2, 2]:
        s = math.sqrt(1 + r[0, 0] - r[1, 1] - r[2, 2]) * 2
        q = np.array([0.25 * s, (r[0, 1] + r[1, 0]) / s,
                      (r[0, 2] + r[2, 0]) / s,
                      (r[2, 1] - r[1, 2]) / s])
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1 + r[1, 1] - r[0, 0] - r[2, 2]) * 2
        q = np.array([(r[0, 1] + r[1, 0]) / s, 0.25 * s,
                      (r[1, 2] + r[2, 1]) / s,
                      (r[0, 2] - r[2, 0]) / s])
    else:
        s = math.sqrt(1 + r[2, 2] - r[0, 0] - r[1, 1]) * 2
        q = np.array([(r[0, 2] + r[2, 0]) / s,
                      (r[1, 2] + r[2, 1]) / s, 0.25 * s,
                      (r[1, 0] - r[0, 1]) / s])
    q /= np.linalg.norm(q)
    if q[3] < 0:
        q = -q
    return q


def _basalt_pose(matrix):
    transform = checked_transform(matrix)
    values = [*transform[:3, 3], *_quaternion_xyzw(transform[:3, :3])]
    return dict(zip(("px", "py", "pz", "qx", "qy", "qz", "qw"),
                    map(float, values)))


def _pinhole(matrix, resolution, name):
    k = np.asarray(matrix, dtype=float)
    if k.shape != (3, 3) or not np.isfinite(k).all():
        raise ValueError(f"{name} intrinsics must be a finite 3x3 matrix")
    if not np.allclose(k[2], [0, 0, 1], atol=1e-6):
        raise ValueError(f"{name} intrinsics have an invalid last row")
    if not np.allclose([k[0, 1], k[1, 0]], 0, atol=1e-6):
        raise ValueError(f"{name} intrinsics have unsupported skew")
    width, height = resolution
    if k[0, 0] <= 0 or k[1, 1] <= 0:
        raise ValueError(f"{name} focal lengths must be positive")
    if not (0 <= k[0, 2] < width and 0 <= k[1, 2] < height):
        raise ValueError(f"{name} principal point is outside the output image")
    return {"camera_type": "pinhole", "intrinsics": {
        "fx": float(k[0, 0]), "fy": float(k[1, 1]),
        "cx": float(k[0, 2]), "cy": float(k[1, 2]),
    }}


def make_calibration(calibration, left_K, right_K, resolution=(640, 480),
                     imu_name="BMI270", imu_rate_hz=200.0, dai=None, imu_extrinsics="nominal"):
    """Return ``(Basalt cereal JSON, calibration provenance)``.

    DepthAI ``getCameraExtrinsics(source, destination, False, METER)`` maps
    source points into destination coordinates.  Basalt's ``T_imu_cam`` maps
    camera points into IMU coordinates.  The board definition supplies nominal
    BMI270-to-CAM_A geometry as a provisional alternative to the EEPROM transform.
    This is a usable initial calibration, not a measured camera-IMU calibration.
    """
    if dai is None:
        import depthai as dai

    factory = calibration.eepromToJson()
    product = str(factory.get("productName", ""))
    if imu_name != "BMI270" or not product.startswith("OAK-D-LITE"):
        raise ValueError("This provisional calibration requires OAK-D-LITE and raw BMI270")
    if (len(resolution) != 2 or any(isinstance(n, bool) or int(n) != n or n <= 0
                                   for n in resolution)):
        raise ValueError("resolution must contain two positive integers")
    resolution = tuple(map(int, resolution))
    imu_rate_hz = float(imu_rate_hz)
    if not math.isfinite(imu_rate_hz) or imu_rate_hz <= 0:
        raise ValueError("imu_rate_hz must be finite and positive")

    # Board JSON translation units are centimeters. These numbers are the full
    # BMI270 spec transform; the cause of the EEPROM discrepancy is unconfirmed.
    T_a_raw_imu = np.eye(4)
    T_a_raw_imu[:3, :3] = np.diag([1., -1., -1.])
    T_a_raw_imu[:3, 3] = np.array([-3.15, 0.6439, -0.2]) / 100.0

    if imu_extrinsics == "factory":
        T_a_raw_imu = checked_transform(calibration.getImuToCameraExtrinsics(
            dai.CameraBoardSocket.CAM_A, False, dai.LengthUnit.METER), "factory raw IMU -> CAM_A")
    elif imu_extrinsics != "nominal":
        raise ValueError("imu_extrinsics must be factory or nominal")

    cameras = (dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C)
    T_camera_a = [checked_transform(calibration.getCameraExtrinsics(
        dai.CameraBoardSocket.CAM_A, camera, False, dai.LengthUnit.METER),
        f"CAM_A -> {camera}") for camera in cameras]
    centers_in_a = [invert_transform(transform)[:3, 3] for transform in T_camera_a]
    baseline_m = float(np.linalg.norm(centers_in_a[1] - centers_in_a[0]))
    if not 0.05 <= baseline_m <= 0.1:
        raise ValueError(f"OAK-D-Lite stereo baseline is implausible: {baseline_m:.6g} m")

    T_raw_imu_cameras = [invert_transform(transform @ T_a_raw_imu)
                         for transform in T_camera_a]
    T_raw_imu_cam_a = invert_transform(T_a_raw_imu)
    value = {
        "T_imu_cam": [_basalt_pose(transform) for transform in T_raw_imu_cameras],
        "intrinsics": [_pinhole(left_K, resolution, "left"),
                       _pinhole(right_K, resolution, "right")],
        "resolution": [list(resolution), list(resolution)],
        "calib_accel_bias": [0.0] * 9,
        "calib_gyro_bias": [0.0] * 12,
        "imu_update_rate": imu_rate_hz,
        # Basalt source defaults, intentionally not called measured BMI270 noise.
        "accel_noise_std": [0.016] * 3,
        "gyro_noise_std": [0.000282] * 3,
        "accel_bias_std": [0.001] * 3,
        "gyro_bias_std": [0.0001] * 3,
        "cam_time_offset_ns": 0,
        "vignette": [],
    }
    provenance = {
        "product_name": product,
        "factory_board_revision": factory.get("boardRev"),
        "imu_name": imu_name,
        "image_geometry": "SDK undistorted, unrectified CAM_B/C; final output K",
        "imu_axes": "raw BMI270; no rotation applied to accel or gyro",
        "raw_to_published_imu_rotation": np.eye(3).tolist(),
        "T_a_raw_imu": T_a_raw_imu.tolist(),
        "T_raw_imu_cam_a": T_raw_imu_cam_a.tolist(),
        "T_imu_cam_A": T_raw_imu_cam_a.tolist(),
        "T_raw_imu_cameras": [t.tolist() for t in T_raw_imu_cameras],
        "T_camera_a": [t.tolist() for t in T_camera_a],
        "stereo_baseline_m": baseline_m,
        "imu_extrinsics_source": "device EEPROM" if imu_extrinsics == "factory" else BOARD_SOURCE,
        "imu_extrinsics_mode": imu_extrinsics,
        "nominal_source_board_revision": BOARD_SOURCE_REVISION,
        "precision_camera_imu_alignment_verified": False,
        "factory_imu_extrinsics_used": imu_extrinsics == "factory",
        "nominal_board_revision_matches": factory.get("boardRev") == BOARD_SOURCE_REVISION,
        "noise_source": "Basalt generic source defaults; not calibrated for this BMI270",
        "noise_source_url": BASALT_SOURCE,
        "camera_imu_time_offset_measured": False,
        "camera_imu_time_offset_ns_assumed": 0,
        "limitations": [
            "Camera-IMU extrinsics use " + imu_extrinsics + " geometry; independent alignment unverified.",
            "Zero camera-IMU time offset and generic noise are initial assumptions.",
            "Stationary output alone cannot validate motion tracking accuracy.",
        ],
    }
    return {"value0": value}, provenance
