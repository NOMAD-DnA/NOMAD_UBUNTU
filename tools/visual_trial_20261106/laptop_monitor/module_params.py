"""모듈 세부 설정 (2026-11-16, user request): VIO(Basalt), 위치 융합, GNSS 수신, OAK·점군, 인지.

Like the planner, the board's own config files are never rewritten. Each value lives on the board
that runs the module (board_settings.RosOverrideModule / BasaltModule):
  Pi5    ~/nomad_settings/vio/basalt_vio_config.json   full Basalt JSON, only edited keys changed
         ~/nomad_settings/fusion/overrides.yaml        node "fusion", changed keys only
         ~/nomad_settings/gnss/overrides.yaml          node "nomad_gnss", changed keys only
  Jetson ~/nomad_settings/oak/overrides.yaml           node "/oak"
         ~/nomad_settings/cloud/overrides.yaml         node "nomad_external_stereo_cloud"
         ~/nomad_settings/perception/overrides.yaml    node "/**" (semantic node name not confirmed)
(+ ~/nomad_settings/<module>/current.json with the table values.) The GUI start scripts copy an
override into the run folder and add it as a later --params-file at the next Start only. A file
without changes holds only a comment (no "ros__parameters:") or "{}" (Basalt), and then every
start command line stays exactly as before.
Defaults come from the laptop copies of the board configs (보드코드_읽기_20261111, 보드코드_20261114);
the dialog always shows the values read from the board. Calibration, topics, frames, *_confirmed
flags, ports and OAK fps / IMU rate are not editable here.
"""
from __future__ import annotations

import math
from pathlib import Path

from module_settings import SettingsDialog as _Dialog, SettingsSpec

HERE = Path(__file__).resolve().parent


class _OakSpec(SettingsSpec):
    """external_stereo_cloud refuses to start unless these hold (external_stereo_cloud.cpp)."""

    def validate(self, configs: dict) -> dict:
        out = super().validate(configs)
        cloud = out["cloud"]
        if cloud["num_disparities"] % 16:
            raise ValueError("시차 범위: 16의 배수여야 합니다")
        if cloud["block_size"] % 2 == 0:
            raise ValueError("블록 크기: 홀수여야 합니다")
        if not cloud["max_depth_m"] > cloud["min_depth_m"]:
            raise ValueError("최대 거리는 최소 거리보다 커야 합니다")
        return out


class _FusionSpec(SettingsSpec):
    """fusion_node.cpp refuses to start when window_s * keyframe_hz > 500 ("Invalid window or integer limit")."""

    def validate(self, configs: dict) -> dict:
        out = super().validate(configs)
        fusion = out["fusion"]
        if fusion["window_s"] * fusion["keyframe_hz"] > 500:
            raise ValueError("최적화 창 길이×키프레임 주기는 500 이하여야 합니다")
        return out


SPECS = {
    "vio": SettingsSpec(HERE / "vio_config", (("vio", "VIO(Basalt)"),), "module_vio_settings.json", "VIO(Basalt)"),
    "fusion": _FusionSpec(HERE / "fusion_config", (("fusion", "위치 융합"),), "module_fusion_settings.json", "위치 융합"),
    "gnss": SettingsSpec(HERE / "gnss_config", (("gnss", "GNSS 수신"),), "module_gnss_settings.json", "GNSS 수신"),
    "oak": _OakSpec(HERE / "oak_config", (("oak", "OAK 카메라"), ("cloud", "OAK 점군")), "module_oak_settings.json", "OAK·점군"),
    "perception": SettingsSpec(HERE / "perception_config", (("semantic", "의미분할"),), "module_perception_settings.json", "인지"),
}
BOARD = {"vio": "pi5", "fusion": "pi5", "gnss": "pi5", "oak": "jetson", "perception": "jetson"}
BOARD_LABEL = {"pi5": "Pi5", "jetson": "Jetson"}
# module -> ((part, ROS node key, board default file or None, override file), ...), paths under the board home.
ROS_PARTS = {
    "fusion": (("fusion", "fusion", "nomad_ws/config/fusion_independent.yaml", "nomad_settings/fusion/overrides.yaml"),),
    "gnss": (("gnss", "nomad_gnss", "nomad_ws/config/gnss_receiver.yaml", "nomad_settings/gnss/overrides.yaml"),),
    "oak": (("oak", "/oak", "nomad_ws/config/oak_cpp.yaml", "nomad_settings/oak/overrides.yaml"),
            ("cloud", "nomad_external_stereo_cloud", None, "nomad_settings/cloud/overrides.yaml")),
    "perception": (("semantic", "/**", None, "nomad_settings/perception/overrides.yaml"),),
}
BASALT_DEFAULT = "nomad_ws/config/basalt_vio_config.json"
BASALT_OVERRIDE = "nomad_settings/vio/basalt_vio_config.json"
BASALT_PREFIX = "config."
NO_CHANGES = "# 바꾼 값 없음 (보드 기본값 사용)\n"
NOTE_NAMES = {"fusion": "위치 융합", "gnss": "GNSS", "oak": "OAK", "cloud": "OAK 점군", "perception": "인지", "vio": "VIO"}
VIO_UNSUPPORTED = "VIO 설정 미적용: Pi5 실행 스크립트 미지원"

INTROS = {
    "vio": "Pi5 Basalt VIO 값입니다. 보드 원래 설정(basalt_vio_config.json)은 그대로 두고, 바꾼 값만 넣은 사본을 "
           "Pi5에 저장해 다음 시작(및 VIO 재시작)부터 씁니다. Pi5 실행 스크립트가 이 사본을 아직 못 읽으면 시작 때 "
           "'" + VIO_UNSUPPORTED + "'로 알립니다. 보정값은 여기서 바꾸지 않습니다.",
    "fusion": "Pi5 위치 융합(VIO+GNSS) 값입니다. 바꾼 값만 Pi5에 저장하고 다음 시작부터 적용합니다. "
              "토픽·프레임·장착 보정값은 여기서 바꾸지 않습니다. [안전] 값은 위치 출력 정지와 관련 있습니다.",
    "gnss": "Pi5 GNSS 수신기 값입니다. 바꾼 값만 Pi5에 저장하고 다음 시작(GNSS 켬)부터 적용합니다. 포트·속도는 바꾸지 않습니다.",
    "oak": "Jetson OAK 카메라 큐와 OAK 점군 값입니다. 바꾼 값만 Jetson에 저장하고 다음 시작부터 적용합니다. "
           "점군 주기·방식은 '시험 설정'이 우선합니다. OAK fps·IMU 주기는 VIO 보정과 묶여 있어 여기서 바꾸지 않습니다.",
    "perception": "Jetson 의미분할 값입니다. 바꾸면 다음 시작부터 적용합니다(바꾼 값이 없으면 기존처럼 1 Hz). "
                  "모델 선택은 '시험 설정'에서 합니다.",
}


def board_of(key: str) -> str:
    return BOARD[key]


def local_save(key: str):
    return lambda configs, state_root: SPECS[key].save(configs, state_root)


def yaml_value(value) -> str:
    """ROS 2 parameter value text; a float always keeps a decimal point (double parameters refuse 20)."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError("유한한 숫자만 저장할 수 있습니다")
        text = repr(value)
        return text if any(c in text for c in ".e") else text + ".0"
    if isinstance(value, str):
        import json
        return json.dumps(value, ensure_ascii=False)
    raise ValueError("지원하지 않는 값 형식")


def overrides_yaml(node: str, changed: dict) -> str:
    """Parameter file with only the changed keys; NO_CHANGES (no ros__parameters) when nothing changed."""
    if not changed:
        return NO_CHANGES
    lines = [f"{node}:", "  ros__parameters:"]
    lines += [f"    {key}: {yaml_value(value)}" for key, value in changed.items()]
    return "\n".join(lines) + "\n"


def start_notes(*stdouts: str) -> list[str]:
    """Korean lines from start/preflight output: 'MODULE_SETTINGS applied=<name>' and VIO_SETTINGS=unsupported."""
    applied, notes = [], []
    for stdout in stdouts:
        for line in (stdout or "").splitlines():
            if line.startswith("MODULE_SETTINGS applied="):
                name = line.split("=", 1)[1].strip()
                label = NOTE_NAMES.get(name, name)
                if label not in applied:
                    applied.append(label)
            elif line.startswith("MODULE_SETTINGS invalid="):
                name = line.split("=", 1)[1].strip()
                notes.append(f"{NOTE_NAMES.get(name, name)} 설정 파일 형식 오류: 보드 기본값으로 실행")
            elif line.strip() == "VIO_SETTINGS=unsupported" and VIO_UNSUPPORTED not in notes:
                notes.append(VIO_UNSUPPORTED)
    if applied:
        notes.insert(0, "모듈 세부 설정 적용: " + ", ".join(applied))
    return notes


class SettingsDialog(_Dialog):
    def __init__(self, parent, state_root, key: str, board=None):
        where = BOARD_LABEL[BOARD[key]]
        super().__init__(parent, state_root, SPECS[key], f"{SPECS[key].label} 설정 · 다음 시작부터 {where}에 적용",
                         INTROS[key], board=board)
