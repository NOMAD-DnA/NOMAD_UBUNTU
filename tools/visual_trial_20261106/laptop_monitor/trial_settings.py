"""Trial settings chosen before 'Start' and changed later in a popup (2026-11-12).

One place for the options that used to live only in the live window:
perception model, OAK cloud source, slope compute mode, motor control mode,
planner placement and GNSS. The launcher saves the last choice per laptop and
passes it to startup.py (boards start with it) and to the live window (defaults
for later changes). No board is contacted here.
"""
from __future__ import annotations

import json
import os
from pathlib import Path

# key: (label, {value: text}, default)
SETTINGS = {
    "semantic_model": ("인지 모델", {"b2": "SegFormer · 야지", "indoor": "SegFormer · 실내", "fcn": "FCN · 실내"}, "b2"),
    # 2026-11-15: "opencv3way" = OpenCV SGBM 3WAY (시차 128, 왼쪽 패딩) — 근거리 바닥 칸이 더 많음. 시작 때만 정함
    # (실행 중 '점군 방식 적용'은 OpenCV/OAK 전환만). Jetson external_stereo_cloud에 sgbm_mode 패치가 있어야 함.
    "cloud_source": ("OAK 점군", {"opencv": "OpenCV StereoSGBM", "device": "OAK 내부 StereoDepth",
                                 "opencv3way": "OpenCV SGBM 3WAY (근거리 넓음 · 시작 때 결정)"}, "opencv"),
    # 2026-11-13: OAK depth cloud rate (was fixed 1 Hz). Higher = fewer empty cells, more Jetson CPU (OpenCV).
    "oak_cloud_hz": ("OAK 점군 주기", {"1": "1 Hz (예전)", "2": "2 Hz", "3": "3 Hz", "5": "5 Hz"}, "3"),
    "slope_mode": ("경사 계산", {"sequential": "기존 방식 (1스레드)", "parallel": "병렬 방식 (3스레드)"}, "sequential"),
    "control_mode": ("제어 방식", {"rpm": "erpm 제어 (VESC 속도)", "vio_duty": "VIO duty 제어"}, "rpm"),
    "placement": ("플래너 위치", {"pi4": "플래너 Pi4 (기존)", "jetson": "플래너 Jetson · 제어 Pi4"}, "pi4"),
    "gnss": ("GNSS", {"on": "GNSS 사용", "off": "GNSS 사용 안 함 (VIO만)"}, "on"),
    # 2026-11-13: new mapper (height map, mapper2) is a trial option; details in mapper2_settings.py.
    # 2026-11-13: "terrain" = legacy map logic in one process (terrain_settings.py).
    "mapper": ("지도(매퍼)", {"legacy": "기존 매퍼", "terrain": "기존 매퍼 개선 (단일 프로세스)",
                             "indoor": "실내 매퍼 (LiDAR 전용)",
                             "mapper2": "새 매퍼 (높이 지도 · 시험용)",
                             # 2026-11-15: 새 매퍼3 v3 (Jetson SSD /mnt/ssd/nomad/nomad_mapper3_ws, 추천 설정 그대로).
                             "mapper3": "새 매퍼3 v3 (경사 정확도·넓이 개선)"}, "legacy"),
}
# Applied when the boards start; the others apply when the planner connects.
START_KEYS = ("semantic_model", "cloud_source", "oak_cloud_hz", "slope_mode", "gnss", "mapper")
PLANNER_KEYS = ("control_mode", "placement")


def defaults() -> dict:
    return {key: spec[2] for key, spec in SETTINGS.items()}


def validate(values) -> dict:
    """Known keys with allowed values only; anything else falls back to the default."""
    out = defaults()
    if isinstance(values, dict):
        for key, (_label, choices, _default) in SETTINGS.items():
            if values.get(key) in choices:
                out[key] = values[key]
    return out


def path(state_root=None) -> Path:
    """state_root = the GUI's nomad-trial state folder (launcher STATE_ROOT)."""
    if state_root is None:
        state_root = Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))) / "nomad-trial"
    return Path(state_root) / "trial_settings.json"


def load(state_root=None) -> dict:
    try:
        return validate(json.loads(path(state_root).read_text(encoding="utf-8")))
    except (OSError, ValueError):
        return defaults()


def save(values, state_root=None) -> dict:
    values = validate(values)
    target = path(state_root)
    target.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    temporary = target.with_suffix(".tmp")
    temporary.write_text(json.dumps(values, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    temporary.chmod(0o600)
    os.replace(temporary, target)
    return values


def summary(values) -> str:
    values = validate(values)
    hidden = {"semantic_model", "cloud_source", "oak_cloud_hz", "slope_mode"} if values["mapper"] == "indoor" else set()
    return " / ".join(SETTINGS[key][1][values[key]] for key in SETTINGS if key not in hidden)


def start_arguments(values) -> list:
    """startup.py arguments for the settings applied at board start."""
    values = validate(values)
    args = ["--semantic-model", values["semantic_model"], "--cloud-source", values["cloud_source"],
            "--oak-cloud-hz", values["oak_cloud_hz"],
            "--slope-mode", values["slope_mode"]]
    if values["gnss"] == "off":
        args.append("--no-gnss")
    if values["mapper"] in ("mapper2", "terrain", "mapper3", "indoor"):
        args += ["--mapper", values["mapper"]]
    return args


def monitor_arguments(values) -> list:
    values = validate(values)
    return ["--trial-settings", json.dumps(values, separators=(",", ":"))]


def form(parent=None, values=None, keys=None):
    """QWidget with one combo per setting. Returns (widget, combos dict)."""
    from PyQt5 import QtWidgets
    values = validate(values)
    widget = QtWidgets.QWidget(parent)
    grid = QtWidgets.QGridLayout(widget)
    grid.setContentsMargins(0, 0, 0, 0)
    combos = {}
    for index, key in enumerate(keys or SETTINGS):
        label, choices, _default = SETTINGS[key]
        combo = QtWidgets.QComboBox()
        for value, text in choices.items():
            combo.addItem(text, value)
        combo.setCurrentIndex(list(choices).index(values[key]))
        combos[key] = combo
        row, column = divmod(index, 3)
        grid.addWidget(QtWidgets.QLabel(label), row, column * 2)
        grid.addWidget(combo, row, column * 2 + 1)
    if "mapper" in combos:
        def refresh_indoor():
            indoor = combos["mapper"].currentData() == "indoor"
            for key in ("semantic_model", "cloud_source", "oak_cloud_hz", "slope_mode"):
                if key in combos:
                    combos[key].setVisible(not indoor)
                    index = list(keys or SETTINGS).index(key)
                    row, column = divmod(index, 3)
                    grid.itemAtPosition(row, column * 2).widget().setVisible(not indoor)
        combos["mapper"].currentIndexChanged.connect(refresh_indoor)
        refresh_indoor()
    return widget, combos


def read_form(combos, base=None) -> dict:
    values = validate(base)
    for key, combo in combos.items():
        values[key] = combo.currentData()
    return validate(values)
