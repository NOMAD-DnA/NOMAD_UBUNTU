"""'새 매퍼3 v3 설정' (2026-11-16, user request): mapper3 class-boundary px, initial safe area, slope k.

History: on 2026-11-16 this was first a separate '매퍼 설정' window for terrain + mapper3 with its own
mapper choice. User correction the same day ("기존 매퍼설정에 그냥 넣으라고"): the improved legacy mapper
(terrain) values moved into its existing '기존 매퍼 개선 설정' dialog (terrain_settings.py, terrain.json
"gui_overrides"), the mapper is chosen only in the trial-settings dialog, and this module is the mapper3
dialog next to the other per-mapper buttons.

Values live on the Jetson in ~/nomad_settings/mapper_overrides/current.json
(board_settings.MapperOverridesModule), so every laptop shares them:
  {"mapper3": {"erode_px": null|int, "start_free_radius_m": null|float, "slope_k": null|float}}
null = keep the board file value. A "terrain" section written by the earlier window is ignored (dropped on
the next save); remote/start_jetson.sh only prints a note when it still has values.
remote/start_jetson.sh applies the values at Start to the per-run copy $run/mapper3_cfg/mapper3.json
(sem_fit.erode_px, lidar.start_free_radius_m; slope_k scales cost.slope_free/mid/high_deg by 4/k).
With every value null nothing is rewritten (same start as before).
start_note() still reads the start_jetson.sh 'MAPPER_SETTINGS kind=terrain|mapper3 ...' line for both.
"""
from __future__ import annotations

from pathlib import Path

from module_settings import SettingsDialog as _Dialog, SettingsSpec

CONFIG_DIR = Path(__file__).resolve().parent / "mapper_config"
PARTS = (("mapper3", "새 매퍼3 v3"),)
REMOTE_PATH = "nomad_settings/mapper_overrides/current.json"   # relative to the Jetson user's home
# start_note() reads the start_jetson.sh line for both mappers (terrain values: terrain_settings.py).
PX_KEYS = (("terrain", "boundary_margin_px"), ("mapper3", "erode_px"))
RADIUS_KEYS = (("terrain", "startup_radius_m"), ("mapper3", "start_free_radius_m"))
MAPPERS = ("terrain", "mapper3")


class _Spec(SettingsSpec):
    def validate(self, configs: dict) -> dict:
        out = super().validate(configs)
        for part, key in PX_KEYS:
            if part not in out:
                continue
            value = out[part][key]
            if value is not None:
                if value != int(value):
                    raise ValueError("클래스 경계 px: 정수여야 합니다")
                out[part][key] = int(value)
        return out


SPEC = _Spec(CONFIG_DIR, PARTS, "mapper_settings.json", "새 매퍼3 v3 설정")


def defaults() -> dict:
    return SPEC.defaults()


def validate(configs: dict) -> dict:
    return SPEC.validate(configs)


def load(state_root) -> dict:
    return SPEC.load(state_root)


def save(configs: dict, state_root) -> dict:
    return SPEC.save(configs, state_root)


def _number(value, unit) -> str:
    if value is None:
        return "보드값"
    return f"{int(value)}px" if unit == "px" else f"{float(value):g} m"


def summary(configs: dict, mapper: str = "mapper3") -> str:
    """Short text for the launcher (this laptop's last-read copy). terrain: terrain_settings.gui_summary."""
    if mapper != "mapper3":
        return ""
    configs = validate(configs)
    px = dict(PX_KEYS)[mapper]
    radius = dict(RADIUS_KEYS)[mapper]
    text = f"경계 {_number(configs[mapper][px], 'px')} · 초기영역 {_number(configs[mapper][radius], 'm')}"
    k = configs[mapper].get("slope_k")
    if mapper == "mapper3" and k is not None and abs(k - 4.0) > 1e-9:
        text += f" · 경사계수 {k:g}"
    return text


def start_note(stdout: str) -> str:
    """Korean line from start_jetson.sh 'MAPPER_SETTINGS kind=... key=value ... overridden=...'."""
    for line in (stdout or "").splitlines():
        if not line.startswith("MAPPER_SETTINGS "):
            continue
        fields = dict(item.split("=", 1) for item in line.split()[1:] if "=" in item)
        kind = fields.get("kind")
        if kind not in MAPPERS:
            return ""
        px, radius = dict(PX_KEYS)[kind], dict(RADIUS_KEYS)[kind]
        show = lambda v, unit: "파일 기본" if v in (None, "-") else (v + ("px" if unit == "px" else " m"))
        text = f"매퍼 설정: 클래스 경계 {show(fields.get(px), 'px')} · 초기 안전영역 {show(fields.get(radius), 'm')}"
        overridden = fields.get("overridden", "none")
        return text + (" (GUI 값 적용)" if overridden != "none" else " (보드 파일 값)")
    return ""


class SettingsDialog(_Dialog):
    """Jetson-stored mapper3 values (no mapper choice here: that is the trial-settings 'mapper' box)."""

    def __init__(self, parent, state_root, board=None):
        super().__init__(parent, state_root, SPEC, "새 매퍼3 v3 설정 · 시험 시작 때 Jetson에 적용",
                         "Jetson에 저장된 '새 매퍼3 v3' 값입니다. 클래스 경계 px, 초기 안전영역 반경, 경사 점수 계수를 "
                         "정합니다. 비워 두면(맨 아래 값) Jetson mapper3.json 값을 그대로 씁니다. 실행 때마다 사본에만 "
                         "적용합니다. '시험 설정'에서 지도를 '새 매퍼3 v3'로 고르면 시작할 때 이 값으로 켜집니다.", board=board)
        for (_part, key), (widget, _default) in self.widgets.items():
            if key in dict(PX_KEYS).values():
                widget.setDecimals(0)
                widget.setSingleStep(1.0)
