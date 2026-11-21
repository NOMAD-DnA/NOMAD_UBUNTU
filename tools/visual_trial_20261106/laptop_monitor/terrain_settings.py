"""Improved legacy mapper (단일 프로세스 기존 매퍼, 2026-11-13) settings.

The board mapper keeps the legacy runtime files (semantic.real.json, slope.json) unchanged and
reads one small override file, terrain.json: map size, slope angle reference, slope freezing and
semantic label accumulation. Defaults and Korean descriptions ship in terrain_config/ (copied from
the board package nomad_terrain_mapper/config). Like mapper2, the Jetson's
~/nomad_terrain_ws/active_config/terrain.json is the source of truth (board_settings.py); the trial
start only fills it in when it is missing.
"""
from __future__ import annotations

import json
from pathlib import Path

from module_settings import SettingsDialog as _Dialog, SettingsSpec

CONFIG_DIR = Path(__file__).resolve().parent / "terrain_config"
PARTS = (("terrain", "기존 매퍼 개선"),)
REMOTE_DIR = "nomad_terrain_ws/active_config"   # relative to the Jetson user's home
# 2026-11-16 (slope_k.py): slope score multiplier k and derived block angle B = 80/k. GUI-only keys
# while k = 4: board_json() leaves them out, so older terrain binaries (which reject unknown
# terrain.json keys) keep starting; the board package config/terrain.json does not have them.
SLOPE_KEYS = ("slope_score_per_deg", "slope_block_deg")
# 2026-11-16 (user: "기존 매퍼설정에 그냥 넣으라고"): class-boundary px and initial safe area of the
# improved mapper live in this dialog, in terrain.json under "gui_overrides" (null = keep the Jetson's
# semantic.real.json value). GUI-only object: the terrain binary rejects unknown keys, so board_json()
# leaves it out while both are null and remote/start_jetson.sh removes it from the per-run copy
# ($run/terrain_cfg/terrain.json) and applies the values to the per-run legacy cfg copy instead.
GUI_KEY = "gui_overrides"
GUI_PX = "boundary_margin_px"
GUI_RADIUS = "startup_radius_m"
# Concrete defaults from the installed moving-region configuration.
# BoardStore resolves old inherited values from the runtime file before showing the table.
MOVING_DEFAULTS = {
    "moving_front_enabled": "켬",
    "moving_front_radius_m": 0.5,
    "moving_rear_length_m": 0.35,
    "moving_rear_half_width_m": 0.2,
}


def without_inherited_moving(configs: dict) -> dict:
    """Migrate the previous null / 보드값 controls to numeric board or GUI defaults."""
    import copy
    out = copy.deepcopy(configs)
    gui = out.get("terrain", {}).get(GUI_KEY, {})
    for key in MOVING_DEFAULTS:
        if gui.get(key) is None or gui.get(key) == "보드값":
            gui.pop(key, None)
    return out


class _Spec(SettingsSpec):
    def validate(self, configs: dict) -> dict:
        import slope_k
        import math
        out = super().validate(without_inherited_moving(configs))
        cfg = out["terrain"]
        distance = cfg["oak_max_horizontal_distance_m"]
        if not math.isfinite(distance) or not 0.1 <= distance <= 30.0:
            raise ValueError("OAK 점군 누적 최대 수평거리: 0.1~30m의 유한한 값이어야 합니다")
        radius_cm = cfg["oak_lidar_xy_radius_cm"]
        if not math.isfinite(radius_cm) or not 0.1 <= radius_cm <= 100.0:
            raise ValueError("OAK/LiDAR 수평 검색 반경: 0.1~100cm의 유한한 값이어야 합니다")
        height_cm = cfg["oak_lidar_max_abs_z_difference_cm"]
        if not math.isfinite(height_cm) or not 0.1 <= height_cm <= 100.0:
            raise ValueError("OAK/LiDAR 높이 차이 제거 기준: 0.1~100cm의 유한한 값이어야 합니다")
        cfg["slope_score_per_deg"] = slope_k.check_k(cfg["slope_score_per_deg"])
        cfg["slope_block_deg"] = slope_k.block_deg(cfg["slope_score_per_deg"])
        px = cfg[GUI_KEY][GUI_PX]
        if px is not None:
            if px != int(px):
                raise ValueError("클래스 경계 px: 정수여야 합니다")
            cfg[GUI_KEY][GUI_PX] = int(px)
        moving = cfg[GUI_KEY]
        for key in MOVING_DEFAULTS:
            value = moving[key]
            if key != "moving_front_enabled" and value is not None and not math.isfinite(value):
                raise ValueError("이동 비용 0 영역: 유한한 값을 입력하세요")
        radius = moving["moving_front_radius_m"]
        if radius is not None:
            for key in ("moving_rear_length_m", "moving_rear_half_width_m"):
                if moving[key] is not None and moving[key] > radius:
                    raise ValueError("이동 영역 뒤쪽 길이와 좌우 각각의 폭은 전방 반경 이하여야 합니다")
        return out


SPEC = _Spec(CONFIG_DIR, PARTS, "terrain_settings.json", "기존 매퍼 개선 설정")


def defaults() -> dict:
    return SPEC.defaults()


def validate(configs: dict) -> dict:
    return SPEC.validate(configs)


def load(state_root) -> dict:
    return SPEC.load(state_root)


def save(configs: dict, state_root) -> dict:
    return SPEC.save(configs, state_root)


def summary(configs: dict) -> str:
    return SPEC.summary(configs)


def board_json(configs: dict) -> dict:
    """terrain.json content for the Jetson: the slope k keys only when k != 4, gui_overrides only
    with explicit moving-region values. Other GUI-only values remain optional."""
    import slope_k
    cfg = validate(configs)["terrain"]
    if slope_k.is_default(cfg["slope_score_per_deg"]):
        for key in SLOPE_KEYS:
            cfg.pop(key, None)
    return cfg


def gui_summary(configs: dict) -> str:
    """Launcher text for the class-boundary px / initial safe area (this laptop's last-read copy)."""
    gui = validate(configs)["terrain"][GUI_KEY]
    px, radius = gui[GUI_PX], gui[GUI_RADIUS]
    return (f"경계 {'보드값' if px is None else f'{int(px)}px'} · "
            f"초기영역 {'보드값' if radius is None else f'{float(radius):g} m'}")


def upload_script(configs: dict, if_missing: bool = False) -> str:
    """Bash text that writes terrain.json on the Jetson (atomic). if_missing: the saved board file wins."""
    text = json.dumps(board_json(configs), ensure_ascii=False, indent=1)
    if "NOMAD_TERRAIN_EOF" in text:
        raise ValueError("설정 내용에 사용할 수 없는 글자가 있습니다")
    write = ["cat > \"$dir/terrain.json.tmp\" <<'NOMAD_TERRAIN_EOF'", text, "NOMAD_TERRAIN_EOF",
             'mv -f "$dir/terrain.json.tmp" "$dir/terrain.json"']
    if if_missing:
        write = ['if [ ! -e "$dir/terrain.json" ]; then'] + write + ["fi"]
    lines = ["set -eo pipefail", f'dir="$HOME/{REMOTE_DIR}"', 'mkdir -p "$dir"'] + write
    lines.append('echo "TERRAIN_CONFIG_UPLOADED=$dir"')
    return "\n".join(lines) + "\n"


class SettingsDialog(_Dialog):
    def __init__(self, parent, state_root, board=None):
        super().__init__(parent, state_root, SPEC, "기존 매퍼 개선 설정 · 시험 시작 때 Jetson에 적용",
                         "Jetson에 저장된 '기존 매퍼 개선' 값입니다. 경사·인지·비용 계산은 기존 매퍼와 같고, "
                         "여기서는 지도 크기, 경사 각도 기준, 경사 고정, 의미분할 이미지 누적, 경사 점수 계수, "
                         "OAK 점군 누적 최대 수평거리(기본 2m), OAK/LiDAR 높이 차이 제거 기준(기본 12cm), 수평 검색 반경(기본 20cm), 클래스 경계 px, 초기 안전영역 반경과 이동 비용 0 영역의 사용 여부, 전방 반경, 뒤쪽 길이와 좌우 폭을 바꿉니다. "
                         "'시험 설정'에서 지도를 '기존 매퍼 개선'으로 고르면 시작할 때 이 값으로 켜집니다.", board=board)
        # 2026-11-16: '경사 막힘 각도' follows '경사 점수 계수' (80/k) and is read-only.
        import slope_k
        slope_k.link_dialog(self, "terrain", "slope_score_per_deg", "slope_block_deg")
        px_box = self.widgets[("terrain", f"{GUI_KEY}.{GUI_PX}")][0]
        px_box.setDecimals(0)
        px_box.setSingleStep(1.0)
