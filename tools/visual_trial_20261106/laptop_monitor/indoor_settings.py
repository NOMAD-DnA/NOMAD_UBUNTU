"""Indoor mapper scalar settings. Hidden sensor mounts are retained on every board save."""
from __future__ import annotations
import copy
import json
import math
from pathlib import Path
from module_settings import SettingsDialog as _Dialog, SettingsSpec, leaves, set_dotted

CONFIG_DIR = Path(__file__).resolve().parent / "indoor_config"
PARTS = (("indoor", "실내 매퍼"),)
REMOTE_DIR = "nomad_indoor_ws/active_config"


class _Spec(SettingsSpec):
    def validate(self, configs):
        raw = leaves(configs.get("indoor", {}))
        for key, value in raw.items():
            if isinstance(value, (int, float)) and not isinstance(value, bool):
                if not math.isfinite(value):
                    raise ValueError(f"{key}: 유한한 값이어야 합니다")
                if key in ("max_pending_frames", "max_map_cells") and value != int(value):
                    raise ValueError(f"{key}: 정수여야 합니다")
        out = super().validate(configs)
        c = out["indoor"]
        if c["map_length_m"] != 0 and c["map_length_m"] < 4:
            raise ValueError("지도 길이는 0 또는 4m 이상이어야 합니다")
        if c["map_length_m"] and round(c["map_length_m"] / c["resolution_m"]) ** 2 > 4_000_000:
            raise ValueError("지도 셀 수가 400만개를 초과합니다")
        if c["max_map_cells"] != 0 and c["max_map_cells"] < 1000:
            raise ValueError("셀 수 상한은 0 또는 1000 이상이어야 합니다")
        if c["slope_score_per_deg"] * c["slope_block_deg"] > 100 + 1e-9:
            raise ValueError("경사 점수 계수와 차단 각도의 곱은 100 이하여야 합니다")
        m = c["moving_free_region"]
        if max(m["rear_length_m"], m["rear_half_width_m"], m["driven_front_m"]) > m["radius_m"]:
            raise ValueError("뒤쪽 길이와 좌우 폭 및 주행 전방 길이는 전방 반경 이하여야 합니다")
        if m["driven_path_memory"] and min(m["rear_length_m"], m["rear_half_width_m"]) <= 0:
            raise ValueError("주행 경로 기억을 켜려면 뒤쪽 길이와 폭이 양수여야 합니다")
        return out


SPEC = _Spec(CONFIG_DIR, PARTS, "indoor_settings.json", "실내 매퍼 설정")
def defaults(): return SPEC.defaults()
def validate(configs): return SPEC.validate(configs)
def load(state_root): return SPEC.load(state_root)
def save(configs, state_root): return SPEC.save(configs, state_root)
def summary(configs): return SPEC.summary(configs)


def runtime_config(configs, base=None):
    """Merge documented scalar fields; never replace board-specific calibration with GUI defaults."""
    c = validate(configs)["indoor"]
    runtime = copy.deepcopy(base if base is not None else json.loads((CONFIG_DIR / "runtime.json").read_text()))
    for key, value in leaves(c).items():
        set_dotted(runtime, key, value)
    return runtime


def upload_script(configs, if_missing=False):
    text = json.dumps(runtime_config(configs), ensure_ascii=False, indent=1)
    lines = ["set -eo pipefail", 'dir="$HOME/' + REMOTE_DIR + '"', 'mkdir -p "$dir"']
    if if_missing:
        lines.append('if [ ! -e "$dir/indoor.json" ]; then')
    lines += ['cat > "$dir/indoor.json.tmp" <<\'NOMAD_INDOOR_EOF\'', text, "NOMAD_INDOOR_EOF",
              'mv -f "$dir/indoor.json.tmp" "$dir/indoor.json"']
    if if_missing:
        lines.append("fi")
    lines.append('echo "INDOOR_CONFIG=$dir/indoor.json"')
    return "\n".join(lines) + "\n"


class SettingsDialog(_Dialog):
    def __init__(self, parent, state_root, board=None):
        super().__init__(parent, state_root, SPEC, "실내 매퍼 설정",
            "카메라 정면 기준 LiDAR 사용 반각을 ±10~±100도 사이에서 직접 입력할 수 있습니다. "
            "차량 반경 2m 안 종점은 높이와 관계없이 장애물이며 광선 통과 셀은 포장면입니다. "
            "장애물 셀 고정을 켜면 광선이 통과해도 장애물을 유지합니다. XY와 회전 및 3차원 점군은 유지합니다. 저장값은 다음 실내 매퍼 시작부터 적용됩니다.", board=board)

        angle = self.widgets[("indoor", "lidar_half_fov_deg")][0]
        angle.setSingleStep(0.5)
        self.fov_summary = self.QtWidgets.QLabel()
        self.dialog.layout().insertWidget(1, self.fov_summary)
        def update_fov(value):
            self.fov_summary.setText(f"LiDAR 사용 범위: 정면 기준 ±{value:g}도 / 전체 {2 * value:g}도")
        angle.valueChanged.connect(update_fov)
        update_fov(angle.value())
