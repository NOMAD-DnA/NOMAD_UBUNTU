"""New mapper (mapper2) settings: per-part tables with descriptions, saved per laptop (2026-11-13).

Defaults and Korean descriptions ship in mapper2_config/ (copied from the board packages:
common/height/semantic/cost .json + *_doc.json). The user's edits are saved under the GUI state
folder and uploaded to the Jetson (~/nomad_mapper2_ws/active_config) when a trial starts with
"새 매퍼". The board reads them once at start; values outside the documented range are refused
here and again on the board. Shared table/validation code lives in module_settings.py.
2026-11-13 (user request): the Jetson's active_config is the source of truth (board_settings.py):
the dialog reads it, saves there, keeps presets there; the start only fills in missing files.
"""
from __future__ import annotations

import json
from pathlib import Path

from module_settings import (SettingsDialog as _Dialog, SettingsSpec, check_value, get_dotted,  # noqa: F401
                             leaves, set_dotted)

CONFIG_DIR = Path(__file__).resolve().parent / "mapper2_config"
PARTS = (("common", "공통 (지도·위치)"), ("height", "A 높이·경사"), ("semantic", "B 의미 쓰기"), ("cost", "C 점수"))
REMOTE_DIR = "nomad_mapper2_ws/active_config"   # relative to the Jetson user's home


class _Spec(SettingsSpec):
    """2026-11-16 (slope_k.py): cost slope.limit_deg is derived from slope.score_per_deg (80/k)."""

    def validate(self, configs: dict) -> dict:
        import slope_k
        out = super().validate(configs)
        slope = out["cost"]["slope"]
        slope["score_per_deg"] = slope_k.check_k(slope["score_per_deg"])
        slope["limit_deg"] = slope_k.block_deg(slope["score_per_deg"])
        return out


SPEC = _Spec(CONFIG_DIR, PARTS, "mapper2_settings.json", "새 매퍼 설정")


def defaults() -> dict:
    return SPEC.defaults()


def docs() -> dict:
    return SPEC.docs()


def validate(configs: dict) -> dict:
    return SPEC.validate(configs)


def path(state_root) -> Path:
    return SPEC.path(state_root)


def load(state_root) -> dict:
    return SPEC.load(state_root)


def save(configs: dict, state_root) -> dict:
    return SPEC.save(configs, state_root)


def changed_keys(configs: dict) -> list[str]:
    return SPEC.changed_keys(configs)


def summary(configs: dict) -> str:
    return SPEC.summary(configs)


def upload_script(configs: dict, if_missing: bool = False) -> str:
    """Bash text that writes the four config files on the Jetson (atomic per file).
    if_missing (2026-11-13): the Jetson's saved values win; only absent files are written."""
    configs = validate(configs)
    lines = ["set -eo pipefail", f'dir="$HOME/{REMOTE_DIR}"', 'mkdir -p "$dir"']
    for part, _ in PARTS:
        text = json.dumps(configs[part], ensure_ascii=False, indent=1)
        if "NOMAD_MAPPER2_EOF" in text:
            raise ValueError("설정 내용에 사용할 수 없는 글자가 있습니다")
        write = [f"cat > \"$dir/{part}.json.tmp\" <<'NOMAD_MAPPER2_EOF'", text, "NOMAD_MAPPER2_EOF",
                 f'mv -f "$dir/{part}.json.tmp" "$dir/{part}.json"']
        if if_missing:
            write = [f'if [ ! -e "$dir/{part}.json" ]; then'] + write + ["fi"]
        lines += write
    lines.append('echo "MAPPER2_CONFIG_UPLOADED=$dir"')
    return "\n".join(lines) + "\n"


class SettingsDialog(_Dialog):
    def __init__(self, parent, state_root, board=None):
        super().__init__(parent, state_root, SPEC, "새 매퍼 세부 설정 · 시험 시작 때 Jetson에 적용",
                         "Jetson에 저장된 새 매퍼 값입니다. '시험 설정'에서 지도를 '새 매퍼'로 고르면, 시작할 때 "
                         "Jetson에 저장된 값으로 켜집니다 (실행 중인 매퍼는 바뀌지 않음). "
                         "각 줄의 설명을 읽고 값을 바꾸세요. 범위를 벗어난 값은 저장되지 않습니다.", board=board)
        # 2026-11-16: '경사 한계' follows '경사 점수 계수' (80/k) and is read-only.
        import slope_k
        slope_k.link_dialog(self, "cost", "slope.score_per_deg", "slope.limit_deg")
        k = self.exact.get(("cost", "slope.score_per_deg"))
        limit = self.exact.get(("cost", "slope.limit_deg"))
        if isinstance(k, (int, float)) and isinstance(limit, (int, float)) and slope_k.K_MIN <= k <= slope_k.K_MAX \
                and abs(limit - slope_k.block_deg(k)) > 1e-3:
            self.error.setText(f"지금 경사 한계 {limit:g}°가 80 ÷ 계수({k:g}) = {slope_k.block_deg(k):g}°와 다릅니다. "
                               f"저장하면 {slope_k.block_deg(k):g}°로 바뀝니다.")
