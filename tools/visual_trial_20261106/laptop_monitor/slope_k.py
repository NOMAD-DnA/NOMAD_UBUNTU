"""'경사 점수 계수' k (2026-11-16, user request): one definition for every adjustable mapper.

Below the block angle the slope score is k x slope_deg (< 80); at/above the block angle
B = 80/k the cell is an obstacle (100). k = 4 -> B = 20 deg = the behaviour before this change.
Allowed k: 1.78 .. 16 (B about 45 .. 5 deg; mapper2 limit_deg itself allows 1 .. 45).

Where k lives (one place per mapper, the dialog that already configures that mapper):
  mapper2  '새 매퍼 세부 설정' C 점수: slope.score_per_deg; slope.limit_deg is derived (80/k, read-only).
  terrain  '기존 매퍼 개선 설정': slope_score_per_deg + derived slope_block_deg in terrain.json. Both keys
           are written to the Jetson only when k != 4, because terrain binaries built before the
           terrain_slope_k C++ patch reject unknown terrain.json keys; remote/start_jetson.sh also
           removes them from the per-run copy when the installed binary does not know them.
  mapper3  '새 매퍼3 v3 설정' (mapper_settings.py, mapper_overrides) mapper3.slope_k (null = board file). mapper3 has its own
           piecewise slope cost (cost.slope_free/mid/high_deg, code defaults 8/20/25 deg, 100 above
           high); start_jetson.sh scales those three angles by 4/k (= B/20) on the per-run copy only
           when k != 4, so mapper3's obstacle angle becomes 25 x 4/k (its default 25 deg stays at k = 4).
  legacy   not adjustable.
"""
from __future__ import annotations
import math

SCORE_AT_BLOCK = 80.0
K_DEFAULT = 4.0
K_MIN = 1.78
K_MAX = 16.0
MAPPER3_COST_DEFAULTS = (("slope_free_deg", 8.0), ("slope_mid_deg", 20.0), ("slope_high_deg", 25.0))
UNSUPPORTED_WARNING = "경사 계수 미적용: 개선 매퍼 재빌드 필요"


def display_scale(event):
    """Read the scale carried with the PNG, including relay and recorded packets.

    Old collectors/recordings used 30 degrees. Never infer a running scale from
    this laptop's editable next-trial settings.
    """
    value = event.get("slope_display") if isinstance(event, dict) else None
    if isinstance(value, dict):
        maximum = value.get("max_deg")
        if (isinstance(maximum, (int, float)) and not isinstance(maximum, bool)
                and math.isfinite(maximum) and 0 < maximum <= 90):
            return float(maximum)
    return 30.


def display_legend(event, *, slope):
    maximum = display_scale(event)
    if slope:
        return f"Slope | green 0° / yellow {maximum/2:g}° / red ≥{maximum:g}° | gray: unknown"
    text = "Cost | green 0 / yellow 50 / red 100 | gray: unknown"
    info = event.get("slope_display", {}) if isinstance(event, dict) else {}
    if (isinstance(info, dict) and info.get("applied") is True
            and info.get("mapper") == "terrain" and maximum == info.get("max_deg")):
        text += f"\n경사 막힘 ≥{maximum:g}° / 의미 장애물도 비용 100"
    return text


def is_default(k) -> bool:
    return k is None or abs(float(k) - K_DEFAULT) < 1e-9


def check_k(k, title: str = "경사 점수 계수") -> float:
    if isinstance(k, bool) or not isinstance(k, (int, float)) or k != k:
        raise ValueError(f"{title}: 숫자여야 합니다")
    k = float(k)
    if not K_MIN <= k <= K_MAX:
        raise ValueError(f"{title}: {K_MIN} ~ {K_MAX:g} 범위여야 합니다 (입력 {k:g}; 막힘 각도 약 45° ~ 5°)")
    return k


def block_deg(k) -> float:
    """B = 80/k, rounded for the JSON files (k=4 -> exactly 20.0)."""
    return round(SCORE_AT_BLOCK / float(k), 4)


def mapper3_cost(cost: dict, k) -> dict:
    """mapper3 cost.* slope angles for k (same math as the start_jetson.sh helper)."""
    factor = K_DEFAULT / float(k)
    return {key: round(float(cost.get(key, default)) * factor, 4) for key, default in MAPPER3_COST_DEFAULTS}


def link_dialog(dialog, part: str, k_key: str, b_key: str) -> None:
    """Settings dialog: the block-angle box follows k (80/k) and cannot be edited by hand."""
    k_widget = dialog.widgets[(part, k_key)][0]
    b_widget = dialog.widgets[(part, b_key)][0]
    b_widget.setEnabled(False)
    b_widget.setToolTip("경사 점수 계수에서 자동 계산 (80 ÷ 계수)")

    def follow(value):
        if K_MIN <= value <= K_MAX:
            b_widget.setValue(block_deg(value))
            dialog.exact[(part, b_key)] = block_deg(value)

    k_widget.valueChanged.connect(follow)


def start_note(stdout: str) -> tuple[str, bool]:
    """(Korean line, ok) from start_jetson.sh 'MAPPER_SLOPE ...' lines; ('', True) when none."""
    for line in (stdout or "").splitlines():
        if not line.startswith("MAPPER_SLOPE "):
            continue
        fields = dict(item.split("=", 1) for item in line.split()[1:] if "=" in item)
        status = fields.get("status")
        if status == "unsupported":
            return UNSUPPORTED_WARNING + " (이번 시험은 계수 4·막힘 20° 기존 동작)", False
        k = fields.get("slope_k", "?")
        if fields.get("kind") == "terrain":
            return f"경사 점수 계수 {k} · 막힘 {fields.get('slope_block_deg', '?')}° (기존 매퍼 개선)", True
        if fields.get("kind") == "mapper3":
            return (f"경사 점수 계수 {k} · mapper3 경사 비용 {fields.get('slope_free_deg', '?')}/"
                    f"{fields.get('slope_mid_deg', '?')}/{fields.get('slope_high_deg', '?')}° (마지막 값 이상 100)"), True
    return "", True
