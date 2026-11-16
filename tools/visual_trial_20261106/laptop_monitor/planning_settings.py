"""Planner settings (Jetson planner), per-part tables with descriptions (2026-11-13).

Defaults equal the Jetson planner's installed planning_output.yaml (incl. terrain_weight 5.0
and the restored observation_turn_weight 80.0).
Topics, frames and commissioning/prototype flags are not editable here. When the planner is
started on the Jetson ("플래너 Jetson · 제어 Pi4"), the saved values are sent with the start
command as a ROS parameter override file (later --params-file wins). The Pi4 planner is not
changed by these settings.
2026-11-13 (user request): the values live on the Jetson (board_settings.py): the dialog reads them
as its defaults, saves edits there (~/nomad_settings/planner/overrides.yaml, applied by
remote/jetson_planner.sh) and keeps named presets there. The laptop file is only an offline cache.
"""
from __future__ import annotations

import base64
import copy
import json
import os
import warnings
from pathlib import Path

from module_settings import SettingsDialog as _Dialog, SettingsSpec, leaves

CONFIG_DIR = Path(__file__).resolve().parent / "planning_config"
# 2026-11-13 (source audit): the "corridor" part is the GPP side (Hybrid A* search budget and the
# intermediate-goal selection); footprint checks use the observed_guard box, not body_length/width.
PARTS = (("vehicle", "차량·속도·도착"), ("timing", "주기"), ("safety", "안전·시간 제한"), ("cost", "전역 비용"),
         ("corridor", "전역 경로(GPP)·중간 목표"), ("rpp", "경로 따라가기(RPP)"), ("guard", "차체 검사 상자·관측 감속"))
SPEC = SettingsSpec(CONFIG_DIR, PARTS, "planning_settings.json", "플래너 설정")
NODE = "nomad_planning_cpp"
OBSERVATION_WEIGHT_REVISION = "_observation_turn_weight_revision"


def effective_lookaheads(rpp: dict) -> list[float]:
    """Match the C++ legacy ordering exactly unless an explicit ordered array exists."""
    if rpp.get("lookahead_candidates"):
        return list(rpp["lookahead_candidates"])
    first = rpp["lookahead"]
    extra = [x for x in (0.6, 0.8, 1.0, 1.2, 1.5) if abs(x - first) > 1e-9]
    return [first] + (sorted(extra, key=lambda x: abs(x - first)) if rpp["adaptive_lookahead"] else [])


def defaults() -> dict:
    return SPEC.defaults()


def load(state_root) -> dict:
    configs = SPEC.load(state_root)
    try:
        raw = json.loads(SPEC.path(state_root).read_text(encoding="utf-8"))
    except (OSError, ValueError, TypeError):
        return configs
    # User-requested restoration: old laptops saved the former default 0 and
    # would otherwise override the corrected board config on every connection.
    # Migrate once; subsequent explicitly saved values, including 0, stay intact.
    if isinstance(raw, dict) and raw.get(OBSERVATION_WEIGHT_REVISION) != 1:
        if configs["corridor"]["corridor"]["observation_turn_weight"] == 0:
            configs["corridor"]["corridor"]["observation_turn_weight"] = 80.0
        try:
            save(configs, state_root)
        except OSError:
            warnings.warn("관측 회전 가중치 복원 설정을 저장하지 못했습니다. 이번 연결에는 복원된 값을 사용합니다.", RuntimeWarning)
    return configs


def save(configs: dict, state_root) -> dict:
    configs = SPEC.validate(configs)
    target = SPEC.path(state_root)
    target.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    payload = dict(configs)
    payload[OBSERVATION_WEIGHT_REVISION] = 1
    temporary = target.with_suffix(".tmp")
    temporary.write_text(json.dumps(payload, ensure_ascii=False, indent=1) + "\n", encoding="utf-8")
    temporary.chmod(0o600)
    os.replace(temporary, target)
    return configs


def summary(configs: dict) -> str:
    return SPEC.summary(configs)


def _yaml_value(value) -> str:
    if isinstance(value, list):
        return json.dumps(value, allow_nan=False)
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        return repr(value)
    raise ValueError("unsupported planner parameter type")


def overrides_yaml(configs: dict) -> str:
    """ROS 2 parameter file with every editable value (dotted names = planner parameter names)."""
    configs = SPEC.validate(configs)
    lines = [f"{NODE}:", "  ros__parameters:"]
    for part, _ in PARTS:
        for key, value in leaves(configs[part]).items():
            if key == "rpp.lookahead_candidates":
                legacy = dict(configs["rpp"]["rpp"], lookahead_candidates=[])
                if not value or value == effective_lookaheads(legacy):
                    continue  # Unchanged GUI arrays keep the legacy runtime settings exactly.
            lines.append(f"    {key}: {_yaml_value(value)}")
    return "\n".join(lines) + "\n"


def overrides_b64(configs: dict) -> str:
    return base64.b64encode(overrides_yaml(configs).encode("utf-8")).decode("ascii")


def state_root() -> Path:
    import os
    return Path(os.environ.get("XDG_STATE_HOME", str(Path.home() / ".local/state"))).expanduser() / "nomad-trial"


class SettingsDialog(_Dialog):
    def __init__(self, parent, state_root, board=None):
        super().__init__(parent, state_root, SPEC, "플래너 세부 설정 · Jetson 플래너 연결 때 적용",
                         "Jetson에 저장된 플래너 값입니다. 플래너 위치가 'Jetson'일 때, 플래너를 연결(시작)하면 "
                         "Jetson에 저장된 값이 적용됩니다 (실행 중인 플래너는 바뀌지 않음). "
                         "[안전] 표시는 정지·감속·속도와 관련된 값이니 바꾸기 전에 설명을 꼭 읽으세요. "
                         "Pi4 플래너에는 적용되지 않습니다.", board=board)

    def _table(self, part, cfg):
        if part == "rpp":
            cfg = copy.deepcopy(cfg)
            cfg["rpp"]["lookahead_candidates"] = effective_lookaheads(cfg["rpp"])
        table = super()._table(part, cfg)
        if part == "vehicle":
            # Keep the loaded values and serialization; hide only these GUI rows.
            for row in range(table.rowCount()):
                if table.item(row, 0).toolTip() in {"body_length", "body_width"}:
                    table.setRowHidden(row, True)
        if part == "rpp":
            table.setColumnWidth(1, 310)
            for row in range(table.rowCount()):
                key = table.item(row, 0).toolTip()
                if key in {"rpp.lookahead", "rpp.adaptive_lookahead"}:
                    table.setRowHidden(row, True)
                elif key == "rpp.lookahead_candidates":
                    editor, _ = self.widgets[(part, key)]
                    editor.setMaximumHeight(16777215)
                    table.item(row, 2).setText("m (0.1 ~ 3.0) · 1~32개\n기본 " +
                                             json.dumps(effective_lookaheads(self.base[part]["rpp"])))
            table.resizeRowsToContents()
        return table

    def _widget(self, default, doc, value):
        if isinstance(default, list):
            from planning_candidate_editor import CandidateEditor
            return CandidateEditor(doc, value)
        return super()._widget(default, doc, value)

    def set_values(self, configs, parts=None):
        configs = copy.deepcopy(configs)
        configs["rpp"]["rpp"]["lookahead_candidates"] = effective_lookaheads(configs["rpp"]["rpp"])
        super().set_values(configs, parts)

    def reload_board(self):
        try:
            super().reload_board()
        except ValueError as exc:
            self.error.setText(str(exc))
