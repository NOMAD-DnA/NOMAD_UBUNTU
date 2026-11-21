"""Jetson-stored module settings (2026-11-13, user request).

The Jetson is the one place where the planner and new-mapper (mapper2) settings live:
- read():  the board's current values (GUI defaults when the dialog opens), the board's own
           installed defaults ("기본값으로"), the saved presets and a change fingerprint.
- save():  writes edited values back to the Jetson. The previous files are copied to
           ~/nomad_settings/backup/<module>/<time>/ first, and nothing is written when another
           laptop changed the files after this one read them (fingerprint mismatch).
- presets: named snapshots in ~/nomad_settings/presets/<module>/<name>.json ("잘 되던 값").

Board files (all under the Jetson user's home):
  planner  current  nomad_settings/planner/current.json (+ overrides.yaml for the planner start)
           default  nomad_planning_ws/install/.../config/planning_output.yaml
  mapper2  current  nomad_mapper2_ws/active_config/<part>.json (read by start_jetson.sh)
           default  nomad_mapper2_ws/src/<package>/config/<part>.json
  terrain  current  nomad_terrain_ws/active_config/terrain.json (improved legacy mapper; its
                    "gui_overrides" object is GUI-only and removed by start_jetson.sh before the mapper runs)
           default  nomad_terrain_ws/src/nomad_terrain_mapper/config/terrain.json
The laptop keeps the last board values in its state folder only as an offline cache.
Only bash/coreutils run on the board (scripts are sent over SSH like the other remote/ scripts).
"""
from __future__ import annotations

import base64
import hashlib
import json
import re
import shlex
import tempfile
import time
from pathlib import Path

from module_settings import check_value, leaves, set_dotted

SETTINGS_DIR = "nomad_settings"
PRESET_NAME = re.compile(r"^[0-9A-Za-z가-힣][0-9A-Za-z가-힣 _.()+-]{0,47}$")
EOF_MARK = "NOMAD_SETTINGS_EOF"


class BoardError(RuntimeError):
    """Korean message for the dialog."""


def _hash_lines(contents: list) -> str:
    """Fingerprint of the current files, the same on laptop and board (see _fingerprint_bash)."""
    lines = [hashlib.sha256(c).hexdigest() if c is not None else "missing" for c in contents]
    return hashlib.sha256(("\n".join(lines) + "\n").encode()).hexdigest()


def _fingerprint_bash(paths: list) -> str:
    items = " ".join('"$HOME"/' + shlex.quote(p) for p in paths)
    return ("{ for p in " + items + "; do if [ -e \"$p\" ]; then sha256sum < \"$p\" | cut -c1-64; "
            "else echo missing; fi; done; } | sha256sum | cut -c1-64")


def parse_ros_params(text: str) -> dict:
    """Flat scalar values under ros__parameters of a one-node ROS 2 parameter file."""
    out, inside = {}, False
    for raw in text.splitlines():
        line = raw.split(" #", 1)[0].rstrip()
        if not line.strip() or line.lstrip().startswith("#"):
            continue
        if line.strip() == "ros__parameters:":
            inside = True
            continue
        if not inside:
            continue
        if not line.startswith("    ") or ":" not in line:
            if not line.startswith(" "):
                inside = False
            continue
        key, value = (s.strip() for s in line.split(":", 1))
        if value.startswith("[") and key == "rpp.lookahead_candidates":
            try:
                out[key] = json.loads(value)
            except ValueError as exc:
                raise ValueError("잘못된 캐럿 후보 배열") from exc
            continue
        if not value or value.startswith("["):
            continue
        if value in ("true", "false"):
            out[key] = value == "true"
            continue
        try:
            out[key] = int(value)
            continue
        except ValueError:
            pass
        try:
            out[key] = float(value)
            continue
        except ValueError:
            pass
        out[key] = value.strip("'\"")
    return out


def merge_values(spec, base: dict, given_parts: dict | None = None, flat: dict | None = None):
    """Board values over `base`, one documented key at a time.
    Returns (configs, problems): a value outside the documented range keeps the base value and is
    listed in problems (the dialog shows it, so saving does not silently change it)."""
    import copy
    out, notes, problems = copy.deepcopy(base), spec.docs(), []
    for part, _label in spec.parts:
        given = {}
        if flat is not None:
            given = flat
        elif isinstance((given_parts or {}).get(part), dict):
            given = leaves(given_parts[part])
        for key, default in leaves(base[part]).items():
            if key not in given:
                continue
            try:
                set_dotted(out[part], key, check_value(default, notes[part].get(key, {}), given[key]))
            except (ValueError, TypeError) as exc:
                problems.append(f"{key} = {given[key]!r} ({exc})")
    return out, problems


class Module:
    """One settings module on a board (Jetson unless `board` says pi5, 2026-11-16)."""
    board, board_label = "jetson", "Jetson"

    def __init__(self, key, label, spec, current_paths, default_paths, local_save):
        self.key, self.label, self.spec = key, label, spec
        self.current_paths = list(current_paths)
        self.default_paths = list(default_paths)
        self.local_save = local_save

    # -- board file shapes -------------------------------------------------
    def configs_from_current(self, files: list):
        raise NotImplementedError

    def configs_from_default(self, files: list):
        raise NotImplementedError

    def current_files(self, configs: dict) -> list:
        raise NotImplementedError


class PlannerModule(Module):
    def __init__(self):
        import planning_settings as ps
        super().__init__(
            "planner", "플래너", ps.SPEC,
            [f"{SETTINGS_DIR}/planner/current.json", f"{SETTINGS_DIR}/planner/overrides.yaml"],
            ["nomad_planning_ws/install/nomad_planning_cpp/share/nomad_planning_cpp/config/planning_output.yaml"],
            ps.save)
        self.overrides_yaml = ps.overrides_yaml

    def configs_from_default(self, files):
        if files[0] is None:
            return None, []
        return merge_values(self.spec, self.spec.defaults(), flat=parse_ros_params(files[0].decode("utf-8", "replace")))

    def configs_from_current(self, files):
        if files[0] is None:
            return None, []
        data = json.loads(files[0].decode("utf-8"))
        return data, []

    def current_files(self, configs):
        configs = self.spec.validate(configs)
        return [json.dumps(configs, ensure_ascii=False, indent=1) + "\n", self.overrides_yaml(configs)]


class Mapper2Module(Module):
    PACKAGES = {"common": "nomad_mapper2_bringup", "height": "nomad_height_layer",
                "semantic": "nomad_semantic_layer", "cost": "nomad_cost_layer"}

    def __init__(self):
        import mapper2_settings as m2
        parts = [p for p, _ in m2.PARTS]
        super().__init__(
            "mapper2", "새 매퍼", m2.SPEC,
            [f"{m2.REMOTE_DIR}/{p}.json" for p in parts],
            [f"nomad_mapper2_ws/src/{self.PACKAGES[p]}/config/{p}.json" for p in parts],
            m2.save)
        self.parts = parts

    def _parts(self, files):
        if all(f is None for f in files):
            return None
        return {p: json.loads(f.decode("utf-8")) for p, f in zip(self.parts, files) if f is not None}

    def configs_from_default(self, files):
        given = self._parts(files)
        return (None, []) if given is None else merge_values(self.spec, self.spec.defaults(), given)

    def configs_from_current(self, files):
        given = self._parts(files)
        return (None, []) if given is None else (given, [])

    def current_files(self, configs):
        configs = self.spec.validate(configs)
        return [json.dumps(configs[p], ensure_ascii=False, indent=1) + "\n" for p in self.parts]


class TerrainModule(Mapper2Module):
    """Improved legacy mapper (2026-11-13): one terrain.json under nomad_terrain_ws/active_config."""
    PACKAGES = {"terrain": "nomad_terrain_mapper"}

    def __init__(self):
        import terrain_settings as t
        parts = [p for p, _ in t.PARTS]
        Module.__init__(
            self, "terrain", "기존 매퍼 개선", t.SPEC,
            [f"{t.REMOTE_DIR}/{p}.json" for p in parts],
            [f"nomad_terrain_ws/src/{self.PACKAGES[p]}/config/{p}.json" for p in parts] +
            ["nomad_ws/tools/visual_trial_20261106/runtime-map-preview/share/nomad_costmap_cpp/config/semantic.real.json"],
            t.save)
        self.parts = parts
        self.board_json = t.board_json

    def configs_from_default(self, files):
        import terrain_settings as t
        base = t.defaults()
        if len(files) > 1 and files[1] is not None:
            runtime = json.loads(files[1].decode("utf-8"))
            moving = runtime.get("moving_front_free_region", {})
            for key, field in (("moving_front_enabled", "enabled"),
                               ("moving_front_radius_m", "radius_m"),
                               ("moving_rear_length_m", "rear_length_m"),
                               ("moving_rear_half_width_m", "rear_half_width_m")):
                if field in moving:
                    value = moving[field]
                    if field == "enabled":
                        if not isinstance(value, bool):
                            raise ValueError("moving_front_free_region.enabled must be boolean")
                        value = "켬" if value else "끔"
                    base["terrain"][t.GUI_KEY][key] = value
        given = self._parts(files[:1])
        if given is not None:
            given = t.without_inherited_moving(given)
        if files[0] is None and (len(files) < 2 or files[1] is None):
            return None, []
        base, problems = merge_values(self.spec, t.defaults(), base)
        if given is not None:
            base, more = merge_values(self.spec, base, given)
            problems += more
        return base, problems

    def configs_from_current(self, files):
        import terrain_settings as t
        given = self._parts(files)
        return (None, []) if given is None else (t.without_inherited_moving(given), [])

    def current_files(self, configs):
        # 2026-11-16: slope k keys only when k != 4 (older terrain binaries reject unknown keys).
        return [json.dumps(self.board_json(configs), ensure_ascii=False, indent=1) + "\n"]


class MapperOverridesModule(Module):
    """'새 매퍼3 v3 설정' (2026-11-16): mapper3 class-boundary px, initial safe area and slope k, one JSON
    file nomad_settings/mapper_overrides/current.json (null = board file value). No board default file:
    the defaults are all null. Applied by remote/start_jetson.sh to the per-run mapper3 copy only.
    A "terrain" section from the earlier combined window is ignored and dropped on the next save (the
    improved mapper's values are in terrain.json "gui_overrides", TerrainModule)."""

    def __init__(self):
        import mapper_settings as ms
        super().__init__("mapper_overrides", "새 매퍼3 v3", ms.SPEC, [ms.REMOTE_PATH], [], ms.save)

    def configs_from_default(self, files):
        return None, []

    def configs_from_current(self, files):
        if files[0] is None:
            return None, []
        data = json.loads(files[0].decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("형식이 올바르지 않음")
        return data, []

    def current_files(self, configs):
        configs = self.spec.validate(configs)
        return [json.dumps(configs, ensure_ascii=False, indent=1) + "\n"]


class RosOverrideModule(Module):
    """모듈 세부 설정 (2026-11-16): ROS 2 nodes started by the GUI scripts (module_params.ROS_PARTS).
    Files: nomad_settings/<key>/current.json (table values) + one overrides.yaml per node holding only
    the keys that differ from the board's own config file (or the GUI default when the node has none).
    The start scripts add an override as a later --params-file; NO_CHANGES files are skipped."""

    def __init__(self, key):
        import module_params as mp
        self.ros_parts = mp.ROS_PARTS[key]
        self.board = mp.board_of(key)
        self.board_label = mp.BOARD_LABEL[self.board]
        self._base = None
        super().__init__(key, mp.SPECS[key].label, mp.SPECS[key],
                         [f"{SETTINGS_DIR}/{key}/current.json"] + [p[3] for p in self.ros_parts],
                         [p[2] for p in self.ros_parts if p[2]], mp.local_save(key))

    def configs_from_default(self, files):
        given, index = {}, 0
        for part, _node, default, _override in self.ros_parts:
            if default:
                if files[index] is not None:
                    given[part] = parse_ros_params(files[index].decode("utf-8", "replace"))
                index += 1
        if not given:
            self._base = None
            return None, []
        configs, problems = merge_values(self.spec, self.spec.defaults(), given)
        self._base = json.loads(json.dumps(configs))   # the dialog edits the returned dict
        return configs, problems

    def configs_from_current(self, files):
        if files[0] is None:
            return None, []
        data = json.loads(files[0].decode("utf-8"))
        if not isinstance(data, dict):
            raise ValueError("형식이 올바르지 않음")
        return data, []

    def changed(self, configs) -> dict:
        """{part: {key: value}} for values that differ from the board default (validated configs)."""
        base = self._base or self.spec.defaults()
        out = {}
        for part, _label in self.spec.parts:
            defaults = leaves(base[part])
            out[part] = {k: v for k, v in leaves(configs[part]).items() if defaults.get(k) != v}
        return out

    def current_files(self, configs):
        import module_params as mp
        configs = self.spec.validate(configs)
        changed = self.changed(configs)
        return ([json.dumps(configs, ensure_ascii=False, indent=1) + "\n"] +
                [mp.overrides_yaml(node, changed[part]) for part, node, _d, _o in self.ros_parts])


class BasaltModule(Module):
    """모듈 세부 설정 (2026-11-16): Pi5 Basalt VIO. Basalt's cereal JSON needs every key, so the saved
    file is the board's whole basalt_vio_config.json with only the edited keys changed (key order and
    int/float kinds kept); "{}" when nothing differs. Used only when run_basalt_cpp.sh reads
    NOMAD_BASALT_VIO_CONFIG (preflight_pi.sh reports it otherwise)."""

    def __init__(self):
        import module_params as mp
        self.board = mp.board_of("vio")
        self.board_label = mp.BOARD_LABEL[self.board]
        self._raw = None
        self._base = None
        super().__init__("vio", mp.SPECS["vio"].label, mp.SPECS["vio"],
                         [f"{SETTINGS_DIR}/vio/current.json", mp.BASALT_OVERRIDE], [mp.BASALT_DEFAULT],
                         mp.local_save("vio"))

    def configs_from_default(self, files):
        import module_params as mp
        self._raw = self._base = None
        if files[0] is None:
            return None, []
        data = json.loads(files[0].decode("utf-8"))
        if not isinstance(data, dict) or not isinstance(data.get("value0"), dict):
            raise ValueError("value0 항목이 없음")
        flat = {k[len(mp.BASALT_PREFIX):]: v for k, v in data["value0"].items() if k.startswith(mp.BASALT_PREFIX)}
        configs, problems = merge_values(self.spec, self.spec.defaults(), {"vio": flat})
        self._raw, self._base = data, json.loads(json.dumps(configs))
        return configs, problems

    configs_from_current = RosOverrideModule.configs_from_current

    def current_files(self, configs):
        import copy
        import module_params as mp
        configs = self.spec.validate(configs)
        base = leaves((self._base or self.spec.defaults())["vio"])
        changed = {k: v for k, v in leaves(configs["vio"]).items() if base.get(k) != v}
        text = "{}\n"
        if changed:
            if self._raw is None:
                raise BoardError("Pi5 basalt_vio_config.json을 읽지 못해 바꾼 VIO 값을 저장할 수 없습니다")
            data = copy.deepcopy(self._raw)
            for key, value in changed.items():
                if mp.BASALT_PREFIX + key not in data["value0"]:
                    raise BoardError(f"Pi5 Basalt 설정에 {key} 항목이 없어 저장하지 않았습니다")
                data["value0"][mp.BASALT_PREFIX + key] = value
            text = json.dumps(data, indent=2) + "\n"
        return [json.dumps(configs, ensure_ascii=False, indent=1) + "\n", text]


MODULES = {"planner": PlannerModule, "mapper2": Mapper2Module, "terrain": TerrainModule,
           "mapper_overrides": MapperOverridesModule, "vio": BasaltModule,
           "fusion": lambda: RosOverrideModule("fusion"), "gnss": lambda: RosOverrideModule("gnss"),
           "oak": lambda: RosOverrideModule("oak"), "perception": lambda: RosOverrideModule("perception")}


def module_board(module_key: str) -> str:
    """'pi5' or 'jetson': the board that stores (and runs) this module's settings."""
    return MODULES[module_key]().board


def _heredoc(path_expr: str, text: str) -> list:
    if EOF_MARK in text:
        raise BoardError("설정 내용에 사용할 수 없는 글자가 있습니다")
    return [f"cat > {path_expr}.tmp <<'{EOF_MARK}'", text.rstrip("\n"), EOF_MARK,
            f"mv -f {path_expr}.tmp {path_expr}"]


def preset_path(module_key: str, name: str) -> str:
    if not PRESET_NAME.match(name or "") or name.strip() != name:
        raise BoardError("프리셋 이름은 한글·영문·숫자로 시작하고 48자 이내여야 합니다 (/ 와 특수문자 불가)")
    return f"{SETTINGS_DIR}/presets/{module_key}/{name}.json"


class BoardStore:
    """Read/save one module's settings on its board (module.board) through control.run_remote."""

    def __init__(self, module_key: str, host: str, password: str | None, state_root, runner=None):
        self.module = MODULES[module_key]()
        self.host, self.password, self.state_root = host, password or None, state_root
        if runner is None:
            from control import run_remote as runner
        self.runner = runner
        self.fingerprint = None

    # -- transport ---------------------------------------------------------
    def _run(self, script: str, timeout_s=15) -> dict:
        with tempfile.NamedTemporaryFile("w", suffix=".sh", prefix="nomad-settings-", delete=False) as handle:
            handle.write(script)
        try:
            return self.runner(self.host, Path(handle.name), self.password, timeout_s=timeout_s)
        finally:
            Path(handle.name).unlink(missing_ok=True)

    @staticmethod
    def _failure(result: dict, where: str = "Jetson") -> str:
        reason = result.get("reason", "")
        account = "user@주소" if where == "Pi5" else "nvidia@주소"
        return {"authentication_failed": "SSH 비밀번호가 맞지 않습니다",
                "ssh_unreachable": f"{where}에 연결하지 못했습니다 (주소·네트워크 확인)",
                "timeout": f"{where} 응답이 늦어 중단했습니다",
                "host_key_check_failed": f"{where} 호스트 키 확인 실패",
                "invalid_host_script_or_options": f"{where} 주소({account})가 비어 있거나 올바르지 않습니다"}.get(reason, f"{where} 작업 실패 (" + reason + ")")

    # -- read --------------------------------------------------------------
    def read(self) -> dict:
        m = self.module
        lines = ["set -o pipefail",
                 'emit() { if [ -r "$HOME/$2" ]; then printf "NOMAD_FILE %s %s\\n" "$1" "$(base64 -w0 < "$HOME/$2")"; fi; }']
        for i, p in enumerate(m.current_paths):
            lines.append(f"emit cur{i} {shlex.quote(p)}")
        for i, p in enumerate(m.default_paths):
            lines.append(f"emit def{i} {shlex.quote(p)}")
        lines += [f'for f in "$HOME"/{SETTINGS_DIR}/presets/{m.key}/*.json; do',
                  '  [ -r "$f" ] || continue',
                  '  printf "NOMAD_PRESET %s %s\\n" "$(printf %s "$(basename "$f" .json)" | base64 -w0)" "$(base64 -w0 < "$f")"',
                  "done", "echo NOMAD_READ_DONE"]
        result = self._run("\n".join(lines) + "\n")
        if not result.get("ok") or "NOMAD_READ_DONE" not in result.get("stdout", ""):
            raise BoardError(self._failure(result, self.module.board_label))
        files, presets = {}, {}
        for line in result["stdout"].splitlines():
            parts = line.split(" ")
            if len(parts) == 3 and parts[0] == "NOMAD_FILE":
                files[parts[1]] = base64.b64decode(parts[2])
            elif len(parts) == 3 and parts[0] == "NOMAD_PRESET":
                try:
                    presets[base64.b64decode(parts[1]).decode("utf-8")] = json.loads(base64.b64decode(parts[2]))
                except (ValueError, UnicodeDecodeError):
                    continue
        current = [files.get(f"cur{i}") for i in range(len(m.current_paths))]
        default = [files.get(f"def{i}") for i in range(len(m.default_paths))]
        self.fingerprint = _hash_lines(current)
        problems = []
        try:
            board_default, p1 = m.configs_from_default(default)
        except (ValueError, UnicodeDecodeError) as exc:
            board_default, p1 = None, [f"보드 기본값 파일을 읽지 못함: {exc}"]
        base = board_default or m.spec.defaults()
        try:
            given, _ = m.configs_from_current(current)
        except (ValueError, UnicodeDecodeError) as exc:
            given, problems = None, [f"보드 현재값 파일을 읽지 못함: {exc}"]
        if given is None:
            values, p2, source = base, [], "default"
        else:
            values, p2 = merge_values(m.spec, base, given)
            source = "current"
        try:
            m.local_save(values, self.state_root)   # offline cache for summaries
        except (OSError, ValueError):
            pass
        return {"values": values, "defaults": base, "source": source, "board_default": board_default is not None,
                "problems": p1 + problems + p2, "presets": presets, "read_at": time.strftime("%H:%M:%S")}

    # -- save --------------------------------------------------------------
    def save(self, configs: dict, force=False) -> None:
        m = self.module
        texts = m.current_files(configs)
        if self.fingerprint is None and not force:
            raise BoardError(f"먼저 {m.board_label} 값을 읽어야 저장할 수 있습니다")
        lines = ["set -eo pipefail", "umask 022", f'mkdir -p "$HOME/{SETTINGS_DIR}"',
                 f'exec 9>"$HOME/{SETTINGS_DIR}/.lock"', "flock -w 5 9"]
        if not force:
            lines += [f"fp=$({_fingerprint_bash(m.current_paths)})",
                      f'if [ "$fp" != {self.fingerprint} ]; then echo NOMAD_SETTINGS_CONFLICT; exit 3; fi']
        lines += [f'backup="$HOME/{SETTINGS_DIR}/backup/{m.key}/$(TZ=Asia/Seoul date +%Y%m%d_%H%M%S)_$$"',
                  'mkdir -p "$backup"']
        for p in m.current_paths:
            q = '"$HOME"/' + shlex.quote(p)
            lines += [f'if [ -e {q} ]; then cp -p {q} "$backup"/; fi', f"mkdir -p \"$(dirname {q})\""]
        for p, text in zip(m.current_paths, texts):
            lines += _heredoc('"$HOME"/' + shlex.quote(p), text)
        lines.append(f"echo \"NOMAD_FINGERPRINT $({_fingerprint_bash(m.current_paths)})\"")
        lines.append("echo NOMAD_SETTINGS_SAVED")
        result = self._run("\n".join(lines) + "\n")
        out = result.get("stdout", "")
        if "NOMAD_SETTINGS_CONFLICT" in out:
            raise BoardError(f"다른 노트북이 그사이 {m.board_label} 값을 바꿨습니다. '{m.board_label}에서 다시 읽기' 후 다시 저장해 주세요")
        if not result.get("ok") or "NOMAD_SETTINGS_SAVED" not in out:
            raise BoardError(self._failure(result, self.module.board_label))
        for line in out.splitlines():
            if line.startswith("NOMAD_FINGERPRINT "):
                self.fingerprint = line.split()[1]
        try:
            m.local_save(configs, self.state_root)
        except (OSError, ValueError):
            pass

    # -- presets -----------------------------------------------------------
    def save_preset(self, name: str, configs: dict, note: str = "", overwrite=False) -> None:
        m = self.module
        path = preset_path(m.key, name)
        body = {"module": m.key, "name": name, "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
                "note": note, "configs": m.spec.validate(configs)}
        q = '"$HOME"/' + shlex.quote(path)
        lines = ["set -eo pipefail", "umask 022", f"mkdir -p \"$(dirname {q})\""]
        if not overwrite:
            lines.append(f"if [ -e {q} ]; then echo NOMAD_PRESET_EXISTS; exit 4; fi")
        lines += _heredoc(q, json.dumps(body, ensure_ascii=False, indent=1) + "\n")
        lines.append("echo NOMAD_PRESET_SAVED")
        result = self._run("\n".join(lines) + "\n")
        out = result.get("stdout", "")
        if "NOMAD_PRESET_EXISTS" in out:
            raise FileExistsError(name)
        if not result.get("ok") or "NOMAD_PRESET_SAVED" not in out:
            raise BoardError(self._failure(result, self.module.board_label))

    def preset_values(self, preset: dict, base: dict):
        """Configs of a preset over `base` (board defaults); out-of-range values are reported."""
        configs = preset.get("configs") if isinstance(preset, dict) else None
        if not isinstance(configs, dict):
            raise BoardError("프리셋 파일 형식이 올바르지 않습니다")
        return merge_values(self.module.spec, base, configs)
