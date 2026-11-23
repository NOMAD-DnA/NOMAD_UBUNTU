"""Traversability developer window. Displays existing snapshots, without I/O."""
import math
import time
import numpy as np
from PyQt5 import QtGui, QtWidgets
from module_logs import attach_module_logs, update_module_logs
from render_utils import history_signature, stable_metadata
import developer_debug as dbg
from slope_k import display_scale, display_legend

# Costmap value buckets (0..100 → 0..1) and slope buckets (|deg|/30 → 0..1).
COST_BUCKETS = (("비용 0 · 자유", 0., .002), ("비용 1~49", .005, .495), ("비용 50~79", .495, .795),
                ("비용 80~99 · 막힘 판단 참고", .795, .995), ("비용 100 · 치명", .995, 1.))
SLOPE_BUCKETS = (("0~10°", 0., 1/3), ("10~20°", 1/3 + 1e-6, 2/3), ("20~30°", 2/3 + 1e-6, .999), ("30° 이상", .999 + 1e-6, 1.))


def age(snapshot, key):
    values = (snapshot.get("now", time.monotonic()), snapshot.get("received", {}).get(key))
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in values):
        return None
    return max(0., values[0] - values[1])


def age_text(value, threshold=2.):
    return "미수신" if value is None else f"수신 후 {value:.1f}초" + (" · 오래됨" if value > threshold else "")


def valid_grid(event):
    """Same physical XY-grid requirements as the injected MapPanel."""
    grid = event.get("grid", {})
    if event.get("georeferenced") is not True or not isinstance(grid, dict):
        return None
    if grid.get("image_y") != "high_to_low" or not isinstance(grid.get("frame_id"), str) or not grid["frame_id"]:
        return None
    origin = grid.get("origin", {})
    if not isinstance(origin, dict):
        return None
    numbers = [grid.get(k) for k in ("resolution", "width", "height")]
    numbers += [origin.get(k) for k in ("x", "y", "z", "qx", "qy", "qz", "qw")]
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in numbers):
        return None
    if min(numbers[:3]) <= 0 or abs(origin["qx"]) > .001 or abs(origin["qy"]) > .001:
        return None
    if abs(sum(origin[k] ** 2 for k in ("qx", "qy", "qz", "qw")) - 1.) > .03:
        return None
    return grid


class TraversabilityWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None, components=None):
        super().__init__(parent)
        self.setWindowTitle("NOMAD | 주행가능성 개발")
        self.resize(1220, 760)
        self.setStyleSheet("QWidget {background:#111827;color:#e2e8f0;} QLabel, QCheckBox {font-size:12px;}")
        self.panels, self._image_keys, self._pose_keys, self._map_received = {}, {}, {}, {}
        body = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(body)
        self.status = QtWidgets.QLabel("주행가능성: 미수신")
        self.position = QtWidgets.QLabel("공용 위치: 미수신")
        self.heading = QtWidgets.QLabel("화살표 전방축: —")
        for label in (self.status, self.position, self.heading):
            label.setWordWrap(True)
            layout.addWidget(label)
        maps = QtWidgets.QHBoxLayout()
        panel_type = (components or {}).get("MapPanel")
        for source, title in (("costmap", "코스트맵 | XY 상단뷰"), ("slope", "경사도 | XY 상단뷰")):
            panel = panel_type(title, slope=(source == "slope"), heading_note="통합 창의 전방축") if panel_type else QtWidgets.QLabel(title + "\nMapPanel 구성요소 미연결")
            if panel_type:
                self.panels[source] = panel
                panel.meta.setWordWrap(True)
                stable_metadata(panel.meta)
            maps.addWidget(panel, 1)
        layout.addLayout(maps, 1)
        self.show_pose = QtWidgets.QCheckBox("지도 위에 공용 위치·이동 경로 겹쳐 보기")
        self.show_pose.setChecked(True)
        self.show_pose.setToolTip("이미 받은 공용 위치만 켜고 끕니다. 지도 값은 바꾸지 않습니다.")
        layout.addWidget(self.show_pose)
        footer = QtWidgets.QLabel("청록색: 동일 좌표계의 공용 위치·이동 경로 | 회색: 값 미확정(미관측과 관측 후 계산 미완료를 포함할 수 있음)")
        footer.setWordWrap(True)
        footer.setStyleSheet("color:#94a3b8;")
        layout.addWidget(footer)
        self.setCentralWidget(body)
        attach_module_logs(self)
        self._analysis = {}
        self.debug = dbg.DebugTools(self, "traversability", "NOMAD 주행가능성(지도) 개발", (
            {"key": "image:costmap", "label": "코스트맵", "limit": 2., "rate": ("jetson", "costmap")},
            {"key": "image:slope", "label": "경사도 지도", "limit": 2., "rate": ("jetson", "slope")},
            {"key": "status:perception", "label": "인지/지도 상태", "limit": 3., "rate": ("jetson", "perception")},
            {"key": "pose:public", "label": "공용 위치", "limit": 1., "rate": ("pi", "public")},
            {"key": "rates:jetson", "label": "젯슨 Hz 보고", "limit": 3.},
        ), log_dock=self.log_dock)
        self.debug.clear_hooks.append(self._analysis.clear)

    def append_module_log(self, event):
        return self.log_panel.append_event(event)

    def update_snapshot(self, snapshot):
        self.debug.observe(snapshot)
        if self.debug.frozen:
            return
        self._update_view(snapshot)
        self.debug.publish(snapshot, self._debug_checks(snapshot), self._debug_sections(snapshot))

    @staticmethod
    def _latest_pose(snapshot):
        history = snapshot.get("histories", {}).get("public", ())
        return history[-1] if history else None

    def _vehicle_cell(self, snapshot, grid):
        """Grid cell (col,row from low Y) of the latest same-frame public pose."""
        pose = self._latest_pose(snapshot)
        if not grid or not pose or pose.get("frame_id") != grid.get("frame_id"):
            return None
        origin = grid["origin"]
        yaw = 2. * math.atan2(origin["qz"], origin["qw"])
        dx, dy = pose["x"] - origin["x"], pose["y"] - origin["y"]
        lx = math.cos(-yaw) * dx - math.sin(-yaw) * dy
        ly = math.sin(-yaw) * dx + math.cos(-yaw) * dy
        return lx / grid["resolution"], ly / grid["resolution"]

    def _analyse(self, snapshot, source):
        image = snapshot.get("images", {}).get(source)
        event = snapshot.get("image_meta", {}).get(source, {})
        grid = valid_grid(event) if isinstance(event, dict) else None
        if not isinstance(image, QtGui.QImage) or image.isNull():
            return self._analysis.get(source)
        maximum = display_scale(event)
        key = (image.cacheKey(), event.get("t"), maximum)
        cached = self._analysis.get(source)
        if cached and cached["key"] == key:
            return cached
        pixels = dbg.image_rgb(image)
        if pixels is None:
            return cached
        values = dbg.heat_values(pixels)
        slope_buckets = ((f"0~{maximum/3:g}°", 0., 1/3),
                         (f"{maximum/3:g}~{2*maximum/3:g}°", 1/3 + 1e-6, 2/3),
                         (f"{2*maximum/3:g}~{maximum:g}°", 2/3 + 1e-6, .999),
                         (f"{maximum:g}° 이상", .999 + 1e-6, 1.))
        total, rows = dbg.bucket_counts(values, COST_BUCKETS if source == "costmap" else slope_buckets)
        result = {"key": key, "values": values, "total": total, "rows": rows, "grid": grid,
                  "width": image.width(), "height": image.height(), "slope_max_deg": maximum}
        self._analysis[source] = result
        return result

    def _value_near_vehicle(self, snapshot, analysis, radius_m=1.):
        grid = analysis.get("grid") if analysis else None
        cell = self._vehicle_cell(snapshot, grid)
        if cell is None:
            return None
        values, height, width = analysis["values"], analysis["height"], analysis["width"]
        cx, cy = cell
        if not (0 <= cx < grid["width"] and 0 <= cy < grid["height"]):
            return {"inside": False, "cell": cell}
        sx, sy = width / grid["width"], height / grid["height"]
        col, row = int(cx * sx), int((grid["height"] - 1 - cy) * sy)
        reach_x = max(1, int(radius_m / grid["resolution"] * sx))
        reach_y = max(1, int(radius_m / grid["resolution"] * sy))
        here = values[min(row, height - 1), min(col, width - 1)]
        patch = values[max(0, row - reach_y):row + reach_y + 1, max(0, col - reach_x):col + reach_x + 1]
        known = patch[~np.isnan(patch)]
        edge = min(cx, cy, grid["width"] - cx, grid["height"] - cy) * grid["resolution"]
        return {"inside": True, "cell": cell, "here": None if np.isnan(here) else float(here),
                "max": float(known.max()) if known.size else None,
                "unknown_ratio": float(np.isnan(patch).mean()) if patch.size else None, "edge_m": edge}

    def _debug_checks(self, snapshot):
        status = snapshot.get("statuses", {}).get("perception", {})
        meta = snapshot.get("image_meta", {})
        checks = [dbg.status_check("인지/지도 모듈 상태 (READY 사유)", status, age(snapshot, "status:perception")),
                  dbg.freshness("코스트맵 수신", age(snapshot, "image:costmap"), 2., 5.),
                  dbg.freshness("경사도 지도 수신", age(snapshot, "image:slope"), 2., 5.),
                  dbg.rate_check("코스트맵 토픽 Hz (보드)", dbg.reported_rate(snapshot, "jetson", "costmap")),
                  dbg.rate_check("경사도 토픽 Hz (보드)", dbg.reported_rate(snapshot, "jetson", "slope"))]
        for source, name in (("costmap", "코스트맵 좌표 정보"), ("slope", "경사도 좌표 정보")):
            event = meta.get(source, {}) if isinstance(meta.get(source), dict) else {}
            if not event:
                checks.append((name, "fail", "지도 미수신"))
            elif event.get("decode_error"):
                checks.append((name, "warn", "새 영상 해독 실패 · 마지막 정상 지도 유지"))
            elif valid_grid(event):
                g = valid_grid(event)
                checks.append((name, "ok", f"{g['frame_id']} · {g['width']}×{g['height']} · {g['resolution']:.2f}m/cell"))
            else:
                checks.append((name, "fail", "좌표 미확정 · " + dbg.plain(event.get("georeference_reason") or "격자 좌표 정보 없음")))
        checks.append(dbg.freshness("공용 위치 수신", age(snapshot, "pose:public"), 1., 3.))
        grid = valid_grid(meta.get("costmap", {})) if isinstance(meta.get("costmap"), dict) else None
        pose = self._latest_pose(snapshot)
        if not grid or not pose:
            checks.append(("위치·지도 좌표계 일치", "unknown", "지도 또는 위치 미수신"))
        elif pose.get("frame_id") != grid["frame_id"]:
            checks.append(("위치·지도 좌표계 일치", "fail", f"위치 {pose.get('frame_id') or '—'} ≠ 지도 {grid['frame_id']}"))
        else:
            checks.append(("위치·지도 좌표계 일치", "ok", grid["frame_id"]))
        near = self._value_near_vehicle(snapshot, self._analyse(snapshot, "costmap"))
        if near is None:
            checks.append(("차량이 지도 안에 있음", "unknown", "같은 좌표계 위치·지도 필요"))
        elif not near["inside"]:
            checks.append(("차량이 지도 안에 있음", "fail", "차량 위치가 코스트맵 범위 밖"))
        else:
            level = "ok" if near["edge_m"] >= 2. else "warn"
            checks.append(("차량이 지도 안에 있음", level, f"가장자리까지 {near['edge_m']:.1f} m (2 m 미만이면 주의)"))
        checks.append(dbg.log_check(self.log_panel))
        return checks

    def _debug_sections(self, snapshot):
        meta = snapshot.get("image_meta", {})
        wall = snapshot.get("wall_now")
        sections, columns = {}, []
        for source in ("costmap", "slope"):
            event = meta.get(source, {}) if isinstance(meta.get(source), dict) else {}
            g = valid_grid(event) if event else None
            stamp = event.get("t")
            latency = wall - stamp if dbg.finite(wall) and dbg.finite(stamp) and stamp > 0 else None
            shown = self._map_received.get(source)
            now = snapshot.get("now", time.monotonic())
            shown_age = None if shown is None else max(0., now - shown)
            if g:
                yaw = math.degrees(2. * math.atan2(g["origin"]["qz"], g["origin"]["qw"]))
                values = [f"{g['width']}×{g['height']}", f"{g['resolution']:.3f} m",
                          f"{g['width']*g['resolution']:.1f}×{g['height']*g['resolution']:.1f} m",
                          f"({g['origin']['x']:.2f}, {g['origin']['y']:.2f}) · {yaw:.1f}°", g["frame_id"]]
            else:
                values = ["—"] * 4 + [dbg.plain(event.get("georeference_reason") or "좌표 미확정") if event else "미수신"]
            columns.append(values + [f"{event.get('width', '—')}×{event.get('height', '—')}",
                                     dbg.wall_clock(stamp), dbg.number_text(latency, 2, "초"), dbg.age_text(shown_age)])
        names = ("셀 수", "해상도", "실제 크기", "원점 · 회전", "좌표계", "영상 크기", "보드 생성 시각",
                 "생성→지금 (시계 차 포함)", "표시 지도 나이")
        sections["격자 정보"] = (["항목", "코스트맵", "경사도"], [[n, *v] for n, *v in zip(names, *columns)],
                              "생성→지금은 보드 시각과 이 PC 시각의 차이를 포함합니다.")
        for source, title, unit in (("costmap", "코스트맵 셀 분포", "비용"), ("slope", "경사도 분포", "경사")):
            analysis = self._analyse(snapshot, source)
            rows = []
            if analysis:
                g = analysis["grid"]
                scale = (g["width"] * g["height"]) / max(1, analysis["total"]) if g else None
                for label, count in analysis["rows"]:
                    rows.append([label, f"{count/analysis['total']*100:.1f}%", f"{count:,}",
                                 f"약 {count*scale:,.0f}" if scale else "—"])
            rule = display_legend(meta.get(source, {}), slope=(source == "slope"))
            sections[title + " · 표시 영상 기준"] = (["구간", "비율", "영상 픽셀", "추정 셀 수"], rows,
                f"보드가 보낸 무손실 PNG 색을 보드 색 규칙({rule}, 회색 미확정)으로 되돌려 셉니다. "
                "영상이 원본 격자보다 작으면 가장 가까운 셀만 표본이 되므로 셀 수는 추정입니다.")
        near_rows = []
        for source, title in (("costmap", "코스트맵"), ("slope", "경사도")):
            near = self._value_near_vehicle(snapshot, self._analysis.get(source))
            if near is None:
                near_rows.append([title, "같은 좌표계 위치·지도 필요", "—", "—", "—"])
            elif not near["inside"]:
                near_rows.append([title, "차량이 지도 밖", "—", "—", "—"])
            else:
                fmt = (lambda v: "미확정" if v is None else f"{v*100:.0f}") if source == "costmap" else \
                      (lambda v: "미확정" if v is None else f"{v*self._analysis[source]['slope_max_deg']:.1f}°")
                ratio = near["unknown_ratio"]
                near_rows.append([title, f"셀 ({near['cell'][0]:.0f}, {near['cell'][1]:.0f})", fmt(near["here"]),
                                  fmt(near["max"]), "—" if ratio is None else f"{ratio*100:.0f}%"])
        sections["차량 위치 주변 값 · 반경 1 m"] = (["지도", "차량 셀", "차량 위치 값", "주변 최대", "주변 미확정 비율"], near_rows,
            "가장 최근 공용 위치 기준입니다. 지도와 위치의 수신 시각은 다를 수 있습니다. 색상 상한 이상의 경사는 상한값으로 표시됩니다. 상한은 해당 영상의 범례를 따릅니다.")
        sections["지도 관련 프로세스 · 보드 부하 계측"] = (dbg.PROCESS_HEADERS,
            dbg.module_processes(snapshot, ("mapping", "slope", "stereo_cloud", "lidar")), dbg.PROCESS_NOTE)
        return sections

    def _update_view(self, snapshot):
        update_module_logs(self, snapshot, "traversability")
        status = snapshot.get("statuses", {}).get("perception", {})
        self.status.setText("주행가능성: " + str(status.get("state", "미수신")) + " | " + age_text(age(snapshot, "status:perception")))
        self.status.setToolTip(str(status.get("detail", "")))
        pose_age = age(snapshot, "pose:public")
        history = snapshot.get("histories", {}).get("public", ())
        latest = history[-1] if history else {}
        self.position.setText("공용 위치: " + str(latest.get("frame_id", "—")) + " | " + age_text(pose_age, 1.))
        self.heading.setText("화살표 전방축: " + str(snapshot.get("heading_note", "—")))
        for source, panel in self.panels.items():
            image = snapshot.get("images", {}).get(source)
            event = snapshot.get("image_meta", {}).get(source, {})
            grid = valid_grid(event)
            image_age = age(snapshot, "image:" + source)
            if isinstance(image, QtGui.QImage) and not image.isNull() and grid and not event.get("decode_error"):
                key = (image.cacheKey(), event.get("t"), snapshot.get("received", {}).get("image:" + source))
                if self._image_keys.get(source) != key:
                    panel.set_image(image, event)
                    self._image_keys[source] = key
                    self._map_received[source] = snapshot.get("received", {}).get("image:" + source)
            shown_grid = valid_grid(panel.event)
            frame = shown_grid.get("frame_id") if shown_grid else None
            matching = history if frame and latest.get("frame_id") == frame and self.show_pose.isChecked() else ()
            pose_key = (history_signature(matching), pose_age is not None and pose_age <= 1., frame)
            if self._pose_keys.get(source) != pose_key:
                panel.set_pose(matching, pose_age)
                self._pose_keys[source] = pose_key
            forward_axis = snapshot.get("forward_axis", (1., 0., 0.))
            if panel.forward_axis != forward_axis:
                panel.forward_axis = forward_axis
                panel.dirty = True
            panel.redraw()
            info = (f"{shown_grid['width']}×{shown_grid['height']} · {shown_grid['resolution']:.2f}m/cell · {frame}"
                    if shown_grid else "격자 좌표 미수신")
            shown_received = self._map_received.get(source)
            shown_age = None if shown_received is None else max(0., snapshot.get("now", time.monotonic()) - shown_received)
            timing = "표시 지도 " + age_text(shown_age)
            if event.get("decode_error"):
                timing += " | 새 영상 오류 · 마지막 정상 영상 유지"
            elif event and not grid:
                timing += " | 새 영상 좌표 미확정 · 마지막 지도 유지 (" + age_text(image_age) + ")"
            position = ("위치 표시 끔" if not self.show_pose.isChecked() else
                        "위치 좌표계 일치" if matching else "동일 좌표계 위치 미수신")
            panel.meta.setText(timing + " | " + info + " | " + position)
