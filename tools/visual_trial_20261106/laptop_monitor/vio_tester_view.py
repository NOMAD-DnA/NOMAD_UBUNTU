"""2026-11-17 VIO 시험기(vio_tester.py)의 데이터 쪽: 관전 중계 수신·녹화 bag 읽기·화면 상태.
보기 전용. 차량 명령·SSH·보드 구독을 만들지 않는다(중계는 운영 GUI의 relay.RelayClient를 그대로 사용)."""
import argparse, json, math, sys, threading, time, collections
import numpy as np
FWD = np.array([-0.00453401, -0.02442952, -0.99969127])  # 카메라 앞(OAK IMU 축)
BLUE, GREEN, ORANGE, GRAY, RED, INK = "#2a78d6", "#1baf7a", "#eb6834", "#b9b8b3", "#e34948", "#0b0b0b"
FIX_NAMES = {-1: "위치 없음", 0: "단독", 1: "SBAS", 2: "RTK"}

class State:
    def __init__(s):
        s.lock = threading.Lock(); s.reset()
        s.conn = "대기"
    def reset(s):
        s.fus = collections.deque(maxlen=20000); s.gnss = collections.deque(maxlen=5000)
        s.vals = {}; s.mode = "?"; s.gtel = {}; s.hist = collections.deque(maxlen=5000); s.t0 = None; s.yaw = None; s.last_gt = None
    def pose(s, t, x, y, q):
        with s.lock:
            if s.t0 is None: s.t0 = t
            s.fus.append((t, x, y))
            x, y, z, w = q
            m = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                          [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)]])
            f = m @ FWD; s.yaw = math.atan2(f[1], f[0])
    def fusion(s, t, mode, vals):
        with s.lock:
            if s.t0 is None: s.t0 = t
            s.mode = mode or "?"; s.vals = vals
            try:
                gt = float(vals.get("viewer_wxy_gnss_t", "nan"))
                if math.isfinite(gt) and gt != s.last_gt:
                    s.last_gt = gt
                    s.gnss.append((gt, float(vals["viewer_wxy_gnss_out_x"]), float(vals["viewer_wxy_gnss_out_y"]),
                                   float(vals.get("viewer_wxy_gnss_sigma_m", "nan")), vals.get("viewer_wxy_aligned") == "true"))
            except (KeyError, ValueError): pass
            def f(k):
                try: return float(vals.get(k, "nan"))
                except ValueError: return float("nan")
            s.hist.append((t, f("viewer_wxy_gnss_sigma_m"), f("viewer_wxy_fit_sigma_yaw_deg")))
    def gnss_tel(s, ev):
        with s.lock: s.gtel = ev

def relay_thread(st, host, port):
    from relay import RelayClient  # 같은 폴더의 읽기 전용 관전 클라이언트
    def on_packet(board, ev):
        k, src = ev.get("kind"), ev.get("source")
        if k == "pose" and src == "fusion":
            st.pose(ev["t"], ev["x"], ev["y"], [ev["qx"], ev["qy"], ev["qz"], ev["qw"]])
        elif k == "telemetry" and src == "fusion" and ev.get("received"):
            st.fusion(ev.get("t") or time.time(), ev.get("mode"), ev.get("values") or {})
        elif k == "telemetry" and src == "gnss":
            st.gnss_tel(ev)
    def on_progress(p): st.conn = f"{p.get('state')} {p.get('host')}:{p.get('port')}" + (f" ({p['error']})" if p.get("error") else "")
    RelayClient(host, port, on_packet, on_progress).run()

def bag_thread(st, bag, speed):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    r = rosbag2_py.SequentialReader(); r.open(rosbag2_py.StorageOptions(uri=bag, storage_id=""), rosbag2_py.ConverterOptions("cdr", "cdr"))
    ty = {t.name: t.type for t in r.get_all_topics_and_types()}
    r.set_filter(rosbag2_py.StorageFilter(topics=[t for t in ("/fusion/odometry", "/fusion/status", "/gps/fix", "/gps/quality") if t in ty]))
    st.conn = f"녹화 재생 ×{speed:g}: {bag}"
    wall0 = time.monotonic(); b0 = None; fix = {}
    while r.has_next():
        tp, d, tr = r.read_next(); m = deserialize_message(d, get_message(ty[tp])); tb = tr * 1e-9
        if b0 is None: b0 = tb
        wait = (tb - b0) / speed - (time.monotonic() - wall0)
        if wait > 0: time.sleep(wait)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        if tp == "/fusion/odometry":
            p, q = m.pose.pose.position, m.pose.pose.orientation; st.pose(t, p.x, p.y, [q.x, q.y, q.z, q.w])
        elif tp == "/fusion/status":
            vals = {v.key: v.value for s in m.status if s.name == "fusion" for v in s.values}; st.fusion(t, vals.get("mode"), vals)
        elif tp == "/gps/fix":
            fix.update(fix_type=m.status.status, h_acc_m=math.sqrt(m.position_covariance[0]) if m.position_covariance[0] > 0 else None); st.gnss_tel(dict(fix))
        elif tp == "/gps/quality":
            fix.update(satellites=getattr(m, "satellites_used", None), pdop=getattr(m, "pdop", None)); st.gnss_tel(dict(fix))
    st.conn += " — 끝"

