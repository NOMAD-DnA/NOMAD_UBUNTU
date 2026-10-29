"""ROS-free comparison policy; failed runs never become fast successes."""
import math
from statistics import median

ALGORITHMS = {
    'hybrid_astar+rpp': '1. Hybrid A* + RPP',
    'hybrid_astar+rollout': '2. Hybrid A* + Ackermann Rollout',
    'state_lattice+rpp': '3. State Lattice + RPP',
    'dstar_lite+rollout': '4. D* Lite + Ackermann Rollout',
    'astar+rollout': '5. A* + Ackermann Rollout',
}
HEADERS = ['알고리즘','기록 수','목표 도착','기대 충족','실패·중단',
           '여유 p05\n중앙값(m)','최소 여유(m)','주행시간\n중앙값(sim s)',
           'GPP CPU\n중앙값(core·s)','LPP CPU\n중앙값(core·s)','LPP 연산 p95\n중앙값(ms)','지표 표본 수']


def comparison(records):
    rows=[]
    for key,label in ALGORITHMS.items():
        runs=[d for d in records if d.get('algorithm')==key]
        accepted=[d for d in runs if d.get('expected_result_met') and d.get('cleanup_ok') is True]
        def values(field, cpu=False):
            result=[(d.get('cpu') or {}).get(field) if cpu else d.get(field) for d in accepted]
            return [v for v in result if isinstance(v,(int,float)) and not isinstance(v,bool) and math.isfinite(v)]
        def med(field,cpu=False):
            v=values(field,cpu);return median(v) if v else None
        clear=values('clearance_min_m')
        latencies=[d.get('strategy_timing',{}).get('lpp_strategy',{}).get('p95_ms') for d in accepted]
        latencies=[v for v in latencies if isinstance(v,(int,float)) and math.isfinite(v)]
        rows.append([label,len(runs),sum(bool(d.get('completed')) for d in runs),len(accepted),
                     len(runs)-len(accepted),med('clearance_p05_m'),min(clear) if clear else None,
                     med('elapsed_sim_s'),med('gpp',True),med('lpp',True),median(latencies) if latencies else None,len(accepted)])
    return rows
