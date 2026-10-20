"""Offline preference fit on measured cost layers; never supplies a map to live ROS.

Uses the implemented Grid.path_cost quadrature and compose() feature priors.
Reference is a human preference, not shortest-path ground truth. Nonnegative
linear weights, fixed distance coefficient 1. Exact feasibility on the sampled
paths does not imply global optimality or vehicle feasibility.
"""
import argparse,json,math,hashlib
from pathlib import Path
import numpy as np
from scipy.optimize import linprog
from nomad_path_planning.gpp.multicost import compose
from nomad_path_planning.gpp.base import Grid,Request

def feature_vector(grid,slope,terrain,path):
    vectors=[]
    for w in [(0,0,0,0),(1,0,0,0),(0,1,0,0),(0,0,1,0),(0,0,0,1)]:
        g=dict(grid);g['cell_weights']=compose(g,slope,terrain,obstacle_weight=w[0],slope_weight=w[1],terrain_weight=w[2],unknown_surcharge=w[3])
        vectors.append(Grid(Request(g,path[0],path[-1])).path_cost(path))
    if not all(math.isfinite(v) for v in vectors):return None
    return [vectors[0]]+[v-vectors[0] for v in vectors[1:]]

def fit(reference,candidates,margin=.1):
    delta=np.asarray(reference)-np.asarray(candidates)
    result=linprog(np.ones(4),A_ub=delta[:,1:],b_ub=-delta[:,0]-margin,
                   bounds=[(0,None)]*4,method='highs',options={'time_limit':10})
    return result,delta

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--map',required=True);parser.add_argument('--route',required=True)
    parser.add_argument('--candidates',required=True);parser.add_argument('--output',required=True);args=parser.parse_args()
    a=np.load(args.map);g={k:a[k].item() for k in ('width','height','resolution','origin_x','origin_y')};g['data']=a['data'].tolist()
    start=a['start'].tolist();goal=a['goal'].tolist()
    if not np.isfinite(start+goal).all():raise ValueError('Missing start/goal')
    route=np.loadtxt(args.route,delimiter=',',skiprows=1)[:,:2]
    # Authored map4 loop starts at west and proceeds along the southern half.
    end=int(np.argmin(np.linalg.norm(route-np.array(goal),axis=1)))
    lower=route[:end+1]
    if np.max(lower[:,1])>1e-6 or np.min(lower[:,1])>=-1:raise ValueError('Not lower half of loop')
    first=int(np.argmin(np.linalg.norm(lower-np.array(start),axis=1)))
    reference=[start]+lower[first:].tolist()+[goal]
    r=feature_vector(g,a['slope'],a['terrain'],reference)
    rows=json.loads(Path(args.candidates).read_text());features=[]
    for row in rows:
        if row.get('result')!='path':continue
        f=feature_vector(g,a['slope'],a['terrain'],row['path'])
        if f:features.append(dict(condition=row['condition'],features=f))
    report=dict(feature_order=['length','observed_proximity_integral','slope_with_missing_prior_integral','terrain_with_missing_prior_integral','unknown_length'],reference=reference,reference_features=r,candidates=features,
        inputs={p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in [args.map,args.route,args.candidates]},
        assumptions=['Reference is training preference only, never a live goal sequence','Only measured frozen map used; no world geometry cost map','Unknown slope and terrain use existing missing prior1; proximity0 in unknown','Distance coefficient fixed1; other weights constrained nonnegative','Reference connector begins at actual trial start; not Ackermann feasibility validation','Grid path quadrature used, not a proof about all possible paths'])
    if r is None:report.update(result='reference_infeasible',reason='Reference or its start connector intersects blocked/out-of-map cells; weights cannot remove hard constraints')
    elif not features:report.update(result='no_valid_candidates')
    else:
        solved,delta=fit(r,[z['features'] for z in features]);report.update(result='fitted' if solved.success else 'no_nonnegative_fit',solver_status=int(solved.status),solver_message=solved.message)
        report['dominating_candidates']=[z['condition'] for z,d in zip(features,delta) if np.all(d>=-1e-7) and d[0]>.1]
        if solved.success:
            report.update(weights=dict(zip(['obstacle','slope','terrain','unknown'],solved.x.tolist())),margins=(-delta[:,0]-delta[:,1:]@solved.x).tolist())
    output=Path(args.output);output.parent.mkdir(parents=True,exist_ok=True);output.write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps({k:v for k,v in report.items() if k not in ['reference','candidates','inputs','assumptions']},indent=2))
if __name__=='__main__':main()
