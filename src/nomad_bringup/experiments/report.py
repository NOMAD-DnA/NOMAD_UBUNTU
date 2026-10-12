"""Offline evaluation of saved trials; no simulator or planner invocation."""
import csv
import html
import math
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from common import read, save, metric_values


def wilson(success,total):
    if not total:return None,None
    p=success/total;z=1.96;denom=1+z*z/total
    center=(p+z*z/(2*total))/denom
    half=z*math.sqrt(p*(1-p)/total+z*z/(4*total*total))/denom
    return max(0.,center-half),min(1.,center+half)


def evaluate(directory):
    directory=Path(directory);manifest=read(directory/'manifest.json')
    result_dir=directory/'results';result_dir.mkdir(exist_ok=True)
    figures=result_dir/'figures';figures.mkdir(exist_ok=True)
    rows=[];attempts=[]
    numbers=sorted({t['pair']['number'] for t in manifest['trials']})
    maps=list(manifest['references'])
    for trial in manifest['pilots']+manifest['trials']:
        found=sorted((directory/'runs'/trial['id']).glob('attempt_*/outcome.json'))
        for file in found:
            attempts.append(dict(trial=trial['id'],attempt=file.parent.name,**read(file)))
        if not found:continue
        valid_files=[f for f in found if read(f).get('valid') and read(f).get('bag_verified')]
        chosen=valid_files[-1] if valid_files else found[-1]
        outcome=read(chosen)
        reference=manifest['references'].get(trial['map'])
        metrics=metric_values(outcome,reference)
        if all(metrics[k] is not None for k in ('safe_success','spl','sct')):
            metrics['score']=100*sum(manifest['weights'][k]*metrics[v] for k,v in [('sr','safe_success'),('spl','spl'),('sct','sct')])
        rows.append(dict(trial=trial['id'],map=trial['map'],pair=trial['pair']['number'],
                         label=trial['pair']['label'],repeat=trial['repeat'],pilot=trial['pilot'],
                         attempt=str(chosen.parent.relative_to(directory)),**outcome,**metrics))
    columns=['trial','map','pair','label','repeat','pilot','attempt','result','valid','bag_verified','safe_success',
             'distance_xy_m','distance_3d_m','duration_sim_s','duration_wall_s','reverse_distance_m',
             'first_path_latency_wall_s','max_tilt_deg','waypoints_reached','total_waypoints','spl','sct','score']
    with (result_dir/'metrics.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=columns,extrasaction='ignore');writer.writeheader();writer.writerows(rows)
    summary=[]
    main=[r for r in rows if not r['pilot']]
    for number in numbers:
        trials=[r for r in main if r['pair']==number and r.get('valid') and r.get('bag_verified')]
        wins=sum(r['safe_success'] for r in trials)
        sr=wins/len(trials) if trials else None
        lo,hi=wilson(wins,len(trials))
        planned=sum(t['pair']['number']==number for t in manifest['trials'])
        complete=len(trials)==planned
        successful=[r for r in trials if r['safe_success']]
        mean=lambda key: float(np.mean([r[key] for r in successful])) if successful else None
        waypoint_ratios=[r.get('waypoints_reached',0)/r['total_waypoints'] for r in trials if r.get('total_waypoints')]
        scores=[r['score'] for r in trials]
        summary.append(dict(pair=number,valid_runs=len(trials),planned_runs=planned,successes=wins,sr=sr,
                            mean_success_distance_m=mean('distance_xy_m'),mean_success_duration_sim_s=mean('duration_sim_s'),
                            mean_success_reverse_m=mean('reverse_distance_m'),
                            mean_waypoint_completion=float(np.mean(waypoint_ratios)) if waypoint_ratios else None,
                            sr_ci95=[lo,hi],complete=complete,
                            weighted_score=float(np.mean(scores)) if complete and all(s is not None for s in scores) else None))
    save(result_dir/'summary.json',dict(weights=manifest['weights'],summary=summary,attempts=attempts,
                                      references=manifest['references'],scope='Gazebo simulation only'))
    fig,axes=plt.subplots(2,2,figsize=(12,8),layout='constrained')
    image=np.full((len(numbers),len(maps)),np.nan)
    for pi,number in enumerate(numbers):
        for mi,mid in enumerate(maps):
            subset=[r for r in main if r['pair']==number and r['map']==mid and r.get('valid')]
            if subset:image[pi,mi]=np.mean([r['safe_success'] for r in subset])
    cmap=plt.get_cmap('RdYlGn').copy();cmap.set_bad('#eeeeee')
    im=axes[0,0].imshow(np.ma.masked_invalid(image),vmin=0,vmax=1,cmap=cmap,aspect='auto')
    axes[0,0].set(xticks=range(len(maps)),xticklabels=[m[:2] for m in maps],yticks=range(len(numbers)),yticklabels=[f'Pair {i}' for i in numbers],title='Safe arrival rate (valid main trials only)',xlabel='Map')
    for pi in range(len(numbers)):
        for mi in range(len(maps)):axes[0,0].text(mi,pi,'N/A' if np.isnan(image[pi,mi]) else f'{image[pi,mi]:.2f}',ha='center',va='center')
    fig.colorbar(im,ax=axes[0,0])
    for row in summary:
        if row['sr'] is not None:
            lo,hi=row['sr_ci95'];axes[0,1].errorbar(row['pair'],row['sr'],yerr=[[row['sr']-lo],[hi-row['sr']]],fmt='o')
    axes[0,1].set(title='Safe arrival: Wilson 95% intervals',xlabel='Pair',ylabel='Rate',xticks=numbers,ylim=(-.05,1.05))
    for number in numbers:
        subset=[r for r in main if r['pair']==number and r.get('valid')]
        if subset:
            axes[1,0].scatter([r['distance_xy_m'] for r in subset],[r['duration_sim_s'] for r in subset],label=f'Pair {number}',alpha=.8)
    axes[1,0].set(title='Distance / duration (failures included)',xlabel='Actual XY distance [m]',ylabel='Simulation duration [s]')
    if any(r.get('valid') for r in main):axes[1,0].legend()
    reasons={}
    for row in main:reasons[row['result']]=reasons.get(row['result'],0)+1
    if reasons:axes[1,1].barh(list(reasons),list(reasons.values()))
    else:axes[1,1].text(.5,.5,'No main trials yet',ha='center',va='center',transform=axes[1,1].transAxes)
    axes[1,1].set(title='Latest selected trial outcomes',xlabel='Count')
    fig.suptitle(f"{manifest['name']} | simulation only | pilots excluded",fontsize=14)
    fig.savefig(figures/'comparison.png',dpi=160);plt.close(fig)
    fig,ax=plt.subplots(figsize=(9,4),layout='constrained')
    actual=[s for s in summary if s['weighted_score'] is not None]
    if actual:
        ax.bar([s['pair'] for s in actual],[s['weighted_score'] for s in actual]);ax.set(ylim=(0,100),xlabel='Pair',ylabel='Weighted score')
    else:
        ax.axis('off');ax.text(.5,.6,'Composite ranking withheld',ha='center',fontsize=18,transform=ax.transAxes)
        ax.text(.5,.4,'Requires complete trials and independently validated distance/time references.\nWeights: safe arrival 50% / path efficiency 25% / time efficiency 25%',ha='center',transform=ax.transAxes)
    fig.savefig(figures/'weighted_score.png',dpi=160);plt.close(fig)
    fig,axes=plt.subplots(1,len(maps),figsize=(max(6,4*len(maps)),5),layout='constrained',squeeze=False)
    for ax,mid in zip(axes[0],maps):
        if manifest.get('comparison'):
            goals=np.array(manifest['comparison']['goals'])
            ax.plot(goals[:,0],goals[:,1],'k:',alpha=.4,label='Clicked goals (not driven path)')
        for row in [r for r in main if r['map']==mid]:
            trace=directory/row['attempt']/'trajectory.csv'
            if trace.exists():
                with trace.open() as stream:points=[p for p in csv.DictReader(stream) if p['phase']=='trial']
                if points:ax.plot([float(p['x']) for p in points],[float(p['y']) for p in points],label=f"P{row['pair']} R{row['repeat']}",alpha=.7)
        ax.set(title=mid,xlabel='x [m]',ylabel='y [m]');ax.set_aspect('equal',adjustable='datalim')
        if ax.lines:ax.legend(fontsize=6)
    fig.savefig(figures/'trajectories.png',dpi=160);plt.close(fig)
    with (result_dir/'pair_summary.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=list(summary[0]) if summary else ['pair'])
        writer.writeheader();writer.writerows(summary)
    timing_rows=timing_summary(directory,main)
    with (result_dir/'timing_summary.csv').open('w',newline='') as stream:
        fields=['pair','stage','calls','mean_wall_ms','p95_wall_ms','mean_thread_cpu_ms']
        writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(timing_rows)
    def esc(v):return html.escape(str(v))
    pair_table=''.join('<tr>'+''.join(f'<td>{esc(row.get(k))}</td>' for k in ('pair','valid_runs','planned_runs','successes','sr','mean_waypoint_completion','mean_success_distance_m','mean_success_duration_sim_s','weighted_score'))+'</tr>' for row in summary)
    table=''.join('<tr>'+''.join(f'<td>{esc(row.get(k,""))}</td>' for k in ('trial','result','valid','distance_xy_m','duration_sim_s','score'))+'</tr>' for row in rows)
    report=f'''<!doctype html><html lang="ko"><meta charset="utf-8"><title>{esc(manifest['name'])}</title>
<style>body{{font-family:sans-serif;max-width:1100px;margin:40px auto;line-height:1.6}}img{{width:100%}}td,th{{border:1px solid #ccc;padding:6px}}table{{border-collapse:collapse}}.note{{background:#fff4d0;padding:18px}}</style>
<h1>{esc(manifest['name'])}</h1><p>기준 코드: {esc(manifest['commit'])}</p>
<p class="note">시뮬레이션 결과이며 실차 성능을 증명하지 않습니다. 안전 도착률은 금지 접촉·전복 없이 목표 반경 0.35m에서 정지한 비율입니다. 기록 누락·실행 오류는 평가 불가로 분리합니다. 기준 최단거리·최소시간이 검증되기 전 SPL·SCT·종합 순위는 보류합니다. 파일럿은 본실험 집계에서 제외합니다.</p>
<p>본실험 유효 기록: {sum(r.get('valid',False) for r in main)}/{len(manifest['trials'])}. 가중치: 안전 도착률 50%, 경로 효율 25%, 시간 효율 25%.</p>
<h2>조합별 비교</h2><table><tr><th>조합</th><th>유효</th><th>계획</th><th>성공</th><th>SR</th><th>경유점 도달 비율</th><th>성공 평균 거리(m)</th><th>성공 평균 시간(s)</th><th>종합 점수</th></tr>{pair_table}</table><p>계획 계산시간은 timing_summary.csv에 stage별로 저장합니다. 서로 다른 stage의 평균을 전체 CPU 사용률로 해석하지 않습니다.</p>
<img src="figures/comparison.png"><img src="figures/weighted_score.png"><img src="figures/trajectories.png">
<table><tr><th>실행</th><th>결과</th><th>유효</th><th>XY 거리(m)</th><th>시뮬레이션 시간(s)</th><th>종합 점수</th></tr>{table}</table>
<p>실패와 성공을 포함한 거리·시간입니다. 짧은 실패 경로를 효율적인 경로로 해석하지 마세요. 안전 성공 조건을 적용한 SPL/SCT는 도착만 요구하는 문헌 지표와 구분합니다.</p></html>'''
    (result_dir/'report.html').write_text(report)
    (result_dir/'notion_summary.md').write_text(f"# {manifest['name']}\n\n상태: 본실험 유효 기록 {sum(r.get('valid',False) for r in main)}/{len(manifest['trials'])}\n\n기준 코드: `{manifest['commit']}`\n\n시뮬레이션 결과이며 실차 성능의 증명이 아님.\n\n가중치: 안전 도착률 50% / 경로 효율 25% / 시간 효율 25%.\n\nSPL·SCT 및 종합 순위: 독립 기준값 검증 및 전체 유효 기록 확보 전 보류.\n\n"+'\n'.join(f"- 조합 {s['pair']}: 유효 {s['valid_runs']}/{sum(t['pair']['number']==s['pair'] for t in manifest['trials'])}, 안전 도착 {s['successes']}회" for s in summary)+'\n')
    return rows


def timing_summary(directory, rows):
    """Aggregate calls by pair and stage; exclude pilots/invalid data, not failures."""
    grouped={}
    for row in rows:
        if not row.get('valid') or not row.get('bag_verified'):continue
        for file in (directory/row['attempt']/'timing').glob('*.csv'):
            with file.open() as stream:
                for sample in csv.DictReader(stream):
                    key=(row['pair'],sample['stage'])
                    grouped.setdefault(key,[]).append((int(sample['elapsed_wall_ns'])/1e6,
                                                       int(sample['thread_cpu_ns'])/1e6))
    result=[]
    for (pair,stage),samples in sorted(grouped.items()):
        a=np.asarray(samples)
        result.append(dict(pair=pair,stage=stage,calls=len(samples),
                           mean_wall_ms=float(np.mean(a[:,0])),p95_wall_ms=float(np.percentile(a[:,0],95)),
                           mean_thread_cpu_ms=float(np.mean(a[:,1]))))
    return result
