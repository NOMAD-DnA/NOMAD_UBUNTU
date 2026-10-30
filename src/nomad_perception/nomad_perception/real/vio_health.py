"""Hand-carried mapping diagnostics. Bounds are engineering guards, not accuracy claims."""
import numpy as np
from scipy.spatial.transform import Rotation


class PoseHealth:
    def __init__(self, max_speed_m_s=10., consecutive=5):
        self.max_speed=max_speed_m_s;self.consecutive=consecutive
        self.last=None;self.speed=None;self.fast_count=0;self.error=None

    def observe(self,row):
        if self.last is not None:
            dt=(int(row[0])-int(self.last[0]))/1e9
            if dt>0:
                self.speed=float(np.linalg.norm(np.array(row[1:4])-self.last[1:4])/dt)
                self.fast_count=self.fast_count+1 if self.speed>self.max_speed else 0
                if self.fast_count>=self.consecutive:
                    self.error=f'VIO 위치 추정 발산: 추정 속도 {self.speed:.1f} m/s. 모노 카메라가 주변 고정 물체를 보는지 확인하고 프로그램을 다시 실행하세요.'
        self.last=row


def stationary_metrics(rows,window_ns=2_000_000_000):
    if len(rows)<2:return {'ready':False,'translation_m':None,'rotation_deg':None,'span_s':0.,'samples':len(rows)}
    recent=[r for r in rows if r[0]>=rows[-1][0]-window_ns]
    xyz=np.array([r[1:4]for r in recent]);rot=Rotation.from_quat([r[4:8]for r in recent])
    spread=float(np.linalg.norm(xyz-xyz[-1],axis=1).max())
    turn=float(np.rad2deg((rot[-1].inv()*rot).magnitude()).max())
    span=(recent[-1][0]-recent[0][0])/1e9
    return {'ready':len(recent)>=10 and span>=1.8,'translation_m':spread,'rotation_deg':turn,'span_s':span,'samples':len(recent)}
