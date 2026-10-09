"""회전교차로 끼어들기 몬테카를로: 충돌 지점(idx 1830)으로 왼쪽에서 NPC 흐름이 무작위 간격으로 진입.
자차는 Frenet(속도 상한) + 단순 PP. 지표: 충돌률, 평균/최대 대기, 최소 여유."""
import math, sys, numpy as np
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..', '..', 'scripts'))
from aisw_common import load_map_fields
from planner.frenet_planner import FrenetPlanner, RefPath, HALF_W, FRONT, REAR
rx,ry=[np.array(a) for a in load_map_fields()[:2]]
def P(i):
    j=min(i+1,len(rx)-1); yaw=math.atan2(ry[j]-ry[i],rx[j]-rx[i]); return rx[i],ry[i],yaw
def episode(seed, gap_lo=2.0, gap_hi=8.0, vmin=4.0, vmax=7.0, ego_v=4.2):
    r=np.random.default_rng(seed)
    cx,cy,cyaw=P(1830); nx,ny=-math.sin(cyaw),math.cos(cyaw)   # NPC 는 경로 왼쪽→오른쪽으로 가로지름
    npcs=[]; t0=r.uniform(0,4)
    for k in range(8):
        v=r.uniform(vmin,vmax); npcs.append((t0,v)); t0+=r.uniform(gap_lo,gap_hi)
    ref=RefPath(rx,ry); pl=FrenetPlanner(ref)
    x,y,yaw=P(1760); v=ego_v; sa=0; t=0; dt=0.1; minclear=1e9; wait=0; i=1760
    while t<60:
        objs=[]
        for (ts,vv) in npcs:
            s=-30+vv*(t-ts)          # 충돌점 기준 진행거리 (-30 m 에서 출발)
            if -30<=s<=30: objs.append((cx+nx*(-s),cy+ny*(-s),-nx*vv,-ny*vv,2.3,1.0,True))
        res=pl.plan(x,y,yaw,v,objs)
        px,py=res['x'],res['y']; ld=max(4.0,0.6*v+3); k=int(min(len(px)-1,ld/0.5))
        a=(math.atan2(py[k]-y,px[k]-x)-yaw+math.pi)%(2*math.pi)-math.pi
        st=max(-0.69,min(0.69,math.atan2(6*math.sin(a),ld))); sa+=(st-sa)*min(1,dt/0.2)
        vt=min(ego_v,res['speed_cap']); v=max(0,v+max(-4,min(2,(vt-v)*1.2))*dt)
        if v<0.5: wait+=dt
        x+=v*math.cos(yaw)*dt; y+=v*math.sin(yaw)*dt; yaw+=v/3*math.tan(sa)*dt; t+=dt
        i=int(np.argmin((rx[i-10:i+60]-x)**2+(ry[i-10:i+60]-y)**2))+i-10
        for o in objs:
            ox,oy=o[0],o[1]; lx=(ox-x)*math.cos(yaw)+(oy-y)*math.sin(yaw); ly=-(ox-x)*math.sin(yaw)+(oy-y)*math.cos(yaw)
            dx=max(-REAR-lx-1.0,0,lx-FRONT-1.0); dy=max(abs(ly)-HALF_W-1.0,0)   # NPC 2x4.6 근사(반폭 1.0, 앞뒤 여유 1 m 근사)
            minclear=min(minclear, math.hypot(dx,dy) if (dx>0 or dy>0) else -1)
        if i>=1865: break
    return minclear, wait, t, i>=1865
res=[episode(s) for s in range(200)]
mc=np.array([r[0] for r in res]); w=np.array([r[1] for r in res]); done=np.array([r[3] for r in res])
print('에피소드 200: 충돌 %d (%.1f%%), 근접(<0.5 m) %d, 완료 %d, 대기 평균 %.1fs 최대 %.1fs, 통과시간 평균 %.1fs'%((mc<0).sum(),100*(mc<0).mean(),((mc>=0)&(mc<0.5)).sum(),done.sum(),w.mean(),w.max(),np.mean([r[2] for r in res])))
