import math, sys, numpy as np
sys.path.insert(0, __import__('os').path.join(__import__('os').path.dirname(__import__('os').path.abspath(__file__)), '..', '..', 'scripts'))
from aisw_common import load_map_fields
from planner.frenet_planner import FrenetPlanner, RefPath, HALF_W, FRONT, REAR
rx,ry=[np.array(a) for a in load_map_fields()[:2]]
def lat_off(x,y):
    i=int(np.argmin((rx-x)**2+(ry-y)**2)); j=min(i+1,len(rx)-1); yaw=math.atan2(ry[j]-ry[i],rx[j]-rx[i])
    return i,-math.sin(yaw)*(x-rx[i])+math.cos(yaw)*(y-ry[i])
def run(i0,i1,objs_fn,v_target=8.3,T=90):
    ref=RefPath(rx,ry); pl=FrenetPlanner(ref)
    yaw=math.atan2(ry[i0+1]-ry[i0],rx[i0+1]-rx[i0]); x,y=rx[i0],ry[i0]; v=v_target; sa=0; t=0; dt=0.1
    maxoff=0; minclear=1e9; minv=1e9; steer_hist=[]; i=i0
    while t<T:
        objs=objs_fn(t)
        res=pl.plan(x,y,yaw,v,[(o[0],o[1],o[2],o[3],o[4],o[5]) for o in objs])
        px,py=res['x'],res['y']
        ld=max(4.0,0.6*v+3); k=int(min(len(px)-1,ld/0.5))
        a=(math.atan2(py[k]-y,px[k]-x)-yaw+math.pi)%(2*math.pi)-math.pi
        st=max(-0.69,min(0.69,math.atan2(6*math.sin(a),ld)))
        st=sa+max(-0.13,min(0.13,st-sa))   # 75°/s
        vt=min(v_target,res['speed_cap'])
        v=max(0,v+max(-4,min(2,(vt-v)*1.2))*dt)
        sa+= (st-sa)*min(1,dt/0.2); steer_hist.append(st)
        x+=v*math.cos(yaw)*dt; y+=v*math.sin(yaw)*dt; yaw+=v/3*math.tan(sa)*dt; t+=dt
        i,off=lat_off(x,y); maxoff=max(maxoff,abs(off)); minv=min(minv,v)
        for o in objs:
            cx=(o[0]-x)*math.cos(yaw)+(o[1]-y)*math.sin(yaw); cy=-(o[0]-x)*math.sin(yaw)+(o[1]-y)*math.cos(yaw)
            dx=max(-REAR-cx,0,cx-FRONT-o[4]); dy=max(abs(cy)-HALF_W-o[5],0); minclear=min(minclear,math.hypot(dx,dy) if (dx>0 or dy>0) else -1)
        if i>=i1: break
    sr=np.abs(np.diff(steer_hist))/dt
    return dict(t=round(t,1),reached=i>=i1,max_off=round(maxoff,2),end_off=round(abs(off),2),min_clear=round(minclear,2) if minclear<1e8 else None,min_v=round(minv,1),steer_rate95=round(math.degrees(np.percentile(sr,95)),1))
def P(i,dd=0.0):
    j=min(i+1,len(rx)-1); yaw=math.atan2(ry[j]-ry[i],rx[j]-rx[i]); return rx[i]-dd*math.sin(yaw), ry[i]+dd*math.cos(yaw), yaw
print('1 clean S-curve      ', run(100,330,lambda t:[]))
ox,oy,_=P(661,0.79)
print('2 static obstacle     ', run(560,780,lambda t:[(ox,oy,0,0,1.5,1.0)]))
lx,ly,lyaw=P(450,0.0)
print('3 slow lead car 3 m/s ', run(380,620,lambda t:[(lx+3*t*math.cos(lyaw),ly+3*t*math.sin(lyaw),3*math.cos(lyaw),3*math.sin(lyaw),2.3,1.0)],T=60))
cx,cy,cyaw=P(1830,0.0); nx,ny=-math.sin(cyaw),math.cos(cyaw)
print('4 crossing NPC 5 m/s  ', run(1740,1880,lambda t:[(cx+nx*(-25+5*t),cy+ny*(-25+5*t),5*nx,5*ny,2.3,1.0)]))
