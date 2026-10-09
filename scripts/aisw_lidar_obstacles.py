#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
[2026_AISW] VLP16 UDP → 간이 클러스터링 → /tracked_objects_3d (Detection3DArray)
- 대회 허용 센서(LiDAR UDP 2368)만 사용. velodyne 드라이버 불필요(패킷 직접 파싱).
- lattice_planner_v2.obs_callback 기대 포맷에 맞춤:
    bbox.center/size = ego(후륜축) 기준 로컬 좌표, results[0].id = 트랙 id,
    source_cloud.data[0:4] = 상대속도 float (미상 → 0.0 = 정적 취급)
파라미터: ~lidar_port(2368) ~min_pts(5) ~grid(0.5m) ~z_hi/0.8 ~max_range(30)
          ~lidar_x(1.5: 후륜축→라이다 전방 오프셋)
          ~h_min(0.3) ~h_max(2.5): 추정 지면 위 높이 [m] 범위만 장애물로 사용
- [2026_AISW] 지면 제거: 매 회전 지면 평면을 맞춰 높이로 거른다. 고정 z 임계(라이다 기준 -1.4 m = 지면 위
  0.15 m)는 오르막/둔덕에서 노면을 장애물로 잡아 도심 구간에서 1.5~2.5 m 앞 가짜 장애물이 계속 떴다(2026-10-09).
"""
import socket, struct, math
import numpy as np
import rospy
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from sensor_msgs.msg import PointCloud2, LaserScan

VERT = np.deg2rad(np.array([-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15], dtype=np.float32))

def main():
    rospy.init_node('aisw_lidar_obstacles')
    gp = lambda n,d: rospy.get_param('~'+n, d)
    port=int(gp('lidar_port',2368)); MINP=int(gp('min_pts',5)); GRID=float(gp('grid',0.5))
    ZHI=float(gp('z_hi',0.8)); RMAX=float(gp('max_range',30.0))
    HMIN=float(gp('h_min',0.3)); HMAX=float(gp('h_max',2.5))
    LX=float(gp('lidar_x',1.5))
    pub=rospy.Publisher('/tracked_objects_3d', Detection3DArray, queue_size=1)
    # [2026_AISW] AI 구간(ai_zone_controller) 입력용 2D 거리 스캔: 장애물 높이대 점의 방위별 최소거리
    scan_pub=rospy.Publisher('/aisw/lidar_scan', LaserScan, queue_size=1)
    NBIN=int(gp('scan_bins',72))
    s=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1); s.settimeout(1.0)
    s.bind(('0.0.0.0',port))
    rospy.loginfo('[lidar_obstacles] UDP %d 수신 대기', port)
    sweep=[]; last_azi=-1.0
    while not rospy.is_shutdown():
        try: raw,_=s.recvfrom(2048)
        except socket.timeout: continue
        except OSError: break
        if len(raw)<1200: continue
        b=np.frombuffer(raw[:1200],dtype=np.uint8).reshape(-1,100)
        azi=(b[:,2].astype(np.float32)+256*b[:,3].astype(np.float32))/100.0   # deg, 12블록
        dist=((b[:,4::3].astype(np.float32)+256*b[:,5::3].astype(np.float32))*2/1000.0)  # m, 12x32
        # 2발사 분리(각 16ch), 방위각 보간 +0.2deg
        d0,d1=dist[:,:16],dist[:,16:]
        a0,a1=np.deg2rad(azi),np.deg2rad(azi+0.2)
        for d,a in ((d0,a0),(d1,a1)):
            r=d  # (12,16)
            xy=r*np.cos(VERT)[None,:]
            x=xy*np.cos(a)[:,None]; y=-xy*np.sin(a)[:,None]  # 우회전 방위각 → y 부호 반전(차량 좌+)
            z=r*np.sin(VERT)[None,:]
            m=(r>0.5)&(r<RMAX)&(z<ZHI)   # 지면 포함 (높이 필터는 remove_ground)
            if m.any(): sweep.append(np.stack([x[m],y[m],z[m]],1))
        if azi[0]<last_azi:  # 방위각 랩 = 한 바퀴 완료
            if sweep:
                pts=remove_self(remove_ground(np.concatenate(sweep), HMIN, HMAX), LX); sweep=[]
                publish_scan(pts, scan_pub, NBIN, RMAX)
                process(pts, pub, GRID, MINP, LX)
            else: sweep=[]
        last_azi=azi[0]

LIDAR_H = 1.55   # 평지 기준 라이다 높이 [m] (공식 센서 파일 z=1.55)

def remove_ground(pts, hmin, hmax, cell=2.0, rfit=25.0):
    """지면 평면 z=ax+by+c 를 2 m 칸별 최저점에 맞추고(잔차 큰 칸 2회 제거), 지면 위 높이 [hmin,hmax] 점만 남긴다."""
    if len(pts)==0: return pts
    near=pts[np.hypot(pts[:,0],pts[:,1])<rfit]
    plane=None
    if len(near)>50:
        ij=np.floor(near[:,:2]/cell).astype(np.int64)
        key=ij[:,0]*100000+ij[:,1]
        order=np.lexsort((near[:,2],key)); key=key[order]; nz=near[order]
        first=np.r_[True, key[1:]!=key[:-1]]
        g=nz[first]                      # 칸별 최저점
        g=g[g[:,2]<-LIDAR_H+1.0]          # 지면에서 1 m 이상 뜬 칸(차량 지붕 등)은 후보 제외
        for _ in range(3):
            if len(g)<6: break
            A=np.c_[g[:,0],g[:,1],np.ones(len(g))]
            coef,*_=np.linalg.lstsq(A,g[:,2],rcond=None)
            res=g[:,2]-A@coef
            plane=coef
            g=g[np.abs(res)<0.15]
        if plane is not None and (abs(plane[0])>0.15 or abs(plane[1])>0.15 or abs(plane[2]+LIDAR_H)>0.6):
            plane=None                   # 경사 8.5° 초과/높이 이상 → 잘못 맞춘 것으로 보고 평지 가정
    ground=(plane[0]*pts[:,0]+plane[1]*pts[:,1]+plane[2]) if plane is not None else -LIDAR_H
    h=pts[:,2]-ground
    return pts[(h>hmin)&(h<hmax)]

# 자차 차체 상자 (후륜축 기준): 뒤 오버행 0.79, 휠베이스+앞 오버행 3.845, 반폭 0.946 + 미러/여유
SELF_X = (-1.1, 4.1)
SELF_Y = 1.45

def remove_self(pts, lx):
    """자차 차체에 맞은 점 제거. 2026-10-09 실주행에서 좌측 앞(후륜축 기준 x 2~3.5, y 1.1~1.2)에
    차를 따라다니는 '장애물'이 계속 잡혀 플래너가 서행/정지했다."""
    xr=pts[:,0]+lx
    inside=(xr>SELF_X[0])&(xr<SELF_X[1])&(np.abs(pts[:,1])<SELF_Y)
    return pts[~inside]

def publish_scan(pts, pub, nbin, rmax):
    """라이다 프레임 기준 방위 nbin 칸(-180°~+180°, 좌+)별 최소 수평거리. 빈 칸 = rmax."""
    r=np.hypot(pts[:,0],pts[:,1])
    b=((np.arctan2(pts[:,1],pts[:,0])+math.pi)/(2*math.pi)*nbin).astype(np.int64)%nbin
    ranges=np.full(nbin, rmax, dtype=np.float32)
    np.minimum.at(ranges, b, r.astype(np.float32))
    m=LaserScan(); m.header.stamp=rospy.Time.now(); m.header.frame_id='lidar'
    m.angle_min=-math.pi; m.angle_increment=2*math.pi/nbin; m.angle_max=math.pi-m.angle_increment
    m.range_min=0.5; m.range_max=float(rmax); m.ranges=ranges.tolist()
    pub.publish(m)

def process(pts, pub, GRID, MINP, LX):
    # 2D 그리드 연결요소 클러스터링
    ij=np.floor(pts[:,:2]/GRID).astype(np.int64)
    keys=ij[:,0]*100000+ij[:,1]
    order=np.argsort(keys); keys=keys[order]; pts=pts[order]
    cells={}
    uk,idx=np.unique(keys,return_index=True)
    for n,k in enumerate(uk):
        cells[k]=(idx[n], idx[n+1] if n+1<len(uk) else len(keys))
    seen=set(); msg=Detection3DArray(); msg.header.stamp=rospy.Time.now(); msg.header.frame_id='ego'
    tid=0
    for k in uk:
        if k in seen: continue
        stack=[k]; comp=[]
        while stack:
            c=stack.pop()
            if c in seen or c not in cells: continue
            seen.add(c); comp.append(c)
            ci,cj=divmod(c,100000) if c>=0 else (c//100000, c%100000)
            for di in (-1,0,1):
                for dj in (-1,0,1):
                    nb=(ci+di)*100000+(cj+dj)
                    if nb in cells and nb not in seen: stack.append(nb)
        sel=np.concatenate([pts[cells[c][0]:cells[c][1]] for c in comp])
        if len(sel)<MINP: continue
        lo=sel.min(0); hi=sel.max(0)
        if (hi[0]-lo[0])>8 or (hi[1]-lo[1])>8: continue  # 벽/가드레일 제외
        cx0=float((lo[0]+hi[0])/2 + LX); cy0=float((lo[1]+hi[1])/2)
        # [2026_AISW] 경로 근처만: 전방 0.5~25m, 측면 |y|<4m (도로변 가로등/신호등/표지판 오탐 제거)
        if not (0.5 < cx0 < 25.0 and abs(cy0) < 4.0):
            continue
        det=Detection3D()
        det.bbox.center.position.x=cx0  # 라이다→후륜축 프레임
        det.bbox.center.position.y=cy0
        det.bbox.center.orientation.w=1.0
        det.bbox.size.x=float(max(hi[0]-lo[0],0.3)); det.bbox.size.y=float(max(hi[1]-lo[1],0.3)); det.bbox.size.z=float(max(hi[2]-lo[2],0.3))
        h=ObjectHypothesisWithPose(); h.id=tid; h.score=1.0; det.results.append(h)
        pc=PointCloud2(); pc.data=struct.pack('f',0.0); det.source_cloud=pc
        msg.detections.append(det); tid+=1
    msg.detections=drop_boundary_lines(msg.detections)
    pub.publish(msg)
    rospy.loginfo_throttle(2.0,'[lidar_obstacles] 장애물 %d개', len(msg.detections))

def drop_boundary_lines(dets, dy=0.5, min_n=3, min_span=5.0, max_w=0.8):
    """도로 경계석/가드레일 제거: 폭이 좁은(<max_w) 조각들이 같은 옆 거리(±dy)에 min_n개 이상,
    전후 min_span m 이상 늘어서 있으면 경계선으로 보고 뺀다. 성긴 VLP16 점 때문에 경계석이
    8 m 벽 필터를 피해 0.3 m 조각들로 잡혀 플래너가 서행했다(2026-10-09, 인덱스 1600 부근)."""
    thin=[d for d in dets if d.bbox.size.y<max_w]
    drop=set()
    for a in thin:
        ya=a.bbox.center.position.y
        line=[b for b in thin if abs(b.bbox.center.position.y-ya)<dy]
        xs=[b.bbox.center.position.x for b in line]
        if len(line)>=min_n and max(xs)-min(xs)>=min_span:
            drop.update(id(b) for b in line)
    return [d for d in dets if id(d) not in drop]

if __name__=='__main__':
    main()
