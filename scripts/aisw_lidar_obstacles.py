#!/home/inji2/.local/rospython/python3
# -*- coding: utf-8 -*-
"""
[2026_AISW] VLP16 UDP → 간이 클러스터링 → /tracked_objects_3d (Detection3DArray)
- 대회 허용 센서(LiDAR UDP 2368)만 사용. velodyne 드라이버 불필요(패킷 직접 파싱).
- lattice_planner_v2.obs_callback 기대 포맷에 맞춤:
    bbox.center/size = ego(후륜축) 기준 로컬 좌표, results[0].id = 트랙 id,
    source_cloud.data[0:4] = 상대속도 float (미상 → 0.0 = 정적 취급)
파라미터: ~lidar_port(2368) ~min_pts(5) ~grid(0.5m) ~z_lo/-1.4 ~z_hi/0.8 ~max_range(30)
          ~lidar_x(1.5: 후륜축→라이다 전방 오프셋)
"""
import socket, struct, math
import numpy as np
import rospy
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from sensor_msgs.msg import PointCloud2

VERT = np.deg2rad(np.array([-15,1,-13,3,-11,5,-9,7,-7,9,-5,11,-3,13,-1,15], dtype=np.float32))

def main():
    rospy.init_node('aisw_lidar_obstacles')
    gp = lambda n,d: rospy.get_param('~'+n, d)
    port=int(gp('lidar_port',2368)); MINP=int(gp('min_pts',5)); GRID=float(gp('grid',0.5))
    ZLO=float(gp('z_lo',-1.4)); ZHI=float(gp('z_hi',0.8)); RMAX=float(gp('max_range',30.0))
    LX=float(gp('lidar_x',1.5))
    pub=rospy.Publisher('/tracked_objects_3d', Detection3DArray, queue_size=1)
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
            m=(r>0.5)&(r<RMAX)&(z>ZLO)&(z<ZHI)
            if m.any(): sweep.append(np.stack([x[m],y[m],z[m]],1))
        if azi[0]<last_azi:  # 방위각 랩 = 한 바퀴 완료
            if sweep:
                pts=np.concatenate(sweep); sweep=[]
                process(pts, pub, GRID, MINP, LX)
            else: sweep=[]
        last_azi=azi[0]

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
        det=Detection3D()
        det.bbox.center.position.x=float((lo[0]+hi[0])/2 + LX)  # 라이다→후륜축 프레임
        det.bbox.center.position.y=float((lo[1]+hi[1])/2)
        det.bbox.center.orientation.w=1.0
        det.bbox.size.x=float(max(hi[0]-lo[0],0.3)); det.bbox.size.y=float(max(hi[1]-lo[1],0.3)); det.bbox.size.z=float(max(hi[2]-lo[2],0.3))
        h=ObjectHypothesisWithPose(); h.id=tid; h.score=1.0; det.results.append(h)
        pc=PointCloud2(); pc.data=struct.pack('f',0.0); det.source_cloud=pc
        msg.detections.append(det); tid+=1
    pub.publish(msg)
    rospy.loginfo_throttle(2.0,'[lidar_obstacles] 장애물 %d개', len(msg.detections))

if __name__=='__main__':
    main()
