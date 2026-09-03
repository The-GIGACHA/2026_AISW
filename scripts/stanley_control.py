#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import numpy as np
import math
import json
import bisect
from tf.transformations import euler_from_quaternion
from pyproj import Proj
from sensor_msgs.msg import Imu
from morai_msgs.msg import CtrlCmd, EgoVehicleStatus, GPSMessage, EventInfo, CollisionData
from morai_msgs.srv import MoraiEventCmdSrv
from collections import deque
from nav_msgs.msg import Path
# RVIZ
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


class Parameter:
    # 아이오닉5 모델 스펙
    vehicle_wheelbase = 3.000       # 차량 휠베이스 [m]
    vehicle_length = 4.635          # 차량 전체 길이 [m]
    vehicle_width = 1.892           # 차량 폭 [m]
    vehicle_height = 2.434          # 차량 전체 높이 [m]
    vehicle_front_overhang = 0.845  # 앞바퀴부터 앞 범퍼까지 거리 [m]
    vehicle_rear_overhang = 0.79    # 뒷바퀴부터 뒤 범퍼까지 거리 [m]
    vehicle_min_radius = 5.87       # 차량 최소회전반경 [m]

    max_wheel_angle = 40.0          # 차량 최대조향각 [deg]
    max_velocity = 46.5 / 3.6       # 최대 속도 [kph] : 단 throttle제어이므로 어느정도 여유둬야 함

    default_target_index = 30       # 0.2m 간격이므로 30개는 6m 전방

    # 추종할 맵 json파일 경로
    INPUT_JSONS = [
         '/home/taegang/catkin_ws/src/2025_HL_MORAI_FINAL-ROUND/map/sangam_bonseon_hdmap.json'
    ]
    # 초기 맵 설정
    INITIAL_MAP_INDEX = 0

    road_width = 5.0                # 도로 너비 폭

    curve_range_start = 20           # 곡률판단 시작부 (ego ind 기준) :  1m
    curve_range_end = 40            # 곡률판단 종료부 (ego ind 기준) :  8m
    curve_filter = 5                # 이동평균필터 반영 개수
    mu = 0.5                       # 마찰계수

class PATH:
    def __init__(self, cx, cy, cyaw, ck, cv, cmission, cgear):
        self.cx = cx
        self.cy = cy
        self.cyaw = cyaw
        self.ck = ck
        self.cv = [v / 3.6 for v in cv]
        self.cmission = cmission
        self.cgear = cgear
        self.length = len(cx)

class MissionState:
    mission_0 = "NORMAL_DRIVING"
    mission_1 = "E_STOP"
    mission_2 = "LANE_DETECTION_BASED_DRIVING"
    mission_3 = "PERPENDICULAR_PARKING"
    mission_4 = "PARALLEL_PARKING"
    mission_5 = "STATIC_OBSTACLE"
    mission_6 = "DYNAMIC_OBSTACLE"

class Stanley_Control:
    def __init__(self, path):
        self.wb = Parameter.vehicle_wheelbase
        self.k = 2.5  # Stanley controller gain (cross track error gain)
        self.path = path
        
        self.ego_gear = 4
        self.ego_pose = np.array([0.0, 0.0, 0.0])
        self.front_axle_pose = np.array([0.0, 0.0, 0.0])  # 전륜 중심 좌표
        self.ego_ind = 0
        self.target_pose = np.array([0.0, 0.0, 0.0])
        self.target_ind = 0
        self.target_velocity = 0.0
        self.cross_track_error = 0.0
        self.heading_error = 0.0
        self.prev_heading_error = 0.0

    def normalize_180(self, deg):
        """Normalize angle to be within [-180, 180) degrees."""
        angle = (deg + 180) % 360 - 180
        return angle
    
    def angle_diff(self, target, current):
        # 두 각도의 차이를 [-pi, pi]로 정규화
        return math.atan2(math.sin(target - current), math.cos(target - current))
    
    
    def update_path(self, path):
        self.path = path
    
    def rear_to_front_axle(self, rear_x, rear_y, yaw_deg):
        """후륜 중심 좌표를 전륜 중심 좌표로 변환"""
        yaw_rad = math.radians(yaw_deg)
        front_x = rear_x + self.wb * math.cos(yaw_rad)
        front_y = rear_y + self.wb * math.sin(yaw_rad)
        return front_x, front_y
    
    def calculate_cross_track_error(self, ego_x, ego_y, path_x1, path_y1, path_x2, path_y2):
        """Calculate cross track error using point to line distance"""
        # 경로 선분의 벡터
        path_vector = np.array([path_x2 - path_x1, path_y2 - path_y1])
        path_length = np.linalg.norm(path_vector)
        
        if path_length < 1e-6:  # 거의 0인 경우
            return math.sqrt((ego_x - path_x1)**2 + (ego_y - path_y1)**2)
        
        # 정규화된 경로 방향 벡터
        path_unit = path_vector / path_length
        
        # 차량에서 경로 시작점까지의 벡터
        ego_to_path = np.array([ego_x - path_x1, ego_y - path_y1])
        
        # 경로에 수직인 거리 (cross track error)
        cross_track = np.cross(ego_to_path, path_unit)
        
        return cross_track

    def run(self, ego_x, ego_y, ego_yaw, ego_gear, ego_ind, ego_vel, curvedvelocity):
        print("Stanley Control Running")
        if self.path is not None:
            self.ego_gear = ego_gear
            self.ego_ind = ego_ind
            self.target_ind = self.ego_ind + Parameter.default_target_index
            if self.target_ind >= self.path.length:
                self.target_ind = self.path.length - 1

            # 후륜 중심 좌표를 전륜 중심 좌표로 변환
            front_x, front_y = self.rear_to_front_axle(ego_x, ego_y, ego_yaw)
            
            self.ego_pose = np.array([ego_x, ego_y, math.radians(ego_yaw)])
            self.front_axle_pose = np.array([front_x, front_y, math.radians(ego_yaw)])
            self.target_pose = np.array([self.path.cx[self.target_ind], self.path.cy[self.target_ind], math.radians(self.path.cyaw[self.target_ind])])
            self.target_velocity = curvedvelocity
            
            # 후진의 경우
            if self.ego_gear == 2:
                self.target_velocity *= -1

            # 속도에 따른 게인 조정
            if abs(self.target_velocity) > 35.0:
                self.k = 1.5
            elif abs(self.target_velocity) > 20.0:
                self.k = 2.7
            elif abs(self.target_velocity) > 15.0:
                self.k = 2.9
            elif abs(self.target_velocity) > 10.0:
                self.k = 3.0
            else:
                self.k = 3.5

            # Cross track error 계산 (전륜 위치와 가장 가까운 경로점 사용)
            current_ind = self.ego_ind
            next_ind = min(current_ind + 1, self.path.length - 1)
            
            self.cross_track_error = self.calculate_cross_track_error(
                front_x, front_y,
                self.path.cx[current_ind], self.path.cy[current_ind],
                self.path.cx[next_ind], self.path.cy[next_ind]
            )

            # Heading error 계산 (lookahead 거리를 늘려서 안정화)
            lookahead_distance = 3  # 3점 앞을 봄
            lookahead_ind = min(current_ind + lookahead_distance, self.path.length - 1)
            
            path_heading = math.atan2(
                self.path.cy[lookahead_ind] - self.path.cy[current_ind],
                self.path.cx[lookahead_ind] - self.path.cx[current_ind]
            )
            heading_error_raw = self.angle_diff(path_heading, math.radians(ego_yaw))
            
            # 헤딩 오차 필터링 (급격한 변화 억제)
            alpha = 0.5  # 더 부드럽게 필터링
            self.heading_error = alpha * heading_error_raw + (1 - alpha) * self.prev_heading_error
            self.prev_heading_error = self.heading_error

            # Stanley controller 공식
            # δ = φ + arctan(k * e / v)
            # φ: heading error, e: cross track error, v: velocity, k: gain
            
            velocity_for_stanley = max(abs(self.target_velocity), 0.1)  # 0으로 나누는 것 방지
            cross_track_term = math.atan2(self.k * self.cross_track_error, velocity_for_stanley)
            
            steering_rad = self.heading_error + cross_track_term
            steering_deg = self.normalize_180(math.degrees(steering_rad))
            
            # 조향각 제한
            steering_deg = np.clip(steering_deg, -Parameter.max_wheel_angle, Parameter.max_wheel_angle)
            
            print(f"Front Axle: ({front_x:.2f}, {front_y:.2f}), Cross track error: {self.cross_track_error:.3f}, Heading error: {math.degrees(self.heading_error):.3f}, Steering: {steering_deg:.3f}")
            
            return np.clip(abs(self.target_velocity), 0.2, Parameter.max_velocity), steering_deg

# Throttle제어기
class AccelCmd_Converter:
    def __init__(self, rate_hz):
        self.p_gain = 0.35
        self.i_gain = 0.06
        self.d_gain = 0.03
        self.prev_error = 0
        self.i_control = 0
        self.controlTime = 1 / rate_hz
        self.output = 0.0

    def run(self, target_vel, current_vel, ego_gear):
        error = target_vel - current_vel
        p_control = self.p_gain * error
        if error <= 5 and abs(self.output) < 1.0:
            self.i_control += self.i_gain * error * self.controlTime
        d_control = self.d_gain * (error-self.prev_error) / self.controlTime
        self.output = p_control + self.i_control + d_control
        self.prev_error = error

        if self.output > 0:
            accel_cmd = self.output
            brake_cmd = 0.0
        else:
            accel_cmd = 0.0
            brake_cmd = -self.output
        return accel_cmd, brake_cmd
    
class CurvedBased_Velocity:
    def __init__(self, path):
        # 큐를 사용하여 최근 곡률값 몇 개만 저장
        self.curve_history = deque(maxlen=Parameter.curve_filter)   # 이동평균필터
        self.ego_index = 0
        self.path = path

    def update_path(self, path):
        self.path = path

    def run(self, ego_ind):
        self.ego_index = ego_ind
        x_list = []
        y_list = []
        for box in range(Parameter.curve_range_start, Parameter.curve_range_end):
            idx = self.ego_index + box
            if idx < 0 or idx >= self.path.length:
                continue  # 경로 범위를 벗어나는 인덱스는 건너뜀
            x = self.path.cx[idx]
            y = self.path.cy[idx]
            x_list.append([-2*x, -2*y ,1])
            y_list.append((-x*x) - (y*y))
            
        x_matrix = np.array(x_list)
        y_matrix = np.array(y_list)
        x_trans = x_matrix.T
        try:
            a_matrix = np.linalg.inv(x_trans @ (x_matrix)) @ x_trans @ y_matrix
            a = a_matrix[0]
            b = a_matrix[1]
            c = a_matrix[2]
            r = math.sqrt(abs(a*a+b*b-c))
        except np.linalg.LinAlgError:
            r = 0.01
        
        self.curve_history.append(r)

        #이동평균필터
        if len(self.curve_history) >= 2:
            smoothed_r = sum(self.curve_history) / len(self.curve_history)
        else:
            smoothed_r = r
            
        v_max = math.sqrt(smoothed_r * 9.81 * Parameter.mu)

        if v_max > self.path.cv[self.ego_index]:
            v_max = self.path.cv[self.ego_index]

        return float(v_max)

class Stanley_Control_Node:
    def __init__(self):
        rospy.init_node('stanley_control_node', anonymous = True)

        rospy.Subscriber("/Competition_topic",EgoVehicleStatus, self.comp_callback)
        rospy.Subscriber("/gps", GPSMessage, self.gps_callback)
        rospy.Subscriber("/imu", Imu, self.imu_callback)
        rospy.Subscriber("/CollisionData", CollisionData, self.coll_callback)
        self.ctrl_cmd_pub = rospy.Publisher('/ctrl_cmd',CtrlCmd, queue_size=1)
        self.ctrl_cmd_msg=CtrlCmd()
        self.ctrl_cmd_msg.longlCmdType=1

        # ServiceProxy를 한 번만 생성
        rospy.wait_for_service('/Service_MoraiEventCmd')
        self._event_cmd = rospy.ServiceProxy('/Service_MoraiEventCmd', MoraiEventCmdSrv)

        self.proj_UTM = Proj(proj='utm', zone=52, ellps='WGS84', preserve_units=False)

        self.rate_hz = 30   # 메인루프 주기 설정

        # 초기 맵 설정
        self.initial_map_index = Parameter.INITIAL_MAP_INDEX
        # 모든 맵을 미리 불러와 저장
        self.all_paths = [ self.load_ref_map(p) for p in Parameter.INPUT_JSONS ]
        # 맵 인덱스 clamp & 현재 ref_path 설정
        self.map_index = max(0, min(self.initial_map_index, len(self.all_paths)-1))
        self.ref_path  = self.all_paths[self.map_index]

        # ================= RViz용 Publisher 추가 =================
        self.map_marker_pub = rospy.Publisher("/ref_path_marker", MarkerArray, queue_size=1)
    
        self.ego_x = 0.0
        self.ego_y = 0.0
        self.ego_yaw = 0.0
        self.ego_gear = 4
        self.ego_vel = 0.0
        self.ego_index = 0

        self.velocity = 0.0
        self.steering = 0.0
        self.curvedvelocity = 0.0

        self.gps_fix = 0
        self.odom_flag = False
        self.gps_flag = False
        self.imu_flag = False
        self.coll_flag = False

        # 후진 관련 변수
        self.adjusted_yaw = 0.0
        self.prev_gear = self.ref_path.cgear[0]

        self.ego_mission = MissionState.mission_0

        # 사용할 클래스 객체 초기화
        self.cmd_converter = AccelCmd_Converter(self.rate_hz)
        self.stanley_controller = Stanley_Control(self.ref_path)
        self.curvebased_vel = CurvedBased_Velocity(self.ref_path)

    ########################################## SENSOR CALLBACK ##########################################

    def comp_callback(self, msg):
        self.ego_vel = msg.velocity.x   # 상대좌표계 기준 : 전진속도 / 후진시 음수로 출력됨 [m/s]
        self.odom_flag = True

    def gps_callback(self, msg):
        """
        차량 후륜중심 (x:0.0, y:0.0, z:1.2) / 30Hz
        """
        # UTM기반 ENU좌표계 (WGS84->UTM->ENU)
        latitude = msg.latitude
        longitude = msg.longitude
        utm_x, utm_y = self.proj_UTM(longitude, latitude)
        self.ego_x = utm_x - msg.eastOffset
        self.ego_y = utm_y - msg.northOffset
        self.gps_fix = msg.status           # 0 : no fix / 1 : 2D fix / 2 : 3D fix / 3 : RTK fix / 4 : RTK Float

        self.gps_flag = True

    def imu_callback(self, msg):
        """
        차량 후륜중심 (x:0.0, y:0.0, z:1.3) / 30Hz
        """
        orientation_q = msg.orientation
        quaternion = (
            orientation_q.x,
            orientation_q.y,
            orientation_q.z,
            orientation_q.w
        )
        roll, pitch, yaw = euler_from_quaternion(quaternion)
        self.ego_yaw = math.degrees(yaw)

        self.imu_flag = True

    def coll_callback(self, msg):
        self.coll_flag = hasattr(msg, "collision_object") and len(msg.collision_object) > 0

    #####################################################################################################

    # 기어 변경
    def change_gear(self, gear_value):
        try:
            event_msg = EventInfo()
            event_msg.option = 2  # gear 변경
            event_msg.gear = gear_value
            self._event_cmd(event_msg)
        
            gear_str = {1: 'P', 2: 'R', 3: 'N', 4: 'D'}.get(gear_value, str(gear_value))
            print(f"Gear changed to {gear_str}")
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")

    def change_to_parking(self):
        self.change_gear(1)  # P

    def change_to_drive(self):
        self.change_gear(4)  # D

    def change_to_reverse(self):
        self.change_gear(2)  # R
    
    # 컨트롤 모드 변경
    def change_ctrl_mode(self, mode_value):
        """
        mode_value:
         1 Keyboard
         2 GameWheel
         3 automode
         4 cruisemode
        """
        try:
            evt = EventInfo()
            evt.option    = 1        # ctrl_mode 변경 옵션
            evt.ctrl_mode = mode_value
            self._event_cmd(evt)
            print(f"Control mode changed to {mode_value}")
        except rospy.ServiceException as e:
            print(f"Ctrl mode change failed: {e}")

    # json파일 불러와 ref 정보 PATH객체로 저장
    def load_ref_map(self, json_file):
        with open(json_file, 'r') as f:
                data = json.load(f)

        keys = sorted(data.keys(), key=lambda k: int(k))

        rx      = [data[k]['x']         for k in keys]
        ry      = [data[k]['y']         for k in keys]
        ryaw    = [data[k]['yaw']       for k in keys]
        rk      = [data[k]['curvature'] for k in keys]
        rvel    = [data[k]['velocity']  for k in keys]
        rm      = [data[k]['mission']   for k in keys]
        rgear   = [data[k]['gear']      for k in keys]

        self.ref_path = PATH(rx, ry, ryaw, rk, rvel, rm, rgear)

        return self.ref_path

    # 현재 위치의 경로에서의 인덱스 검색
    def nearest_index(self, path, ego_x, ego_y):
        dx = [ego_x - x for x in path.cx]
        dy = [ego_y - y for y in path.cy]
        dist = np.hypot(dx, dy)

        ind = int(np.argmin(dist))

        return ind

    def normalize_180(self, deg):
        """Normalize angle to be within [-180, 180) degrees."""
        angle = (deg + 180) % 360 - 180
        return angle
    
    # RViz Marker 생성 함수
    def publish_map_marker(self, path: PATH):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "ref_path"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # 굵기/색/가시성
        marker.scale.x = 1.5           # 선 두께 [m]
        marker.color.r = 0.0           # 파랑으로 변경 (Stanley 구분용)
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.z = 0.1   # z-fighting 방지

        for x, y in zip(path.cx, path.cy):
            p = Point(x=x, y=y, z=0.0)
            marker.points.append(p)

        marker_array.markers.append(marker)
        self.map_marker_pub.publish(marker_array)
    
    def main(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            if not (self.odom_flag and self.gps_flag and self.imu_flag):
                rate.sleep()
                continue

            # ref_path 시각화
            self.publish_map_marker(self.ref_path)

            self.ego_index = self.nearest_index(self.ref_path, self.ego_x, self.ego_y)
            self.curvedvelocity = self.curvebased_vel.run(self.ego_index)

            # 맵 끝에 다다르면 다음 맵으로 전환 : 마지막 인덱스 2개 이내
            if self.ego_index >= self.ref_path.length - 2 and self.map_index < len(self.all_paths) - 1:
                self.map_index += 1
                self.ref_path = self.all_paths[self.map_index]
                print(f">>> Switch to map #{self.map_index+1}")
                # 컨트롤러 맵 업데이트
                self.stanley_controller.update_path(self.ref_path)
                # 인덱스 및 기어 다시 계산
                self.ego_index = self.nearest_index(self.ref_path, self.ego_x, self.ego_y)
                self.prev_gear = self.ref_path.cgear[self.ego_index]

            # 기존/변경된 맵 기준 미션과 기어 업데이트 
            self.ego_mission = self.ref_path.cmission[self.ego_index]
            self.ego_gear = self.ref_path.cgear[self.ego_index]

            # 중간에 맵상에서 기어가 바뀌는 시점 감지
            if self.ego_gear != self.prev_gear:
                self.ctrl_cmd_msg.accel = 0.0
                self.ctrl_cmd_msg.brake = 1.0
                self.ctrl_cmd_msg.steering = 0.0
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                # 속도 멈출 때까지 기다림
                if abs(self.ego_vel) < 0.2:
                    if self.ego_gear == 2:
                        self.change_to_reverse()
                    elif self.ego_gear == 4:
                        self.change_to_drive()
                    elif self.ego_gear == 1:
                        self.change_to_parking()
                    self.prev_gear = self.ego_gear
                continue

            self.adjusted_yaw = self.ego_yaw
            
            # 미션별 적용
            if self.ego_mission == MissionState.mission_0:
                self.velocity, self.steering = self.stanley_controller.run(self.ego_x, self.ego_y, self.adjusted_yaw, self.ego_gear, self.ego_index, self.ego_vel, self.curvedvelocity)
            elif self.ego_mission == MissionState.mission_1:
                self.ctrl_cmd_msg.accel = 0.0
                self.ctrl_cmd_msg.brake = 1.0
                self.ctrl_cmd_msg.steering = 0.0
                self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)
                continue
            elif self.ego_mission == MissionState.mission_2:
                pass
            elif self.ego_mission == MissionState.mission_3:
                pass
            elif self.ego_mission == MissionState.mission_4:
                pass
            elif self.ego_mission == MissionState.mission_5:
                pass
            elif self.ego_mission == MissionState.mission_6:
                pass

            accel_cmd, brake_cmd = self.cmd_converter.run(self.velocity, self.ego_vel, self.ego_gear)

            self.ctrl_cmd_msg.accel = accel_cmd
            self.ctrl_cmd_msg.brake = brake_cmd
            self.ctrl_cmd_msg.steering = math.radians(self.normalize_180(self.steering))
            self.ctrl_cmd_pub.publish(self.ctrl_cmd_msg)

            print("-------------------------------------")
            print(f" Map       = #{self.map_index+1}")
            print(f" Index     = {self.ego_index}")
            print(f" Mission   = {self.ego_mission}")
            print(f" Collision = {self.coll_flag}")
            print(f" Gear      = { {1: 'P', 2: 'R', 3: 'N', 4: 'D'}.get(self.ego_gear, str(self.ego_gear)) }")
            print(f" Velocity  = {self.ego_vel:.3f}")
            print(f" Accel (%) = {100 if self.ctrl_cmd_msg.accel * 100 >= 100 else self.ctrl_cmd_msg.accel * 100:.3f}")
            print(f" Brake (%) = {100 if self.ctrl_cmd_msg.brake * 100 >= 100 else self.ctrl_cmd_msg.brake * 100:.3f}")
            print(f" Steering  = {self.steering:.3f}")
            print("-------------------------------------\n")

            rate.sleep()


if __name__ == "__main__":
    stanley = Stanley_Control_Node()
    stanley.main()