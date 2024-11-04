#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import numpy as np

# 카메라 매트릭스 파라미터 (내적 매트릭스)
K = np.array([
    [205.46963709898583, 0.0, 320.5],
    [0.0, 205.46963709898583, 240.5],
    [0.0, 0.0, 1.0]
])

# 투영 행렬
P = np.array([
    [205.46963709898583, 0.0, 320.5, -14.382874596929009],
    [0.0, 205.46963709898583, 240.5, 0.0],
    [0.0, 0.0, 1.0, 0.0]
])

# 드론의 나침반 Yaw 각도 (방위각)을 저장하는 전역 변수
yaw = 0.0

# 카메라의 고정된 피치 각도 (40도)
fixed_camera_pitch = -40.0

def pixel_to_camera_coords(cx, cy, K):
    fx = K[0, 0]
    fy = K[1, 1]
    cx0 = K[0, 2]
    cy0 = K[1, 2]

    x = (cx - cx0) / fx
    y = (cy - cy0) / fy

    return np.array([x, y, 1.0])

def get_rotation_matrix(yaw, fixed_pitch):
    """
    Yaw 각도 및 고정된 카메라 피치 각도에 대한 회전 행렬을 계산합니다.
    """
    # 각도를 라디안으로 변환
    pitch_rad = np.deg2rad(fixed_pitch)  # 카메라의 고정된 피치 각도
    yaw_rad = np.deg2rad(yaw)  # 드론의 Yaw (콤파스) 각도
    
    # 회전 행렬 계산
    R_pitch = np.array([
        [np.cos(pitch_rad), 0, np.sin(pitch_rad)],
        [0, 1, 0],
        [-np.sin(pitch_rad), 0, np.cos(pitch_rad)]
    ])
    
    R_yaw = np.array([
        [np.cos(yaw_rad), -np.sin(yaw_rad), 0],
        [np.sin(yaw_rad), np.cos(yaw_rad), 0],
        [0, 0, 1]
    ])
    
    # 총 회전 행렬은 R_yaw * R_pitch 순서로 곱합니다.
    R = np.dot(R_yaw, R_pitch)
    
    return R

def camera_to_world_coords(camera_coords, R, T):
    world_coords = np.dot(R, camera_coords) + T
    return world_coords

def world_to_gps_coords(world_coords, drone_gps, drone_altitude):
    earth_radius = 6378137.0  # 지구 반경 (미터)
    
    dlat = world_coords[1] / earth_radius
    dlon = world_coords[0] / (earth_radius * np.cos(np.pi * drone_gps[0] / 180))

    gps_lat = drone_gps[0] + (dlat * 180 / np.pi)
    gps_lon = drone_gps[1] + (dlon * 180 / np.pi)
    gps_alt = drone_gps[2] + drone_altitude

    return gps_lat, gps_lon, gps_alt

def calculate_gps_from_image_center(K, P, drone_gps, drone_altitude, yaw):
    image_center = (K[0, 2], K[1, 2])  # (cx, cy)
    camera_coords = pixel_to_camera_coords(image_center[0], image_center[1], K)
    
    # 드론의 Yaw 각도 및 고정된 카메라 피치 각도를 반영한 회전 행렬 생성
    R = get_rotation_matrix(yaw, fixed_camera_pitch)
    
    T = np.array([P[0, 3], P[1, 3], P[2, 3]])  # Translation vector
    world_coords = camera_to_world_coords(camera_coords, R, T)
    gps_coords = world_to_gps_coords(world_coords, drone_gps, drone_altitude)
    return gps_coords

def compass_callback(data):
    global yaw
    yaw = data.data  # 콤파스에서 Yaw (방위각) 값을 가져옴

def gps_callback(data):
    # 드론의 현재 GPS 좌표
    drone_gps = (data.latitude, data.longitude, data.altitude)
    drone_altitude = data.altitude  # 드론 고도

    # 이미지 중심 픽셀 좌표의 GPS 좌표 계산
    gps_coords = calculate_gps_from_image_center(K, P, drone_gps, drone_altitude, yaw)
    rospy.loginfo("Image Center GPS Coordinates with yaw {}: {}".format(yaw, gps_coords))

def gps_calculator_node():
    rospy.init_node('gps_calculator_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, compass_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_calculator_node()
    except rospy.ROSInterruptException:
        pass
