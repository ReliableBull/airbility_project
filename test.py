#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
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

def pixel_to_camera_coords(cx, cy, K):
    fx = K[0, 0]
    fy = K[1, 1]
    cx0 = K[0, 2]
    cy0 = K[1, 2]

    x = (cx - cx0) / fx
    y = (cy - cy0) / fy

    return np.array([x, y, 1.0])

def apply_pitch_rotation(camera_coords, pitch_angle):
    """
    주어진 피치 각도(라디안)를 사용하여 카메라 좌표계를 회전시킵니다.
    """
    pitch_rad = np.deg2rad(pitch_angle)
    R_pitch = np.array([
        [1, 0, 0],
        [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
        [0, np.sin(pitch_rad), np.cos(pitch_rad)]
    ])
    return np.dot(R_pitch, camera_coords)

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

def calculate_gps_from_image_center(K, P, drone_gps, drone_altitude, pitch_angle):
    image_center = (K[0, 2], K[1, 2])  # (cx, cy)
    camera_coords = pixel_to_camera_coords(image_center[0], image_center[1], K)
    
    # 피치 각도를 고려한 회전 적용
    camera_coords = apply_pitch_rotation(camera_coords, pitch_angle)
    
    # 회전 행렬 (단위 행렬로 초기화)
    R = np.eye(3)
    
    T = np.array([P[0, 3], P[1, 3], P[2, 3]])  # Translation vector
    world_coords = camera_to_world_coords(camera_coords, R, T)
    gps_coords = world_to_gps_coords(world_coords, drone_gps, drone_altitude)
    return gps_coords

def gps_callback(data):
    # 드론의 현재 GPS 좌표
    drone_gps = (data.latitude, data.longitude, data.altitude)
    drone_altitude = data.altitude  # 드론 고도

    # 짐벌 카메라의 피치 각도를 여기에 정의하거나 ROS에서 받아옵니다.
    # 예시로 10도로 설정
    pitch_angle = 40.0

    # 이미지 중심 픽셀 좌표의 GPS 좌표 계산
    gps_coords = calculate_gps_from_image_center(K, P, drone_gps, drone_altitude, pitch_angle)
    rospy.loginfo("Image Center GPS Coordinates with pitch {}: {}".format(pitch_angle, gps_coords))

def gps_calculator_node():
    rospy.init_node('gps_calculator_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_calculator_node()
    except rospy.ROSInterruptException:
        pass
