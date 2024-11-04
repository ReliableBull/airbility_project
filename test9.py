#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
import numpy as np
from geopy.distance import geodesic

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

# 타겟의 실제 GPS 좌표 (위도, 경도)
target_lat = 47.397739
target_lon = 8.5455746

def pixel_to_camera_coords(cx, cy, K):
    fx = K[0, 0]
    fy = K[1, 1]
    cx0 = K[0, 2]
    cy0 = K[1, 2]

    x = (cx - cx0) / fx
    y = (cy - cy0) / fy
    # print("x :" , x , "  y: ", y)
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

    print("center : " ,camera_coords)
    return np.dot(R_pitch, camera_coords)

def camera_to_world_coords(camera_coords, R, T):
    world_coords = np.dot(R, camera_coords) + T
    return world_coords

def world_to_gps_coords(world_coords, drone_gps):
    earth_radius = 6378137.0  # 지구 반경 (미터)
    
    dlat = world_coords[1] / earth_radius
    dlon = world_coords[0] / (earth_radius * np.cos(np.pi * drone_gps[0] / 180))

    gps_lat = drone_gps[0] + (dlat * 180 / np.pi)
    gps_lon = drone_gps[1] + (dlon * 180 / np.pi)

    return gps_lat, gps_lon, drone_gps[2]  # 고도는 그대로 유지

def calculate_gps_from_image_center(K, P, drone_gps, pitch_angle):
    image_center = (320, 240)  # (cx, cy)
    camera_coords = pixel_to_camera_coords(image_center[0], image_center[1], K)
    
    # 피치 각도를 고려한 회전 적용
    camera_coords = apply_pitch_rotation(camera_coords, pitch_angle)
    
    # 회전 행렬 (단위 행렬로 초기화)
    R = np.eye(3)
    
    T = np.array([P[0, 3], P[1, 3], P[2, 3]])  # Translation vector
    world_coords = camera_to_world_coords(camera_coords, R, T)
    gps_coords = world_to_gps_coords(world_coords, drone_gps)
    return gps_coords

def calculate_distance_to_target(calculated_gps, target_gps):
    """
    계산된 GPS 좌표와 실제 타겟 GPS 좌표 간의 거리 차이를 계산합니다.
    """
    calculated_point = (calculated_gps[0], calculated_gps[1])
    target_point = target_gps
    return geodesic(calculated_point, target_point).meters

def gps_callback(data):
    # 드론의 현재 GPS 좌표
    drone_gps = (data.latitude, data.longitude, data.altitude)

    # 피치 각도가 20도인 경우
    pitch_angle = 20.0

    # 이미지 중심 픽셀 좌표의 GPS 좌표 계산
    gps_coords = calculate_gps_from_image_center(K, P, drone_gps, pitch_angle)

    # 타겟과의 거리 계산
    distance_to_target = calculate_distance_to_target(gps_coords, (target_lat, target_lon))

    rospy.loginfo("Image Center GPS Coordinates: {}, Target Distance: {:.2f} meters".format(gps_coords, distance_to_target))

def gps_calculator_node():
    rospy.init_node('gps_calculator_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_calculator_node()
    except rospy.ROSInterruptException:
        pass
