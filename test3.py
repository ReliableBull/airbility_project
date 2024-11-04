#!/usr/bin/env python3

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
fixed_camera_pitch = 40.0

def pixel_to_camera_coords(cx, cy, K):
    fx = K[0, 0]
    fy = K[1, 1]
    cx0 = K[0, 2]
    cy0 = K[1, 2]

    x = (cx - cx0) / fx
    y = (cy - cy0) / fy

    return np.array([x, y, 1.0])

def calculate_world_direction(yaw, pitch):
    """
    Yaw 및 피치 각도를 사용하여 카메라의 월드 좌표계 방향을 계산합니다.
    """
    yaw_rad = np.deg2rad(yaw)
    pitch_rad = np.deg2rad(pitch)

    # Yaw와 Pitch를 사용하여 월드 좌표계에서의 방향 벡터를 계산
    direction_x = np.cos(pitch_rad) * np.cos(yaw_rad)
    direction_y = np.cos(pitch_rad) * np.sin(yaw_rad)
    direction_z = np.sin(pitch_rad)

    return np.array([direction_x, direction_y, direction_z])

def world_to_gps_coords(world_coords, drone_gps, drone_altitude):
    earth_radius = 6378137.0  # 지구 반경 (미터)
    
    dlat = world_coords[1] / earth_radius
    dlon = world_coords[0] / (earth_radius * np.cos(np.pi * drone_gps[0] / 180))

    gps_lat = drone_gps[0] + (dlat * 180 / np.pi)
    gps_lon = drone_gps[1] + (dlon * 180 / np.pi)
    gps_alt = drone_altitude + world_coords[2]

    return gps_lat, gps_lon, gps_alt

def calculate_gps_from_image_center(K, P, drone_gps, drone_altitude, yaw):
    image_center = (K[0, 2], K[1, 2])  # (cx, cy)
    camera_coords = pixel_to_camera_coords(image_center[0], image_center[1], K)
    
    # Yaw 및 고정된 피치 각도를 사용하여 월드 방향을 계산
    world_direction = calculate_world_direction(yaw, fixed_camera_pitch)
    
    # 카메라 좌표에서 월드 좌표로 변환
    world_coords = camera_coords * world_direction
    
    # 월드 좌표를 GPS 좌표로 변환
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