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

# 투영 행렬 (P)
P = np.array([
    [205.46963709898583, 0.0, 320.5, -14.382874596929009],
    [0.0, 205.46963709898583, 240.5, 0.0],
    [0.0, 0.0, 1.0, 0.0]
])

# 드론의 나침반 Yaw 각도 (방위각)을 저장하는 전역 변수
yaw = 0.0

# 카메라의 고정된 피치 각도 (40도)
fixed_camera_pitch = 40.0

def calculate_ground_distance(drone_altitude, pitch_angle):
    """
    드론의 고도와 카메라 피치 각도를 사용하여 지면까지의 거리를 계산합니다.
    """
    pitch_rad = np.deg2rad(pitch_angle)
    ground_distance = drone_altitude / np.tan(pitch_rad)
    return ground_distance

def calculate_world_coords(yaw, ground_distance, P):
    """
    Yaw 및 지면 거리 정보를 사용하여 월드 좌표계를 계산합니다.
    """
    yaw_rad = np.deg2rad(yaw)

    # 월드 좌표계에서의 거리 계산
    world_x = ground_distance * np.cos(yaw_rad)
    world_y = ground_distance * np.sin(yaw_rad)

    # 투영 행렬 P에서 변환 벡터를 고려
    T = np.array([P[0, 3], P[1, 3], P[2, 3]])

    # 월드 좌표에 변환 벡터 적용
    world_coords = np.array([world_x, world_y, 0]) + T
    
    return world_coords

def world_to_gps_coords(world_coords, drone_gps):
    earth_radius = 6378137.0  # 지구 반경 (미터)
    
    dlat = world_coords[1] / earth_radius
    dlon = world_coords[0] / (earth_radius * np.cos(np.pi * drone_gps[0] / 180))

    gps_lat = drone_gps[0] + (dlat * 180 / np.pi)
    gps_lon = drone_gps[1] + (dlon * 180 / np.pi)

    return gps_lat, gps_lon, drone_gps[2]  # 고도는 변화가 없다고 가정

def calculate_gps_from_image_center(drone_gps, drone_altitude, yaw, fixed_camera_pitch, P):
    # 지면까지의 거리 계산
    ground_distance = calculate_ground_distance(drone_altitude, fixed_camera_pitch)
    
    # 월드 좌표계 계산
    world_coords = calculate_world_coords(yaw, ground_distance, P)
    
    # 월드 좌표를 GPS 좌표로 변환
    gps_coords = world_to_gps_coords(world_coords, drone_gps)
    return gps_coords

def compass_callback(data):
    global yaw
    yaw = data.data  # 콤파스에서 Yaw (방위각) 값을 가져옴

def gps_callback(data):
    # 드론의 현재 GPS 좌표
    drone_gps = (data.latitude, data.longitude, data.altitude)

    # 이미지 중심 픽셀 좌표의 GPS 좌표 계산
    gps_coords = calculate_gps_from_image_center(drone_gps, drone_gps[2], yaw, fixed_camera_pitch, P)
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
