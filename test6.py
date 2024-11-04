#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
import numpy as np

# 드론의 나침반 Yaw 각도 (방위각)을 저장하는 전역 변수
yaw = 0.0

# 카메라의 고정된 피치 각도 (예: 40도)
fixed_camera_pitch = 40.0

# 지역 지면 고도 (해발 고도)
local_ground_altitude = 100.0  # 예를 들어 100m로 가정

def calculate_relative_altitude(drone_altitude, ground_altitude):
    """
    해수면 고도를 기준으로 한 드론의 고도에서 지면 고도를 빼서 상대 고도를 계산합니다.
    """
    return drone_altitude - ground_altitude

def calculate_ground_distance(relative_altitude, pitch_angle):
    """
    드론의 상대 고도와 카메라 피치 각도를 사용하여 지면까지의 거리를 계산합니다.
    """
    pitch_rad = np.deg2rad(pitch_angle)
    ground_distance = relative_altitude / np.tan(pitch_rad)
    return ground_distance

def calculate_world_coords(yaw, ground_distance):
    """
    Yaw 및 지면 거리 정보를 사용하여 ENU 좌표계를 기준으로 월드 좌표를 계산합니다.
    """
    yaw_rad = np.deg2rad(yaw)

    # ENU 좌표계에서의 월드 좌표계 변환
    world_x = ground_distance * np.cos(yaw_rad)  # 동쪽(East) 방향
    world_y = ground_distance * np.sin(yaw_rad)  # 북쪽(North) 방향

    return np.array([world_x, world_y])

def world_to_gps_coords(world_coords, drone_gps):
    """
    ENU 좌표계를 GPS 좌표로 변환합니다.
    """
    earth_radius = 6378137.0  # 지구 반경 (미터)
    
    dlat = world_coords[1] / earth_radius
    dlon = world_coords[0] / (earth_radius * np.cos(np.pi * drone_gps[0] / 180))

    gps_lat = drone_gps[0] + (dlat * 180 / np.pi)
    gps_lon = drone_gps[1] + (dlon * 180 / np.pi)

    return gps_lat, gps_lon, drone_gps[2]  # 고도는 해수면 고도값 사용

def calculate_gps_from_image_center(drone_gps, drone_altitude, yaw, fixed_camera_pitch):
    # 지면으로부터 드론의 상대 고도 계산
    relative_altitude = calculate_relative_altitude(drone_altitude, local_ground_altitude)

    # 지면까지의 거리 계산
    ground_distance = calculate_ground_distance(relative_altitude, fixed_camera_pitch)
    
    # 월드 좌표계 (ENU 좌표계) 계산
    world_coords = calculate_world_coords(yaw, ground_distance)
    
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
    gps_coords = calculate_gps_from_image_center(drone_gps, drone_gps[2], yaw, fixed_camera_pitch)
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
