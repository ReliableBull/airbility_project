#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist, PoseStamped , Vector3, Point
from std_msgs.msg import Float64
import numpy as np
from geopy.distance import geodesic

# 드론의 나침반 Yaw 각도 (방위각)을 저장하는 전역 변수
yaw = 0.0

# 카메라의 고정된 피치 각도 (예: 40도)
fixed_camera_pitch = 20.0

# 지역 지면 고도 (해발 고도)
local_ground_altitude = 535.0

# 타겟의 실제 GPS 좌표 (위도, 경도)
target_lat = 47.3977416
target_lon = 8.545659

# 카메라 매트릭스 파라미터 (내적 매트릭스)
K = np.array([
    [205.46963709898583, 0.0, 320.5],
    [0.0, 205.46963709898583, 240.5],
    [0.0, 0.0, 1.0]
])

gimbal_angle = Point()
gimbal_angle.z = -1


def pixel_to_camera_coords(cx, cy, K):
    """
    이미지 좌표를 카메라 좌표로 변환합니다.
    """
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

def calculate_ground_distance(relative_altitude, pitch_angle):
    global gimbal_angle
    """
    드론의 상대 고도와 카메라 피치 각도를 사용하여 지면까지의 거리를 계산합니다.
    """
    current_gimbal_pitch = gimbal_angle.y
    pitch_rad = np.deg2rad(90 - abs(current_gimbal_pitch))
    print(current_gimbal_pitch)
    # pitch_rad = np.deg2rad(90 - pitch_angle)
    ground_distance = relative_altitude * np.tan(pitch_rad)
    return ground_distance

def calculate_world_coords(yaw, ground_distance):
    global gimbal_angle
    """
    Yaw 및 지면 거리 정보를 사용하여 ENU 좌표계를 기준으로 월드 좌표를 계산합니다.
    """
    current_gimbal_yaw = gimbal_angle.x
    yaw_rad = np.deg2rad(yaw + current_gimbal_yaw)

    # ENU 좌표계에서의 월드 좌표계 변환
    world_x = ground_distance * np.sin(yaw_rad)  # 동쪽(East) 방향
    world_y = ground_distance * np.cos(yaw_rad)  # 북쪽(North) 방향

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

def calculate_gps_from_image_center(K, drone_gps, drone_altitude, yaw, fixed_camera_pitch):
    # 이미지 센터 픽셀 좌표 (가정)
    # image_center = (K[0, 2], K[1, 2])  # (cx, cy)
    image_center = (320, 240)  # (cx, cy)
    
    # 이미지 좌표를 카메라 좌표로 변환
    camera_coords = pixel_to_camera_coords(image_center[0], image_center[1], K)
    
    # 피치 각도를 고려한 회전 적용
    camera_coords = apply_pitch_rotation(camera_coords, fixed_camera_pitch)
    
    # 지면으로부터 드론의 상대 고도 계산
    relative_altitude = calculate_relative_altitude(drone_altitude, local_ground_altitude)
    
    # 지면까지의 거리 계산
    ground_distance = calculate_ground_distance(relative_altitude, fixed_camera_pitch)
    
    # 월드 좌표계 (ENU 좌표계) 계산
    world_coords = calculate_world_coords(yaw, ground_distance)
    
    # 월드 좌표를 GPS 좌표로 변환
    gps_coords = world_to_gps_coords(world_coords, drone_gps)
    return gps_coords

def calculate_relative_altitude(drone_altitude, ground_altitude):
    """
    해수면 고도를 기준으로 한 드론의 고도에서 지면 고도를 빼서 상대 고도를 계산합니다.
    """
    return drone_altitude - ground_altitude

def calculate_distance_to_target(calculated_gps, target_gps):
    """
    계산된 GPS 좌표와 실제 타겟 GPS 좌표 간의 거리 차이를 계산합니다.
    """
    calculated_point = (calculated_gps[0], calculated_gps[1])
    # print("!@#!@#!", target_gps)
    target_point = target_gps
    return geodesic(calculated_point, target_point).meters

def compass_callback(data):
    global yaw
    yaw = data.data  # 콤파스에서 Yaw (방위각) 값을 가져옴

def gps_callback(data):
    # 드론의 현재 GPS 좌표
    drone_gps = (data.latitude, data.longitude, data.altitude)

    # 이미지 중심 픽셀 좌표의 GPS 좌표 계산
    calculated_gps_coords = calculate_gps_from_image_center(K, drone_gps, drone_gps[2], yaw, fixed_camera_pitch)

    # 타겟과의 거리 계산
    distance_to_target = calculate_distance_to_target(calculated_gps_coords, (target_lat, target_lon))

    rospy.loginfo("Calculated GPS Coordinates: {}, Target Distance: {:.2f} meters".format(calculated_gps_coords, distance_to_target))

    # gimbal callback 
def callBackGimbalAngle(data):
    global gimbal_angle
        # x : yaw
        # y : pitch
    gimbal_angle = data
    print(gimbal_angle)


def gps_calculator_node():
    rospy.init_node('gps_calculator_node', anonymous=True)
    rospy.Subscriber("/mavros/global_position/global", NavSatFix, gps_callback)
    rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, compass_callback)
    rospy.Subscriber("/angle_of_gimbal", Point, callBackGimbalAngle)
    rospy.spin()

if __name__ == '__main__':
    try:
        gps_calculator_node()
    except rospy.ROSInterruptException:
        pass
