#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from mavros_msgs.srv import CommandLong
# from utils import ARUCO_DICT, aruco_display
# gimbal_control 
# import asyncio
# from mavsdk import System
# from mavsdk.gimbal import GimbalMode, ControlMode

from geometry_msgs.msg import Point
from std_msgs.msg import Int32

# Print "Hello!" to terminal
# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('gimbal_control', anonymous=True)

  # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("gimbal control start!!")

  # Initialize the CvBridge class
bridge = CvBridge()
command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)

# Initial PWM values
yaw_pwm = 1495
pitch_pwm = 1495
  # Send pwm values to gimbal

    
    # if abs(yaw_error) < 3:
    #   output_yaw = 0.0

    # if abs(pitch_error) <current_angle 3:
    #   output_picth = 0.0

    # if detect_flag == -1:
    #     output_picth = output_yaw = 0

    # # go to destination
    # gimbal_angle.z = detect_flag
    # gimbal_angle_pub.publish(gimbal_angle)

# center_sub = rospy.Subscriber('center_of_image', Point, center_callback,queue_size=1)
#    print(msg)

# a = 0.0
# async def run():
#     # Init the drone

#     global outputY
#     global outputP
#     global a

#     global output_picth 
#     global output_yaw 
#     global mission_flag

#     drone = System()
#     await drone.connect(system_address="udp://:14599")

#     # Start printing gimbal position updates
#     print_gimbal_position_task = \
#         asyncio.ensure_future(print_gimbal_position(drone))

#     print("Taking control of gimbal")
#     await drone.gimbal.take_control(ControlMode.PRIMARY)

#     #print(outputY) 
    
#     #await drone.gimbal.set_pitch_and_yaw(-90, 0)
#     while 1:
#         #a = a+1
#         # print(output_picth,":::",output_yaw)
#         # if mission_flag == 3:
#         #     await drone.gimbal.set_pitch_and_yaw(-90, 0)
#         # else:
#         #     await drone.gimbal.set_pitch_rate_and_yaw_rate(output_picth , output_yaw)
#         await drone.gimbal.set_pitch_rate_and_yaw_rate(output_picth , output_yaw)
#         await asyncio.sleep(0.000001)
#         True

# async def print_gimbal_position(drone):
#     global gimbal_angle_pub
#     global current_angle
#     global detect_flag
#     # Report gimbal position updates asynchronously
#     # Note that we are getting gimbal position updates in
#     # euler angles; we can also get them as quaternions
#     async for angle in drone.telemetry.camera_attitude_euler():
#       current_angle.x = angle.yaw_deg
#       current_angle.y = angle.pitch_deg
#       current_angle.z = detect_flag
#       # print("OK")
#       gimbal_angle_pub.publish(current_angle)
      
#       # print("1")
#       print(f"Gimbal pitch: {angle.pitch_deg}, yaw: {angle.yaw_deg}")

# # end gimbal control method

#   # Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
# #sub_image = rospy.Subscriber("/standard_vtol/c14540amera/rgb/image_raw", Image, image_callback)
# # sub_image = rospy.Subscriber("/cgo3_camera/image_raw", Image, image_callback)

# #   # Initialize an OpenCV Window named "Image Window"
# # cv2.namedWindow("Image Window", 1)

#   # Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed


# # print("GGG")
# asyncio.run(run())
# print("GGG")
command_service(
            broadcast=False,
            command=183,  # MAV_CMD_DO_SET_SERVO
            param1=12,  # 서보 채널 (예: 9)
            param2=2000,      # 원하는 PWM 값
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
command_service(
            broadcast=False,
            command=183,  # MAV_CMD_DO_SET_SERVO
            param1=13,  # 서보 채널 (예: 9)
            param2=2000,      # 원하는 PWM 값
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )

rate = rospy.Rate(50)
while not rospy.is_shutdown():
    
    

    rate.sleep()
