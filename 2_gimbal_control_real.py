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


# yaw
errorY_last = 0
outputY =0

# pitch
errorP_last = 0
outputP =0

output_picth = 0
output_yaw = 0

current_angle = Point()

current_angle.x = -1
current_angle.y = -1
current_angle.z = -1
# Print "Hello!" to terminal
# Initialize the ROS Node named 'opencv_example', allow multiple nodes to be run with this name
rospy.init_node('gimbal_control', anonymous=True)

  # Print "Hello ROS!" to the Terminal and to a ROS Log file located in ~/.ros/log/loghash/*.log
rospy.loginfo("gimbal control start!!")

  # Initialize the CvBridge class
bridge = CvBridge()
command_service = rospy.ServiceProxy('/mavros/cmd/command', CommandLong)


def callBackGimbalAngle(data):
    global gimbal_angle
    gimbal_angle = data
    
    #print("gimbal angles : " , gimbal_angle)


class PID:
    def __init__(self, kp, ki, kd, max_output, min_output, max_integ, min_integ, sample_time):
        self.kp = kp  # Proportional Gain
        self.ki = ki  # Integral Gain
        self.kd = kd  # Derivative Gain
        self.max_output = max_output  # Maximum Output
        self.min_output = min_output  # Minimum Output
        self.max_integ = max_integ  # Maximum Integral Term
        self.min_integ = min_integ  # Minimum Integral Term
        self.sample_time = sample_time  # Sample Time

        self.target = 0.0  # Target Value
        self.integ = 0.0  # Integral Term
        self.last_error = 0.0  # Last Error
        self.last_time = rospy.Time.now()  # Last Time

      
    def update(self, feedback_value):
        error = self.target - feedback_value  # Error
        dt = (rospy.Time.now() - self.last_time).to_sec()  # Time Step

        if dt == 0:
            dt = 0.003

        # Proportional Term
        P = self.kp * error

        # Integral Term
        self.integ += error * dt
        self.integ = max(self.min_integ, min(self.max_integ, self.integ))
        I = self.ki * self.integ

        # Derivative Term
        D = self.kd * (error - self.last_error) / dt

       
        # PID Output
        output = P + I + D

        # print(self.min_output,"::",self.max_output,":::",output)
        output = max(self.min_output, min(self.max_output, output))

        # Update Last Error and Last Time
        self.last_error = error
        self.last_time = rospy.Time.now()

        return output
  # Define a function to show the image in an OpenCV Window



gimbal_pitch_pid = PID(2.5, 0.00, 0.2, 1000.0, -1000.0, 1.0, -1.0, 0.1) # with vision
gimbal_yaw_pid = PID(2.3, 0.00, 0.1, 1000.0, -1000.0, 1.0, -1.0, 0.1) # with vision
detect_flag= -1
gimbal_angle_pub = rospy.Publisher("/angle_of_gimbal", Point,queue_size=10)
mission_flag = -1 # mission flag
# command_flag = -1 # command flag

# command_to_gimbal_pub = rospy.Publisher("/angle_of_gimbal", Int32, queue_size=10)

def mission_callback(msg):
    global mission_flag
    mission_flag = msg.data

# Frome gimbal encoder
rospy.Subscriber("/encoder_angles", Point, callBackGimbalAngle,queue_size=1)

# From move_drone
rospy.Subscriber('/missionTogimbal', Int32, mission_callback)

gimbal_angle = Point()
gimbal_angle.z = -1

# Initial PWM values
yaw_pwm = 1495
pitch_pwm = 1495

# PWM limits
yaw_min, yaw_max = 1000, 2000
pitch_min, pitch_max = 2000, 1000

image_width = 640
image_height = 480

def show_image(img):
    cv2.imshow("Image Window", img)   
    cv2.waitKey(1)

def center_callback(msg):
    global command_service
    global command_flag
    global gimbal_pitch_pid
    global gimbal_yaw_pid

    global output_picth 
    global output_yaw 
    global detect_flag
    global mission_flag
    global gimbal_angle
    
    DEAD_ZONE_LEFT = 1445
    DEAD_ZONE_RIGHT = 1560

    DEAD_ZONE_DOWN = 1560
    DEAD_ZONE_UP=1420
    x = msg.x
    y = msg.y
    detect_flag = msg.z

    yaw_error =  image_width/2 - x 
    pitch_error =  y - image_height/2  

    
    output_pitch = gimbal_pitch_pid.update(pitch_error)
    output_yaw = gimbal_yaw_pid.update(yaw_error)

    print(str(output_pitch) +":: " + str(output_yaw))
    
    if output_yaw < 0:
       yaw_pwm = DEAD_ZONE_LEFT + output_yaw
    else:
       yaw_pwm = DEAD_ZONE_RIGHT + output_yaw

#    if output_pitch< 0:    
#        pitch_pwm = DEAD_ZONE_DOWN + output_pitch
#    else:
#        pitch_pwm = DEAD_ZONE_UP - output_pitch

    # Adjust and clamp PWM values
    yaw_pwm = int(min(max(yaw_pwm, yaw_min), yaw_max))
    pitch_pwm = int(min(max(1495-output_pitch, pitch_max), pitch_min))

    print(f"Yaw PWM: {yaw_pwm}, Pitch PWM: {pitch_pwm}")

    # Break the loop if target is near center (within tolerance)
    # print(yaw_error,":::",pitch_error)
    if abs(yaw_error) < 3:
        yaw_pwm = 1495
    if abs(yaw_error) < 3 and abs(pitch_error) < 3:
        print("Target centered.")

    if gimbal_angle.y >= 90.0: # if gimbal pitch angle is 90 degrees
        pitch_pwm = 1495
        yaw_pwm = 1495
        print("downward")
#    if 1445 < yaw_pwm < 1560:
#        if yaw_pwm > 1495:
#            yaw_pwm = 1590
#        else:
#            yaw_pwm = 143335
#    if yaw_pwm == 1495:
 #       pass
 #   elif abs(yaw_pwm - 1495) < 65:
  #      yaw_pwm = 1495 + (70 if yaw_pwm > 1495 else -60)
#    pitch_pwm = 1560

    if mission_flag == 2: # go to destination
        # fix yaw angle
        yaw_pwm = 1495
    elif mission_flag == 3: # fixed gimbal and align horizontal
        picth_pwm = yaw_pwm = 1495  

    # Send pwm values to gimbal
    command_service(
            broadcast=False,
            command=183,  # MAV_CMD_DO_SET_SERVO
            param1=9,  # 서보 채널 (예: 9)
            param2=pitch_pwm,      # 원하는 PWM 값
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )

    command_service(
            broadcast=False,
            command=183,  # MAV_CMD_DO_SET_SERVO
            param1=10,  # 서보 채널 (예: 9)
            param2=yaw_pwm,      # 원하는 PWM 값
            param3=0,
            param4=0,
            param5=0,
            param6=0,
            param7=0
        )
    
    # if abs(yaw_error) < 3:
    #   output_yaw = 0.0

    # if abs(pitch_error) <current_angle 3:
    #   output_picth = 0.0

    # if detect_flag == -1:
    #     output_picth = output_yaw = 0

    # # go to destination
    # gimbal_angle.z = detect_flag
    # gimbal_angle_pub.publish(gimbal_angle)

center_sub = rospy.Subscriber('center_of_image', Point, center_callback,queue_size=1)
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
rate = rospy.Rate(50)
while not rospy.is_shutdown():
    
    gimbal_angle_pub.publish(gimbal_angle)
    rate.sleep()
