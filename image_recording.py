from ultralytics import YOLO
import cv2
import math
import sys
import rospy
import pyproj
import numpy as np
from geometry_msgs.msg import Point
from sensor_msgs.msg import NavSatFix , Image
import timeit
from datetime import datetime

# init point
point_msg = Point()
point_msg.x = -1
point_msg.y = -1
point_msg.z = -1

fps = 50
width = 640
height = 480

# start webcam

# cap = cv2.VideoCapture('/dev/video4')
cap = cv2.VideoCapture('rtsp://192.168.2.119:554')
# LTE
# out = cv2.VideoWriter('appsrc ! videoconvert' + \
#     ' ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=40' + \
#     ' ! rtspclientsink location=rtsp://118.67.132.33:8554/mystream protocols=tcp',
#     cv2.CAP_GSTREAMER, 0, fps, (width, height), True)
# RF
# out = cv2.VideoWriter('appsrc ! videoconvert' + \
#     ' ! x264enc speed-preset=ultrafast bitrate=600 key-int-max=40' + \
#     ' ! rtspclientsink location=rtsp://127.0.0.1:8554/drone protocols=tcp',
#     cv2.CAP_GSTREAMER, 0, fps, (640, 480), True)

out = cv2.VideoWriter('appsrc ! videoconvert' + \
    ' ! video/x-raw, format=BGRx' + \
    ' ! nvvidconv' + \
    ' ! video/x-raw(memory:NVMM), format=NV12' + \
    ' ! nvv4l2h264enc bitrate=16000000 preset-level=1 insert-sps-pps=true qp-range=15,30:20,35:25,40' + \
    ' ! h264parse' + \
    ' ! rtspclientsink location=rtsp://118.67.132.33:8554/mystream protocols=tcp',
    cv2.CAP_GSTREAMER, 0, fps, (width, height), True)

if not out.isOpened():
    raise Exception("can't open video writer")


rospy.init_node('image_info')

rate = rospy.Rate(50)

pub = rospy.Publisher('center_of_image', Point, queue_size=10)

while not rospy.is_shutdown():
    success, img = cap.read()
    img = cv2.resize(img,(width,height))
    cv_image = cv2.resize(img, (width, height))
    # print(img.shape)
            # 알고리즘 시작 시점
 # 이미지를 HSV 색상 공간으로 변환
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # 빨간색 범위 정의 (HSV 범위)
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

        # 두 범위에 대해 마스크 생성
    mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
    mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # 두 마스크 합치기
    mask = mask1 + mask2

        # 마스크에 대한 컨투어 찾기
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 컨투어가 존재하는지 확인
    if contours:
            # 가장 큰 컨투어 선택
        c = max(contours, key=cv2.contourArea)

            # 컨투어의 외곽선을 따라 경계 상자 계산
        x, y, w, h = cv2.boundingRect(c)

            # 빨간색 영역에 사각형 그리기
        cv2.rectangle(cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # 중심점 계산 및 그리기
        M = cv2.moments(c)
        if M["m00"] != 0:
            centerX = int(M["m10"] / M["m00"])
            centerY = int(M["m01"] / M["m00"])
            cv2.circle(cv_image, (centerX, centerY), 1, (255, 0, 0), -1)
            # rospy.loginfo(f"빨간색 영역의 중심: ({centerX}, {centerY})")
            point_msg.x = centerX
            point_msg.y = centerY
            point_msg.z = 1  # 감지된 상태를 나타내기 위해 z를 1로 설정
        else:
            point_msg.x = -1
            point_msg.y = -1
            point_msg.z = -1
    else:
        point_msg.x = -1
        point_msg.y = -1
        point_msg.z = -1    
    
    pub.publish(point_msg)
    #cv2.imshow('frame', cv_image)
    out.write(img)
    

    #if cv2.waitKey(1) == ord('q'):
    #    break

cap.release()
cv2.destroyAllWindows()
