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



# start webcam
capEO = cv2.VideoCapture('rtsp://192.168.2.119/live1')
# capIR = cv2.VideoCapture('rtsp://192.168.2.119/live2')


rospy.init_node('image_info111')

rate = rospy.Rate(100)

pub = rospy.Publisher('center_of_image', Point, queue_size=10)
# cv2.namedWindow('EO')
# cv2.namedWindow('IR')
while not rospy.is_shutdown():
    
    success, img_EO = capEO.read()
    # success, img_IR = capIR.read()

    # cv_image = cv2.resize(img, (width, height))
    # print(img.shape)
    # cv2.imshow('IR', img_IR)
    cv2.imshow('EO', img_EO)
    
    # out.write(cv_image)
    

    if cv2.waitKey(1) == ord('q'):
       break

cap.release()
cv2.destroyAllWindows()
