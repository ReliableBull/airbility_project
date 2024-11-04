#!/usr/bin/env python

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge, CvBridgeError


# init point
point_msg = Point()
point_msg.x = -1
point_msg.y = -1
point_msg.z = -1

class RedBoxDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/uav0/rrbot/camera1/image_raw", Image, self.image_callback)
        self.pub = rospy.Publisher('center_of_image', Point, queue_size=10)
        self.rate = rospy.Rate(10)  # 10Hz로 메시지 발행

    def image_callback(self, data):
        global point_msg
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

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
                rospy.loginfo(f"빨간색 영역의 중심: ({centerX}, {centerY})")
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

        # 결과 이미지를 보여줌 (디버깅 용도)
        cv2.imshow("Red Box Detection", cv_image)
        cv2.waitKey(1)

    def publish_point(self):
        while not rospy.is_shutdown():
            self.pub.publish(point_msg)
            self.rate.sleep()

def main():
    rospy.init_node('red_box_detector', anonymous=True)

    detector = RedBoxDetector()

    try:
        detector.publish_point()  # while 루프에서 point_msg를 발행
    except rospy.ROSInterruptException:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
