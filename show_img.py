#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2

class ImageConverter:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/uav0/rrbot/camera1/image_raw", Image, self.callback)

    def callback(self, data):
        try:
            # ROS Image 메시지를 OpenCV 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # 이미지의 중심점 계산
        height, width, _ = cv_image.shape
        center_x = width // 2
        center_y = height // 2

        # 중심점에 원 그리기
        cv2.circle(cv_image, (center_x, center_y), 1, (0, 255, 0), -1)

        # OpenCV 윈도우에 이미지 표시
        cv2.imshow("Image Window", cv_image)
        cv2.waitKey(3)

def main():
    rospy.init_node('image_converter', anonymous=True)
    ic = ImageConverter()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
