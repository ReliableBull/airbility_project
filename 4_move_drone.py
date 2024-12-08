#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, PoseStamped, Point
from mavros_msgs.msg import State
from mavros_msgs.srv import SetMode, CommandBool , SetModeRequest
from mavros_msgs.srv import SetMode, CommandBool , SetModeRequest , SetMavFrame , SetMavFrameRequest
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64
from sensor_msgs.msg import NavSatFix
import tf.transformations
import threading, requests, time
import math
from geopy.distance import geodesic
from std_msgs.msg import Int32

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
        #D = 0

        # PID Output
        output = P + I + D
        output = max(self.min_output, min(self.max_output, output))

        # Update Last Error and Last Time
        self.last_error = error
        self.last_time = rospy.Time.now()

        return output

class MoveDrone:
    def __init__(self):
        rospy.init_node('move_drone', anonymous=True)

        # Initialize ROS Subscriber
        rospy.Subscriber('/mavros/state', State, self.state_cb)
        rospy.Subscriber('/mavros/local_position/odom', Odometry, self.position_cb)
        rospy.Subscriber("/mavros/global_position/global" , NavSatFix , self.callBaclkGlobalPosition)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.pose_callback)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/estimation_gps', NavSatFix, self.target_gps_callback)
        # rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.pose_callback)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.sub_compass_hdg)
        # Initialize ROS Publisher
        self.vel_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
        self.alt_pub = rospy.Publisher('/mavros/setpoint_position/rel_alt', Float32, queue_size=10)
        self.mission_pub = rospy.Publisher('/missionTogimbal', Int32, queue_size=10)


        
        # Initialize ROS Service
        self.arm_service = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flight_mode_service = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.gimbal_angle_sub = rospy.Subscriber("/angle_of_gimbal", Point, self.callBackGimbalAngle)
        self.center_sub = rospy.Subscriber('center_of_image', Point, self.center_callback)
        

        # Initialize Variables
        self.current_state = State()
        self.current_pose = None
        self.current_global_position = None
        self.target_pose = PoseStamped()
        self.target_pose.header.frame_id = "home"
        self.target_pose.pose.position.x = 10.0
        self.target_pose.pose.position.y = 10.0
        self.target_pose.pose.position.z = 0.0
        self.offb_set_mode = SetModeRequest()
        self.offb_set_mode.custom_mode = 'OFFBOARD'

        # tracking 관련 target_lat = 35.8934302
        self.target_lat = None    
        self.target_lng = None   

        # target 이전 좌표
        self.prev_target_lat = None    
        self.prev_target_lng = None
        self.destination=None
                # gimbal angle
        self.gimbal_angle = Point()
        self.gimbal_angle.z = -1

        self.yaw = None

        # Initialize PID Controllers
        self.x_pid = PID(0.8, 0.00, 0.05, 1.0, -1.0, 1.0, -1.0, 0.1)
        self.y_pid = PID(0.8, 0.00, 0.05, 1.0, -1.0, 1.0, -1.0, 0.1)
        
        self.z_pid = PID(0.5, 0.00, 0.01, 0.5, -0.5, 1.0, -1.0, 0.1)
        self.height_pid = PID(0.2, 0.01, 0.01, 0.3, -0.3, 1.0, -1.0, 0.1)
        self.z_angular_pid = PID(0.3, 0.01, 0.03, 0.02, -0.05, 1.0, -1.0, 0.1) # only gimbal

        # horizontal PID controllers
        self.x_pid_horizontal = PID(0.03, 0.00, 0.001, 0.25, -0.25, 1.0, -1.0, 0.1)
        self.y_pid_horizontal = PID(0.03, 0.00, 0.001, 0.25, -0.25, 1.0, -1.0, 0.1)
        self.height_pid_horizontal = PID(0.5, 0.00, 0.01, 1.0, -1.0, 1.0, -1.0, 0.1)



        # params
        self.start_height =60.5 
        
        
        self.flag = 0
        self.horizontal_x_error= None
        self.horizontal_y_error= None

        self.frame_set_mode = SetMavFrameRequest()
        self.frame_set_mode.mav_frame = 8
                # Change frame
        self.set_mav_frame =rospy.ServiceProxy("/mavros/setpoint_velocity/mav_frame", SetMavFrame)     
        
        self.current_hegiht = None

        self.current_hdg=None
        self.target_hdg = None


        # self.t1 = threading.Thread(target=self.getHtml)
        # self.t1.start()
        # gimbal callback 
    # def pose_callback(self, data):
    #     # 메시지 데이터를 처리하는 콜백 함수
    #     # rospy.loginfo("Position: [x: %f, y: %f, z: %f]", 
    #     #           data.pose.position.x, 
    #     #           data.pose.position.y, 
    #     #           data.pose.position.z)
    #     self.current_hegiht = data.pose.position.z

    def sub_compass_hdg(self, msg):
        self.current_hdg = msg.data

    def center_callback(self, msg):
        image_width = 640
        image_height = 480

        x = msg.x
        y = msg.y
        self.horizontal_y_error = image_width/2 - x 
        self.horizontal_x_error =  y - image_height/2  

    def callBackGimbalAngle(self, data):
        # x : yaw
        # y : pitch
        self.gimbal_angle = data
        #print(self.gimbal_angle)

    def target_gps_callback(self,msg):

        if self.target_lat == None:
            self.target_lat = msg.latitude
            self.target_lng = msg.longitude

        # print(self.target_lat,":::",self.target_lng)

    def pose_callback(self,msg):
        self.current_hegiht = msg.pose.position.z
        quaternion = (
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        )


        euler = tf.transformations.euler_from_quaternion(quaternion)
        yaw_radian = euler[2]  

        yaw_degree = math.degrees(yaw_radian)

        yaw_degree = (-yaw_degree + 360) % 360

        self.yaw = (yaw_degree + 90 ) % 360

    #rospy.loginfo(yaw_degree)
    # 여기서 yaw 각도를 추출합니다.

    def callBaclkGlobalPosition(self,data):
        #print(data)     
        self.current_global_position = data

        #sub_topics_ready['global_pos'] = True

    # 현재 position <-> Target position 각도 계산
    def calcAngle(self,current_lat , current_lng , target_lat , target_lng):
        lat1 = current_lat
        lon1 = current_lng

        # 목적지
        lat2 = target_lat
        lon2 = target_lng

        # 위도, 경도를 라디안 단위로 변환
        pi_1 = lat1 * math.pi / 180
        pi_2 = lat2 * math.pi / 180
        gamma_1 = lon1 * math.pi / 180
        gamma_2 = lon2 * math.pi / 180

        y = math.sin(gamma_2 - gamma_1) * math.cos(pi_2)
        x = math.cos(pi_1) * math.sin(pi_2) - math.sin(pi_1) * math.cos(pi_2) * math.cos(gamma_2 - gamma_1)
        theta = math.atan2(y, x); # 방위각 (라디안)

        bearing = (theta * 180 / math.pi + 360) % 360; # 방위각 (디그리, 정규화 완료)
        
        # return (bearing * math.pi / 180)
        return bearing

    def setOffboard(self):
        last_req = rospy.Time.now()
        
        while(not rospy.is_shutdown()):
            if(self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0)):
                if((self.flight_mode_service.call(self.offb_set_mode).mode_sent == True)):
                    #self.flight_mode_service.call(self.offb_set_mode)
                    rospy.loginfo("OFFBOARD enabled")
                    break
            # else:
            #     break

    def state_cb(self, state_msg):
        self.current_state = state_msg

    def position_cb(self, odom_msg):
        #print(odom_msg)
        self.current_pose = odom_msg.pose.pose

    def arm(self):
        #self.setOffboard()
        
        rospy.loginfo("Arming Drone")
        
        while not rospy.is_shutdown():
            if self.current_state.armed:
                break
            self.arm_service(True)
            rospy.sleep(1)
      

    def takeoff(self):
        rospy.loginfo("Taking off")
        while not rospy.is_shutdown():
            # print(self.current_state.mode)
            # if self.current_pose.position.z >= 3.0:
            #     break
            # self.alt_pub.publish(3.0)

            vel_msg = Twist()
            vel_msg.linear.x = 0.1
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0
            self.vel_pub.publish(vel_msg)
            rospy.sleep(0.1)

    def change_mav_frame(self, frame_number):
        frame_set_mode = SetMavFrameRequest()
        frame_set_mode.mav_frame = frame_number              
        
        while not rospy.is_shutdown():
            if (self.set_mav_frame.call(frame_set_mode).success == True):
               rospy.loginfo("update mav frame")
               break

        return True
    def move_drone(self):
        rospy.loginfo("Moving Drone")

        rate = rospy.Rate(30)

        # align yaw of drone
        self.flag = 1
        alignYawCnt = 0
        arrivalCnt = 0
        # 1: global
        # 8 : BODYFRAME
        self.change_mav_frame(1)
        stop_cnt = 0
        while not rospy.is_shutdown():

            if self.current_global_position is None:
                rospy.loginfo("Waiting for global position update...")
                continue
            
            if self.target_lat is None:
                rospy.loginfo("Waiting for target position update...")
                continue
            
            
            # To do
            current_gimbal_yaw = self.gimbal_angle.z # yaw
            current_gimbal_pitch = self.gimbal_angle.y # pitch

            yaw_error = abs(current_gimbal_yaw)
            yaw_output = self.z_angular_pid.update(yaw_error*0.1)

            if current_gimbal_yaw < 0: # rotate right
                yaw_output = -yaw_output

            pointA = (self.current_global_position.latitude ,self.current_global_position.longitude)

            pointB = (self.target_lat ,self.target_lng)

            # 이동 방향(베어링) 계산
            bearing = self.calculate_bearing(pointB, pointA)
            
            # # 이동 방향 반대쪽 3m 지점 좌표 계산
            destination_point = self.calculate_destination_coordinate(pointB, bearing, 0)

            x_error , y_error = self.distance_on_xy(pointB[0] , pointB[1]  ,self.current_global_position.latitude, self.current_global_position.longitude)

            # 헤딩각을 위한 계산
            diff_angle = self.calcAngle(self.current_global_position.latitude, self.current_global_position.longitude,self.target_lat , self.target_lng )
            
            # 타겟과 현재 드론의 거리 추출
            diff_distance = geodesic((self.current_global_position.latitude , self.current_global_position.longitude) , (self.target_lat,self.target_lng)).meters
            
            height_error = (self.start_height +(self.current_global_position.altitude-self.start_height)) - self.current_global_position.altitude 

            height_output = self.height_pid.update(int(height_error))

            # print("x_error : " , x_error , "  y_error : " , y_error)
            x_output = self.x_pid.update(x_error)
            y_output = self.y_pid.update(y_error)

            # z_output = 0.0

            # Align yaw
            if self.flag == 1:
                x_output = y_output = 0.0
                if current_gimbal_yaw != 0.0 and abs(current_gimbal_yaw) < 1.0:
                    alignYawCnt = alignYawCnt + 1
                else:
                    alignYawCnt = 0.0
                if alignYawCnt > 5:
                    # Set flag 2
                    stop_cnt = 0
                    self.flag = 2
                    print("\033[91mDrone yaw alignment complete. \033[0m")
                    print("\033[91mGo to the destination. \033[0m")                                       
            elif self.flag == 2: # go to destination
                yaw_output = 0.0
                if diff_distance < 2.0: # m
                    arrivalCnt = arrivalCnt + 1
                else:
                    arrivalCnt = 0.
                if arrivalCnt > 5:
                    print("\033[91mArrived at the destination. \033[0m")
                    self.flag = 3
                    self.change_mav_frame(8) # change body frame
                    x_output = y_output = 0.0
                    print("\033[91mApproaching the destination. \033[0m")              

            elif self.flag == 4: # Arrive at the destination
                # Align for target
                x_output = self.x_pid_horizontal.update(self.horizontal_x_error*0.1)  # drone body frame x-axis
                y_output = self.y_pid_horizontal.update(self.horizontal_y_error*0.1)  # drone body frame y-axis
                
                if abs(self.horizontal_x_error) > 10 or abs(self.horizontal_y_error > 10):
                    print("Align again")
                    height_output = 0.0
                else:
                    height_output = 0.5                

                y_output = -y_output
                yaw_output = 0.0
                # If altitude of drone is below a threshold
                if self.current_hegiht < 15.0: # m
                    self.flag = 5
                    print("\033[91mFire extinguisher bomb drop. \033[0m")
            else: # DONE
                x_output = y_output = yaw_output = 0.0
                height_output = -0.5 # up speed
            
            vel_msg = Twist()
            vel_msg.linear.x = x_output
            vel_msg.linear.y = y_output
            # vel_msg.linear.x = 0.0
            # vel_msg.linear.y = 0.0
            # vel_msg.linear.z = 0.0
            vel_msg.linear.z = -height_output
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            # vel_msg.angular.z = 0.0
            vel_msg.angular.z = yaw_output
            # vel_msg.angular.z = 0.2
            # + : left
            # - : right
            print(vel_msg)
            self.vel_pub.publish(vel_msg)
            self.mission_pub.publish(self.flag)    
            if self.flag == 3:
                stop_cnt = stop_cnt + 1
            if stop_cnt > 50: # 1hz
                self.flag = 4           
            rate.sleep()

    # tracking 관련 함수
    def distance_on_xy(self, lat1, lon1, lat2, lon2):
        R = 6371000  # 지구의 반지름 (m)
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)
        dx = R * math.cos(lat1_rad) * (lon2_rad - lon1_rad)
        dy = R * (lat2_rad - lat1_rad)
        return dx, dy
    

    def calculate_bearing(self,pointA, pointB):
        lat1, lon1 = pointA
        lat2, lon2 = pointB

        # 위도와 경도를 라디안 단위로 변환
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        # 경도 차이 계산
        dLon = lon2 - lon1

        y = math.sin(dLon) * math.cos(lat2)
        x = math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(dLon)

        # 방위각 계산
        bearing = math.atan2(y, x)

        # 라디안 값을 도 단위로 변환
        bearing = math.degrees(bearing)

        # 0 ~ 360도 사이로 조정
        bearing = (bearing + 360) % 360

        return bearing
    
    
    def calculate_destination_coordinate(self, start_point, bearing, distance):
        # 시작 지점의 위도와 경도 추출
        lat1, lon1 = start_point

        # 이동 거리를 이동 방향의 벡터로 변환
        dx = distance * math.cos(math.radians(bearing))
        dy = distance * math.sin(math.radians(bearing))

        # 새로운 좌표 계산
        lat2 = lat1 + (dx / 111111)  # 1도의 위도 차이는 약 111,111 미터
        lon2 = lon1 + (dy / (111111 * math.cos(math.radians(lat1))))  # 1도의 경도 차이는 약 111,111 * cos(위도) 미터
        return lat2, lon2
if __name__ == "__main__":

    move_drone = MoveDrone()

     # Arm Drone
    #move_drone.arm()

    # Takeoff Drone
    #move_drone.takeoff()
    #rospy.sleep(2.0)
    move_drone.move_drone()
