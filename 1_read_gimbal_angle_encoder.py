import serial
import struct
import time
import rospy
from geometry_msgs.msg import Point

BUF_SIZE = 47

rospy.init_node('Read gimbal angle', anonymous=True)

rospy.loginfo("Start gimbal angle")

#gimbal_angle_pub = rospy.Publisher("/angle_of_gimbal", Point,queue_size=10)
gimbal_angle_pub = rospy.Publisher("/angle_of_gimbal_temp", Point,queue_size=10)

current_angle = Point()


class GimbalGetAnglesExtReq:
    def __init__(self):
        self.timestamp = 0
        self.rollIMUangle = 0
        self.pitchIMUangle = 0
        self.yawIMUangle = 0
        self.rollIMUspeed = 0
        self.pitchIMUspeed = 0
        self.yawIMUspeed = 0
        self.rollStatorRotorAngle = 0
        self.pitchStatorRotorAngle = 0
        self.yawStatorRotorAngle = 0
def hex_to_signed_decimal(hex_value):
    # uint16(16비트 무부호 정수)를 int16(16비트 부호 있는 정수)로 변환
    if hex_value >= 0x8000:
        return hex_value - 0x10000
    return hex_value
def main():
    global current_angle
    global gimbal_angle_pub
    port = "/dev/ttyUSB1"
    baud_rate = 115200
    # command = bytearray([0x55, 0xAA, 0xDC, 0x04, 0x10, 0x00, 0x14])
    command = bytearray([0xAA, 0x55, 0x2F, 0x42, 0xFF])
    
    try:
        ser = serial.Serial(port, baud_rate,  
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,timeout=1)
        ser.flush()
        ser.write(command)
        while True:
            # Write the command to the serial port
            # ser.write(command)
            # print("Sent command")

            # Read BUF_SIZE bytes from the serial port
            buf = ser.read(BUF_SIZE)
            print(len(buf))
            if len(buf) == 0:
                continue
            # print(buf[0] ," ",buf[1] ," ",buf[2] ," ",buf[4] ," " )
            # if buf[0] == 0x55 and buf[1] == 0xAA and buf[2] == 0xDC and buf[4] == 0x40:
            #     print("Response received:")
            #     for i in range(BUF_SIZE):
            #         print(f"{i:02}: {hex(buf[i])}")

            #     # 데이터 추출
            #     roll = ((buf[28] & 0x0F) << 8) + buf[29]
            #     yaw = (buf[32] << 8) + buf[33]
            #     pitch = (buf[30] << 8) + buf[31]

            #     roll_angle = (roll * 180.0) / 4095.0
            #     pitch_angle = (pitch * 360.0) / 65536.0
            #     yaw_angle = (hex_to_signed_decimal(yaw) * 360.0) / 65536.0

            #     print(f"Roll: {roll_angle - 90.0}°")
            #     print(f"Pitch: {pitch_angle}°")
            #     print(f"Yaw: {yaw_angle}°")

            # if len(buf) != BUF_SIZE:
            #     print("Received incomplete data")
            #     continue
            
            print("Response received:")
            for i in range(BUF_SIZE):
                print(f"{i:02}: {hex(buf[i])}")

            # 데이터 추출
            roll = ((buf[28] & 0x0F) << 8) + buf[29]
            yaw = (buf[32] << 8) + buf[33]
            pitch = (buf[30] << 8) + buf[31]

            roll_angle = (roll * 180.0) / 4095.0
            pitch_angle = (pitch * 360.0) / 65536.0
            yaw_angle = (hex_to_signed_decimal(yaw) * 360.0) / 65536.0

            print(f"Roll: {roll_angle - 90.0}°")
            print(f"Pitch: {pitch_angle}°")
            print(f"Yaw: {yaw_angle}°")

            if len(buf) != BUF_SIZE:
                print("Received incomplete data")
                continue
            # Display each byte in the buffer
            # for i, byte in enumerate(buf):
            #     print(f"{i} {hex(byte)}")

            # # Extract pitch and yaw angles from the buffer
            # pitch = struct.unpack_from('<h', buf, 24)[0]
            # yaw = struct.unpack_from('<h', buf, 42)[0]

            # # Store values in the structure and convert angles
            # t = GimbalGetAnglesExtReq()
            # t.pitchIMUangle = pitch

            # new_yaw = yaw * 0.02197
            # t.yawIMUangle = yaw

            # # Print the angles after conversion
            # #print(f"pitch : {new_yaw* 0.02197}")
            # #print(f"yaw : {t.yawIMUangle * 0.02197}")
            
            # current_angle.x = t.yawIMUangle * 0.02197
            # current_angle.y = t.pitchIMUangle * 0.02197
            # current_angle.z = 1
            # print(f"pitch : {t.pitchIMUangle * 0.02197}")
            # print(f"new yaw : {new_yaw % 360}")
            # print(f"current yaw : {yaw * 0.02197}")
            # gimbal_angle_pub.publish(current_angle)
            time.sleep(0.2)


    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        if 'ser' in locals() and ser.is_open:
            ser.close()

if __name__ == "__main__":
    main()
