import serial
import struct
import time

BUF_SIZE = 47

def calculate_angle(servo_status):
    # 8비트 값을 12비트로 확장
    expanded_value = servo_status & 0x0F
    print(f"Expanded Value: {hex(expanded_value)}")

    new_yaw = (expanded_value << 8) + 0xFF
    print(f"New Yaw: {hex(new_yaw)}")

    # 각도 계산
    angle = (new_yaw * 180.0) / 4095.0
    print(f"Angle Before Subtracting 90: {angle}")

    # 기준값(90도)을 뺌
    angle -= 90.0

    return angle

def hex_to_signed_decimal(hex_value):
    # uint16(16비트 무부호 정수)를 int16(16비트 부호 있는 정수)로 변환
    if hex_value >= 0x8000:
        return hex_value - 0x10000
    return hex_value

def main():
    command = bytes([0x55, 0xAA, 0xDC, 0x04, 0x10, 0x00, 0x14])
    buf = bytearray(BUF_SIZE)

    # 시리얼 포트 설정
    ser = serial.Serial(
        port="/dev/ttyUSB1",
        baudrate=115200,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1
    )

    if not ser.is_open:
        ser.open()

    while True:
        # 명령 전송
        ser.write(command)
        

        # 응답 읽기
        res = ser.read(BUF_SIZE)

        if len(res) >= BUF_SIZE and res[0] == 0x55 and res[1] == 0xAA and res[2] == 0xDC and res[4] == 0x40:
            print("Response received:")
            for i in range(BUF_SIZE):
                print(f"{i:02}: {hex(res[i])}")

            # 데이터 추출
            roll = ((res[28] & 0x0F) << 8) + res[29]
            yaw = (res[32] << 8) + res[33]
            pitch = (res[30] << 8) + res[31]

            roll_angle = (roll * 180.0) / 4095.0
            pitch_angle = (pitch * 360.0) / 65536.0
            yaw_angle = (hex_to_signed_decimal(yaw) * 360.0) / 65536.0

            print(f"Roll: {roll_angle - 90.0}°")
            print(f"Pitch: {pitch_angle}°")
            print(f"Yaw: {yaw_angle}°")

        time.sleep(0.1)

    ser.close()

if __name__ == "__main__":
    main()
