import socket
import struct
import time

BUF_SIZE = 47

def viewlink_protocol_checksum(viewlink_data_buf):
    """
    Calculate the checksum for the given data buffer.
    :param viewlink_data_buf: bytearray containing the data
    :return: Calculated checksum as an integer
    """
    length = viewlink_data_buf[2]
    checksum = length
    print(length)
    # Calculate XOR of the specified bytes
    for i in range(length - 3):
        checksum ^= viewlink_data_buf[4 + i]

    return checksum

def hex_to_signed_decimal(hex_value):
    """
    Convert a 16-bit unsigned value to a signed 16-bit integer.
    :param hex_value: Unsigned 16-bit integer
    :return: Signed 16-bit integer
    """
    if hex_value >= 0x8000:
        return hex_value - 0x10000
    return hex_value

def calculate_angle_from_data(data):
    """
    Extract roll, pitch, and yaw angles from the response data.
    :param data: Response data from the device
    :return: Roll, Pitch, Yaw angles
    """
    roll = ((data[28] & 0x0F) << 8) + data[29]
    pitch = (data[30] << 8) + data[31]
    yaw = (data[32] << 8) + data[33]

    roll_angle = (roll * 180.0) / 4095.0 - 90.0
    pitch_angle = (pitch * 360.0) / 65536.0
    yaw_angle = (hex_to_signed_decimal(yaw) * 360.0) / 65536.0

    return roll_angle, pitch_angle, yaw_angle

def main():
    # TCP connection details
    ip_address = "192.168.2.119"
    port = 2000

    # Command data
    command = bytearray([0xEB, 0x90, 0x07, 0x55, 0xAA, 0xDC, 0x04, 0x10, 0x00, 0x14,0x03])
    # command[-1] = viewlink_protocol_checksum(command)  # Calculate checksum

    print(f"Sending Command: {[hex(byte) for byte in command]}")

    # Open TCP connection
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as tcp_socket:
        tcp_socket.connect((ip_address, port))
        print(f"Connected to {ip_address}:{port}")

        # Send command
        # tcp_socket.sendall(command)
        print("Command sent!")
        tcp_socket.sendall(command)
        while True:
            

            # Read response
            response = tcp_socket.recv(BUF_SIZE)

            if len(response) >= BUF_SIZE and response[0] == 0x55 and response[1] == 0xAA:
                print("Response received:")
                for i in range(BUF_SIZE):
                    print(f"{i:02}: {hex(response[i])}")

                # Calculate angles
                roll_angle, pitch_angle, yaw_angle = calculate_angle_from_data(response)

                print(f"Roll: {roll_angle:.2f}°")
                print(f"Pitch: {pitch_angle:.2f}°")
                print(f"Yaw: {yaw_angle:.2f}°")
            # time.sleep(1.1)  # Delay between reads
            

if __name__ == "__main__":
    main()

