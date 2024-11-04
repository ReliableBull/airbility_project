class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def update(self, current_value):
        # Calculate error
        error = self.setpoint - current_value

        # Calculate integral
        self.integral += error

        # Calculate derivative
        derivative = error - self.previous_error

        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Update previous error
        self.previous_error = error

        return output

# Set up PID controllers for yaw and pitch
yaw_pid = PIDController(kp=0.1, ki=0.01, kd=0.05, setpoint=0)   # Yaw target is the image center (0 offset)
pitch_pid = PIDController(kp=0.1, ki=0.01, kd=0.05, setpoint=0) # Pitch target is the image center (0 offset)

# Initial PWM values
yaw_pwm = 1495
pitch_pwm = 1495

# PWM limits
yaw_min, yaw_max = 900, 2000
pitch_min, pitch_max = 1000, 2000

# Image center (assuming image width W and height H)
W, H = 640, 480
center_x, center_y = W / 2, H / 2

# Sample target coordinates (e.g., from image processing)
target_x, target_y = 500, 300  # example target position

# Main control loop
while True:
    # Calculate offsets from image center
    x_offset = target_x - center_x
    y_offset = target_y - center_y

    # Update PWM using PID controller
    yaw_adjustment = yaw_pid.update(x_offset)
    pitch_adjustment = pitch_pid.update(y_offset)

    # Adjust and clamp PWM values
    yaw_pwm = int(min(max(1495 + yaw_adjustment, yaw_min), yaw_max))
    pitch_pwm = int(min(max(1495 + pitch_adjustment, pitch_min), pitch_max))

    # Output PWM to gimbal (this part depends on your actual hardware control library)
    print(f"Yaw PWM: {yaw_pwm}, Pitch PWM: {pitch_pwm}")

    # Break the loop if target is near center (within tolerance)
    if abs(x_offset) < 5 and abs(y_offset) < 5:
        print("Target centered.")
        break


class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.previous_error = 0
        self.integral = 0

    def update(self, current_value):
        # Calculate error
        error = self.setpoint - current_value

        # Calculate integral
        self.integral += error

        # Calculate derivative
        derivative = error - self.previous_error

        # PID output
        output = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Update previous error
        self.previous_error = error

        return output

# Set up PID controllers for yaw and pitch
yaw_pid = PIDController(kp=0.1, ki=0.01, kd=0.05, setpoint=0)   # Yaw target is the image center (0 offset)
pitch_pid = PIDController(kp=0.1, ki=0.01, kd=0.05, setpoint=0) # Pitch target is the image center (0 offset)

# Initial PWM values
yaw_pwm = 1495
pitch_pwm = 1495

# PWM limits
yaw_min, yaw_max = 900, 2000
pitch_min, pitch_max = 2000, 1000  # Adjusted to reflect max up at 2000 and max down at 1000

# Image center (assuming image width W and height H)
W, H = 640, 480
center_x, center_y = W / 2, H / 2

# Sample target coordinates (e.g., from image processing)
target_x, target_y = 500, 300  # example target position

# Main control loop
while True:
    # Calculate offsets from image center
    x_offset = target_x - center_x
    y_offset = target_y - center_y

    # Update PWM using PID controller
    yaw_adjustment = yaw_pid.update(x_offset)
    pitch_adjustment = pitch_pid.update(y_offset)

    # Adjust and clamp PWM values
    yaw_pwm = int(min(max(1495 + yaw_adjustment, yaw_min), yaw_max))
    pitch_pwm = int(min(max(1495 - pitch_adjustment, pitch_max), pitch_min))  # Invert adjustment for pitch direction

    # Output PWM to gimbal (this part depends on your actual hardware control library)
    print(f"Yaw PWM: {yaw_pwm}, Pitch PWM: {pitch_pwm}")

    # Break the loop if target is near center (within tolerance)
    if abs(x_offset) < 5 and abs(y_offset) < 5:
        print("Target centered.")
        break

