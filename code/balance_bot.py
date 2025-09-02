# Import libraries
from XRPLib.defaults import *
from XRPLib.pid import PID
import time

# Wheel diameter in cm
drivetrain.wheel_diam = 10

# Loop period in seconds. The main loop needs to run relatively fast to ensure
# the PID controllers can react quickly
loop_period = 0.01

# Create PID controller that maintains the robot's pitch angle by actuating the
# motors
angle_pid = PID(
    kp = 0.1, # 0.1 corresponds to full effort per 10 degrees
    ki = 0.25, # 0.25 corresponds to full effort per 4 degree seconds
    kd = 0.005, # 0.005 corresponds to full effort per 200 degrees per second
    
    # Although the motor's max effort is 1, let the PID controller's output be
    # larger to detect when it goes extremely high, which likely indicates the
    # robot has fallen over
    max_output = 10
)

# Create PID controller that maintains the robot's position by adjusting the
# target pitch angle
position_pid = PID(
    kp = -0.1, # 0.1 corresponds to 1 degree per 10 cm
    ki = -0.025, # 0.025 corresponds to 1 degree per 40 cm seconds
    kd = -0.025, # 0.025 corresponds to 1 degree per 40 cm per second

    # Although the angle of the robot will never realistically be more than a
    # few degrees, let the PID controller's output be a lot larger to give
    # plenty of margin
    max_output = 90
)

# Loop forever, allowing the user to stand up the robot and start balancing
# over and over again
while(True):
    # Inform user to stand up the robot
    print("Ready, stand up robot!")
    board.set_rgb_led(0,64,0)

    # Wait for robot to stand. The balancing point is roughly straight up, at
    # which point the z-axis of the accelerometer should be around 0. So we'll
    # wait until it's within 10 milli-g of zero to start balancing
    while(abs(imu.get_acc_z()) > 10):
        pass

    # Inform user that we're starting to balance
    print("Starting to balance!")
    board.set_rgb_led(0,0,64)

    # Get current pitch of robot. We know that we're close to the balancing
    # point, so this represents the approximate zero pitch angle of the robot
    angle_offset = imu.get_pitch()

    # Reset encoder values to zero, this is our initial target position to hold
    drivetrain.reset_encoder_position()

    # Reset PID controllers
    angle_pid.clear_history()
    position_pid.clear_history()

    # Reset other values
    target_position = 0
    target_speed = 0
    turn_effort = 0
    
    # Reset loop timer
    last_loop_ticks_ms = time.ticks_ms()

    # Balance loop
    while(True):
        # Wait until the loop period has passed to ensure the loop runs at a
        # consistent rate
        if(time.ticks_ms() < last_loop_ticks_ms + (loop_period * 1000)):
            continue

        # Update loop timer
        last_loop_ticks_ms += loop_period * 1000

        # Update sensor values
        angle = imu.get_pitch() - angle_offset
        left_position = drivetrain.get_left_encoder_position()
        right_position = drivetrain.get_right_encoder_position()

        # Compute robot position by averaging the left and right wheel positions
        bot_position = (left_position + right_position) / 2

        # Update target position based on target velocity
        target_position -= target_speed * loop_period

        # Compute target angle from robot's position with the PID controller
        targetAngle = position_pid.update(target_position - bot_position)

        # Compute forward effort from robot's angle with the PID controller
        forwardEffort = angle_pid.update(targetAngle - angle)

        # Set motor effort
        drivetrain.arcade(forwardEffort, turn_effort)

        # Check the angle and forward effort. If either is too large, the robot
        # has likely fallen over, so stop the motors
        if(abs(angle) > 45 or abs(forwardEffort) > 5):
            print("Effort too large, stopping!")
            drivetrain.stop()
            break
