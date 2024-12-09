#!/usr/bin/env pybricks-micropython

# Import the necessary libraries
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor,
    TouchSensor,
    ColorSensor,
    UltrasonicSensor
)
from pybricks.parameters import Port, Button, Stop, Direction
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import _thread

# Initialize devices
ev3 = EV3Brick()
left_motor = Motor(Port.A, positive_direction=Direction.COUNTERCLOCKWISE)
right_motor = Motor(Port.D, positive_direction=Direction.COUNTERCLOCKWISE)
robot = DriveBase(left_motor, right_motor, wheel_diameter=55, axle_track=200)
robot.settings(straight_speed=200, straight_acceleration=100, turn_rate=100)

color_sensor_in1 = ColorSensor(Port.S4)
obstacle_sensor = UltrasonicSensor(Port.S1)
touch_sensor1 = TouchSensor(Port.S2)
pickup_motor = Motor(Port.C)

# Calibration values (adjust based on your testing)
"""BLACK_THRESHOLD = 10
WHITE_THRESHOLD = 95
MIDPOINT = (BLACK_THRESHOLD + WHITE_THRESHOLD) // 2
SHARP_TURN_THRESHOLD = 20  # Adjust for sharper turns

# Driving speeds
BASE_SPEED = 150  # Speed for driving forward
TURN_GAIN = 1.5   # Gain factor for turning adjustment
SHARP_TURN_GAIN = 2.5  # Stronger gain for sharp turns

# Line-following logic
def follow_line():
    while True:
        # Read sensor value
        reflected_light = color_sensor_in1.reflection()
        error = reflected_light - MIDPOINT

        if reflected_light < BLACK_THRESHOLD + SHARP_TURN_THRESHOLD:
            # Sharp turn: Reverse right motor for sharper steering
            left_motor.run(BASE_SPEED)
            right_motor.run(-BASE_SPEED)
        else:
            # Proportional control for smoother steering
            turn_rate = error * TURN_GAIN
            left_motor.run(BASE_SPEED - turn_rate)
            right_motor.run(BASE_SPEED + turn_rate)

try:
    follow_line()
except KeyboardInterrupt:
    # Stop motors when interrupted
    left_motor.stop(Stop.BRAKE)
    right_motor.stop(Stop.BRAKE)"""

def pickupThread():
    while True:
        pickup_motor.run(-650)
        wait(2000)
        pickup_motor.run(650)
        wait(100)

ev3.speaker.beep()
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
    
_thread.start_new_thread(pickupThread, ())

# Calibration values (adjust based on testing)
BLACK_THRESHOLD = 10
WHITE_THRESHOLD = 95
MIDPOINT = (BLACK_THRESHOLD + WHITE_THRESHOLD) // 2
SHARP_TURN_THRESHOLD = 20  # Threshold for sharp turns

# PD controller constants (tune these values for your robot)
Kp = 1.9  # Proportional gain
Kd = 0.9  # Derivative gain

# Driving speed
BASE_SPEED = 200  # Base speed for motors
SHARP_TURN_SPEED = BASE_SPEED  # Speed for sharp turns

# Variables for PD control
last_error = 0  # Stores the previous error for derivative calculation

def stopAll():
    left_motor.stop()
    right_motor.stop()

def drive_straight_with_ultrasonic():
    desired_distance = 30  # Desired distance in millimeters (5 cm)
    # Measure the distance using the ultrasonic sensor
    current_distance = obstacle_sensor.distance()
    print(current_distance)
    if current_distance > desired_distance:
        # If the distance is greater than the desired distance, move forward
        robot.drive(100, 30)
    elif current_distance < desired_distance:
        robot.drive(100, -30)
    else:
        robot.drive(100, 0)

# Line-following logic with PD control and sharp turns
def follow_line():
    global last_error
    # Read the reflected light intensity
    reflected_light = color_sensor_in1.reflection()

    # Sharp turn detection
    if reflected_light < BLACK_THRESHOLD + SHARP_TURN_THRESHOLD:
        # Sharp turn: Reverse right motor for sharper steering
        left_motor.run(SHARP_TURN_SPEED)
        right_motor.run(-SHARP_TURN_SPEED)
    else:
        # Calculate the error (difference from midpoint)
        error = reflected_light - MIDPOINT

        # Calculate the derivative (rate of change of error)
        derivative = error - last_error

        # PD formula: Turn correction
        turn_rate = (Kp * error) + (Kd * derivative)

        # Update the last error
        last_error = error

        # Adjust motor speeds using the turn rate
        left_motor.run(BASE_SPEED - turn_rate)
        right_motor.run(BASE_SPEED + turn_rate)


while not touch_sensor1.pressed():
    follow_line()
    wait(5)
stopAll()
# zacouvání od stěny a otočka směrem k 3. čáře
ev3.speaker.beep(500, 500)
robot.drive(-100,0)
wait(50)
robot.turn(-80)
ev3.speaker.beep(500, 500)
robot.stop()
while color_sensor_in1.reflection() > 15:
    drive_straight_with_ultrasonic()
    wait(5)
robot.drive(100, 0)
wait(200)
robot.stop()
robot.turn(-80)
robot.stop()
while not touch_sensor1.pressed():
    follow_line()
    wait(5)
stopAll()