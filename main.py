#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (
    Motor,
    TouchSensor,
    ColorSensor,
    UltrasonicSensor
)
from pybricks.parameters import Port, Button
from pybricks.tools import wait
from pybricks.robotics import DriveBase
import _thread

left_motor = Motor(Port.A)
right_motor = Motor(Port.D)
pickup_motor = Motor(Port.C)
ev3 = EV3Brick()

color_sensor = ColorSensor(Port.S1)
ultrasonic_sensor = UltrasonicSensor(Port.S3)
touch_sensor1 = TouchSensor(Port.S2)

wheel_diameter = 55  # whell diam in mm
axle_track = 200  # distance between wheels in mm
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
robot.settings(100, 100, 100, 100)
ev3.speaker.beep()

def pickupThread():
    while True:
        pickup_motor.run(-650)
        wait(2000)
        pickup_motor.run(650)
        wait(100)

robot.reset()

white_intensity = 75
gray_intensity = 65

def followBlack():
    intensity = color_sensor.reflection()
    if intensity > white_intensity:
        robot.drive(100, -15)
    else:
        robot.drive(100, (gray_intensity - intensity) * 1.5)

while Button.CENTER not in ev3.buttons.pressed():
    wait(10)

_thread.start_new_thread(pickupThread, ())

while robot.distance() < 1000:
    followBlack()
    wait(5)
while not touch_sensor1.pressed():
    followBlack()
    wait(5)
robot.stop()
robot.drive(-100,0)
wait(500)
robot.turn(-70)
while ultrasonic_sensor.distance() < 495:
    robot.drive(100, 0)
    wait(5)
robot.stop()
robot.turn(-70)
while not touch_sensor1.pressed():
    followBlack()
    wait(5)
robot.stop()
robot.drive(-100, 0)
wait(500)
robot.turn(-40)
wait(2000)
robot.turn(40)
wait(1000)
robot.drive(-100, -30)
wait(2500)
robot.drive(-100, -8)
wait(1000)
while ultrasonic_sensor.distance() > 40:
    robot.drive(-100, 0)
    wait(10)
robot.drive(-100, 0)
wait(500)
robot.drive(100, 0)
wait(300)
robot.drive(-100, 0)
wait(300)
robot.drive(100, 0)
wait(300)
robot.stop()