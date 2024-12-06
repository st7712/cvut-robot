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

left_motor = Motor(Port.D)
right_motor = Motor(Port.A)
pickup_motor = Motor(Port.C)
ev3 = EV3Brick()

color_sensor = ColorSensor(Port.S4)
ultrasonic_sensor = UltrasonicSensor(Port.S1)
touch_sensor1 = TouchSensor(Port.S2)

wheel_diameter = 55  # whell diam in mm
axle_track = 200  # distance between wheels in mm
robot = DriveBase(left_motor, right_motor, wheel_diameter, axle_track)
robot.settings(-100, -100, 100, 100)
ev3.speaker.beep()

def pickupThread():
    while True:
        pickup_motor.run(-650)
        wait(2000)
        pickup_motor.run(650)
        wait(100)

robot.reset()

white_intensity = 90
gray_intensity = 55

def followBlack(whiteAngle = -15):
    intensity = color_sensor.reflection()
    if intensity > white_intensity:
        robot.drive(-100, whiteAngle)
    else:
        robot.drive(-100, (gray_intensity - intensity) * 0.4)

def checkIfBlackCross(ultra_distance, driven_distance, color_intensity, max_ultra, max_driven, max_color):
    ultra = 0
    driven = 0 
    color = 0
    if ultra_distance > max_ultra:
        ultra = 1
    if driven_distance < max_driven:
        driven = 1
    if color_intensity < max_color:
        color = 1
    if ultra + driven + color >= 2:
        return False
    return True

while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
    
_thread.start_new_thread(pickupThread, ())


# 1. otočka
while not touch_sensor1.pressed():
    robot.drive(-100, 0)
    # wait(5)
robot.stop()
robot.drive(100, 0)
wait(150)
robot.turn(70)
robot.reset()
robot.drive(-100, 0)
# vedeni po 1. čáře
wait(2000)
while checkIfBlackCross(ultrasonic_sensor.distance(), robot.distance(), color_sensor.reflection(), 860, -830, 15):
    print("Driven: ", robot.distance())
    print("Distance: ", ultrasonic_sensor.distance())
    followBlack()
    # wait(5)
robot.stop()
ev3.speaker.beep(500, 500)
# otočka směrem k 1. spojce
robot.turn(90)
robot.reset()
# vedení po 1. spojce
while checkIfBlackCross(ultrasonic_sensor.distance(), robot.distance(), color_sensor.reflection(), 610, -230, 15):
    print("Driven: ", robot.distance())
    print("Distance: ", ultrasonic_sensor.distance())
    followBlack(-30)
    # wait(5)
robot.stop()
ev3.speaker.beep()
# otočka směrem k 2. čáře
robot.turn(80)
robot.reset()
# vedení po 2. čáře až do stěny
while not touch_sensor1.pressed():
    followBlack()
    # wait(5)
robot.stop()
# zacouvání od stěny a otočka směrem k 3. čáře
robot.drive(100,0)
wait(50)
robot.turn(-80)
# približování k 3. čáře
while ultrasonic_sensor.distance() < 590:
    print("Distance: ", ultrasonic_sensor.distance())
    robot.drive(-100, 0)
    # wait(5)
robot.stop()
# otočka směrem na 3. čáru
robot.turn(-85)
robot.reset()
# vedení po 3. čáře
while checkIfBlackCross(ultrasonic_sensor.distance(), robot.distance(), color_sensor.reflection(), 860, -810, 15):
    print("Driven: ", robot.distance())
    print("Distance: ", ultrasonic_sensor.distance())
    followBlack()
    # wait(5)
robot.stop()
ev3.speaker.beep()
# otočka směrem ke 2. spojce
robot.turn(70)
robot.reset()
# vedení po 2. spojce
while checkIfBlackCross(ultrasonic_sensor.distance(), robot.distance(), color_sensor.reflection(), 999999, -240, 15):
    print("Driven: ", robot.distance())
    print("Distance: ", ultrasonic_sensor.distance())
    followBlack()
    # wait(5)
robot.stop()
ev3.speaker.beep()
# otočka směrem ke 4. čáře
robot.turn(90)
robot.reset()
# vedení po 4. čáře až do stěny
while not touch_sensor1.pressed():
    followBlack()
    # wait(5)
robot.stop()
# manévr k 1. zásobníku
robot.drive(100,0)
wait(500)
robot.turn(-120)
robot.drive(100,-3)
wait(7000)
"""while not touch_sensor1.pressed():
    followBlack()
    # wait(5)
robot.stop()
robot.drive(100,0)
wait(500)
robot.turn(-70)
while ultrasonic_sensor.distance() < 495:
    robot.drive(-100, 0)
    # wait(5)
robot.stop()
robot.turn(-70)
while not touch_sensor1.pressed():
    followBlack()
    # wait(5)
robot.stop()
robot.drive(100, 0)
wait(500)
robot.turn(-40)
wait(2000)
robot.turn(40)
wait(1000)
robot.drive(100, -30)
wait(2500)
robot.drive(100, -8)
wait(1000)
while ultrasonic_sensor.distance() > 40:
    robot.drive(100, 0)
    wait(10)
robot.drive(100, 0)
wait(500)
robot.drive(-100, 0)
wait(300)
robot.drive(100, 0)
wait(300)
robot.drive(-100, 0)
wait(300)
robot.stop()"""