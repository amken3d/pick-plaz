#from pick_plaz_robot import Robot

import math
from save_robot import SaveRobot
from random import random


robot = None
try:
    robot = SaveRobot("/dev/ttyUSB0")
except Exception as e:
    print(e)
#robot = SaveRobot("COM5")


def test1():
    robot.home()
    robot.light_topdn(True)
    robot.drive(0,0)
    robot.drive(400,0)
    robot.drive(400,400)
    robot.drive(0,400)

    n = 100
    r = 200
    for i in range(n):
        angle = 2*math.pi*i/n
        x = 200 + math.sin(angle) * r
        y = 200 + math.cos(angle) * r
        robot.drive(x=x, y=y)
        robot.light_topdn(False if i % 2 == 0 else True)

    robot.done()
    print("this line reached")
    robot.vacuum(True)
    robot.drive(10,10)
    robot.vacuum(False)
    robot.light_topdn(False)
    robot.steppers(False)

def test2():
    robot.home()
    robot.light_topdn(True)
    for i in range(20):
        x = 5+395*random()
        y = 5+395*random()
        robot.drive(x, y)
    robot.drive(5, 5)
    robot.light_topdn(False)
    robot.steppers(False)

def test3():
    """pick, rotate, place"""
    robot.valve(False)
    # robot.steppers(True)
    robot.home("y")

    for _ in range(2):

        robot.vacuum(True)
        robot.valve(False)
        robot.drive(x=100, e=0)
        robot.drive(z=-15)
        robot.valve(True)
        robot.drive(z=0)
        robot.drive(x=0, e=90)
        robot.drive(z=-15)
        robot.valve(False)
        robot.vacuum(False)
        robot.drive(z=0)

        robot.vacuum(True)
        robot.drive(x=0)
        robot.drive(z=-15)
        robot.valve(True)
        robot.drive(z=0)
        robot.drive(x=100, e=0)
        robot.drive(z=-15)
        robot.valve(False)
        robot.vacuum(False)
        robot.drive(z=0)

    robot.vacuum(False)
    robot.drive(z=0)
    robot.drive(x=0, e=0)
    robot.steppers(False)

def test4():
    import time
    robot.steppers(True)

    t = 1

    time.sleep(t)
    robot.drive(z=-15, f=10)
    robot.drive(z=0)
    robot.default_settings()
    robot.drive(z=-15)
    robot.drive(z=0)
    robot.steppers(False)

def test5():
    robot.steppers(True)
    robot.home()
    for i in range(10):
        robot.drive(100, 100)
        robot.drive(100, 140)
        robot.drive(140, 140)
        robot.drive(140, 100)
        robot.drive(100, 100)
        robot.drive(100, 140)
        robot.drive(140, 140)
        robot.drive(140, 100)
        robot.drive(100, 100, r=10)
        robot.drive(100, 140, r=10)
        robot.drive(140, 140, r=10)
        robot.drive(140, 100, r=10)
        robot.drive(100, 100, r=10)
        robot.drive(100, 140, r=10)
        robot.drive(140, 140, r=10)
        robot.drive(140, 100, r=10)
    robot.drive(100, 100)
    robot.drive(10, 10)
    robot.steppers(False)

def test6():
    robot.steppers(True)
    #robot.home()
    robot.feedrate_multiplier(x=1, y=1, z=1, e=1, a=1, b=1, c=1)  #revert 'apply_area_slowdown'
    for i in range(2):
        robot.drive(e=-360)
        robot.drive(z=0)
        robot.drive(e=0)
        robot.drive(z=-10)
    robot.default_settings()
    for i in range(2):
        robot.drive(e=-360)
        robot.drive(z=0)
        robot.drive(e=0)
        robot.drive(z=-10)
    robot.steppers(False)

test6()
