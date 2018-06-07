#!/usr/bin/env python3

'''
Stater code for Lab 7.

'''

import cozmo
from cozmo.util import degrees, Angle, Pose, distance_mm, speed_mmps
import math
import time
import numpy

# Wrappers for existing Cozmo navigation functions

def cozmo_drive_straight(robot, dist, speed):
    """Drives the robot straight.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        dist -- Desired distance of the movement in millimeters
        speed -- Desired speed of the movement in millimeters per second
    """
    robot.drive_straight(distance_mm(dist), speed_mmps(speed)).wait_for_completed()

def cozmo_turn_in_place(robot, angle, speed):
    """Rotates the robot in place.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        angle -- Desired distance of the movement in degrees
        speed -- Desired speed of the movement in degrees per second
    """
    robot.turn_in_place(degrees(angle), speed=degrees(speed)).wait_for_completed()

def cozmo_go_to_pose(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    robot.go_to_pose(Pose(x, y, 0, angle_z=degrees(angle_z)), relative_to_robot=True).wait_for_completed()

# Functions to be defined as part of the labs

def get_front_wheel_radius():
    """Returns the radius of the Cozmo robot's front wheel in millimeters."""

    # I used the three notches located around the rim of the wheel as reference points.
    # I selected one of the notches arbitrarily and put a piece of tape over it to identify it clearly.
    # Orienting this notch straight down, I played with drive_straight until I got that notch to complete
    # exactly one full rotation. This ended up being approximately 86 mm.
    # Since 86 mm is the circumference of the wheel, the radius will be 86/(2*Pi) mm
    return 86 / (2 * math.pi)

def get_distance_between_wheels():
    """Returns the distance between the wheels of the Cozmo robot in millimeters."""

    # Using drive_wheels, I set the left wheel to 100 mmps and the right wheel to -100 mmps.
    # Using these parameters, it took 3.3 seconds to visually complete one full 360 degree rotation.
    # This means that each wheel travelled 330 mm.
    # If the circumference of the circle is 330 mm, then the radius of the circle is 330/(2*Pi)
    return 330 / (2 * math.pi)

def rotate_front_wheel(robot, angle_deg):
    """Rotates the front wheel of the robot by a desired angle.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        angle_deg -- Desired rotation of the wheel in degrees
    """
    front_wheel_circum = get_front_wheel_radius() * 2 * math.pi
    mm_per_deg = front_wheel_circum / 360
    mm = angle_deg * mm_per_deg
    robot.drive_straight(distance_mm(mm), speed_mmps(20)).wait_for_completed()

def my_drive_straight(robot, dist, speed):
    """Drives the robot straight.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        dist -- Desired distance of the movement in millimeters
        speed -- Desired speed of the movement in millimeters per second
    """
    t = (1/speed) * numpy.abs(dist) + 0.6 # Constant offset to make up for lack of accuracy
    speed = speed if dist > 0 else -speed
    robot.drive_wheels(speed, speed, duration=t)

def my_turn_in_place(robot, angle, speed):
    """Rotates the robot in place.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        angle -- Desired distance of the movement in degrees
        speed -- Desired speed of the movement in degrees per second
    """

    t = (1/speed) * numpy.abs(angle)

    circum = 2 * math.pi * get_distance_between_wheels()
    arc_length = (numpy.abs(angle)/360) * circum
    mm_speed = arc_length / t
    mm_speed = mm_speed if angle>0 else -mm_speed

    robot.drive_wheels(-mm_speed, mm_speed, duration=t)

def my_go_to_pose1(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # Assuming positive y is to the robot's left and positive x is the direction the robot is already facing
    hypotenuse = numpy.sqrt(x*x + y*y)
    angle_offset_of_target_point = numpy.arcsin(y/hypotenuse)*180/math.pi
    my_turn_in_place(robot, angle_offset_of_target_point , 30)
    my_drive_straight(robot, hypotenuse, 50)
    my_turn_in_place(robot, angle_z-angle_offset_of_target_point, 30)
    time.sleep(1)

def my_go_to_pose2(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    # ####
    # TODO: Implement a function that makes the robot move to a desired pose
    # using the robot.drive_wheels() function to jointly move and rotate the 
    # robot to reduce distance between current and desired pose (Approach 2).
    # ####
    pass

def my_go_to_pose3(robot, x, y, angle_z):
    """Moves the robot to a pose relative to its current pose.
        Arguments:
        robot -- the Cozmo robot instance passed to the function
        x,y -- Desired position of the robot in millimeters
        angle_z -- Desired rotation of the robot around the vertical axis in degrees
    """
    if(numpy.abs(angle_z)>90):
        my_turn_in_place(robot, angle_z, 30)
        my_go_to_pose2(robot, x, y, 0)
    else:
        my_go_to_pose2(robot, x, y, angle_z)

def run(robot: cozmo.robot.Robot):

    print("***** Front wheel radius: " + str(get_front_wheel_radius()))
    print("***** Distance between wheels: " + str(get_distance_between_wheels()))

    ## Example tests of the functions

    """cozmo_drive_straight(robot, 62, 50)
    cozmo_turn_in_place(robot, 60, 30)
    cozmo_go_to_pose(robot, 100, 100, 45)

    rotate_front_wheel(robot, 90)
    my_drive_straight(robot, 62, 50)
    my_turn_in_place(robot, 90, 30)

    my_go_to_pose1(robot, 100, 100, 45)
    my_go_to_pose2(robot, 100, 100, 45)
    my_go_to_pose3(robot, 100, 100, 45)"""

    my_go_to_pose1(robot, 100, 100, 180)


if __name__ == '__main__':

    cozmo.run_program(run)



