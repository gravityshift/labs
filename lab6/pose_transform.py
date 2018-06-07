#!/usr/bin/env python3

'''
This is starter code for Lab 6 on Coordinate Frame transforms.

'''

import asyncio
import math
from math import cos, sin, radians
import time

import cozmo
from cozmo.util import degrees

def get_relative_pose(object_pose, reference_frame_pose):
	# ####
	# TODO: Implement computation of the relative frame using numpy.
	# Try to derive the equations yourself and verify by looking at
	# the books or slides before implementing.
	# ####
	robot_pose = (reference_frame_pose.position.x, reference_frame_pose.position.y, reference_frame_pose.rotation.angle_z.degrees)
	cube_pose = (object_pose.position.x, object_pose.position.y, object_pose.rotation.angle_z.degrees)

	x_diff = cube_pose[0] - robot_pose[0]
	y_diff = cube_pose[1] - robot_pose[1]
	z_diff = cube_pose[2] - robot_pose[2]

	robot_cos = cos(radians(robot_pose[2]))
	robot_sin = sin(radians(robot_pose[2]))
	cube_cos = cos(radians(cube_pose[2]))
	cube_sin = sin(radians(cube_pose[2]))

	new_x = x_diff*robot_cos + y_diff*robot_sin
	new_y = y_diff*robot_cos - x_diff*robot_sin

	return cozmo.util.Pose(new_x, new_y, 0, angle_z=degrees(z_diff)) #0 for 3rd position dimension, since we're ignoring

def find_relative_cube_pose(robot: cozmo.robot.Robot):
	'''Looks for a cube while sitting still, prints the pose of the detected cube
	in world coordinate frame and relative to the robot coordinate frame.'''

	robot.move_lift(-3)
	robot.set_head_angle(degrees(0)).wait_for_completed()
	cube = None

	while True:
		time.sleep(1)
		try:
			cube = robot.world.wait_for_observed_light_cube(timeout=30)
			if cube:
				print("Robot pose: %s" % robot.pose)
				print("Cube pose: %s" % cube.pose)
				print("Cube pose in the robot coordinate frame: %s" % get_relative_pose(cube.pose, robot.pose))
		except asyncio.TimeoutError:
			print("Didn't find a cube")


if __name__ == '__main__':

	cozmo.run_program(find_relative_cube_pose)
