pi#!/usr/bin/env python3

import rospy
import numpy as np
from math import pow, atan2, sqrt
import time

from barrier_controller.transformations import *
from barrier_controller.barrier_certificates import *
from barrier_controller.misc import *
from barrier_controller.controllers import *

from phasespace_msgs.msg import Markers

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

from barrier_controller.msg import Robots
from barrier_controller.msg import Robot

curr_pose_1 = Pose2D()
curr_pose_2 = Pose2D()
curr_pose_3 = Pose2D()

curr_vel = np.zeros((2, 3))
curr_flags = np.zeros((3, 1))

def tbPoseCallback_1(msg):
	marker_dyn = msg.markers[0]
	marker_ori = msg.markers[1] 
		
	curr_pose_1.x = round(marker_dyn.x, 4)
	curr_pose_1.y = round(marker_dyn.y, 4)

	dy = marker_dyn.y - marker_ori.y
	dx = marker_dyn.x - marker_ori.x

	angle = round(atan2(dy, dx), 4)
	curr_pose_1.theta = angle

def tbPoseCallback_2(msg):
	marker_dyn = msg.markers[2]
	marker_ori = msg.markers[3] 
		
	curr_pose_2.x = round(marker_dyn.x, 4)
	curr_pose_2.y = round(marker_dyn.y, 4)

	dy = marker_dyn.y - marker_ori.y
	dx = marker_dyn.x - marker_ori.x

	angle = round(atan2(dy, dx), 4)
	curr_pose_2.theta = angle

def tbPoseCallback_3(msg):
	marker_dyn = msg.markers[4]
	marker_ori = msg.markers[5] 
		
	curr_pose_3.x = round(marker_dyn.x, 4)
	curr_pose_3.y = round(marker_dyn.y, 4)

	dy = marker_dyn.y - marker_ori.y
	dx = marker_dyn.x - marker_ori.x

	angle = round(atan2(dy, dx), 4)
	curr_pose_3.theta = angle

def turtleVelCallback_1(msg):
	curr_vel[:, 0] = msg.robots[0].u
	curr_vel[:, 1] = msg.robots[1].u
	curr_vel[:, 2] = msg.robots[2].u

	curr_flags[0] = msg.robots[0].flag
	curr_flags[1] = msg.robots[1].flag
	curr_flags[2] = msg.robots[2].flag

def get_velocities(ids, velocities, max_linear_velocity=0.22, max_angular_velocity=0.22):
	# Threshold linear velocities
	idxs = np.where(np.abs(velocities[0, :]) > max_linear_velocity)
	velocities[0, idxs] = max_linear_velocity*np.sign(velocities[0, idxs])

	# Threshold angular velocities
	idxs = np.where(np.abs(velocities[1, :]) > max_angular_velocity)
	velocities[1, idxs] = max_angular_velocity*np.sign(velocities[1, idxs])
	
	return velocities

def set_velocities(vel):
	vel_msg = Twist()

	vel_msg.linear.x = vel[0]
	vel_msg.linear.y = 0.0
	vel_msg.linear.z = 0.0
	vel_msg.angular.x = 0.0
	vel_msg.angular.y = 0.0
	vel_msg.angular.z = vel[1]

	return vel_msg

def stop_robot():
	vel_msg = Twist()

	vel_msg.linear.x = 0.0
	vel_msg.linear.y = 0.0
	vel_msg.linear.z = 0.0
	vel_msg.angular.x = 0.0
	vel_msg.angular.y = 0.0
	vel_msg.angular.z = 0.0

	return vel_msg

def main():
	rospy.init_node("barrier_uni")

	turtle1_sub = rospy.Subscriber('/phasespace/markers', Markers, tbPoseCallback_1)
	turtle2_sub = rospy.Subscriber('/phasespace/markers', Markers, tbPoseCallback_2)
	turtle3_sub = rospy.Subscriber('/phasespace/markers', Markers, tbPoseCallback_3)

	scots_vel = rospy.Subscriber('/robots', Robots, turtleVelCallback_1)

	turtle1_pub = rospy.Publisher('tb1/cmd_vel', Twist, queue_size=100)
	turtle2_pub = rospy.Publisher('tb2/cmd_vel', Twist, queue_size=100)
	turtle3_pub = rospy.Publisher('tb3/cmd_vel', Twist, queue_size=100)	

	# Instantiate Robotarium object
	N = 3

	# Define goal points by removing orientation from poses
	# goal_points = generate_initial_conditions(N, width=9.5, height=9.5)
	# goal_points = np.array([[3.2, 5.55], [1.5, 5.55], [0.4, 0.4]])
	goal_points = np.array([[0.35, 4.35, 2.5], [2.7, 2.7, 3.8], [0.4, 0.4, 0.4]])
	rospy.loginfo("Goal Points: {}".format(goal_points))

	# Create unicycle position controller
	unicycle_position_controller = create_clf_unicycle_position_controller()

	# Create barrier certificates to avoid collision
	uni_barrier_cert = create_unicycle_barrier_certificate(safety_radius=0.3, projection_distance=0.05)

	# define x initially
	curr_x = np.array([[curr_pose_1.x, curr_pose_2.x, curr_pose_3.x], [curr_pose_1.y, curr_pose_2.y, curr_pose_3.y], [curr_pose_1.theta, curr_pose_2.theta, curr_pose_3.theta]])

	rate = rospy.Rate(20)

	# While the number of robots at the required poses is less
	# than N...
	while not rospy.is_shutdown():

		# Get poses of agents
		curr_x = np.array([[curr_pose_1.x, curr_pose_2.x, curr_pose_3.x], [curr_pose_1.y, curr_pose_2.y, curr_pose_3.y], [curr_pose_1.theta, curr_pose_2.theta, curr_pose_3.theta]])


		# Create single-integrator control inputs
		# dxu = unicycle_position_controller(curr_x, goal_points[:2][:])

		# curr_vel[:, 1][0] = dxu[:, 1][0]
		# curr_vel[:, 1][1] = dxu[:, 1][1]

		# Create safe control inputs (i.e., no collisions)
		dxu = uni_barrier_cert(curr_vel, curr_x)

		# Get and Set the velocities by mapping the single-integrator inputs to unciycle inputs
		dxu = get_velocities(np.arange(N), dxu)

		turtle1_vel = set_velocities(dxu[:, 0])
		turtle2_vel = set_velocities(dxu[:, 1])
		turtle3_vel = set_velocities(dxu[:, 2])

		if(curr_flags[0]):
			turtle1_vel = stop_robot()

		if(curr_flags[1]):
			turtle2_vel = stop_robot()

		if(curr_flags[2]):
			turtle3_vel = stop_robot()

		if (np.size(at_pose(curr_x, goal_points, position_error=0.05, rotation_error=100)) == N) or (curr_flags[0] and curr_flags[1] and curr_flags[2]):
			turtle1_vel = stop_robot()
			turtle2_vel = stop_robot()
			turtle3_vel = stop_robot()
			break

		turtle1_pub.publish(turtle1_vel)
		turtle2_pub.publish(turtle2_vel)
		turtle3_pub.publish(turtle3_vel)

		# Iterate the simulation
		rate.sleep()


if __name__ == '__main__':
	main()