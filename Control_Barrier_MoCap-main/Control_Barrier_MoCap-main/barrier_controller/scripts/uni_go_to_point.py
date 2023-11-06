#!/usr/bin/env python3

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


class ControlBarrier:
	"""docstring for ControlBarrier"""
	def __init__(self, number_of_robots=3, max_linear_velocity=0.22, max_angular_velocity=0.22):
		
		self.N = number_of_robots
		self.max_linear_velocity = max_linear_velocity
		self.max_angular_velocity = max_angular_velocity

		self.phasespace_topic = "/phasespace/markers"
		self.robot_vel_topic = "/robots"
		self.turtle_vel_topics = np.array(["/tb1/cmd_vel", "/tb2/cmd_vel", "/tb3/cmd_vel"])

		self.pose_handler = None
		self.vel_handler = None
		self.pub_handlers = np.array([None] * self.N)
		
		self.curr_pose = np.zeros((3, self.N))
		self.curr_vel = np.zeros((2, self.N))
		self.curr_flags = np.zeros((self.N, 1))

		self.vel_msgs = np.array([None] * self.N)

		self.markers_ids = np.array([[0, 1], [2, 3], [4, 5]])

		self.init_callbacks()

		self.rate = rospy.Rate(20)

	def init_callbacks(self):
		self.pose_handler = rospy.Subscriber(self.phasespace_topic, Markers, self.pose_callback)
		self.vel_handler = rospy.Subscriber(self.robot_vel_topic, Robots, self.vel_callback)

		for i in range(self.N):
			self.pub_handlers[i] = rospy.Publisher(self.turtle_vel_topics[i], Twist, queue_size=100)

	def pose_callback(self, msg):
		for i in range(self.N):
			dyn_id = self.markers_ids[i][0]
			ori_id = self.markers_ids[i][1]

			marker_dyn = msg.markers[dyn_id]
			marker_ori = msg.markers[ori_id]

			self.curr_pose[:, i][0] = round(marker_dyn.x, 4)
			self.curr_pose[:, i][1] = round(marker_dyn.y, 4)

			dy = marker_dyn.y - marker_ori.y
			dx = marker_dyn.x - marker_ori.x

			angle = round(atan2(dy, dx), 4)
			self.curr_pose[:, i][2] = angle

	def vel_callback(self, msg):
		for i in range(self.N):
			self.curr_vel[:, i] = msg.robots[i].u
			self.curr_flags[i] = msg.robots[i].flag

	def get_threshold_velocities(self, ids, velocities):
		# Thresholding the linear velocities
		idxs = np.where(np.abs(velocities[0, :]) > self.max_linear_velocity)
		velocities[0, idxs] = self.max_linear_velocity * np.sign(velocities[0, idxs])

		# Thresholding the angular velocities
		idxs = np.where(np.abs(velocities[1, :]) > self.max_angular_velocity)
		velocities[1, idxs] = self.max_angular_velocity * np.sign(velocities[1, idxs])
		
		return velocities

	def get_vel_msg(self, velocities):
		vel_msg = Twist()

		vel_msg.linear.x = velocities[0]
		vel_msg.linear.y = 0.0
		vel_msg.linear.z = 0.0
		vel_msg.angular.x = 0.0
		vel_msg.angular.y = 0.0
		vel_msg.angular.z = velocities[1]

		return vel_msg

	def run_controller(self, barrier_certificate, goal_points, position_error=0.05, rotation_error=100):
		assert goal_points.shape[1] == self.N, "Goal points size should be (%r, %r)" % (3, self.N)

		rospy.loginfo("Goal Points, {}.".format(goal_points))

		while not rospy.is_shutdown():

			dxu = barrier_certificate(self.curr_vel, self.curr_pose)

			dxu = self.get_threshold_velocities(np.arange(self.N), dxu)

			for i in range(self.N):
				self.vel_msgs[i] = self.get_vel_msg(dxu[:, i])

				if(self.curr_flags[i]):
					self.vel_msgs[i] = self.get_vel_msg(np.array([0.0, 0.0]))

				self.pub_handlers[i].publish(self.vel_msgs[i])
			
			if(np.size(at_pose(self.curr_pose, goal_points, position_error=position_error, rotation_error=rotation_error)) == self.N) or ((self.curr_flags).all()):
				break

			self.rate.sleep()

def main():
	rospy.init_node('barrier_node')

	controller_barrier = ControlBarrier(number_of_robots=3, max_linear_velocity=0.22, max_angular_velocity=0.11)

	barrier_certificate = create_unicycle_barrier_certificate(safety_radius=0.3)

	goal_points = np.array([[0.35, 4.35, 2.5], [2.7, 2.7, 3.8], [0.4, 0.4, 0.4]])

	rospy.loginfo("Waiting for 5 seconds.")
	rospy.sleep(5)

	controller_barrier.run_controller(barrier_certificate, goal_points, position_error=0.05, rotation_error=100)


if __name__ == '__main__':
	main()