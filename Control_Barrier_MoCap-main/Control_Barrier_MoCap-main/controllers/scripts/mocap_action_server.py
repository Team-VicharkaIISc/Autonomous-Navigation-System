#!/usr/bin/env python3

# ROS Node - Mocap Action Server

import rospy
import actionlib
from math import pow, atan2, sqrt

# from phasespace_msgs.msg import Rigids

from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

from phasespace_msgs.msg import Markers

from mocap_action.msg import mocapSimpleAction
from mocap_action.msg import mocapSimpleGoal
from mocap_action.msg import mocapSimpleResult
from mocap_action.msg import mocapSimpleFeedback

class MocapActionServer:
	
	# Constructor
	def __init__(self):

		# Initialize Simple Action Server
		self._sas = actionlib.SimpleActionServer('/mocap_simple',
												 mocapSimpleAction,
												 execute_cb=self.on_goal,
												 auto_start=False)
		'''
		* '/mocap_simple' - The name of the action that will be used by ROS Nodes to communicate with this Simple Action Server.
		* mocapSimpleAction - The Message Class that is used by ROS Actions internally for this Simple Action Server
		* execute_cb - Holds the function pointer to the function which will process incoming Goals from Simple Action Clients.
		* auto_start = False - Only when self._sas.start() will be called then only this Simple Action Server will start.
		'''

		self._turtle_vel_topic = '/cmd_vel'
		self._phasespace_markers_topic = '/phasespace/markers'

		self._distance_tolerance = 0.01

		self._vel_publisher = rospy.Publisher(self._turtle_vel_topic, Twist, queue_size=10)

		self._pose_subscriber = rospy.Subscriber(self._phasespace_markers_topic, Markers, self.phaseapce_markers_callback)
		# self._pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.turtle_pose_callback)

		self._curr_pose = Pose()
		self._curr_pose_z = 0
		self._rate = rospy.Rate(10)

		self._sas.start()
		rospy.loginfo("Started Mocap Action Server.")


	def phaseapce_markers_callback(self, msg):
		marker_dyn = msg.markers[1]
		marker_ori = msg.markers[0] 
		
		self._curr_pose.x = round(marker_dyn.x, 4)
		self._curr_pose.y = round(marker_dyn.y, 4)
		self._curr_pose_z = round(marker_dyn.z, 4)

		dy = marker_dyn.y - marker_ori.y
		dx = marker_dyn.x - marker_ori.x

		angle = round(atan2(dy, dx), 4)
		self._curr_pose.theta = angle
		# rospy.loginfo("Angle, {}".format(angle))


	# def turtle_pose_callback(self, msg):
	# 	self._curr_pose = msg
	# 	self._curr_pose.x = round(self._curr_pose.x, 4)
	# 	self._curr_pose.y = round(self._curr_pose.y, 4)


	def get_euclidean_distance(self, goal_pose):
		return sqrt(pow((goal_pose.x - self._curr_pose.x), 2) + pow((goal_pose.y - self._curr_pose.y), 2))

	def linear_vel(self, goal_pose, constant=1.5):
		return constant * self.get_euclidean_distance(goal_pose)

	def get_steering_angle(self, goal_pose):
		return atan2(goal_pose.y - self._curr_pose.y, goal_pose.x - self._curr_pose.x)

	def angular_vel(self, goal_pose, constant=4):
		return constant * (self.get_steering_angle(goal_pose) - self._curr_pose.theta)

	def go_to_point(self, goal_pose):

		vel_msg = Twist()

		feedback_msg = mocapSimpleFeedback()

		while not rospy.is_shutdown():

			feedback_msg.x = self._curr_pose.x
			feedback_msg.y = self._curr_pose.y
			feedback_msg.angle = self._curr_pose_z

			self._sas.publish_feedback(feedback_msg)

			if (self.get_euclidean_distance(goal_pose) >= self._distance_tolerance):
				# Linear velocity in the x-axis.
				vel_msg.linear.x = self.linear_vel(goal_pose)
				# vel_msg.linear.y = self.linear_vel(goal_pose)
				vel_msg.linear.y = 0
				vel_msg.linear.z = 0

				# Angular velocity in the z-axis.
				vel_msg.angular.x = 0
				vel_msg.angular.y = 0 
				vel_msg.angular.z = self.angular_vel(goal_pose)

				self._vel_publisher.publish(vel_msg)

				self._rate.sleep()
			else:
				break

		# Stop the robot after desired distance is covered
		vel_msg.linear.x = 0
		vel_msg.angular.z = 0
		self._vel_publisher.publish(vel_msg)
		rospy.loginfo('Destination Reached..')

	#-------------------------------------------------------
	# Function to process Goals and send Results
	def on_goal(self, goal_pose):
		rospy.loginfo("Received a Goal from Client.")
		rospy.loginfo("\033[93mGoal Pose: X, Y = {}, {}\033[00m".format(goal_pose.x, goal_pose.y))

		# --- Goal Processing Section ---
		self.go_to_point(goal_pose)

		# Send Result to the Client
		result_msg = mocapSimpleResult()
		result_msg.x = self._curr_pose.x
		result_msg.y = self._curr_pose.y
		result_msg.angle = self._curr_pose_z

		rospy.loginfo("\033[92mGoal Completed, Sending Results..\033[00m")
		self._sas.set_succeeded(result_msg)

# Main Function
def main():
	# 1. Initialize ROS Node
	rospy.init_node('mocap_action_server')

	# 2. Create Mocap Action Server object.
	action_server = MocapActionServer()

	# 3. Do not exit and loop forever.
	rospy.spin() 


if __name__ == '__main__':
	main()