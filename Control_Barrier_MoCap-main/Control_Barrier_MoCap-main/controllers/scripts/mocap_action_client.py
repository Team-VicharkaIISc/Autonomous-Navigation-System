#!/usr/bin/env python3

import rospy
import actionlib
import time
import math
import numpy as np

# from phasespace_msgs.msg import Rigids
from turtlesim.msg import Pose

from mocap_action.msg import mocapSimpleAction
from mocap_action.msg import mocapSimpleGoal
from mocap_action.msg import mocapSimpleResult
from mocap_action.msg import mocapSimpleFeedback

class MocapActionClient:

	# Constructor
	def __init__(self):

		self._ac = actionlib.SimpleActionClient('/mocap_simple', mocapSimpleAction)

		self._ac.wait_for_server()

		rospy.loginfo("Mocap Action server is up, we can send new goals!")

	def send_goal(self, arg_x, arg_y):
		
		# Create Goal message for Mpcap Action Server
		goal = mocapSimpleGoal(x=arg_x, y=arg_y)
		
		'''
			* done_cb is set to the function pointer of the function which should be called once 
				the Goal is processed by the Mocap Action Server.

			* feedback_cb is set to the function pointer of the function which should be called while
				the goal is being processed by the Mocap Action Server.
		''' 
		self._ac.send_goal(goal, done_cb=self.done_callback, feedback_cb=self.feedback_callback)

		# rospy.loginfo("Goal has been sent.")

	# Function print result on Goal completion
	def done_callback(self, status, result):
		# rospy.loginfo("[Status] : " + str(status))
		# rospy.loginfo("[Result] : ({}, {}, {})".format(result.x, result.y, result.angle))
		print("({}, {}, {})".format(result.x, result.y, result.angle))

	# Function to print feedback while Goal is being processed
	def feedback_callback(self, feedback):
		rospy.loginfo("[Feedback] : ({}, {}, {})".format(feedback.x, feedback.y, feedback.angle))

# Main Function
def main():
	
	rospy.init_node('mocap_action_client')

	action_client = MocapActionClient()

	# goal_points = list()

	x = np.linspace(0, 6.05, 11)
	y = np.linspace(0, 0.605 * 8, 8)

	# for i in x:
	# 	for j in y:
	# 		temp = (round(i, 4), round(j, 4))
	# 		goal_points.append(temp)

	goal_points = [(1, 1), (0, 4.84)]

	for point in goal_points:
		action_client.send_goal(point[0], point[1])
		action_client._ac.wait_for_result()
		rospy.sleep(1)

	# rospy.spin()


if __name__ == '__main__':
	main()