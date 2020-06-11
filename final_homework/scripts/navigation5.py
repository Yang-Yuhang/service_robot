#!/usr/bin/env python

"""

    RoboCup@Home Education | oc@robocupathomeedu.org
    navi.py - enable turtlebot to navigate to predefined waypoint location

"""

import rospy
import actionlib

from std_msgs.msg import String
from actionlib_msgs.msg import *
from geometry_msgs.msg import Pose, PoseWithCovarianceStamped, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler




class NavToPoint:
    def __init__(self):


        rospy.Subscriber('/navi_to_point', String, self.goto)
        
        rospy.on_shutdown(self.cleanup)
        
	# Subscribe to the move_base action server
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)

        #publish when reach destination
        self.navigation = rospy.Publisher("/navigation_feed_point", String, queue_size=10)

        rospy.loginfo("Waiting for move_base action server...")

        # Wait for the action server to become available
        self.move_base.wait_for_server(rospy.Duration(120))
        rospy.loginfo("Connected to move base server")
            
        rospy.loginfo("Ready")

	rospy.sleep(1)

	self.locations = dict()

	# Location Origin
	S_x = 0.01
	S_y = 0.01
	S_theta = 0
	quaternionS = quaternion_from_euler(0.0, 0.0, S_theta)
	self.locations['S'] = Pose(Point(S_x, S_y, 0.000), Quaternion(quaternionS[0], quaternionS[1], quaternionS[2], quaternionS[3]))

	# Location A(study)
	A_x = -2.62
	A_y = 2.94
	A_theta = 1.5708
	quaternionA = quaternion_from_euler(0.0, 0.0, A_theta)
	self.locations['A'] = Pose(Point(A_x, A_y, 0.000), Quaternion(quaternionA[0], quaternionA[1], quaternionA[2], quaternionA[3]))

        # Location B(bedroom)
	B_x = -2.74
	B_y = -0.448
	B_theta = 0
	quaternionB = quaternion_from_euler(0.0, 0.0, B_theta)
	self.locations['B'] = Pose(Point(B_x, B_y, 0.000), Quaternion(quaternionB[0], quaternionB[1], quaternionB[2], quaternionB[3]))

	# Location C(living room)
	C_x = 2.54
	C_y = 3.30
	C_theta = 0
	quaternionC = quaternion_from_euler(0.0, 0.0, C_theta)
	self.locations['C'] = Pose(Point(C_x, C_y, 0.000), Quaternion(quaternionC[0], quaternionC[1], quaternionC[2], quaternionC[3]))

	# Location D(kitchen)
	D_x = 3.62
	D_y = -0.523
	D_theta = 0
	quaternionD = quaternion_from_euler(0.0, 0.0, D_theta)
	self.locations['D'] = Pose(Point(D_x, D_y, 0.000), Quaternion(quaternionD[0], quaternionD[1], quaternionD[2], quaternionD[3]))

        # Location E(dining room)
	E_x = 0.993
	E_y = -1.13
	E_theta = 1.23
	quaternionE = quaternion_from_euler(0.0, 0.0, E_theta)
	self.locations['E'] = Pose(Point(E_x, E_y, 0.000), Quaternion(quaternionE[0], quaternionE[1], quaternionE[2], quaternionE[3]))

	self.goal = MoveBaseGoal()
        rospy.loginfo("Starting navigation test")



    def goto(self,data):
		
	self.goal.target_pose.header.frame_id = 'map'
	self.goal.target_pose.header.stamp = rospy.Time.now()


	# Robot will return to the origin
	if data.data == "go back":
	    rospy.loginfo("Going back to the origin")
	    rospy.sleep(2)
	    self.goal.target_pose.pose = self.locations['S']
	    self.move_base.send_goal(self.goal)
	    waiting = self.move_base.wait_for_result(rospy.Duration(300))
            if waiting == 1:
		rospy.loginfo("Returned")
		self.navigation.publish('Returned')
	# Robot will go to the study
	if data.data == "go to the study":
	    rospy.loginfo("Going to the study")
	    rospy.sleep(2)
	    self.goal.target_pose.pose = self.locations['A']
	    self.move_base.send_goal(self.goal)
	    waiting = self.move_base.wait_for_result(rospy.Duration(300))
            if waiting == 1:
		rospy.loginfo("Reached the study")
		self.navigation.publish('Reached the study')
        # Robot will go to the bedroom
	elif data.data=="go to the bedroom":
	    rospy.loginfo("Going to the bedroom")
	    rospy.sleep(2)
	    self.goal.target_pose.pose = self.locations['B']
	    self.move_base.send_goal(self.goal)
	    waiting = self.move_base.wait_for_result(rospy.Duration(300))
	    if waiting == 1:
	        rospy.loginfo("Reached the bedroom")
		self.navigation.publish('reached the bedroom')
        # Robot will go to the livingroom
	elif data.data == "go to the living room":
	    rospy.loginfo("Going to the living room")
	    rospy.sleep(2)
	    self.goal.target_pose.pose = self.locations['C']
	    self.move_base.send_goal(self.goal)
	    waiting = self.move_base.wait_for_result(rospy.Duration(300))
	    if waiting == 1:
		rospy.loginfo("Reached the living room")
		self.navigation.publish('reached the living room')
	# Robot will go to the kitchen
	elif data.data == "go to the kitchen":
	    rospy.loginfo("Going to the kitchen")
	    rospy.sleep(2)
	    self.goal.target_pose.pose = self.locations['D']
	    self.move_base.send_goal(self.goal)
	    waiting = self.move_base.wait_for_result(rospy.Duration(300))
	    if waiting == 1:
		rospy.loginfo("Reached the kitchen")
		self.navigation.publish('reached the kitchen')
	# Robot will go to the dining room
	elif data.data == "go to the dining room":
	    rospy.loginfo("Going to the dining room")
	    rospy.sleep(2)
	    self.goal.target_pose.pose = self.locations['E']
	    self.move_base.send_goal(self.goal)
	    waiting = self.move_base.wait_for_result(rospy.Duration(300))
	    if waiting == 1:
		rospy.loginfo("Reached the dining room")
		self.navigation.publish('reached the dining room')


    def cleanup(self):
        rospy.loginfo("Shutting down navigation	....")

	self.move_base.cancel_goal()


if __name__=="__main__":
    rospy.init_node('navi_point')
    try:
        NavToPoint()
        rospy.spin()
    except:
        pass
