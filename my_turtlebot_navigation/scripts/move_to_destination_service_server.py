#! /usr/bin/env python

import rospy
import time
import actionlib
import os
import re
from my_turtlebot_localization.srv import MyServiceMessage, MyServiceMessageResponse
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

PENDING = 0
ACTIVE = 1
PREEMPTED=2
SUCCEEDED = 3

class MoveToSpot():

    def __init__(self):
        # Initialize all needed variables
        self._ss = rospy.Service('/move_to_spot', MyServiceMessage, self.move_to_spot)
        self.client = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.filename = "spots.txt"
        self.dir_name ="/home/user/catkin_ws/src/my_turtlebot_localization/spots"
        self.response = MyServiceMessageResponse()
        self.response.navigation_successful = False
        self.dest_pos = None
        self.rate = rospy.Rate(1)

    def move_to_spot(self, request):
        # Reset variables
        self.dest_pos = None
        self.response.navigation_successful = False
        self.response.message = ""
        # Connect to move base action server
        rospy.loginfo('Waiting for action Server /move_base')
        self.client.wait_for_server()
        rospy.loginfo('Action Server Found...')
        time.sleep(1)
        # Grab coordinate points for requested point from text file
        points_file = open(os.path.join(self.dir_name, self.filename),"r")
        file_lines = points_file.readlines()
        points_file.close()
        # Look for labeled point in text file. If it is there, grab coordinate points
        for i, line in enumerate(file_lines):
            line = line.rstrip()
            
            if line.lower() == request.label.lower():
                self.dest_pos = MoveBaseGoal()
                self.dest_pos.target_pose.header.frame_id = 'map'
                self.dest_pos.target_pose.pose.position.x = float(re.sub(':|x|y|z|w| ','',file_lines[i+10]))
                self.dest_pos.target_pose.pose.position.y = float(re.sub(':|x|y|z|w| ','',file_lines[i+11]))
                self.dest_pos.target_pose.pose.position.z = float(re.sub(':|x|y|z|w| ','',file_lines[i+12]))
                self.dest_pos.target_pose.pose.orientation.x = float(re.sub(':|x|y|z|w| ','',file_lines[i+14]))
                self.dest_pos.target_pose.pose.orientation.y = float(re.sub(':|x|y|z|w| ','',file_lines[i+15]))
                self.dest_pos.target_pose.pose.orientation.z = float(re.sub(':|x|y|z|w| ','',file_lines[i+16]))
                self.dest_pos.target_pose.pose.orientation.w = float(re.sub(':|x|y|z|w| ','',file_lines[i+17]))
                break

        # If specified point is not in file, return with failure
        if not self.dest_pos:
            output_string = f"Cannot find {request.label} in {self.dir_name}/{self.filename}"
            rospy.logerr(output_string)
            self.response.message = output_string
            return self.response

        # Send pose goal to move_base action server
        self.client.send_goal(self.dest_pos)
        state_result = self.client.get_state()
        
        rospy.loginfo("Waiting for robot to reach position...")
        while state_result < SUCCEEDED:
            self.rate.sleep()
            state_result = self.client.get_state()

        # Verify that robot succesfully reached destination position
        if state_result > SUCCEEDED:
            output_string = f"Robot could not reach {request.label}"
            rospy.logerr(output_string)
            self.response.message = output_string
        else:
            output_string = f"Robot succesfully reached {request.label}"
            rospy.loginfo(output_string)
            self.response.message = output_string
            self.response.navigation_successful = True
        
        return self.response


if __name__ == '__main__':
    # Start node
    rospy.init_node('move_to_spot')
    MoveToSpot()
    rospy.spin() # maintain the service open.

