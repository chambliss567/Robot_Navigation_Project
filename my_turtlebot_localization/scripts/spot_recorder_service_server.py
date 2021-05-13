#! /usr/bin/env python

import rospy
from my_turtlebot_localization.srv import MyServiceMessage, MyServiceMessageResponse
from geometry_msgs.msg import PoseWithCovarianceStamped
import math
import time
import os

class RecordLocation(object):

    def __init__(self):
        # Initialize all needed variables
        self._ss = rospy.Service('/save_spot', MyServiceMessage, self.save_spot)
        self.sub_amcl_pose = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.read_amcl_pose)
        self.positions = {}
        self.curr_position = PoseWithCovarianceStamped()
        self.filename = "spots.txt"
        self.dir_name ="/home/user/catkin_ws/src/my_turtlebot_localization/spots"
        self.response = MyServiceMessageResponse()

    def save_spot(self, request):
        # Wait until subscriber is connected to amcl_pose topic
        rospy.loginfo("Waiting until subscriber is connected to /amcl_pose topic")
        while self.sub_amcl_pose.get_num_connections() < 1 :
            time.sleep(1)
        time.sleep(1)
        rospy.loginfo("Subscriber is listening to topic")
        # Read request - If request is "end", return all recorded positions.
        # If request is other string, record position under string name
        if request.label.lower() != "end":
            self.positions[request.label] = self.curr_position
        else:
            # Create output txt file spots.txt
            os.makedirs(self.dir_name, exist_ok=True)
            output_file = open(os.path.join(self.dir_name, self.filename),"w")
            rospy.loginfo("Writing to output file " + self.dir_name + "/" + self.filename)
            output_message = ""
            for key in self.positions:
                output_file.write(key + '\n')
                output_file.write(str(self.positions[key]) + '\n')
                output_message = output_message + key + ", "
                rospy.loginfo("Position info: " + key + '\n' + str(self.positions[key]))
            # Return service succesful
            self.response.navigation_successful = True
            self.response.message = output_message[:-2] + " recorded in " + self.dir_name + "/" + self.filename
            rospy.loginfo(self.response.message)
            self.positions = {}
            return self.response

    def read_amcl_pose(self, amcle_pose):
        # Read the amcl_pose position
        self.curr_position = amcle_pose

if __name__ == '__main__':
    # Start node
    rospy.init_node('spot_recorder')
    RecordLocation()
    rospy.spin() # maintain the service open.
