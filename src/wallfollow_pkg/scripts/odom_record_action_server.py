#!/usr/bin/env python

import rospy
import actionlib
from wallfollow_pkg.msg import OdomRecordAction, OdomRecordFeedback, OdomRecordResult
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point32
import math
from math import sqrt
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class OdomRecordActionServer:

    def __init__(self):
        # Initialize the action server
        self._as = actionlib.SimpleActionServer("/record_odom", OdomRecordAction, self.execute_cb, False)
        self._as.start()
        # Subscribe to odometry topic
        self.odom_sub = rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Initialize variables
        self.odom_list = []
        self.prev_x = None
        self.prev_y = None
        self.start_x = None
        self.start_y = None
        self.total_distance = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.full_circle_tolerance = 0.3  # Tolerance for returning to start position

        self.start_position = None  # For lap tracking

    def odom_cb(self, msg):
        # Update the current position from odometry
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

    def calculate_distance(self, x1, y1, x2, y2):
        # Calculate the Euclidean distance between two points
        return sqrt((x2 - x1)**2 + (y2 - y1)**2)
    
    def execute_cb(self, goal):
        r = rospy.Rate(1)  # 1 Hz
        success = True

        # Save the starting position
        if self.start_x is None and self.start_y is None:
            self.start_x = self.current_x
            self.start_y = self.current_y

        # Record odometry and check for lap completion
        while not rospy.is_shutdown():
            if self.prev_x is not None and self.prev_y is not None:
                # Calculate the distance traveled since the last position update
                dist = self.calculate_distance(self.prev_x, self.prev_y, self.current_x, self.current_y)
                self.total_distance += dist

                # Record the current odometry point
                point = Point32()
                point.x = self.current_x
                point.y = self.current_y
                self.odom_list.append(point)

            # Update previous positions
            self.prev_x = self.current_x
            self.prev_y = self.current_y

            # Provide feedback to the client
            feedback = OdomRecordFeedback()
            feedback.current_total = self.total_distance
            rospy.loginfo("Current total distance: %.2f meters", self.total_distance)
            self._as.publish_feedback(feedback)

            # Check if a lap has been completed
            if self.is_lap_complete():
                # Prepare the result message
                result = OdomRecordResult()
                result.list_of_odoms = self.odom_list
    
                # Set the action as succeeded
                self._as.set_succeeded(result)
    
                # Log the odometries
                rospy.loginfo("Lap completed! Odometries recorded:")
                for odom in self.odom_list:
                     rospy.loginfo("x: %f, y: %f", odom.x, odom.y)
    
                rospy.loginfo("Robot stopped.")
                self.send_stop_request()
                # Optional: Shut down the action server if needed
                rospy.loginfo("Shutting down the action server...")
                self._as.preempt_request = True  # This flags a preemption request to stop the server
                break  # Exit the loop


            r.sleep()
    def send_stop_request(self):
        rospy.wait_for_service('stop_robot')
        try:
            stop_service = rospy.ServiceProxy('stop_robot', Empty)
            stop_service()
            rospy.loginfo("Stop request sent to wall follower.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
    def is_lap_complete(self):
        #Determine if the robot has returned to the starting point within a threshold.
        if self.current_x is not None and self.current_y is not None:
            distance_to_start = self.calculate_distance(self.start_x, self.start_y, self.current_x, self.current_y)
            return distance_to_start <= self.full_circle_tolerance and self.total_distance > 1.0 
        return False

if __name__ == '__main__':
    rospy.init_node('odom_record_action_server')
    server = OdomRecordActionServer()
    rospy.spin()
