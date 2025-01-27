#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wallfollow_pkg.srv import FindWall
import actionlib
from std_srvs.srv import Empty, EmptyResponse
from wallfollow_pkg.msg import OdomRecordAction, OdomRecordGoal

class WallFollower:
    def __init__(self):
        rospy.init_node('wall_follower')
        global R
        R = 0  #for robot stopping
        # Subscribers and Publishers
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=30)
        self.odom_client = actionlib.SimpleActionClient('/record_odom', OdomRecordAction)
        self.odom_client.wait_for_server()
        self.stop_service = rospy.Service('stop_robot', Empty, self.handle_stop_robot)
        # Service Client
        rospy.wait_for_service('find_wall')
        try:
            self.find_wall = rospy.ServiceProxy('find_wall', FindWall)
            rospy.loginfo("Calling find_wall service...")
            resp = self.find_wall()
            if resp.wallfound:
                rospy.loginfo("Wall found and aligned. Starting wall-following behavior.")
                goal = OdomRecordGoal()
                self.odom_client.send_goal(goal)
                rospy.loginfo("Odom recording started.")


                # Initialize parameters and start wall following
                self.initialize_parameters()
                self.wall_follow()
            else:
                rospy.logwarn("Wall not found. Exiting.")
                rospy.signal_shutdown("Wall not found.")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)
            rospy.signal_shutdown("Service call failed")
        
        rospy.loginfo("Wall Follower Node Initialized")

    def initialize_parameters(self):
        # Wall-following parameters
        self.desired_distance_to_wall = 0.3  
        self.min_distance_to_wall = 0.2      
        self.max_distance_to_wall = 0.3      
        self.front_distance_threshold = 0.5  
        self.linear_speed = 0.07             # Move forward speed
        self.angular_speed = 0.7             # Angular speed for corrections
        
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

    def laser_callback(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def rotate_in_place(self, direction, duration):
        cmd = Twist()
        cmd.angular.z = -self.angular_speed if direction == 'right' else self.angular_speed
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(duration)
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def wall_follow(self):
        rospy.loginfo("Starting wall-following behavior.")
        while not rospy.is_shutdown() and R == 0:
            if not self.ranges:
                rospy.sleep(1)
                continue

            right_angle = -1.57  # -90 degrees in radians
            front_angle = 0.0    # 0 degrees in radians

            right_index = int((right_angle - self.angle_min) / self.angle_increment)
            front_index = int((front_angle - self.angle_min) / self.angle_increment)

            if right_index < 0 or right_index >= len(self.ranges):
                rospy.logwarn("Right index out of range, using closest valid index.")
                right_index = max(0, min(len(self.ranges) - 1, right_index))
            
            if front_index < 0 or front_index >= len(self.ranges):
                rospy.logwarn("Front index out of range, using closest valid index.")
                front_index = max(0, min(len(self.ranges) - 1, front_index))

            right_distance = self.ranges[right_index]
            front_distance = self.ranges[front_index]
            cmd = Twist()

            # Wall following logic
            if front_distance < self.front_distance_threshold:
                cmd.linear.x = self.linear_speed
                cmd.angular.z = self.angular_speed
                rospy.loginfo("Wall crossing detected. Turning left.")
            else:
                if right_distance > self.max_distance_to_wall:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = -self.angular_speed
                    rospy.loginfo("Approaching the wall.")
                elif right_distance < self.min_distance_to_wall:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = self.angular_speed
                    rospy.loginfo("Moving away from the wall.")
                else:
                    cmd.linear.x = self.linear_speed
                    cmd.angular.z = 0.0
                    rospy.loginfo("Maintaining distance to the wall.")
            
            self.cmd_vel_pub.publish(cmd)
            rospy.Rate(10).sleep()  # Adjust rate 
    def handle_stop_robot(self, req):
        rospy.loginfo("Stop request received. Stopping the robot.")
        self.stop_requested = True
        # Stop the robot 
        self.stop_robot()
        return EmptyResponse()
    def stop_robot(self):
        # Publish zero velocities to stop the robot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        rospy.loginfo("Robot has stopped.")
        global R
        R = 1


if __name__ == '__main__':
    try:
        wall_follower = WallFollower()
    except rospy.ROSInterruptException:
        pass
