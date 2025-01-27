#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from wallfollow_pkg.srv import FindWall, FindWallResponse

class FindWallNode:
    def __init__(self):
        rospy.init_node('find_wall')
        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=30)
        self.service = rospy.Service('find_wall', FindWall, self.handle_find_wall)

        # Define distance parameters
        self.min_distance_to_wall = 0.27  # 27 cm minimum distance
        self.max_distance_to_wall = 0.33  # 33 cm maximum distance
        self.max_distance_to_detect_wall = 60 # 60 cm , to look for a wall
        self.laser_ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

        rospy.loginfo("Find Wall Service Node Initialized")

    def laser_callback(self, msg):
        self.laser_ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def rotate_in_place(self, direction, duration):
        cmd = Twist()
        if direction == 'right':
            cmd.angular.z = -0.1  # Adjust this value for robot's turning speed
        elif direction == 'left':
            cmd.angular.z = 0.1  # Adjust this value for robot's turning speed
        self.cmd_vel_pub.publish(cmd)
        rospy.sleep(duration)
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)

    def align_with_wall(self, shortest_ray_index):
        cmd = Twist()
        # angle of the shortest laser ray
        shortest_ray_angle = self.angle_min + (shortest_ray_index * self.angle_increment)
        rospy.loginfo("Aligning with wall. Shortest ray angle: %f", shortest_ray_angle)

        # the range for the front-facing angle (-pi/2 to pi/2 radians)
        forward_min = -1.57  # -90 degrees
        forward_max = 1.57   # 90 degrees

        #Fine-tune alignment to face the wall within a small angle range
        angle_threshold_min = 0.5  # 
        angle_threshold_max = 0  # 
        angle_threshold = 0.05
        num_readings = len(self.laser_ranges)
        front_laser_index = num_readings // 2
        front_laser_angle = self.angle_min + (front_laser_index * self.angle_increment)
        rospy.loginfo("Front laser angle: %f", front_laser_angle)
        # Continue rotating until the robot is properly aligned with the wall
        while abs(shortest_ray_angle - front_laser_angle) > angle_threshold: 
            # Rotate in the appropriate direction based on the shortest ray angle
            angle_to_rotate = shortest_ray_angle - front_laser_angle
            if  angle_to_rotate > 0:
                cmd.angular.z = 0.2  # Rotate right
            else:
                cmd.angular.z = -0.2  # Rotate left
        
            self.cmd_vel_pub.publish(cmd)
            rospy.sleep(0.1)  # Adjust this for finer control

            # Update the shortest ray angle after rotating
            shortest_ray_index = self.laser_ranges.index(min(self.laser_ranges))
            shortest_ray_angle = self.angle_min + (shortest_ray_index * self.angle_increment)
            rospy.loginfo("New shortest ray angle during alignment: %f", shortest_ray_angle)
        # Stop the robot after alignment
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        rospy.loginfo("Robot aligned with wall.")

    def detect_wall(self):
        rospy.loginfo("Scanning for wall...")
        rate = rospy.Rate(1)
        timeout = 60  # Set a timeout for how long to try finding the wall
        start_time = rospy.get_time()

        while not rospy.is_shutdown():
            # Find the shortest laser ray
            if not self.laser_ranges:
                rospy.logwarn("Laser data is not available.")
                return False

            shortest_ray_index = self.laser_ranges.index(min(self.laser_ranges))
            shortest_ray_distance = self.laser_ranges[shortest_ray_index]

            rospy.loginfo("Shortest ray distance: %f", shortest_ray_distance)
            rospy.loginfo("Shortest ray angle: %f", self.angle_min + (shortest_ray_index * self.angle_increment))

            if shortest_ray_distance < self.max_distance_to_detect_wall:
                rospy.loginfo("Wall detected.")
                self.align_with_wall(shortest_ray_index)  # Align robot with wall
                return True

            # Rotate in place to try to detect the wall
            self.rotate_in_place('left', 0.5)  # Rotate left for 0.5 seconds
            rospy.sleep(0.5)  # Wait a bit before the next scan

            # Check for timeout to prevent infinite loop
            if rospy.get_time() - start_time > timeout:
                rospy.logwarn("Timeout reached, wall not detected.")
                return False
   

        rospy.loginfo("Alignment and rotation complete. Ready for wall-following.")
        return FindWallResponse(True)
    def rotate_90_degrees(self,n):
        if n == True :
            shortest_ray_index = self.laser_ranges.index(min(self.laser_ranges))
            shortest_ray_angle = self.angle_min + (shortest_ray_index * self.angle_increment)
            cmd = Twist()
            rospy.loginfo("Start Rotating Loop")
            print("1")
            while abs(shortest_ray_angle) <= 1.57 :
                cmd.angular.z = 0.2 #rotation speed
                rospy.loginfo("Rotating 90 degrees")
                print("2")
                self.cmd_vel_pub.publish(cmd)
                shortest_ray_index = self.laser_ranges.index(min(self.laser_ranges))
                shortest_ray_angle = self.angle_min + (shortest_ray_index * self.angle_increment)
                rospy.loginfo("Current angle to the wall: %f", shortest_ray_angle)
                rospy.sleep(0.2)
            cmd.angular.z = 0 # stop after rotation
        elif n == False:
            return

        rospy.loginfo("Stop Rotating Loop")
        print("2")
        try:
            self.cmd_vel_pub.publish(cmd)
        except:
            return

        cmd.angular.z = 0.0
        


        # Stop movement
    #    cmd.linear.x = 0.0
    #    cmd.angular.z = 0.0
    #    self.cmd_vel_pub.publish(cmd)

    #    return FindWallResponse(True)
    def handle_find_wall(self, req):
        rospy.loginfo("Find Wall Service Called")
        cmd = Twist()
        distant_correct = False
        # Wait until laser data is received
        while not rospy.is_shutdown() and not self.laser_ranges:
            rospy.loginfo("Waiting for laser scan data...")
            rospy.sleep(1)

        # Check for wall detection
        if not self.detect_wall():
            rospy.loginfo("Wall not detected. Exiting.")
            return FindWallResponse(False)

        rospy.loginfo("Adjusting distance to the wall.")

        # Define acceptable distance range in meters (29 cm to 31 cm)
        desired_min_distance = 0.29
        desired_max_distance = 0.31
        rate = rospy.Rate(10)  # Adjusting rate to 10 Hz for smoother control

        while not rospy.is_shutdown():
            # Get updated shortest ray distance
            # shortest_ray_distance = min(self.laser_ranges)
            num_rays = len(self.laser_ranges)
            front_ray_index = num_rays // 2
            front_ray_distance = self.laser_ranges[front_ray_index]
            # Adjust position relative to the wall
            if front_ray_distance < desired_min_distance:
                cmd.linear.x = -0.05  # Move backward to adjust distance
                rospy.loginfo(f"Too close to the wall: {front_ray_distance:.2f} m, moving back.")
            elif front_ray_distance > desired_max_distance:
                cmd.linear.x = 0.05  # Move forward to adjust distance
                rospy.loginfo(f"Too far from the wall: {front_ray_distance:.2f} m, moving forward.")
            else:
                cmd.linear.x = 0.0
                rospy.loginfo(f"Correct distance from wall: {front_ray_distance:.2f} m. Stopping adjustment.")
                distant_correct = True
            self.cmd_vel_pub.publish(cmd)
            if distant_correct == True:
                break  # Exit the loop once the distance is correct
            # Publish the velocity command and sleep
            rate.sleep()
        shortest_ray_distance = min(self.laser_ranges)
        shortest_ray_distance = min(self.laser_ranges)
        shortest_ray_index = self.laser_ranges.index(shortest_ray_distance)  # Find the index of the shortest ray
        if shortest_ray_distance < desired_min_distance: 
            self.align_with_wall(shortest_ray_index)
        
        rate.sleep()
        self.rotate_90_degrees(True)
        rospy.loginfo("Launching Wall Follower")
        return FindWallResponse(True)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        find_wall_node = FindWallNode()
        find_wall_node.run()
    except rospy.ROSInterruptException:
        pass