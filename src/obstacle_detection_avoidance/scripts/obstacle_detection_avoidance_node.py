#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class ObstacleAvoidance:
    def __init__(self):
        # Initialize the ROS node 
        rospy.init_node('obstacle_detection_avoidance_node', anonymous=True)

        # Parameters 
        self.min_distance = rospy.get_param('~min_distance', 0.5)    # meters
        self.forward_speed = rospy.get_param('~forward_speed', 0.2)    # meters per second 
        self.turn_speed = rospy.get_param('~turn_speed', 0.5)    # radians per second 

        # Publisher 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber 
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        
    def scan_callback(self, data):
        # Initialize Twist message
        cmd_vel = Twist()

        # Calculate resolution in degrees
        angle_increment_deg = data.angle_increment * (180.0 / 3.141592653589793) 

        # Calculate the number of measurements per 360 degree scan 
        num_measurements = 360 / angle_increment_deg

        rospy.loginfo(f"Angle increment (degrees): {angle_increment_deg:.3f}")
        rospy.loginfo(f"Number of measurements per 360-degree scan: {num_measurements:.0f}")

        # Divide the scan into 3 regions 
        # regions = {
        #     'left': min(min(data.ranges[0:int(num_measurements*1/3)]), 10),
        #     'front': min(min(data.ranges[int(num_measurements*1/3):int(num_measurements*2/3)]), 10),
        #     'right': min(min(data.ranges[int(num_measurements*2/3):]), 10),
        # }
        regions = {
            'left': min(min(data.ranges[0:int(num_measurements*1/6)]), 10),
            'front': min(min(data.ranges[int(num_measurements*1/6):int(num_measurements*2/6)]), 10),
            'right': min(min(data.ranges[int(num_measurements*2/6):int(num_measurements*3/6)]), 10),
        }

        rospy.loginfo(f"Regions: {regions}")

        # Check the regions and decide the action 
        if regions['front'] < self.min_distance:

            # If an obstacle is too close in front, turn based on which side is clearer 
            if regions['left'] < regions['right']:
                rospy.loginfo("Obstacle ahead! Turning Right...")
                cmd_vel.angular.z = -self.turn_speed   
            else:
                rospy.loginfo("Obstacle ahead! Turning Left...")
                cmd_vel.angular.z = self.turn_speed    
            cmd_vel.linear.x = 0.0
        
        else:
            rospy.loginfo("Path is clear. Moving forward...")

            # If no obstacle is detected in front move forward 
            cmd_vel.linear.x = self.forward_speed
            cmd_vel.angular.z = 0.0

        # Publish the velocity command 
        self.pub.publish(cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObstacleAvoidance()
        node.run()
    except rospy.ROSInterruptException:
        pass