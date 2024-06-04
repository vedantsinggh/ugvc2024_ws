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
        self.max_distance = rospy.get_param('~max_distance', 3.0)    # meters, beyond which the robot moves at max speed
        self.forward_speed = rospy.get_param('~forward_speed', 0.2)    # meters per second 
        self.turn_speed = rospy.get_param('~turn_speed', 0.5)    # radians per second 
        self.log_frequency = rospy.get_param('~log_frequency', 1.0)    # log frequnecy in seconds 


        # Publisher 
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber 
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        # For controlling log output frequency 
        self.last_log_time = rospy.get_time()
        
    def scan_callback(self, data):
        try:
            # Initialize Twist message
            cmd_vel = Twist()

            # Calculate resolution in degrees
            angle_increment_deg = data.angle_increment * (180.0 / 3.141592653589793) 

            # Calculate the number of measurements per 360 degree scan 
            num_measurements = int(360 / angle_increment_deg)

            try:
                # Divide the scan into 3 regions : 30, 90, 30 degrees
                # regions = {
                #     'left': min(min(data.ranges[0:int(num_measurements*1/3)]), 10),
                #     'front': min(min(data.ranges[int(num_measurements*1/3):int(num_measurements*2/3)]), 10),
                #     'right': min(min(data.ranges[int(num_measurements*2/3):]), 10),
                # }
                regions = {
                    'left': min(min(data.ranges[0:int(num_measurements*1/12)]), 5),
                    'front': min(min(data.ranges[int(num_measurements*1/12):int(num_measurements*4/12)]), 5),
                    'right': min(min(data.ranges[int(num_measurements*4/12):int(num_measurements*5/12)]), 5),
                }

            except Exception as e:
                rospy.logwarn(f"[obstacle_detection_node] Incomplete or invalid data: {e}")
                return 


            current_time = rospy.get_time()
            if current_time - self.last_log_time >= self.log_frequency:
                rospy.loginfo(f"Angle increment (degrees): {angle_increment_deg:.3f}")
                rospy.loginfo(f"Number of measurements per 360-degree scan: {num_measurements:.0f}")
                rospy.loginfo(f"Regions: {regions}")
                self.last_log_time = current_time

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

                # Adjust speed based on the closest distance to an obstacle in the front region 
                if (regions['front'] < self.max_distance):
                    # Scale the speed inversely proportional to distance 
                    cmd_vel.linear.x = self.forward_speed * (regions['front'] / self.max_distance)

                else:
                # Move at full speed
                    cmd_vel.linear.x = self.forward_speed
        
                cmd_vel.angular.z = 0.0

            # Publish the velocity command 
            self.pub.publish(cmd_vel)

        except Exception as e:
            rospy.logwarn(f"[obstacle_detection_node] Incomplete or Invalid data recieves from RPLiDAR: {e}")

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = ObstacleAvoidance()
        node.run()
    except rospy.ROSInterruptException:
        pass