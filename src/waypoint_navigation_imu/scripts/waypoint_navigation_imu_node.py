#!/usr/bin/env python3

import rospy
import math 
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Twist

class WaypointNavigationIMU:
    def __init__(self):
        rospy.init_node('waypoint_navigation_imu_node', anonymous=True)

        # Parameters 
        self.current_position = rospy.get_param('~current-position', [0.0, 0.0])    # (x, y)
        self.current_orientation = rospy.get_param('~current_orientation', 0.0)
        polar_waypoints = rospy.get_param('~polar_waypoints', [(1.0, 0.0), (2.0, 45.0)])    # (radius, angle in degrees)
        self.waypoints = [self.polar_to_cartesian(r, theta) for r, theta in polar_waypoints]
        self.waypoints = self.sort_waypoints_by_proximity(self.current_position, self.waypoints)
        self.current_waypoint_index = 0
        self.distance_threshold = rospy.get_param('~distance_threshold', 1.0)    # meters
        self.linear_speed = rospy.get_param('~linear_speed', 1.0)    # meters per second 
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)    # radians per second

        # Publishers 
        self.pub = rospy.Publisher('/cmd/vel', Twist, queue_size=10)

        # Subscribers 
        self.imu_sub = rospy.Subscriber('/imu/data', Imu, self.imu_callback)

        self.last_time = rospy.Time.now()

    def polar_to_cartesian(self, radius, angle):
        angle_rad = math.radians(angle)

        x = radius * math.cos(angle_rad)
        y = radius * math.sin(angle_rad)

        return (x, y)
    
    def sort_waypoints_by_proximity(self, current_position, waypoints):
        return sorted(waypoints, key=lambda waypoint: self.calculate_distance(current_position, waypoint))

    def imu_callback(self, data):
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).secs

        # Integrate accelarations to update position 
        acc_x = data.linear_accelaration.x
        acc_y = data.linear_accelaration.y

        self.current_position[0] += 0.5 * acc_x * dt**2
        self.current_position[1] += 0.5 * acc_y * dt**2

        # Integrate angular velocity to update orientation 
        angular_velocity_z = data.angular_velocity.z

        self.current_orientation += angular_velocity_z * dt 

        self.last_time = current_time

        self.navigate()

    def navigate(self):
        if (self.current_waypoint_index >= len(self.waypoints)):
            rospy.loginfo("All waypoints reached")
            return 
        
        target_position = self.waypoints[self.current_waypoint_index]
        distance_to_waypoint = self.calculate_distance(self.current_position, target_position)
        rospy.loginfo(f"Distance to waypoint: {distance_to_waypoint:.2f} meters")

        if (distance_to_waypoint < self.distance_threshold):
            rospy.loginfo("Waypoint reached")
            self.current_waypoint_index += 1
            return 
        
        heading_to_waypoint = self.calculate_heading(self.current_position, target_position)
        self.move_towards_waypoint(heading_to_waypoint)
        

    def calculate_distance(self, current_position, target_position):
        return math.sqrt((current_position[0] - target_position[0])**2 + (current_position[1] - target_position[1])**2)

    def calculate_heading(current_position, target_position):
        delta_x = target_position[0] - current_position[0] 
        delta_y = target_position[1] - current_position[1] 

        heading = math.atan2(delta_y, delta_x)

        return math.degrees(heading)

    def move_towards_waypoint(self, heading):
        cmd_vel = Twist()

        heading_error = heading - math.degrees(self.current_orientation)
        
        # Normalize the heading error (ensure it lies between -180 to 180 degrees) [ensure robot takes shortest turn to correct it's direction]
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360

        # Set Angular velocity proportional to heading error (Helps to correct robot's direction smoothly)
        cmd_vel.angular.z = self.angular_speed * (heading_error / 180.0)
        cmd_vel.linear.x = self.linear_speed

        self.pub.publish(cmd_vel)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = WaypointNavigationIMU()
        node.run()
    except rospy.ROSInterruptException:
        pass