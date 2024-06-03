#!/usr/bin/env python3 

import rospy 
import math 
from sensor_msgs.msg  import NavSatFix
from geometry_msgs.msg import Twist 
from geopy.distance import geodesic

class WaypointNavigationGPS:
    def __init__(self):
        rospy.init_node('waypoint_navigation_gps', anonymous=True)

        # Parameters 
        self.waypoints = [
            (37.7749, -122.4194),    # Example waypoints (latitude, longitude)
            (37.7750, 122.4180)
        ]
        self.current_waypoint_index = 0
        self.distance_threshold = rospy.get_param('~distance_threshold', 1.0)    # meters
        self.linear_speed = rospy.get_param('~linear_speed', 1.0)    # meters per second 
        self.angular_speed = rospy.get_param('~angular_speed', 0.5)    # radians per second

        # Publishers 
        self.pub = rospy.Publisher('/cmd/vel', Twist, queue_size=10)

        # Subscribers 
        self.sub = rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)

    def gps_callback(self, data):
        if (self.current_waypoint_index >= len(self.waypoints)):
            rospy.loginfo("All waypoints reached")
            return 
        
        current_position = (data.latitude, data.longitude)
        target_position = self.waypoints(self.current_waypoint_index)

        # Calculating distance while taking in account the curvature of the earth 
        distance_to_waypoint = geodesic(current_position, target_position).meters 
        rospy.loginfo(f"Distance to waypoint: {distance_to_waypoint:.2f} meters")

        if (distance_to_waypoint < self.distance_threshold):
            rospy.loginfo("Waypoint reached")
            self.current_waypoint_index += 1
            return 
        
        heading_to_waypoint = self.calculate_heading(current_position, target_position)
        self.move_towards_waypoint(heading_to_waypoint)

    # Function to calculate the initial bearing (or heading) from current GPS position to the target GPS position 
    # Bearing is the direction in our robot needs to tarvel to go from current location to target location 
    def calculate_heading(self, current_position, target_position):
        # Convert latitude and longitude to radians as python expects in radians (But GPS provides in degrees)
        lat_current, lon_current = map(math.radians, current_position)
        lat_target, lon_target = map(math.radians, target_position)
        diff_lon = lon_target - lon_current

        # Spherical Trigonometry to calculate cartesian co-ordinates of target location relative to the current location
        x = math.sin(diff_lon) * math.cos(lat_target)
        y = math.cos(lat_current) * math.sin(lat_target) - (math.sin(lat_current) * math.cos(lat_target) * math.cos(diff_lon))

        # Calculate initial bearing 
        initial_bearing = math.atan2(x, y)    # returns angle between +ve x-axis & point (x, y). This angle is initial bearing in radians 
        initial_bearing = math.degrees(initial_bearing)    # Convert bearing to degrees 

        # Converts bearing to a value between 0 & 360 degrees, suitable for compass directions 
        compass_bearing = (initial_bearing + 360) % 360    

        return compass_bearing   

    # Function to adjust robot's movement based on desired heading 
    # Calculates heading error & adjusts robot's angular velocity to correct it's direction 
    def move_towards_waypoint(self, heading):
        cmd_vel = Twist()

        # current orientation of robot is required to calculate heading error (typically this is obtained using IMU)
        current_yaw = 0.0   # TODO_: Using IMU sensor  
        heading_error = heading - current_yaw

        # Normalize the heading error (ensure it lies between -180 to 180 degrees) [ensure robot takes shortest turn to correct it's direction]
        if heading_error > 180:
            heading_error -= 360 
        elif heading_error < -180:
            heading_error += 360 

        # Set Angular velocity proportional to heading error (Helps to correct robot's direction smoothly)
        cmd_vel.angular.z = self.angular_speed * (heading_error/180.0)
        cmd_vel.linear.x = self.linear_speed

        self.pub.publish(cmd_vel)

    def run(self):
        rospy.spin()

    
if __name__ == "__main__":
    try:
        node = WaypointNavigationGPS()
        node.run()
    except rospy.ROSInterruptException:
        pass