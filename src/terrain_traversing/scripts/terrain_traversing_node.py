#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from sensor_msgs.msg import NavSatFix, Imu
import math

class Point:
	def __init__(self,x,y) -> None:
		self.x = x
		self.y = y

class TerrainTreversing:
	def __init__(self):
		rospy.init_node('terrain_treversing_node')
		self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
		
# ---------------GPS--------------------------
		# self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		# rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
		# rospy.Subscriber('/imu/data', Imu, self.imu_callback)

		#TODO : input target data from params
		# self.targets = [
		# 	NavSatFix(latitude=30.0500, longitude=31.2333),
		# 	NavSatFix(latitude=30.0500, longitude=31.2333),
		# 	NavSatFix(latitude=30.0500, longitude=31.2333)
		# ]

		# self.current_gps = None
		# self.target_gps = targets[0] 


# ---------------GPS--------------------------


# -----FOR TESTING IN TURTLE MAKE SURE TO RUN TURTLESIMNODE FIRST-----
		self.current_pose = None
		self.target_pose = Pose()

		#TODO : implement dikjstra algo to sort the target array 
		self.targets = [
			Point(2,8),
			Point(8,8),
			Point(8,2),
		]

		
		self.point_index = 0
		self.task_done = False

		self.target_reached = False
		self.spiral_search_active = False
		self.spiral_step = .5  
		self.spiral_rotation = 5
		
		self.rate = rospy.Rate(10)

# ---------------GPS-------------------------
	# def gps_callback(self, msg):
	# 	self.current_gps = msg

	#TODO : input IMU data for accuracy 
	# def imu_callback(self, msg):
	# 	pass  # to configure IMU data
# ---------------GPS--------------------------

	def pose_callback(self, msg):
		self.current_pose = msg

	def euclidean_distance(self, x1, y1, x2, y2):
		return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

	def navigate_to_waypoint(self):
		if self.current_pose is None:
			return

		# distance = self.haversine(self.current_gps.latitude, self.current_gps.longitude, self.target_gps.latitude, self.target_gps.longitude) 
		distance = self.euclidean_distance(self.current_pose.x, self.current_pose.y, self.target_pose.x, self.target_pose.y)
		if distance < 0.5:
			self.target_reached = True
			rospy.loginfo("Waypoint reached")
			return

		cmd_vel = Twist()
		cmd_vel.linear.x = 1.0 * distance
		cmd_vel.angular.z = 4.0 * (math.atan2(self.target_pose.y - self.current_pose.y, self.target_pose.x - self.current_pose.x) - self.current_pose.theta)
		self.cmd_vel_pub.publish(cmd_vel)

	def haversine(self, lat1, lon1, lat2, lon2):
		R = 6371000  # Radius of the Earth in meters
		phi1 = math.radians(lat1)
		phi2 = math.radians(lat2)
		delta_phi = math.radians(lat2 - lat1)
		delta_lambda = math.radians(lon2 - lon1)
		a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
		c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
		return R * c

	def spiral_search(self):
		if not self.spiral_search_active:
			return

		cmd_vel = Twist()
		self.spiral_step += 0.1
		cmd_vel.linear.x = self.spiral_step
		cmd_vel.linear.y = 0
		cmd_vel.linear.z = 0
		cmd_vel.angular.x = 0
		cmd_vel.angular.y = 0
		cmd_vel.angular.z = self.spiral_rotation

		#TODO : implement metal detection
		#TODO : write a retraversing fallback in case bot is unable to detect the metal at first go
		if self.spiral_step > 5:
			rospy.sleep(rospy.Duration(2))
			self.point_index += 1 

			if self.point_index >= len(self.targets):
				self.task_done = True
				#TODO: Now it should load node for next task

			self.spiral_search_active = False
			self.target_reached = False
			self.spiral_step = 0.5

		self.cmd_vel_pub.publish(cmd_vel)

	def run(self):
		while not rospy.is_shutdown() and not self.task_done:
			self.target_pose.x = self.targets[self.point_index].x
			self.target_pose.y = self.targets[self.point_index].y
			if not self.target_reached:
				self.navigate_to_waypoint()
			else:
				self.spiral_search_active = True
				self.spiral_search()

			self.rate.sleep()

if __name__ == '__main__':
	try:
		node = TerrainTreversing()
		node.run()
	except rospy.ROSInterruptException:
		pass
