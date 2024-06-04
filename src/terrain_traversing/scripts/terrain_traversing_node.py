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

		self.log_frequency = rospy.get_param('~log_frequency', 1.0)    # log frequnecy in seconds 
		self.last_log_time = rospy.get_time()
		
		self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
		rospy.Subscriber('/gps/fix', NavSatFix, self.gps_callback)
		rospy.Subscriber('/imu/data', Imu, self.imu_callback)

		#TODO : input target data from params
		#TODO : implement dikjstra algo to sort the target array 
		#TODO : normalize angular velocity
		self.targets = [
			NavSatFix(latitude=30.0500, longitude=31.2333),
			NavSatFix(latitude=30.0500, longitude=31.2333),
			NavSatFix(latitude=30.0500, longitude=31.2333)
		]

		self.current_gps = None
		self.target_gps = self.targets[0] 

# ---------------Turtle --------------------------
		# self.cmd_vel_pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
		# self.pose_sub = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
		# self.current_pose = None
		# self.target_pose = Pose()

		# self.targets = [
		# 	Point(2,8),
		# 	Point(8,8),
		# 	Point(8,2),
		# ]
# ---------------Turtle --------------------------

		
		self.point_index = 0
		self.task_done = False

		self.target_reached = False
		self.spiral_search_active = False

		self.spiral_step = rospy.get_param('~spiral_step', 0.5)    
		self.spiral_rotation = rospy.get_param('~spiral_rotation', 5.0)    
		self.linear_multiplier = rospy.get_param('~linear_multiplier', 1.0)    
		self.angular_multiplier = rospy.get_param('~angular_multiplier', 4.0)
		self.spiral_inc = rospy.get_param('~spiral_inc', 0.1)

		self.rate = rospy.Rate(10)

# ---------------GPS-------------------------
	def gps_callback(self, msg):
		try:
			self.current_gps = msg
		except Exception as e:
			rospy.logwarn(f"[terrain_traversing_node] Incomplete or Invalid data recieved from GPS: {e}")

	#TODO : input IMU data for accuracy 
	def imu_callback(self, msg):
		try:
			pass  # to configure IMU data
		except Exception as e:
			rospy.logwarn(f"[terrain_traversing_node] Incomplete or Invalid data recieved from IMU: {e}")
			

# ---------------GPS--------------------------

	# def pose_callback(self, msg):
	# 	self.current_pose = msg

	# def euclidean_distance(self, x1, y1, x2, y2):
	# 	return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

	def navigate_to_waypoint(self):
		if self.current_pose is None:
			return

		current_time = rospy.get_time()
		if current_time - self.last_log_time >= self.log_frequency:
			rospy.loginfo(f"Navigating towards point {self.point_index + 1}, bitch! ")
			self.last_log_time = current_time

		distance = self.haversine(self.current_gps.latitude, self.current_gps.longitude, self.target_gps.latitude, self.target_gps.longitude) 
		# distance = self.euclidean_distance(self.current_pose.x, self.current_pose.y, self.target_pose.x, self.target_pose.y)
		if distance < 0.5:
			self.target_reached = True
			rospy.loginfo("Waypoint reached")
			return
		
		# Normalize the angular difference between current position and target waypoint
		angular_diff_curr_wayp = (math.atan2(self.target_pose.y - self.current_pose.y, self.target_pose.x - self.current_pose.x) - self.current_pose.theta)
		if angular_diff_curr_wayp > math.pi:
				angular_diff_curr_wayp =- 2 * math.pi
		elif angular_diff_curr_wayp <  -math.pi:
				angular_diff_curr_wayp =+ 2 * math.pi

		cmd_vel = Twist()
		cmd_vel.linear.x = self.linear_multiplier * distance
		cmd_vel.angular.z = self.angular_multiplier *  angular_diff_curr_wayp
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
		
		current_time = rospy.get_time()
		if current_time - self.last_log_time >= self.log_frequency:
			rospy.loginfo(f"Finding metal, bitch! ")
			self.last_log_time = current_time

		cmd_vel = Twist()
		self.spiral_step += self.spiral_inc
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
			rospy.loginfo(f"Metal found at {self.current_pose.x} , {self.current_pose.y}")
			#TODO : broadcast it to the station
			self.point_index += 1 

			if self.point_index >= len(self.targets):
				self.task_done = True
				rospy.loginfo(f"Task completed, bitch! ")
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
