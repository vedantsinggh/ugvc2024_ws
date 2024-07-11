#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import serial
import json

def talker():
    pub_accel_x = rospy.Publisher('imu/accel_x', Float32, queue_size=10)
    pub_accel_y = rospy.Publisher('imu/accel_y', Float32, queue_size=10)
    pub_accel_z = rospy.Publisher('imu/accel_z', Float32, queue_size=10)
    pub_gyro_x = rospy.Publisher('imu/gyro_x', Float32, queue_size=10)
    pub_gyro_y = rospy.Publisher('imu/gyro_y', Float32, queue_size=10)
    pub_gyro_z = rospy.Publisher('imu/gyro_z', Float32, queue_size=10)
    pub_temp = rospy.Publisher('imu/temperature', Float32, queue_size=10)
    rospy.init_node('imu_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    ser = serial.Serial('/dev/ttyACM0', 115200)

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            try:
                data = json.loads(line)
                rospy.loginfo(data)

                pub_accel_x.publish(data['accel_x'])
                pub_accel_y.publish(data['accel_y'])
                pub_accel_z.publish(data['accel_z'])
                pub_gyro_x.publish(data['gyro_x'])
                pub_gyro_y.publish(data['gyro_y'])
                pub_gyro_z.publish(data['gyro_z'])
                pub_temp.publish(data['temperature'])
            except json.JSONDecodeError:
                rospy.logwarn("Could not decode JSON")

        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass