#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32
import serial
import json
import serial.tools.list_ports

def find_arduino():
    # VID and PID for Arduino Uno R3
    arduino_vid = '2341'
    arduino_pid = '0043'

    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if arduino_vid in p.hwid and arduino_pid in p.hwid:
            return p.device
    return None

def imu_publisher():
    rospy.init_node('imu_publisher', anonymous=True)
    pub_accel_x = rospy.Publisher('imu/accel_x', Float32, queue_size=10)
    pub_accel_y = rospy.Publisher('imu/accel_y', Float32, queue_size=10)
    pub_accel_z = rospy.Publisher('imu/accel_z', Float32, queue_size=10)
    pub_gyro_x = rospy.Publisher('imu/gyro_x', Float32, queue_size=10)
    pub_gyro_y = rospy.Publisher('imu/gyro_y', Float32, queue_size=10)
    pub_gyro_z = rospy.Publisher('imu/gyro_z', Float32, queue_size=10)
    pub_temp = rospy.Publisher('imu/temperature', Float32, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    port = find_arduino()
    if port is None:
        rospy.logerr("Arduino not found")
        return

    ser = serial.Serial(port, 115200)

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
        imu_publisher()
    except rospy.ROSInterruptException:
        pass