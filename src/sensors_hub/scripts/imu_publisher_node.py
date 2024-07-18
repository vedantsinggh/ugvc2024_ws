#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32
import serial
import json
import serial.tools.list_ports
from geometry_msgs.msg import Vector3

def find_arduino(serial_number):
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        if p.serial_number == serial_number:
            return p.device
    return None

def imu_publisher():
    rospy.init_node('imu_publisher', anonymous=True)
    pub_imu = rospy.Publisher('imu/data', Imu, queue_size=10)
    pub_temp = rospy.Publisher('imu/temperature', Float32, queue_size=10)
    rate = rospy.Rate(10)  # 10hz

    arduino_serial_number = '44236313735351705081'
    port = find_arduino(arduino_serial_number)
    
    if port is None:
        rospy.logerr(f"Arduino with serial number {arduino_serial_number} not found")
        return

    ser = serial.Serial(port, 115200)

    while not rospy.is_shutdown():
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').rstrip()
            try:
                data = json.loads(line)

                imu_msg = Imu()
                imu_msg.header.stamp = rospy.Time.now()
                imu_msg.header.frame_id = "imu_link"
                imu_msg.linear_acceleration = Vector3(
                    x=data['accel_x'],
                    y=data['accel_y'],
                    z=data['accel_z']
                )

                imu_msg.angular_velocity = Vector3(
                    x=data['gyro_x'],
                    y=data['gyro_y'],
                    z=data['gyro_z']
                )

                pub_imu.publish(imu_msg)

                pub_temp.publish(data['temperature'])

            except json.JSONDecodeError:
                rospy.logwarn("Could not decode JSON")

        rate.sleep()

if __name__ == '__main__':
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass