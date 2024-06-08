#!/usr/bin/env python3

import rospy 
from sensor_msgs.msg import NavSatFix
import gpsd
import time 

def gps_publisher():
    rospy.init_node('gps_publisher', anonymous=True)
    pub = rospy.Publisher('gps_topic', NavSatFix, queue_size=10)
    rate = rospy.Rate(1)    # 1 Hz

    gpsd.connect()

    while not rospy.is_shutdown():
        packet = gpsd.get_current()

        try:
            latitude, longitude = packet.position()
            navsatfix_msg = NavSatFix()
            navsatfix_msg.header.stamp = rospy.Time.now()
            navsatfix_msg.header.frame_id = "gps"
            navsatfix_msg.latitude = latitude
            navsatfix_msg.longitude = longitude 
            navsatfix_msg.altitude = packet.altitude()

            rospy.loginfo("Publishing GPS data: (%f, %f)", latitude, longitude)
            pub.publish(navsatfix_msg)
        
        except Exception as e:
            rospy.logwarn("Could not get GPS position: %s", e)

        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException as e:
        rospy.logwarn(f"[gps_publisher_node] Incomplete or Invalid data recieved from GPS: {e}")
        pass