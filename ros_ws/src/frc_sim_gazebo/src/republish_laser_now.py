#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan

# ChatGPT wrote this it saved me 3 minutes and only cost like a 10 pounds of coal

def laser_scan_callback(msg):
    # Modify the timestamp of the original message
    msg.header.stamp = rospy.Time.now()
    msg.header.stamp -= rospy.Duration(rospy.get_param("~repub_latency", 0.0))

    # Publish the modified message
    pub.publish(msg)

def laser_scan_republisher():
    rospy.init_node('laser_scan_republisher')
    rospy.loginfo("Starting simple scan republisher")

    # Subscribe to the original laser scan topic
    rospy.Subscriber('/raw_scan', LaserScan, laser_scan_callback)

    # Publisher for the modified laser scan with the current time
    global pub
    pub = rospy.Publisher('/scan', LaserScan, queue_size=10)

    # Keep the node running
    rospy.spin()

if __name__ == '__main__':
    try:
        laser_scan_republisher()
    except rospy.ROSInterruptException:
        pass