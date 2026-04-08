#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from marta_msgs.msg import Position 

def publisher():
    rospy.init_node('sphere_position_publisher', anonymous=True)
    pub = rospy.Publisher('sphere_position', Position, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    msg = Position()
    msg.latitude = 43.7064396   # ~20 meters north
    msg.longitude = 10.4754444   # ~20 meters easts
    msg.depth = 0.0

    while not rospy.is_shutdown():
        rospy.loginfo("Publishing fixed position: {}".format(msg))
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass
