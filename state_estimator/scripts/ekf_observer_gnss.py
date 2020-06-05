#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def update_v(msg):
    global v
    v = np.sqrt(msg.linear.x**2 + msg.linear.y**2)
   

def publisher(pos):
    global cov, v, i
    if i == 1:
        i = 0
        pub = rospy.Publisher('/ekf_gnss', Odometry, queue_size=1)
        msg = Odometry()
        msg.pose.pose.orientation = pos.pose.orientation
        msg.twist.twist.linear.x = v
        pub.publish(msg)
    elif i < 1:
        i += 1


def main():
    rospy.Subscriber("/an_device/Twist", Twist, update_v, queue_size=1)
    rospy.Subscriber("/gnss_pose", PoseStamped, publisher, queue_size=1)
    rospy.spin()


# Node initiliasation
rospy.init_node('ekf_observer_publisher')

# Global variables
v = 0
i = 1

if __name__ == '__main__':
    try:
    	main()
    except rospy.ROSInterruptException:
        pass
