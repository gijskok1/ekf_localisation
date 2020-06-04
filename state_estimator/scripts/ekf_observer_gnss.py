#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def update_cov(msg):
    global cov
    cov = np.diag([msg.position_covariance[0], msg.position_covariance[4], msg.position_covariance[8], 3e-6, 3e-6, 3e-6])


def update_v(msg):
    global v
    v = np.sqrt(msg.linear.x**2 + msg.linear.y**2)
   

def publisher(pos):
    global cov, v, i
    if (cov is not None) and (i == 1):
        i = 0
        pub = rospy.Publisher('/ekf_gnss', Odometry, queue_size=1)
        msg = Odometry()
	msg.pose.pose.position = pos.pose.position
        msg.pose.pose.orientation = pos.pose.orientation
        msg.pose.covariance = cov.ravel().tolist()
        msg.twist.twist.linear.x = v
        msg.twist.covariance = [0.005**2] + [0]*35
        pub.publish(msg)
    elif i < 1:
        i += 1


def main():
    rospy.Subscriber("/an_device/Twist", Twist, update_v, queue_size=1)
    rospy.Subscriber("/an_device/NavSatFix", NavSatFix, update_cov, queue_size=1)
    rospy.Subscriber("/gnss_pose", PoseStamped, publisher, queue_size=1)
    rospy.spin()


# Node initiliasation
rospy.init_node('ekf_observer_publisher')

# Global variables
cov = None
v = 0
i = 1

if __name__ == '__main__':
    try:
    	main()
    except rospy.ROSInterruptException:
        pass
