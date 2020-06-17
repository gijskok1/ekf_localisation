#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def update_v(msg):
    global v
    v = np.sqrt(msg.linear.x**2+msg.linear.y**2)


def update_cov(msg):
    global cov
    cov = msg.position_covariance
        

def publisher(pos):
    global cov, v, i
    if i == 1 and cov != None:
	i = 0
    	pub = rospy.Publisher('/ekf_gnss', Odometry, queue_size=1)
    	msg = Odometry()
    	msg.pose.pose.position = pos.pose.position
    	msg.pose.pose.orientation = pos.pose.orientation
    	msg.twist.twist.linear.x = v
    	msg.pose.covariance = np.diag([cov[0], cov[4], cov[8], 0, 0, 0]).ravel().tolist()
    	pub.publish(msg)
    elif i < 1:
	i += 1


def main():
    rospy.Subscriber("/an_device/Twist", Twist, update_v, queue_size=1)
    rospy.Subscriber("/an_device/NavSatFix", NavSatFix, update_cov, queue_size=1)
    rospy.Subscriber("/gnss_pose", PoseStamped, publisher, queue_size=1)
    rospy.spin()

# Node initiliasation
rospy.init_node('ekf_gnss_publisher')

# Global variables
started = False
cov = None
v = 0
i = 1


if __name__ == '__main__':
    try:
    	main()
    except rospy.ROSInterruptException:
        pass

