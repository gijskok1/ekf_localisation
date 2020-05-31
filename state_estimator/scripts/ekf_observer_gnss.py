#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def update_cov(msg):
    global cov
    cov = np.diag([msg.position_covariance[0], msg.position_covariance[4], msg.position_covariance[8], msg.position_covariance[8]/(1.56**2), 0.02, msg.position_covariance[0]/(1.56**2)])


def update_v(msg):
    global v
    v = np.sqrt(msg.linear.x**2 + msg.linear.y**2)
   

def publisher(pos):
    global cov, v, i, t_prev
    if (cov is not None) and (i == 1):
        i = 0
        pub = rospy.Publisher('/ekf_gnss', Odometry, queue_size=1)
        msg = Odometry()
	t_new = rospy.Time().now().to_sec()
	t = t_new - t_prev
	t_prev = t_new
	msg.pose.pose.position = pos.pose.position
        msg.pose.pose.orientation = pos.pose.orientation
        msg.pose.covariance = cov.ravel().tolist()
        msg.twist.twist.linear.x = v
        msg.twist.covariance = [np.sqrt(2*cov[0, 0]**2+2*cov[1, 1]**2)/t] + [0]*35
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
t_prev = rospy.Time().now().to_sec()

if __name__ == '__main__':
    try:
    	main()
    except rospy.ROSInterruptException:
        pass
