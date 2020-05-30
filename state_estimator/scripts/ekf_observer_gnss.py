#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Point, Quaternion, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix


def update_cov(msg):
    """Obtain pose covariance from position covariance and estimated orientation covariance"""
    global cov, std_x, std_y
    cov = np.diag([msg.position_covariance[0]+std_x**2, msg.position_covariance[4]+std_y**2, msg.position_covariance[8], msg.position_covariance[8]/(1.56**2), 0.02, (msg.position_covariance[0]+std_x**2)/(1.56**2)])


def update_v(msg):
    """Obtain velocity information from advanced navigation device"""
    global msg_v
    msg_v = msg
   

def publisher(pos):
    """Publish every other GNSS reading as GNSS readings are sent twice. Also calculates estimated velocity variance"""
    global cov, v, i, t_prev, msg_v
    if (cov is not None) and (i == 1):
        i = 0
        pub = rospy.Publisher('/ekf_gnss', Odometry, queue_size=1)
        msg = Odometry()
	t_new = rospy.Time().now().to_sec()
	t = t_new - t_prev
	t_prev = t_new
	v = np.sqrt((s_x/t)**2 + (s_y/t)**2)
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
rospy.init_node('ekf_observer_gnss')

# Global variables
msg_v = None
cov = None
v = 0
i = 1
t_prev = rospy.Time().now().to_sec()

if __name__ == '__main__':
    try:
    	main()
    except rospy.ROSInterruptException:
        pass
