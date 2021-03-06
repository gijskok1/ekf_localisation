#!/usr/bin/env python

import rospy
import numpy as np
from math import cos, sin, asin, atan2
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Quaternion
from nav_msgs.msg import Odometry


def get_euler(x, y, z, w):
    """Convert given quaternion to roll, pitch, yaw. NOTE: Singularity for pitch equals 90 degrees"""
    scale = 1/np.sqrt(x**2+y**2+z**2+w**2)
    x = scale*x
    y = scale*y
    z = scale*z
    w = scale*w

    r = atan2(2*(y*z + w*x), z**2-y**2-x**2+w**2)
    p = -asin(2*(x*z-w*y))
    y = atan2(2*(x*y+w*z), x**2+w**2-z**2-y**2)
    return r, p, y


def update_v(msg):
    global v
    v = np.sqrt(msg.twist.linear.x**2 + msg.twist.linear.y**2)


def publisher(pos):
    global samples, x, y, z, roll, pitch, yaw, cov, v, i, x_pos1, y_pos1, x_pos2, y_pos2, t_prev
    if samples < 100:
        x.append(pos.pose.position.x)
        y.append(pos.pose.position.y)
        z.append(pos.pose.position.z)
        get_roll, get_pitch, get_yaw = get_euler(pos.pose.orientation.z, pos.pose.orientation.y, 
        pos.pose.orientation.z, pos.pose.orientation.w)
        roll.append(get_roll)
        pitch.append(get_pitch)
        yaw.append(get_yaw)
        samples += 1
    elif cov is None:
    	cov = np.diag([np.var(x), np.var(y), np.var(z), np.var(roll), np.var(pitch), np.var(yaw)])
	t_prev = rospy.Time().now().to_sec()
	x_pos1 = pos.pose.position.x
	y_pos1 = pos.pose.position.y
    else:
	t_new = rospy.Time().now().to_sec()
	t = t_new - t_prev
	t_prev = t_new
	if x_pos2 != None and y_pos2 != None:
	    x_pos1 = x_pos2
	    y_pos1 = y_pos2
	x_pos2 = pos.pose.position.x
	y_pos2 = pos.pose.position.y
	var_v = (2*cov[0, 0]*(x_pos1-x_pos2)**2)/(t**2*((x_pos1-x_pos2)**2+(y_pos1-y_pos2)**2))+(2*cov[1, 1]*(y_pos1-y_pos2)**2)/(t**2*((x_pos1-x_pos2)**2+(y_pos1-y_pos2)**2))	

        pub = rospy.Publisher('/ekf_lidar', Odometry, queue_size=1)
        msg = Odometry()
        msg.pose.pose.position = pos.pose.position
        msg.pose.pose.orientation = pos.pose.orientation
        msg.pose.covariance = cov.ravel().tolist()
        msg.twist.twist.linear.x = v
        msg.twist.covariance = [var_v] + [0]*35
        pub.publish(msg)


def main():
    rospy.Subscriber("/estimate_twist", TwistStamped, update_v, queue_size=1)
    rospy.Subscriber("/ndt_pose", PoseStamped, publisher, queue_size=1)
    rospy.spin()


# Node initiliasation
rospy.init_node('ekf_lidar_publisher')

# Global variables
samples = 0
x = []
y = []
z = []
roll = []
pitch = []
yaw = []
cov = None
x_pos1 = None
y_pos1 = None
x_pos2 = None 
y_pos2 = None
v = 0
t_prev = 0

if __name__ == '__main__':
    try:
    	main()
    except rospy.ROSInterruptException:
        pass
