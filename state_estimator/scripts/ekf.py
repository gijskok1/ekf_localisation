#!/usr/bin/env python

import rospy
import numpy as np
from math import cos, sin, asin, atan2
from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovariance, PoseWithCovarianceStamped, Twist, TwistStamped, Point, Quaternion
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry


def get_quaternion(r, p, y):
    """Convert given roll, pitch, yaw to quaternion"""
    c_1 = cos(r*0.5)
    s_1 = sin(r*0.5)
    c_2 = cos(p*0.5)
    s_2 = sin(p*0.5)
    c_3 = cos(y*0.5)
    s_3 = sin(y*0.5)

    w = c_1*c_2*c_3+s_1*s_2*s_3
    x = -c_1*s_2*s_3+c_2*c_3*s_1
    y = c_1*c_3*s_2+s_1*c_2*s_3
    z = c_1*c_2*s_3-s_1*c_3*s_2
    return x, y, z, w


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
    
    
def normalise_angle(angle):
    """Converts angle to the domain (-pi pi)"""
    angle = angle%(2*np.pi)
    if angle > np.pi:
        angle -= 2*np.pi
    return angle


def callback_imu(msg):
    """Updates orientation, angular velocities and linear accelerations via IMU measurement"""
    global q, omega, a
    q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
    omega = [-msg.angular_velocity.x, -msg.angular_velocity.y, -msg.angular_velocity.z]
    a = [msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z]


def callback_gnss(msg):
    """Updates velocity via GNSS measurement"""
    global v
    v = np.sqrt(msg.linear.x**2 + msg.linear.y**2)
    

def callback_lidar(msg):
    """Updates or starts Kalman filter via LiDAR measurement"""
    global EKF, started, lidar_callback_done, old_time
    p = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    o = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
         msg.pose.pose.orientation.w]
    cov = list(msg.pose.covariance)
    v = msg.twist.twist.linear.x
    cov_v = msg.twist.covariance[0]
    if not started:
        started = True
        lidar_callback_done = True
        old_time = rospy.Time().now().to_sec()
        EKF = StateEstimator(p, o, cov, v, cov_v)
    elif not lidar_callback_done:
        lidar_callback_done = True
        EKF.update_lidar(p, o, cov, v, cov_v)


class StateEstimator:

    def __init__(self, P=[0]*3, O=[1]*4, cov=[0]*36, v=0, cov_v=0):
        """"Take position P (x y z), orientation O (x y z w) and 3x3 Position covariance and
        Orientation covariance and make these the initial state (x y z roll pitch yaw) and covariance.
        Also defines tuning variables, Also determines whether the filter has wheel speed avialable"""
        r, p, y = get_euler(O[0], O[1], O[2], O[3])

        # Tuning variables
        self.var_u = np.diag([0.0084, 0.002, 0.002, 0.002, 0.0024, 0.0022, 0.0013])**2 # (a, phi, theta, psi, phi_dot, theta_dot psi_dot)
        self.var_u2 = np.diag([0.005, 0.0084, 0.002, 0.002, 0.002, 0.0024, 0.0022, 0.0013])**2 # (v, a, phi, theta, psi, phi_dot, theta_dot psi_dot)
        self.var_a = np.diag([4.784, 0.10697, 0.4989, 0.36137, 0.10051])**2 # (j, z_dot, phi_ddot, theta_ddot psi_ddot)

        # Initialisation of matrices, vectors and startup boolean
        self.X_prev = np.matrix([[P[0]], [P[1]], [P[2]], [r], [p], [y], [v]])
        self.P_prev = np.diag([cov[0], cov[7], cov[14], cov[21], cov[28], cov[35], cov_v])
        self.H = np.eye(7)
            
        self.X_est = None
        self.P_est = None
        self.starting_up = True


    def predict(self, a, q, omega, t):
        """Kalman filter prediction via IMU measurements"""
	# Get orientation 
	phi, theta, psi = get_euler(q[0], q[1], q[2], q[3])

        # Rear odometric model prediction
        self.X_est = np.matrix([[self.X_prev[0, 0]+(self.X_prev[6, 0]*t+a[0]*t**2/2)*cos(psi+omega[2]*t/2)],
                                [self.X_prev[1, 0]+(self.X_prev[6, 0]*t+a[0]*t**2/2)*sin(psi+omega[2]*t/2)],
                                [self.X_prev[2, 0]+0],
                                [phi+omega[0]*t],
                                [theta+omega[1]*t],
                                [psi+omega[2]*t],
                                [self.X_prev[6, 0]+a[0]*t]])

        # Normalise yaw angle
        self.X_est[5, 0] = normalise_angle(self.X_est[5, 0])

        # Jacobian w.r.t state variables
        F_x = np.matrix([[1, 0, 0, 0, 0, -(t*self.X_prev[6, 0]+a[0]*t**2/2)*sin(psi+omega[2]*t/2), t*cos(psi+omega[2]*t/2)],
                         [0, 1, 0, 0, 0, (t*self.X_prev[6, 0]+a[0]*t**2/2)*cos(psi+omega[2]*t/2), t*sin(psi+omega[2]*t/2)],
                         [0, 0, 1, 0, 0, 0, 0],
                         [0, 0, 0, 1, 0, 0, 0],
                         [0, 0, 0, 0, 1, 0, 0],
                         [0, 0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 0, 0, 1]])

        # Jacobian w.r.t input variables
        F_u = np.matrix([[t**2*cos(psi+omega[2]*t/2)/2, 0, 0, -((a[0]*t**2/2+self.X_prev[6, 0]*t)*sin(psi+omega[2]*t/2)), 0, 0, -((a[0]*t**2/2+self.X_prev[6, 0]*t)*sin(psi+omega[2]*t/2))/2],
                         [t**2*sin(psi+omega[2]*t/2)/2, 0, 0, ((a[0]*t**2/2+self.X_prev[6, 0]*t)*cos(psi+omega[2]*t/2)), 0, 0, ((a[0]*t**2/2+self.X_prev[6, 0]*t)*cos(psi+omega[2]*t/2))/2],
                         [0, 0, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, t, 0, 0],
                         [0, 0, 1, 0, 0, t, 0],
                         [0, 0, 0, 1, 0, 0, t],
                         [t, 0, 0, 0, 0, 0, 0]])

        # Jacobian w.r.t higher order motion variables
        G = np.matrix([[t**3*cos(self.X_prev[5, 0]+omega[2]*t/2)/6, 0, 0, 0, -((a[0]*t**2/2+self.X_prev[6, 0]*t)*sin(self.X_prev[5, 0]+omega[2]*t/2))/4],
                       [t**3*sin(self.X_prev[5, 0]+omega[2]*t/2)/6, 0, 0, 0, ((a[0]*t**2/2+self.X_prev[6, 0]*t)*cos(self.X_prev[5, 0]+omega[2]*t/2))/4],
                       [0, t, 0, 0, 0],
                       [0, 0, t**2/2, 0, 0],
                       [0, 0, 0, t**2/2, 0],
                       [0, 0, 0, 0, t**2/2],
                       [t**2/2, 0, 0, 0, 0]])

        Q = F_u*self.var_u*F_u.T+G*self.var_a*G.T
        self.P_est = F_x*self.P_prev*F_x.T+Q
        self.starting_up = False


    def predict_gnss(self, v, a, q, omega, t):
        """Kalman filter prediction via IMU and GNSS measurements"""
	# Avoid GNSS drift by setting velocity threshold
	if v < 0.06:
	    v = 0

	# Get orientation 
	phi, theta, psi = get_euler(q[0], q[1], q[2], q[3])

        # Rear odometric model prediction
        self.X_est = np.matrix([[self.X_prev[0, 0]+(v*t+a[0]*t**2/2)*cos(psi+omega[2]*t/2)],
                                [self.X_prev[1, 0]+(v*t+a[0]*t**2/2)*sin(psi+omega[2]*t/2)],
                                [self.X_prev[2, 0]+0],
                                [phi+omega[0]*t],
                                [theta+omega[1]*t],
                                [psi+omega[2]*t],
                                [v+a[0]*t]])

        # Normalise yaw angle
        self.X_est[5, 0] = normalise_angle(self.X_est[5, 0])

        # Jacobian w.r.t state variables
        F_x = np.matrix([[1, 0, 0, 0, 0, -(t*v+a[0]*t**2/2)*sin(psi+omega[2]*t/2), t*cos(psi+omega[2]*t/2)],
                         [0, 1, 0, 0, 0, (t*v+a[0]*t**2/2)*cos(psi+omega[2]*t/2), t*sin(psi+omega[2]*t/2)],
                         [0, 0, 1, 0, 0, 0, 0],
                         [0, 0, 0, 1, 0, 0, 0],
                         [0, 0, 0, 0, 1, 0, 0],
                         [0, 0, 0, 0, 0, 1, 0],
                         [0, 0, 0, 0, 0, 0, 1]])

        # Jacobian w.r.t input variables
        F_u = np.matrix([[t*cos(psi+omega[2]*t/2)/2, t**2*cos(psi+omega[2]*t/2)/2, 0, 0, -((a[0]*t**2/2+v*t)*sin(psi+omega[2]*t/2)), 0, 0, -((a[0]*t**2/2+v*t)*sin(psi+omega[2]*t/2))/2],
                         [t*sin(psi+omega[2]*t/2)/2, t**2*sin(psi+omega[2]*t/2)/2, 0, 0, ((a[0]*t**2/2+v*t)*cos(psi+omega[2]*t/2)), 0, 0, ((a[0]*t**2/2+v*t)*cos(psi+omega[2]*t/2))/2],
                         [0, 0, 0, 0, 0, 0, 0, 0],
                         [0, 0, 1, 0, 0, t, 0, 0],
                         [0, 0, 0, 1, 0, 0, t, 0],
                         [0, 0, 0, 0, 1, 0, 0, t],
                         [1, t, 0, 0, 0, 0, 0, 0]])

        # Jacobian w.r.t higher order motion variables
        G = np.matrix([[t**3*cos(psi+omega[2]*t/2)/6, 0, 0, 0, -((a[0]*t**2/2+v*t)*sin(psi+omega[2]*t/2))/4],
                       [t**3*sin(psi+omega[2]*t/2)/6, 0, 0, 0, ((a[0]*t**2/2+v*t)*cos(psi+omega[2]*t/2))/4],
                       [0, t, 0, 0, 0],
                       [0, 0, t**2/2, 0, 0],
                       [0, 0, 0, t**2/2, 0],
                       [0, 0, 0, 0, t**2/2],
                       [t**2/2, 0, 0, 0, 0]])

        Q = F_u*self.var_u2*F_u.T+G*self.var_a*G.T
        self.P_est = F_x*self.P_prev*F_x.T+Q
        self.starting_up = False


    def update_lidar(self, z_P, z_O, cov, v, cov_v):
        """Kalman filter update via LiDAR observation"""
        if not self.starting_up:
            # Converting measurement to correct state
            r, p, y = get_euler(z_O[0], z_O[1], z_O[2], z_O[3])
            z = np.matrix([[z_P[0]], [z_P[1]], [z_P[2]], [r], [p], [y], [v]])
            R = np.diag([cov[0], cov[7], cov[14], cov[21], cov[28], cov[35], cov_v])


            # Kalman update calculations
            S = self.H*self.P_est*self.H.T + R
            K = self.P_est*self.H.T*np.linalg.inv(S)
            inn = z-self.H*self.X_est
            if inn[5, 0] >= np.pi:
                inn[5, 0] -= 2*np.pi
            elif inn[5, 0] <= -np.pi:
                inn[5, 0] += 2*np.pi
            self.X_est += K*inn
            self.P_est = (np.eye(7)-K*self.H)*self.P_est


    def finish_loop(self):
        """Publish estimated position and orientation and preparation for next cycle"""
        pub = rospy.Publisher('/ekf_pose', PoseStamped, queue_size=10)
        msg = PoseStamped()
        msg.header.frame_id = '/map'
        msg.header.stamp = rospy.Time().now()
        
        msg.pose.position = Point(self.X_est[0, 0], self.X_est[1, 0], self.X_est[2, 0])
        x, y, z, w = get_quaternion(self.X_est[3, 0], self.X_est[4, 0], self.X_est[5, 0])
        msg.pose.orientation = Quaternion(x, y, z, w)
        pub.publish(msg)

        self.X_prev = self.X_est
        self.P_prev = self.P_est


# Initialise global variables
EKF = StateEstimator()
q = None
omega = [0, 0, 0]
v = None
a = [0, 0, 0]

old_time = 0
new_time = 0
started = False

# Specify used sensors
gnss = True
lidar = True
imu = True

# Quick fix variable
lidar_callback_done = False

# Initialise node
rospy.init_node('state_estimator')
rate = rospy.Rate(20)


if __name__ == '__main__':
    try:
	old_time = rospy.Time().now().to_sec()
    	while not rospy.is_shutdown():
	    if started:
                new_time = rospy.Time().now().to_sec()
                dt = new_time-old_time
		if v is None:
                    EKF.predict(a, q, omega, dt)
		else:
		    EKF.predict_gnss(v, a, q, omega, dt)
		    v = None
                EKF.finish_loop()
                old_time = new_time
	    if imu:
	    	rospy.Subscriber("/an_device/Imu", Imu, callback_imu, queue_size=1)
	    if gnss:
                rospy.Subscriber("/an_device/Twist", Twist, callback_gnss, queue_size=1)
	    if lidar:
                rospy.Subscriber("/ekf_lidar", Odometry, callback_lidar, queue_size=1)
	    lidar_callback_done = False # Commented out -> Dead reckoning test
	    rate.sleep()
    except rospy.ROSInterruptException:
        pass

