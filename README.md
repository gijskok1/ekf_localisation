# Quick summary
This repository contains 2 versions of the localisation EKF for the BEP "Sensor fusion for self-driving vehicle localisation". The revision contains the latest version. This contains a GNSS merging node, LiDAR merging node, and EKF node.

The scripts are written in Python 2. The only library used is numpy.

# Tuning
In the EKF node, the estimated variances for GNSS orientation and velocity estimates, as well as sensor input noise can be further tuned.

# NOTE
This EKF only uses the GNSS for orientation, velocity and relative displacement estimates as as significant difference between GNSS and map-based LiDAR localisation was discovered when both are supposed to be accurate. The LiDAR was chosen as leading sensor and is thus the the Kalman filter observer.
