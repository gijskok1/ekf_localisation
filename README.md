# Quick summary
This repository contains the entire ROS package for a localisation EKF. In the /src/scripts folder, 3 separate ROS nodes are found. 
These are the LiDAR merging node, GNSS merging node, and EKF node.

The scripts are written in Python 2. The only library used is numpy.

# Tuning
In the GNSS merging node, as well as the EKF node, the estimated variances for GNSS orientation estimates and sensor input noise can be further tuned.
