# Multi-sensor Localisation of Aerial Robots using EKF

[![ROS](https://img.shields.io/badge/ROS-Melodic-brightgreen)](https://wiki.ros.org/melodic)
[![Python](https://img.shields.io/badge/Python-3.6-blue)](https://www.python.org/)
[![Gazebo](https://img.shields.io/badge/Gazebo-9-red)](https://gazebosim.org/)
[![MATLAB](https://img.shields.io/badge/MATLAB-R2018b-red)](https://www.mathworks.com/products/matlab.html)

## Description

This ROS-based project fuses multiple sensor sources (IMU, GPS, etc.) to estimate the position of an aerial robot using an Extended Kalman Filter (EKF). The output is a real-time pose estimation suitable for navigation and control.

## Installation

```bash
# Clone the repository
git clone https://github.com/josgarvil/Multi-sensor-localisation-of-aerial-robots-EKF.git

# Add to your ROS workspace
cd ~/catkin_ws/src
ln -s /path/to/Multi-sensor-localisation-of-aerial-robots-EKF ekf_localisation
cd ~/catkin_ws
catkin_make
source devel/setup.sh
```

## Usage

```bash
# ROS
catkin_make && roscore
source devel/setup.sh && roslaunch ekf ekf.launch
source devel/setup.sh && rosservice call /enable_motors "enable: true" && rosrun teleop_twist_keyboard teleop_twist_keyboard.py
source devel/setup.sh && rosrun ekf ekf.py
# or 
source devel/setup.sh && roslaunch ekf simulation.launch

# MATLAB
# Load EKF.m and run
```

* Subscribes to IMU, GPS, and other sensor topics
* Runs EKF to estimate robot pose
* Publishes output to /robot_pose
* Visualize the map using RViz
* Launch Gazebo simulation
* Recreate in MATLAB environment

## Technologies

* ROS
* Python
* Extended Kalman Filter (EKF)
* Sensor Fusion

## Author
José García Villalón