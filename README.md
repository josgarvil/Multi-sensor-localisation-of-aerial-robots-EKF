
---

## 📘 `Multi-sensor-localisation-of-aerial-robots-EKF`

```markdown
# Multi-sensor Localisation of Aerial Robots using EKF

## 🧠 Description

This ROS-based project fuses multiple sensor sources (IMU, GPS, etc.) to estimate the position of an aerial robot using an Extended Kalman Filter (EKF). The output is a real-time pose estimation suitable for navigation and control.

## 🔧 Installation

```bash
git clone https://github.com/josgarvil/Multi-sensor-localisation-of-aerial-robots-EKF.git

cd ~/catkin_ws/src
ln -s /path/to/Multi-sensor-localisation-of-aerial-robots-EKF ekf_localisation
cd ~/catkin_ws
catkin_make
source devel/setup.bash
