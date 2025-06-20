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
```

## 🚀 Usage

```bash
roslaunch ekf_localisation localisation.launch
```

* Subscribes to IMU, GPS, and other sensor topics

* Runs EKF to estimate robot pose

* Publishes output to /robot_pose

## 📁 Project Structure

```arduino
ekf_localisation/
├── launch/
│   └── localisation.launch
├── scripts/
│   └── ekf_node.py
├── config/
│   └── sensors.yaml
```
## 🛠️ Technologies

* ROS
* Python
* Extended Kalman Filter (EKF)
* Sensor Fusion

## 👨‍💻 Author
José García Villalón – GitHub