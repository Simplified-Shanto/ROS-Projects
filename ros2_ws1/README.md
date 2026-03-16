# ROS 2 IMU Workspace

This workspace contains a ROS 2 Humble package for reading a Hiwonder / WIT-style 10-axis IMU over serial, publishing the data as `sensor_msgs/Imu`, and visualizing it in RViz.

The main package is:

- `src/wit_ros2_imu`

## Purpose

This workspace is meant for:

- reading IMU data from the Hiwonder IM10A-style serial device
- publishing IMU data on `/imu/data`
- visualizing orientation in RViz
- showing a simple rectangular dummy URDF model that moves with the IMU orientation

## Main Components

- `src/wit_ros2_imu/wit_ros2_imu/wit_ros2_imu.py`
  ROS 2 Python driver that reads the serial IMU and publishes `/imu/data`.

- `src/wit_ros2_imu/wit_ros2_imu/imu_tf_broadcaster.py`
  Subscribes to `/imu/data` and publishes a TF transform from `map` to `imu_link` using the IMU orientation.

- `src/wit_ros2_imu/launch/rviz_and_imu.launch.py`
  Launches the IMU driver, TF broadcaster, `robot_state_publisher`, and RViz.

- `src/wit_ros2_imu/rviz/imu.rviz`
  Saved RViz configuration for IMU display and robot model visualization.

- `src/wit_ros2_imu/urdf/im10a_dummy.urdf`
  A simple rectangular dummy model representing the IMU module.

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- serial IMU connected through USB

Useful packages:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-rviz2 \
  ros-humble-rviz-imu-plugin \
  ros-humble-robot-state-publisher \
  python3-serial \
  python3-numpy
```

## Build

From the workspace root:

```bash
cd /home/shanto/ROS-Projects/ros2_ws1
source /opt/ros/humble/setup.bash
colcon build --packages-select wit_ros2_imu
source install/setup.bash
```

## Run

Launch the IMU driver and RViz:

```bash
cd /home/shanto/ROS-Projects/ros2_ws1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wit_ros2_imu rviz_and_imu.launch.py
```

## Check IMU Data

In another terminal:

```bash
cd /home/shanto/ROS-Projects/ros2_ws1
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 topic echo /imu/data
```

Check rate:

```bash
ros2 topic hz /imu/data
```

List frames:

```bash
ros2 run tf2_tools view_frames
```

## Serial Port Notes

The launch file currently uses:

- port: `/dev/ttyUSB0`
- baud: `9600`

If your device appears on a different port, edit:

- `src/wit_ros2_imu/launch/rviz_and_imu.launch.py`

If permissions fail, make sure your user is in the `dialout` group:

```bash
sudo usermod -aG dialout $USER
newgrp dialout
```

## RViz Notes

This workspace uses:

- IMU topic: `/imu/data`
- TF frames: `map` and `imu_link`

The rectangular dummy model is attached to `imu_link` through the URDF, so when the IMU orientation updates, the model should rotate in RViz.

## Git Notes

This workspace should be committed without generated folders:

- `build/`
- `install/`
- `log/`

Those are already ignored in `.gitignore`.
