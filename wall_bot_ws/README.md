# `wall_bot_ws`

ROS 2 Humble workspace for the `wall_bot` robot.

Current active stack:
- `wall_bot_description`: URDF/Xacro, RViz config
- `wall_bot_gazebo`: Gazebo world + `ros2_control` + Ackermann controller
- `wall_follower`: LiDAR wall follower + line counter nodes

## 1. Build The Workspace

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
colcon build --packages-select wall_bot_description wall_bot_gazebo wall_follower
source install/setup.bash
```

## 2. Source In Any New Terminal

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
```

## 3. Kill Old Gazebo Processes

Use this before relaunching Gazebo if a previous run is still alive.

```bash
pkill -f gzserver
pkill -f gzclient
pkill -f gazebo
```

## 4. Package Commands

### `wall_bot_description`

Show the robot in RViz with `robot_state_publisher` and `joint_state_publisher_gui`:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch wall_bot_description display.launch.py
```

### `wall_bot_gazebo`

Launch Gazebo, spawn the robot, start `joint_state_broadcaster`, and start `ackermann_steering_controller`:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 launch wall_bot_gazebo gazebo.launch.py
```

Check controller status:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 control list_controllers
```

Expected active controllers:
- `joint_state_broadcaster`
- `ackermann_steering_controller`

#### Manual Ackermann Command Test

Drive straight:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 topic pub -r 10 /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.25}, angular: {z: 0.0}}}"
```

Turn left:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 topic pub -r 10 /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.20}, angular: {z: 0.50}}}"
```

Turn right:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 topic pub -r 10 /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.20}, angular: {z: -0.50}}}"
```

Stop:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 topic pub --once /ackermann_steering_controller/reference geometry_msgs/msg/TwistStamped "{header: {frame_id: 'base_link'}, twist: {linear: {x: 0.0}, angular: {z: 0.0}}}"
```

### `wall_follower`

Available executables:
- `wall_follower_node`
- `line_counter_node`

Run the wall follower node:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 run wall_follower wall_follower_node
```

Run the wall follower node with a parameter override:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 run wall_follower wall_follower_node --ros-args -p linear_speed:=0.6
```

Run the line counter node:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 run wall_follower line_counter_node
```

#### Recommended Ackermann Runtime Flow

Terminal 1: Gazebo + controllers

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 launch wall_bot_gazebo gazebo.launch.py
```

Terminal 2: Wall follower

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 run wall_follower wall_follower_node
```

Terminal 3: Line counter

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 run wall_follower line_counter_node
```

#### Legacy Combined Launch Files

These launch files are present in the package:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 launch wall_follower wall_follower.launch.py
```

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 launch wall_follower lap_mission.launch.py
```

Parameter override example for the launch file:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 launch wall_follower lap_mission.launch.py linear_speed:=0.6 lap_count:=3
```

Note:
- the recommended current setup is still `wall_bot_gazebo/gazebo.launch.py` plus running `wall_follower_node` and `line_counter_node` separately
- the combined `wall_follower` launch files are older convenience launch files kept in the repo

## 5. Useful Debug Commands

Check package executable names:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 pkg executables wall_follower
```

Check joint states:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 topic echo /joint_states
```

Check odometry:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 topic echo /odom
```

Check line counter output:

```bash
cd ~/Github/wall_bot_ws
source /opt/ros/humble/setup.bash
source /usr/share/gazebo/setup.sh
source install/setup.bash
ros2 topic echo /line_count
```

## 6. Manual Stop In `wall_follower_node`

While `wall_follower_node` is running:
- press `Ctrl+S` to request a manual stop
- if your terminal captures `Ctrl+S`, press `s` instead

