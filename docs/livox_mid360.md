# Livox Mid-360 Setup

This document describes the Livox Mid-360 LiDAR setup used by the cloud
mapping pipeline.

## 1. Hardware

The system uses a **Livox Mid-360** LiDAR with its built-in IMU.

The sensor communicates with the computer over Ethernet. Please follow the setup instruction on Livox's official website:https://www.sachtleben-technology.com/assets/downloads/livox-mid-360-user-manual.pdf 

The IMU data is used by DLIO for LiDAR-inertial odometry and point-cloud
transformation and accumulation.

## 2. ROS 2 Driver

The LiDAR is driven using:

- `livox_ros_driver2`
- ROS 2 Humble

The driver is maintained in a separate ROS 2 workspace:
```bash
~/livox_ws
```
The relevant package in this project is livox_ros_driver2

The repository contains the ROS2 launch files under:
```bash
~/livox_ws/src/livox_ros_driver2/launch_ROS2/
```

### 2.1 Installing the Livox ROS 2 Driver

The pipeline uses the `livox_ros_driver2` ROS 2 driver.

#### 2.1.1 Create a Livox workspace

```bash
mkdir -p ~/livox_ws/src
cd ~/livox_ws/src
```
#### 2.1.3 Clone the driver
Clone the livox_ros_driver2 repository:
git clone https://github.com/Livox-SDK/livox_ros_driver2.git

#### 2.1.3 Build the driver
Source ROS 2 Humble:
```bash
source /opt/ros/humble/setup.bash
```

Then build:
```bash
cd ~/livox_ws
colcon build
```
After a successful build:
```bash
source ~/livox_ws/install/setup.bash
```

#### 2.1.4 Verify the package
Check that ROS 2 can find the driver:
```bash
ros2 pkg list | grep livox
```
You should see:
livox_ros_driver2

## 3. Mid-360 launch file

The pipeline uses:
msg_MID360_launch.py

located at ~/livox_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py

The Mid-360 configuration is loaded from 
```bash
~/livox_ws/src/livox_ros_driver2/config/MID360_config.json
```

## 4. Point Cloud Message Format

An important configuration is 
xfer_format = 0 in msg_MID360_launch.py

The Livox driver supports two output formats:
0 → sensor_msgs/PointCloud2
1 → Livox custom point-cloud format

This pipeline requires xfer_format = 0 because DLIO subscribes to the standard ROS2 sensor_msgs/PointCloud2.

The resulting LiDAR topic is /livox/lidar with message type sensor_msgs/msg/PointCloud2.
The PointCloud2 message contains per-point timing information required by the downstream odometry/ deskewing pipeline

## 5. IMU
The Mid-360's built in IMU is published on /livox/imu. DLIO uses this IMU together with the LiDAR point cloud for LiDAR-inertial odometry

## 6. Starting the Driver

Source ROS2 and the Livox workspace:
```bash
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
```
Launch the Mid-360:
```bash
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

## 7. Verify the LiDAR Topic

Check that the point cloud is being published:
```bash
ros2 topic list | grep livox
```

Check the message type: 
```bash
ros2 topic type /livox/lidar
```
(Expected sensor_msgs/msg/PointCloud2).

Check the IMU: 
```bash
ros2 topic echo /livox/imu --once
```
Check the point cloud: 
```bash
ros2 topic echo /livox/lidar --once
```
## 8. Verify the Point Cloud in RViz2

Start Rviz2: 
```bash
rviz2
```
Add a PointCloud2 display and select: 
```bash
/livox/lidar.
```
Set the appropriate fixed frame according to the current TF configuration. 

## 9. Network configuration

Mid-360 communicates through Ethernet.

If the driver starts but no LiDAR data is received, check:
1. Ethernet connection between the computer and Mid-360
2. Computer network interface configuration
3. Mid_360 IP configuration
4. MID360_config.json
5. Driver startup output

## 10. Troubleshotting

### /livox/lidar has the wrong message type
Check: 
```bash
ros2 topic type /livox/lidar
```
If the result is not: sensor_msgs/msg/PointCloud2
check: 
```bash
~/livox_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
```
and ensure: xfer_format = 0

### No LiDAR data
Check: 
```bash
ros2 topic hz /livox/lidar
```
and: 
```bash
ros2 topic hz /livox/imu
```
If neither topic is publishing, check the Ethernet connection and Livox driver configuration.

### Driver launches but DLIO does not receive the cloud
First verify: 
```bash
ros2 topic type /livox/lidar
```
The topic must be: sensor_msgs/msg/PointCloud2
Then verify that the topic is publishing: 
```bash
ros2 topic hz /livox/lidar
```
Finally check that DLIO is configured to subscribe to:
/livox/lidar
and: /livox/imu
