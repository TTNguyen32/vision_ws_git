# Livox Mid-360 Setup

This document describes the Livox Mid-360 LiDAR setup used by the cloud
mapping pipeline.

## 1. Hardware

The system uses a **Livox Mid-360** LiDAR with its built-in IMU.

The sensor communicates with the computer over Ethernet. Please follow the setup instruction on Livox's official website:https://www.sachtleben-technology.com/assets/downloads/livox-mid-360-user-manual.pdf

The IMU data is used by DLIO for LiDAR-inertial odometry and point-cloud
transformation and accumulation.

### 1.1 Specifications

Figures below are from the Livox Mid-360 User Manual (v1.2, 2024.04). Check
them against the manual for your own unit before wiring anything — this doc is
a convenience copy, not the authority.

| Parameter | Value |
| --- | --- |
| Supply voltage | <cite index="49-1">9–27 V DC, 12 V recommended</cite> |
| Working power | <cite index="51-1">6.5 W at 25 °C ambient</cite> |
| Startup peak power | <cite index="46-1">up to 18 W, for roughly 8 s</cite> |
| Self-heating power | <cite index="50-1">peak up to 14 W, entered automatically between −20 °C and 0 °C</cite> |
| Operating temperature | −20 °C to +55 °C |
| Connector | <cite index="60-1">M12 A-code 12-pin aviation connector (male)</cite> |
| Dimensions / weight | 65 × 65 × 60 mm, 265 g |
| IP rating | IP67 |
| FOV | horizontal 360°, vertical −7° to +52° |
| Detection range | <cite index="51-1">40 m @ 10 % reflectivity, 70 m @ 80 % reflectivity</cite> |
| Close-proximity blind zone | 0.1 m |
| Point rate | 200,000 points/s |
| Frame rate | 10 Hz typical |
| Laser | <cite index="51-1">905 nm, Class 1 (IEC 60825-1:2014)</cite> |
| Built-in IMU | <cite index="46-1">ICM-40609, ±4 g accelerometer, ±2000 °/s gyroscope</cite> |

### 1.2 Powering the sensor

**Size the supply for the startup peak, not the running figure.** The 6.5 W
average is what the Mid-360 draws once running; the <cite index="46-1">startup peak reaches 18 W for
about 8 seconds</cite>. A supply rated only for steady-state will brown out at
power-on and the unit will fail to enumerate. At 12 V that peak is ~1.5 A, so
a 12 V / 2 A supply is a sensible floor.

The vertical FOV constraints relevant to the pipeline's crop sector
(−7° to +52°) are the sensor's, not a software limit — see the FOV section in
the main README.

Points to watch when wiring:

- <cite index="49-1">Do not exceed 27 V. Power cables can generate voltage fluctuation above
  27 V in some scenarios, which may stop the Mid-360 working normally or
  damage it.</cite> If the supply is shared with motors or other inductive loads,
  that is exactly the scenario to worry about.
- The minimum working voltage should be raised in a low-temperature
  environment.
- <cite index="50-1">Between −20 °C and 0 °C the unit enters self-heating mode automatically,
  where the power draw peaks around 14 W</cite> — relevant if this ever flies
  somewhere cold, as the supply has to cover that too.
- Running on battery (as on the drone): check the pack's voltage at the point
  it is *most discharged*, not nominal, against the 9 V floor.

### 1.3 M12 connector and splitter cable

<cite index="59-1">The Livox Aviation Connector 1-to-3 Splitter Cable has an M12 aviation
connector (female) on one end; the other end splits into power, Ethernet, and
function cables.</cite> Power is on <cite index="53-1">pins 1 and 9 (Power+, DC 9–27 V, red)</cite>.

Assembly order:

1. Connect the splitter's M12 female connector to the Mid-360's M12 male
   connector. <cite index="59-1">Tighten the lock nut with a wrench to ensure a secure
   connection.</cite> Align the positioning key with the slot before pushing.
2. Connect the RJ-45 to the computer.
3. Connect the power cable's bare leads to the external supply, observing
   polarity.
4. The function cable is only needed for GPS time synchronisation — unused in
   this pipeline.

<cite index="60-1">Livox's own note is that the 1-to-3 splitter cable is intended for testing and
debugging, and that a customised cable is recommended for scenarios requiring
high reliability</cite> — worth bearing in mind for a flight-worthy build, where
the bare-lead power connection and the cable's weight both matter.

**Laser safety:** Class 1 and eye-safe in normal operation, but do not
disassemble the unit — that defeats the classification.

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
#### 2.1.2 Clone the driver
Clone the livox_ros_driver2 repository:
```bash
git clone https://github.com/Livox-SDK/livox_ros_driver2.git
```

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

In this project the Mid-360 and OptiTrack sit on **different subnets**
(`192.168.1.x` and `192.168.50.x`) and conflict on a single NIC — both run via
two separate Ethernet adapters, one static-IP'd per subnet. See
`docs/optitrack_setup.md`.

If the driver starts but no LiDAR data is received, check:
1. Ethernet connection between the computer and Mid-360
2. Computer network interface configuration
3. Mid_360 IP configuration
4. MID360_config.json
5. Driver startup output

## 10. Troubleshooting

### LiDAR does not power on, or drops out at startup
Check the supply can deliver the **startup peak** (up to 18 W for ~8 s), not
just the 6.5 W running figure. A supply sized for steady state will brown out
at power-on. Then check the voltage at the connector while the unit starts —
not at the supply terminals — since cable resistance can drop it below the
9 V floor under the startup surge.

### LiDAR works on the bench but not on the drone
Usually supply, not software. Measure pack voltage at its most discharged
state against the 9 V minimum, and check whether the Mid-360 shares a rail
with motors or other inductive loads — <cite index="49-1">voltage fluctuation above 27 V on the
power cable can stop the unit working or damage it</cite>.

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

### Points closer than 10 cm are missing
Expected — the close-proximity blind zone is 0.1 m. The pipeline's
`fov_min_range` default (0.1 m) matches it, and also drops the `(0,0,0)`
no-return beams.
