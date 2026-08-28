# Direct LiDAR-Inertial Odometry (DLIO) Setup

This document describes the Direct LiDAR-Inertial Odometry (DLIO) setup used by the cloud mapping pipeline.

## 1. Hardware and Role

The system uses a **Livox Mid-360** LiDAR with its built-in IMU.

DLIO combines the LiDAR point-cloud data and IMU measurements to estimate the sensor's motion and produce a continuously updated odometry estimate. This allows the mapping pipeline to transform and accumulate incoming point clouds into a common coordinate frame.

In this project, DLIO sits between the Livox driver and the cloud mapping pipeline:

```text
Livox Mid-360
     │
     ├── /livox/lidar
     │
     └── /livox/imu
            │
            ▼
           DLIO
            │
            ├── Odometry
            └── Deskewed point cloud
                    │
                    ▼
             cloud_pipeline
```

The main outputs used by the mapping pipeline are the DLIO odometry and deskewed point cloud.

## 2. ROS 2 Package

The system uses:

- **Direct LiDAR-Inertial Odometry (DLIO)**
- **ROS 2 Humble**

DLIO is maintained in a separate ROS 2 workspace:

```text
~/dlio_ws
```

The relevant ROS 2 package is:

```text
direct_lidar_inertial_odometry
```

After building the workspace, the package should be available through:

```bash
ros2 pkg list | grep direct_lidar_inertial_odometry
```

Expected output:

```text
direct_lidar_inertial_odometry
```

## 3. Relationship with the Livox Driver

DLIO receives the data published by the Livox ROS 2 driver.

The cloud mapping pipeline currently uses:

| Data | ROS 2 topic |
|---|---|
| Mid-360 LiDAR | `/livox/lidar` |
| Mid-360 IMU | `/livox/imu` |

The Livox driver must therefore be running before DLIO can initialise correctly.

For this project, the Livox driver is configured with:

```python
xfer_format = 0
```

This causes `/livox/lidar` to be published as a standard:

```text
sensor_msgs/PointCloud2
```

rather than Livox's custom `CustomMsg` format.

This is important because the DLIO configuration used by the pipeline subscribes directly to `/livox/lidar`.

## 4. DLIO Workspace

The DLIO workspace is located at:

```text
~/dlio_ws
```

After installation or rebuilding, source the workspace with:

```bash
source ~/dlio_ws/install/setup.bash
```

It can also be sourced automatically as part of the main pipeline launch script.

## 5. Building DLIO

From a terminal:

```bash
source /opt/ros/humble/setup.bash

cd ~/dlio_ws

colcon build --symlink-install
```

Then source the resulting workspace:

```bash
source ~/dlio_ws/install/setup.bash
```

Verify that ROS 2 can find the package:

```bash
ros2 pkg list | grep direct_lidar_inertial_odometry
```

## 6. Launching DLIO

DLIO is launched using:

```bash
ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
    rviz:=false \
    pointcloud_topic:=/livox/lidar \
    imu_topic:=/livox/imu
```

The project disables DLIO's built-in RViz interface because the main pipeline launches its own RViz2 instance with the project's saved configuration.

The important parameters are:

```text
pointcloud_topic:=/livox/lidar
imu_topic:=/livox/imu
```

These connect DLIO to the Livox Mid-360 driver.

## 7. Important DLIO Outputs

DLIO produces several topics used for monitoring and mapping.

The main outputs used by this project include:

```text
/dlio/odom_node/pose
/odom
/pointcloud/deskewed
```

In particular, `/pointcloud/deskewed` provides the deskewed point cloud in the DLIO odometry frame. This is subsequently used by the cloud mapping pipeline to construct the accumulated global map.

The typical frame associated with the deskewed cloud is:

```text
odom
```

## 8. TF Relationship

DLIO provides the odometry frame used by the mapping pipeline.

The current system also publishes a static transform between the LiDAR-related frames as part of the launch script:

```text
lidar → livox_frame
```

The exact TF configuration should be kept consistent with the physical mounting and the frame names published by the Livox driver and DLIO.

## 9. Running the Complete Pipeline

The preferred method is to use the project's combined launch script rather than launching Livox and DLIO manually.

From the cloud pipeline repository:

```bash
cd ~/vision_ws_git

./scripts/run_mid360_dlio_mappipmocap-poisson.sh
```

The script performs the following sequence:

1. Sources ROS 2 Humble.
2. Sources the Livox driver workspace.
3. Sources the DLIO workspace.
4. Sources the cloud pipeline workspace.
5. Starts the Livox Mid-360 driver.
6. Checks that `/livox/lidar` is publishing `PointCloud2`.
7. Starts
