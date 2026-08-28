# Cloud Mapping Pipeline

A ROS 2-based LiDAR mapping pipeline for collecting, processing, and reconstructing 3D point clouds using a **Livox Mid-360**, **Direct LiDAR-Inertial Odometry (DLIO)**, and custom cloud-processing nodes.

The pipeline is designed to produce a globally aligned point-cloud map, estimate surface normals, identify and lock target contact points, and reconstruct the resulting point cloud as a 3D mesh.

---

## Overview

The system combines LiDAR, inertial odometry, point-cloud processing, and mesh reconstruction into a single pipeline.

```text
                         Livox Mid-360
                         LiDAR + IMU
                              │
                ┌─────────────┴─────────────┐
                │                           │
                ▼                           ▼
         /livox/lidar                 /livox/imu
                │                           │
                └─────────────┬─────────────┘
                              ▼
                             DLIO
                  Direct LiDAR-Inertial
                         Odometry
                              │
                ┌─────────────┴─────────────┐
                │                           │
                ▼                           ▼
          Odometry                  Deskewed cloud
                │                           │
                └─────────────┬─────────────┘
                              ▼
                       cloud_pipeline
                              │
                ┌─────────────┴─────────────┐
                │                           │
                ▼                           ▼
           Global map                 Surface normals
             (.pcd)                         │
                                            ▼
                                     Locked target
                                            │
                                            ▼
                                      Mesh reconstruction
                                            │
                                            ▼
                                          (.stl)
```

The complete system can be launched using a single script.

---

## Repository Structure

```text
vision_ws_git/
├── README.md
│
├── config/
│   └── map_pip.rviz
│
├── docs/
│   ├── livox_mid360_setup.md
│   └── dlio_setup.md
│
├── scripts/
│   ├── pcd_to_stl.py
│   └── run_mid360_dlio_mappipmocap-poisson.sh
│
└── src/
    └── cloud_pipeline/
        ├── CMakeLists.txt
        ├── package.xml
        └── src/
            ├── map_pip.cpp
            └── map_pip_mocap.cpp
```

---

## Hardware

The current system uses:

- **Livox Mid-360 LiDAR**
- Built-in IMU
- Ethernet connection between the LiDAR and computer

The Mid-360's LiDAR and IMU measurements are used together by DLIO to estimate the sensor trajectory and produce deskewed point clouds.

For detailed hardware and driver setup, see:

- [`docs/livox_mid360_setup.md`](docs/livox_mid360_setup.md)
- [`docs/dlio_setup.md`](docs/dlio_setup.md)

---

## Software Requirements

The pipeline has been developed and tested with:

- Ubuntu 22.04
- ROS 2 Humble
- Livox ROS Driver 2
- Direct LiDAR-Inertial Odometry (DLIO)
- PCL
- Python 3
- MeshLab

The system uses three ROS 2 workspaces:

```text
~/livox_ws
~/dlio_ws
~/vision_ws_git
```

### Workspace roles

| Workspace | Purpose |
|---|---|
| `~/livox_ws` | Livox ROS 2 driver |
| `~/dlio_ws` | Direct LiDAR-Inertial Odometry |
| `~/vision_ws_git` | This project's cloud-processing pipeline |

---

## Installation

### 1. Install ROS 2 Humble

Install ROS 2 Humble on Ubuntu 22.04 and ensure that the following works:

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

---

### 2. Install the Livox ROS 2 driver

The Livox driver is maintained in a separate workspace:

```text
~/livox_ws
```

See:

[`docs/livox_mid360_setup.md`](docs/livox_mid360_setup.md)

for the complete installation and configuration procedure.

For this project, the Mid-360 launch configuration must use:

```python
xfer_format = 0
```

This makes `/livox/lidar` publish a standard:

```text
sensor_msgs/PointCloud2
```

message rather than Livox's custom point-cloud message format.

---

### 3. Install DLIO

DLIO is maintained in:

```text
~/dlio_ws
```

See:

[`docs/dlio_setup.md`](docs/dlio_setup.md)

for installation and configuration instructions.

The project expects DLIO to provide:

```text
/livox/lidar
/livox/imu
```

as its input topics.

---

### 4. Build this project

Clone the repository:

```bash
cd ~
git clone https://github.com/TTNguyen32/vision_ws_git.git
```

Enter the workspace:

```bash
cd ~/vision_ws_git
```

Source ROS 2:

```bash
source /opt/ros/humble/setup.bash
```

Build:

```bash
colcon build --symlink-install
```

Then source the workspace:

```bash
source ~/vision_ws_git/install/setup.bash
```

Verify that the package is available:

```bash
ros2 pkg list | grep cloud_pipeline
```

Expected output:

```text
cloud_pipeline
```

---

# Configuration

## Livox Point Cloud Format

The Livox Mid-360 launch file must use:

```python
xfer_format = 0
```

This is important because the mapping pipeline expects:

```text
/livox/lidar
    sensor_msgs/PointCloud2
```

The launch file is located in the Livox workspace:

```text
~/livox_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
```

---

## ROS Topics

The main topics used by the pipeline are:

| Topic | Message type | Purpose |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Raw Mid-360 point cloud |
| `/livox/imu` | IMU message | Mid-360 IMU measurements |
| `/dlio/odom_node/pose` | `geometry_msgs/PoseStamped` | DLIO pose |
| `/odom` | `nav_msgs/Odometry` | DLIO odometry |
| `/pointcloud/deskewed` | `sensor_msgs/PointCloud2` | Deskewed/globalised point cloud |

The mapping pipeline primarily uses DLIO's deskewed point cloud and odometry information to accumulate the global map.

---

## RViz Configuration

The saved RViz2 configuration is stored at:

```text
config/map_pip.rviz
```

The main launch script automatically loads this configuration.

If the configuration is missing, the script will fall back to launching RViz2 with its default configuration.

---

# Running the Pipeline

The recommended way to run the complete system is through the provided launch script.

First, enter the repository:

```bash
cd ~/vision_ws_git
```

Make sure the script is executable:

```bash
chmod +x scripts/run_mid360_dlio_mappipmocap-poisson.sh
```

Then run:

```bash
./scripts/run_mid360_dlio_mappipmocap-poisson.sh
```

The script automatically sources the required workspaces:

```text
/opt/ros/humble
~/livox_ws
~/dlio_ws
~/vision_ws_git
```

It then starts the pipeline in sequence:

1. Livox Mid-360 driver
2. LiDAR topic verification
3. Previous target-normal check
4. DLIO
5. Static TF transform
6. RViz2
7. `cloud_pipeline`
8. Map collection
9. Mesh reconstruction
10. MeshLab

---

# Output Files

Pipeline outputs are stored outside the Git repository in:

```text
~/vision_ws_outputs/
```

The directory structure is:

```text
vision_ws_outputs/
├── maps/
├── meshes/
├── normals/
└── logs/
```

### Maps

Accumulated point clouds are saved in:

```text
~/vision_ws_outputs/maps/
```

Each run receives a timestamped filename:

```text
global_map_YYYYMMDD_HHMMSS.pcd
```

For example:

```text
global_map_20260828_153012.pcd
```

### Meshes

Reconstructed meshes are stored in:

```text
~/vision_ws_outputs/meshes/
```

with timestamped filenames:

```text
map_YYYYMMDD_HHMMSS.stl
```

### Normals

Persisted target-normal information is stored in:

```text
~/vision_ws_outputs/normals/
```

The currently used file is:

```text
locked_target.yaml
```

This allows a previously selected target normal to be reused in subsequent runs when the user confirms that the setup is unchanged.

### Logs

DLIO logs are stored in:

```text
~/vision_ws_outputs/logs/
```

Each run creates a timestamped log:

```text
dlio_YYYYMMDD_HHMMSS.log
```

---

# Target Normal Workflow

The pipeline supports saving and reusing a previously selected target normal.

During startup, the system checks:

```text
~/vision_ws_outputs/normals/locked_target.yaml
```

If a previous target exists, the user is asked whether it should be reused.

If the user chooses:

```text
y
```

the previous target normal is used.

Otherwise, a new normal is estimated from the newly collected map.

This is useful when repeatedly scanning the same physical setup and the previously established target frame remains valid.

---

# Mesh Reconstruction

After the global point cloud has been saved, the pipeline automatically runs:

```text
scripts/pcd_to_stl.py
```

The current reconstruction configuration uses Poisson surface reconstruction with:

```text
voxel size:       0.01
orientation:      sensor
Poisson depth:    8
keep largest:     enabled
remove outliers:  enabled
crop to input:    enabled
```

The command is equivalent to:

```bash
python3 scripts/pcd_to_stl.py \
    input.pcd \
    output.stl \
    --voxel 0.01 \
    --orient sensor \
    --poisson-depth 8 \
    --keep-largest \
    --remove-outliers \
    --crop-to-input
```

The resulting STL file can be opened automatically in MeshLab if MeshLab is installed.

---

# Troubleshooting

## `/livox/lidar` is not `PointCloud2`

Check the Livox Mid-360 launch file:

```text
~/livox_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py
```

Ensure:

```python
xfer_format = 0
```

You can check the topic type with:

```bash
ros2 topic type /livox/lidar
```

Expected:

```text
sensor_msgs/msg/PointCloud2
```

---

## DLIO does not start

Check that the Livox driver is publishing:

```bash
ros2 topic list | grep livox
```

Then check:

```bash
ros2 topic type /livox/lidar
ros2 topic type /livox/imu
```

The LiDAR topic should be:

```text
sensor_msgs/msg/PointCloud2
```

---

## DLIO topics are missing

Check:

```bash
ros2 topic list | grep dlio
```

and:

```bash
ros2 topic list | grep pointcloud
```

The main expected mapping output is:

```text
/pointcloud/deskewed
```

---

## RViz2 shows no point cloud

Check the active TF tree:

```bash
ros2 run tf2_tools view_frames
```

Also check the frame of the point cloud:

```bash
ros2 topic echo /pointcloud/deskewed --once
```

The fixed frame in RViz2 must be compatible with the published point-cloud frame.

---

## Mesh reconstruction fails

Check that the reconstruction script exists:

```bash
ls -l ~/vision_ws_git/scripts/pcd_to_stl.py
```

Check Python:

```bash
python3 --version
```

The script can also be tested independently:

```bash
python3 scripts/pcd_to_stl.py --help
```

---

# Development

The ROS 2 package is located at:

```text
src/cloud_pipeline/
```

After modifying the source code, rebuild with:

```bash
cd ~/vision_ws_git
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Check the working tree:

```bash
git status
```

Before committing changes, it is useful to check for whitespace errors:

```bash
git diff --check
```

---

# Git Workflow

The repository uses `main` as its primary branch.

Typical workflow:

```bash
cd ~/vision_ws_git

git status

git add .
git commit -m "Describe the change"

git push
```

Generated build files and local experiment outputs are excluded using `.gitignore`.

In particular:

```text
build/
install/
log/
vision_ws_outputs/
```

are not committed to the repository.

---

# Documentation

Additional setup documentation is available in the `docs/` directory:

- **Livox Mid-360:** [`docs/livox_mid360_setup.md`](docs/livox_mid360_setup.md)
- **DLIO:** [`docs/dlio_setup.md`](docs/dlio_setup.md)

These documents contain the detailed peripheral installation and configuration instructions, while this README provides the overall project workflow.

---

# Project Status

The current pipeline supports:

- Livox Mid-360 LiDAR input
- Mid-360 built-in IMU input
- DLIO LiDAR-inertial odometry
- Deskewed point-cloud accumulation
- Global map generation
- Surface normal estimation
- Target normal selection and persistence
- Previous normal reuse
- Poisson mesh reconstruction
- STL output
- Automatic MeshLab visualisation
- Timestamped experiment outputs
- Separate logging and output directories

---

## License

Add the appropriate project license here if/when one is selected.

## Author

**Thanh Tin Nguyen**

Cambridge University | ttn32@cam.ac.uk
