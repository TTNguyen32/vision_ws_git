# vision_ws_git — A*STAR Drone Vision Pipeline

A ROS 2 (Humble) LiDAR mapping pipeline for collecting, processing, and reconstructing 3D point clouds, and for selecting tree-contact normals and generating a UAV inspection path from them. Supports two pose sources — **DLIO** (LiDAR-inertial odometry, via a **Livox Mid-360**) and **OptiTrack** motion capture — sharing the same downstream mapping, meshing, normal-selection, and path-generation stages.

---

## Overview

```text
   Livox Mid-360 (LiDAR + IMU)                OptiTrack (rigid body: Tin_LIDAR)
            │                                            │
            ▼                                            ▼
          DLIO                              optitrack_packages_ros2
  (odometry, deskewing, pose)          (fork: src/optitrack_packages_ros2,
            │                           submodule — see docs/optitrack_setup.md)
            │                                            │
            ▼                                            ▼
map_pip_dlio_multinormals          map_pip_optitrack_tf  /  map_pip_optitrack_slerp
            │                                            │
            └────────────────────┬───────────────────────┘
                                  ▼
                       Point-cloud accumulation
                (voxel filter, duplicate removal,
                      statistical outlier removal)
                                  │
                                  ▼
                        Global map (.pcd)
                                  │
                    ┌─────────────┴─────────────┐
                    ▼                           ▼
              Mesh (Poisson)              Surface normals
      scripts/lidar-tools/pcd_to_stl.py         │
                    │                            ▼
                    │              Interactive normal selection (RViz clicks)
                    │                            │
                    │                      locked_target.yaml
                    │                            │
                    └────────────┬───────────────┘
                                  ▼
                      Path generation (offset_spline.py)
                                  │
                                  ▼
                          UAV inspection path
```

Collection, mesh generation, normal selection, and path generation are controlled through **`vision_pip_gui`**, an rqt panel (`src/vision_pip_gui/`). Shell pane scripts (`scripts/collect.sh`, `mesh.sh`, `select.sh`, `path.sh`) are kept as the reference implementation the GUI is ported from, and remain usable standalone.

---

## Repository structure

```text
vision_ws_git/
├── README.md
│
├── config/
│   ├── map_pip.rviz                    # DLIO RViz layout
│   ├── map_pip_optitrack.rviz          # OptiTrack RViz layout
│   └── optitrack/                      # reference copies of the fork's config
│       ├── optitrack_multiplexer_config.yaml
│       ├── optitrack_wrapper_config.yaml
│       └── wrapper_and_multiplexer.launch.py
│
├── docs/
│   ├── livox_mid360_setup.md
│   ├── dlio_setup.md
│   └── optitrack_setup.md              # network, naming, submodule workflow, debugging notes
│
├── scripts/
│   ├── common.sh                       # shared paths/helpers, sourced by every pane script
│   ├── collect.sh  mesh.sh  select.sh  path.sh
│   ├── load_params.py                  # pipeline_params.yaml -> CLI args (shell + node modes)
│   ├── pipeline_params.yaml            # every tunable: node, mesh (live/final), path
│   ├── run_vision_pip_dlio
│   ├── run_vision_pip_optitrack_tf         # LIDAR_EXT = 0,0,0
│   ├── run_vision_pip_optitrack_slerp
│   ├── run_vision_pip_optitrack_naive.sh   # pre-pane fallback, documented not deleted
│   ├── test_mesh.sh  test_path.sh          # standalone tuning harnesses, hardcoded paths
│   └── lidar-tools/
│       ├── pcd_to_stl.py               # PCD -> STL (Poisson / BPA / alpha / delaunay)
│       ├── offset_spline.py            # normals + mesh -> standoff path
│       ├── tf_displacement.py          # TF-edge displacement diagnostics
│       └── setup.sh                    # one-time `lidar` conda env bootstrap
│
└── src/
    ├── cloud_pipeline/                 # C++ nodes (ament_cmake)
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── src/
    │       ├── map_pip.cpp  map_pip_mocap.cpp  map_pip_mocap_normals.cpp
    │       ├── map_pip_dlio_multinormals.cpp  map_pip_dlio_multinormals_test.cpp
    │       ├── map_pip_optitrack_slerp.cpp  map_pip_optitrack_tf.cpp
    │       └── mocap_tf_broadcaster.cpp    # RViz-visualization-only TF bridge
    │
    ├── vision_pip_gui/                 # rqt panel (ament_python)
    │   ├── package.xml  plugin.xml  setup.py  setup.cfg
    │   └── vision_pip_gui/
    │       ├── pipeline_panel.py  common.py  services.py  tools.py  mesh_watcher.py
    │
    └── optitrack_packages_ros2/        # git submodule -> fork, project-specific config
```

---

## Hardware

- **Livox Mid-360 LiDAR** with built-in IMU, Ethernet-connected.
- **OptiTrack** motion capture, tracking a rigid body registered in Motive as `Tin_LIDAR`.

Livox and OptiTrack sit on different subnets (`192.168.1.x` and `192.168.50.x` respectively) and conflict on a single NIC — run both via two separate Ethernet adapters, one static-IP'd per subnet.

Detailed setup:

- [`docs/livox_mid360_setup.md`](docs/livox_mid360_setup.md)
- [`docs/dlio_setup.md`](docs/dlio_setup.md)
- [`docs/optitrack_setup.md`](docs/optitrack_setup.md)

---

## Software requirements

- Ubuntu 22.04, ROS 2 Humble
- Livox ROS Driver 2, Direct LiDAR-Inertial Odometry (DLIO)
- `optitrack_packages_ros2` (submodule, see Setup below)
- PCL, Eigen3, yaml-cpp
- `rqt_gui`, `rqt_gui_py`, `python_qt_binding` (for `vision_pip_gui`)
- Python 3, plus a `lidar` conda environment (Open3D, NumPy) for `scripts/lidar-tools/`
- MeshLab (optional, for viewing `.stl` output)

Three ROS 2 workspaces:

| Workspace | Purpose |
|---|---|
| `~/livox_ws` | Livox ROS 2 driver |
| `~/dlio_ws` | Direct LiDAR-Inertial Odometry |
| `~/vision_ws_git` | This project — `cloud_pipeline`, `vision_pip_gui`, and the OptiTrack submodule |

---

## Setup

### 1. ROS 2 Humble

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

### 2. Livox ROS 2 driver

Maintained in `~/livox_ws` — see [`docs/livox_mid360_setup.md`](docs/livox_mid360_setup.md). The Mid-360 launch config must use `xfer_format = 0`, so `/livox/lidar` publishes a standard `sensor_msgs/PointCloud2` rather than Livox's custom message type.

### 3. DLIO

Maintained in `~/dlio_ws` — see [`docs/dlio_setup.md`](docs/dlio_setup.md). Expects `/livox/lidar` and `/livox/imu` as input.

### 4. Clone this repository (with the OptiTrack submodule)

```bash
cd ~
git clone --recurse-submodules https://github.com/TTNguyen32/vision_ws_git.git
cd vision_ws_git
```

Already cloned without `--recurse-submodules`?

```bash
git submodule update --init
```

The submodule is a fork of [`lis-epfl/optitrack_packages_ros2`](https://github.com/lis-epfl/optitrack_packages_ros2) with project-specific config (rigid body name, world frame, server address) committed directly in it — see [`docs/optitrack_setup.md`](docs/optitrack_setup.md) for the update workflow.

### 5. `lidar-tools` conda environment

```bash
scripts/lidar-tools/setup.sh
```

### 6. Build

**Deactivate conda first** — `colcon`/`cmake` need system Python (which has ROS's apt-installed dependencies like `catkin_pkg` and `PyYAML`); an active conda base env shadows `python3` on `PATH` and breaks the build with `ModuleNotFoundError`.

```bash
conda deactivate
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to cloud_pipeline vision_pip_gui
source install/setup.bash
```

`--packages-up-to`, not `--packages-select` — `cloud_pipeline` depends on message packages generated inside the `optitrack_packages_ros2` submodule, and `--packages-up-to` resolves the full dependency graph from every package's `package.xml` in one pass. `--symlink-install` matters especially for `vision_pip_gui`, a pure-Python package — edits to its `.py` files take effect without a rebuild.

Verify:

```bash
ros2 pkg list | grep -E 'cloud_pipeline|vision_pip_gui'
```

---

## Configuration

### Livox point-cloud format

`~/livox_ws/src/livox_ros_driver2/launch_ROS2/msg_MID360_launch.py` must set `xfer_format = 0`. Check with:

```bash
ros2 topic type /livox/lidar   # expect sensor_msgs/msg/PointCloud2
```

### ROS topics

| Topic | Message type | Purpose |
|---|---|---|
| `/livox/lidar` | `sensor_msgs/PointCloud2` | Raw Mid-360 point cloud |
| `/livox/imu` | IMU message | Mid-360 IMU measurements |
| `/dlio/odom_node/pose` | `geometry_msgs/PoseStamped` | DLIO pose |
| `/dlio/odom_node/pointcloud/deskewed` | `sensor_msgs/PointCloud2` | Deskewed cloud (DLIO path) |
| `/optitrack_multiplexer_node/rigid_body/Tin_LIDAR` | `RigidBodyStamped` | Mocap pose (OptiTrack path) |
| `/global_map` | `sensor_msgs/PointCloud2` | Accumulated global map |
| `/processed/mesh` | `visualization_msgs/MarkerArray` | Live/final mesh, in RViz |
| `/processed/selected_normals`, `/processed/path` | — | Selected contact normals, generated path |

### RViz

- DLIO: `config/map_pip.rviz`
- OptiTrack: `config/map_pip_optitrack.rviz`

### Tuning

All node/mesh/path parameters live in `scripts/pipeline_params.yaml`, read once at launch:

```bash
PARAMS_FILE=~/experiments/deep_mesh.yaml scripts/run_vision_pip_dlio
```

---

## Running

```bash
scripts/run_vision_pip_dlio              # DLIO pose source
scripts/run_vision_pip_optitrack_tf      # OptiTrack, TF lookup + mocap_tf_broadcaster
scripts/run_vision_pip_optitrack_slerp   # OptiTrack, buffered pose interpolation
```

Then, the control panel:

```bash
ros2 run rqt_gui rqt_gui
# Plugins -> Vision Pipeline
```

Start/end collection, undo/save/load/clear normals, and generate/publish a path all happen from the panel. RViz clicks select normals; the panel's buttons drive everything else via `std_srvs/Trigger` service calls to the running node.

---

## Target-normal workflow

Selected normals are saved via the panel's Save button (or `select.sh`'s Enter key), with an option to overwrite the persistent `locked_target.yaml`. On a later run, Load pulls the locked set back into the current selection — useful when re-scanning the same physical setup where the previous contact points remain valid.

```text
run 1 → select normals → Save (+ overwrite locked target) → locked_target.yaml
run 2 → Load locked target → reuse contact targets
```

---

## Output files

Stored outside git, in `~/vision_ws_outputs/`:

```text
vision_ws_outputs/
├── latest -> run_YYYYMMDD_HHMMSS/     # symlink to the current/most recent run
└── run_YYYYMMDD_HHMMSS/
    ├── maps/     global_map_<ts>.pcd, snapshot_NNNNN.pcd
    ├── meshes/   map_<ts>.stl, live_NNNNN.stl
    ├── normals/  selected_normals_<ts>.yaml
    ├── paths/    path_<ts>.yaml, path_<ts>.csv
    ├── logs/     mesh.log, path.log
    └── params.yaml   # pipeline_params.yaml snapshot for this run
locked_target.yaml    # persistent, reused across runs
```

---

## Mesh reconstruction

Driven by `pipeline_params.yaml`'s `mesh.live` / `mesh.final` sections (live: fast, runs every snapshot during collection; final: quality, runs once after collection ends), via:

```bash
scripts/lidar-tools/pcd_to_stl.py
```

Can be run standalone for tuning:

```bash
python3 scripts/lidar-tools/pcd_to_stl.py --help
```

---

## Troubleshooting

**`/livox/lidar` is not `PointCloud2`** — check `xfer_format = 0` in the Mid-360 launch file; confirm with `ros2 topic type /livox/lidar`.

**DLIO does not start** — check the Livox driver is publishing: `ros2 topic list | grep livox`, then `ros2 topic type /livox/lidar` / `/livox/imu`.

**DLIO topics are missing** — `ros2 topic list | grep dlio`; expect `/dlio/odom_node/pointcloud/deskewed`.

**OptiTrack topic has a publisher but zero messages** — `rigid_body_names` in `optitrack_multiplexer_config.yaml` must match Motive's registered rigid-body name *exactly*; a mismatch produces a connected-but-silent topic with no error. Confirm the real name via a data-descriptions service call.

**RViz shows no point cloud** — check the TF tree (`ros2 run tf2_tools view_frames`) and the cloud's actual frame (`ros2 topic echo /dlio/odom_node/pointcloud/deskewed --once`) against RViz's fixed frame.

**`colcon build` fails with `ModuleNotFoundError: No module named 'catkin_pkg'`** (or `'yaml'`) — an active conda environment is shadowing system Python on `PATH`. Run `conda deactivate` before building or running anything through `ros2`/`colcon`.

**`ros2 run cloud_pipeline <exe>` can't find the executable after a rename** — check `project(...)` at the top of `CMakeLists.txt` matches `package.xml`'s `<name>`; `install(TARGETS ... DESTINATION lib/${PROJECT_NAME})` resolves from `project()`, independently of `package.xml`.

**A clean clone fails to build `cloud_pipeline`** — known issue, `map_pip_mocap_normals.cpp` / `map_pip_dlio_multinormals_test.cpp` not yet reliably tracked/pushed; see `docs/optitrack_setup.md`.

Full debugging log from the DLIO → OptiTrack/rqt migration: [`docs/optitrack_setup.md`](docs/optitrack_setup.md).

---

## Development

```bash
cd ~/vision_ws_git
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to cloud_pipeline vision_pip_gui
source install/setup.bash
```

```bash
git status
git diff --check       # whitespace errors before committing
```

---

## Git workflow

```bash
cd ~/vision_ws_git
git status
git add <files>
git diff --cached       # read it before committing
git commit -m "Describe the change"
git push origin HEAD    # not a hardcoded branch name — avoids branch-name mismatches
```

`build/`, `install/`, `log/`, `vision_ws_outputs/` should be excluded via `.gitignore` — verify with `git ls-files | grep -E '^(build|install|log)/'` (should print nothing).

The `optitrack_packages_ros2` submodule is a separate repository — config changes there need their own commit + push before the outer repo's gitlink is updated:

```bash
cd src/optitrack_packages_ros2
git add -A && git commit -m "..." && git push origin HEAD
cd ../..
git add src/optitrack_packages_ros2
git commit -m "Pin optitrack_packages_ros2 submodule to <what changed>"
```

---

## Documentation

- [`docs/livox_mid360_setup.md`](docs/livox_mid360_setup.md) — Livox hardware/driver setup
- [`docs/dlio_setup.md`](docs/dlio_setup.md) — DLIO setup
- [`docs/optitrack_setup.md`](docs/optitrack_setup.md) — network/naming, fork+submodule workflow, build dependency ordering, debugging notes
- [`docs/mesh_and_path_parameters.md`](docs/mesh_and_path_parameters.md) — every `pcd_to_stl.py` / `offset_spline.py` flag, with measured effects

---

## Project status

- DLIO and OptiTrack pose sources, both feeding the same mapping/meshing/normal-selection/path pipeline
- Live and final mesh reconstruction (Poisson), interactive multi-normal selection with undo, locked-target persistence
- `vision_pip_gui` rqt panel controlling collection, selection, and path generation
- **Known issue:** clean-clone build currently fails (see Troubleshooting) — fix pending

## License

Add the appropriate project license here if/when one is selected.

## Author

**Thanh Tin Nguyen**
Cambridge University | ttn32@cam.ac.uk
