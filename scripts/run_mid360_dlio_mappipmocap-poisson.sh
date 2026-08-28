#!/bin/bash
# run_mid360_dlio_mappipmocap_poisson.sh
# Brings up the Livox Mid-360 driver, DLIO (feature/ros2 branch), RViz2 with a
# custom saved config, and finally the map_pip (cloud_pipeline) node.
# Mirrors the workspace-sourcing / trap-cleanup pattern used in run_mid360_fastlio.sh
# and run_mid360_dlio.sh.
#
# Requires: livox_ros_driver2 built in ~/livox_ws, DLIO built in ~/dlio_ws,
# cloud_pipeline built in ~/vision_ws_git, and xfer_format = 0 set in the Livox
# launch file (msg_MID360_launch.py) so /livox/lidar publishes
# sensor_msgs/PointCloud2 with per-point timestamps rather than CustomMsg.

set -e

# --- Timestamp for this run ---
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# --- Output directories ---
OUTPUT_DIR="$HOME/vision_ws_outputs"
MAP_DIR="$OUTPUT_DIR/maps"
MESH_DIR="$OUTPUT_DIR/meshes"
NORMAL_DIR="$OUTPUT_DIR/normals"
LOG_DIR="$OUTPUT_DIR/logs"

# Create output directories if they don't exist
mkdir -p "$MAP_DIR" "$MESH_DIR" "$NORMAL_DIR" "$LOG_DIR"

# --- Path to saved RViz2 config ---
RVIZ_CONFIG="$HOME/vision_ws_git/config/map_pip.rviz"

# --- Path to DLIO log ---
DLIO_LOG="$LOG_DIR/dlio_${TIMESTAMP}.log"

# --- Source workspaces in order ---
source /opt/ros/humble/setup.bash
source ~/livox_ws/install/setup.bash
source ~/dlio_ws/install/setup.bash
source ~/vision_ws_git/install/setup.bash

# --- Cleanup: kill all child processes on exit/ctrl-c ---
DLIO_TAIL_PID=""
cleanup() {
    echo ""
    echo "Shutting down..."
    kill $(jobs -p) 2>/dev/null
    # jobs -p only tracks direct children; the tail's Terminator window is a
    # separate detached process, so kill it explicitly if it was started.
    [[ -n "$DLIO_TAIL_PID" ]] && kill "$DLIO_TAIL_PID" 2>/dev/null
    wait 2>/dev/null
}
trap cleanup EXIT INT TERM

# --- Launch Livox Mid-360 driver ---
echo "[1/4] Starting Livox Mid-360 driver..."
ros2 launch livox_ros_driver2 msg_MID360_launch.py &

# Give the driver time to connect and start publishing before DLIO subscribes
sleep 3

# --- Sanity check: confirm /livox/lidar is PointCloud2, not CustomMsg ---
TOPIC_TYPE=$(ros2 topic type /livox/lidar 2>/dev/null || echo "unknown")
if [[ "$TOPIC_TYPE" != *"PointCloud2"* ]]; then
    echo "WARNING: /livox/lidar is reporting '$TOPIC_TYPE', not PointCloud2."
    echo "Check that xfer_format = 0 in msg_MID360_launch.py."
fi

# ---- Check previous normal and ask if user wants to reuse normal
NORMAL_FILE="$NORMAL_DIR/locked_target.yaml"

USE_PREVIOUS_NORMAL=false

if [[ -f "$NORMAL_FILE" ]]; then
    echo ""
    echo "========================================="
    echo "Previous normal found:"
    echo "$NORMAL_FILE"
    echo "========================================="

    read -r -p "(Assuming same setup) Display previous normal? [y/N]: " DISPLAY_PREVIOUS_NORMAL

    if [[ "$DISPLAY_PREVIOUS_NORMAL" =~ ^[Yy]$ ]]; then
        USE_PREVIOUS_NORMAL=true
        echo "Using previous normal."
    else
        echo "Using newly estimated normal."
    fi
else
    echo ""
    echo "No previous normal found."
    echo "Running new normal estimation."
fi

# --- Launch DLIO ---
# rviz:=false here since we're launching our own RViz2 instance below with
# the custom map_pip.rviz config instead of DLIO's default one.
# Output is redirected to $DLIO_LOG instead of this terminal, and tailed
# live in a separate Terminator window so it stays out of the main terminal
# but is still visible.
# --- Launch DLIO silently ---

echo "[2/4] Starting DLIO..."

ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
    rviz:=false \
    pointcloud_topic:=/livox/lidar \
    imu_topic:=/livox/imu \
    > "$DLIO_LOG" 2>&1 &

DLIO_PID=$!

echo "[2.5/4] Starting TF transform (logging to $DLIO_LOG)..."
ros2 run tf2_ros static_transform_publisher \
    0 0 0 \
    0 0 0 \
    lidar livox_frame &
TF_PID=$!

# Give DLIO time to initialize and start publishing odometry/deskewed cloud
# before map_pip starts subscribing/collecting.
sleep 3

# --- Launch RViz2 with the saved cloud_pipeline config ---
echo "[3/4] Starting RViz2 with map_pip config..."
if [[ ! -f "$RVIZ_CONFIG" ]]; then
    echo "WARNING: RViz2 config not found at $RVIZ_CONFIG — launching with defaults."
    ros2 run rviz2 rviz2 &
else
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" &
fi

# --- Launch map_pip_mocap (cloud_pipeline) ---

# python3 scripts/pcd_to_stl.py in.pcd out.stl \
#  --voxel 0.01 --orient sensor --poisson-depth 8 --keep-largest

MAP_PCD="$MAP_DIR/global_map_${TIMESTAMP}.pcd"

MESH_RECONSTRUCT="$HOME/vision_ws_git/scripts/pcd_to_stl.py"
MESH_OUTPUT="$MESH_DIR/map_${TIMESTAMP}.stl"

echo "[4/4] Starting map_pip_mocap..."
echo "      Map will be saved to: $MAP_PCD"

# run BOTH map_pip_mocap and get the process ID to track if process
# is still running

ros2 run cloud_pipeline map_pip_mocap \
    --ros-args \
    -p save_path:="$MAP_PCD"\
    -p voxel_leaf_size:=0.007\
    -p duplicate_distance:=0.005\
    -p sor_mean_k:=2000\
    -p collection_duration_sec:=20.0\
    -p mesh_path:="$MESH_OUTPUT"\
    -p normal_file_path:="$NORMAL_FILE" \
    -p use_previous_normal:="$USE_PREVIOUS_NORMAL" &

MAP_PIP_PID=$!

# ------------------------------------------------------------
# Wait for map_pip_mocap to create the NEW PCD
# ------------------------------------------------------------

echo ""
echo "[POST] Waiting for map_pip_mocap to save:"
echo "       $MAP_PCD"

# This while loop is to wait for global_map.pcd to appear,
while [[ ! -f "$MAP_PCD" ]]; do
	# check whether map_pip is still alive (process still exists)
	# 2>/dev/null is a standard error message 
    if ! kill -0 "$MAP_PIP_PID" 2>/dev/null; then
        echo ""
        echo "ERROR: map_pip_mocap has exited."
        echo "       global_map.pcd was never created."
        exit 1
    fi

    echo "[POST] Map not ready yet..."
    sleep 1
done

echo ""
echo "=========================================="
echo "[POST] global_map.pcd detected!"
echo "=========================================="

# ------------------------------------------------------------
# Run Mesh reconstruction
# ------------------------------------------------------------

echo ""
echo "[POST] Checking Mesh reconstruction..."

if [[ ! -f "$MESH_RECONSTRUCT" ]]; then
    echo "ERROR: Cannot find reconstruction script:"
    echo "       $MESH_RECONSTRUCT"
    exit 1
fi

echo "[POST] Running:"
echo "       $MESH_RECONSTRUCT $MAP_PCD"

(
    python3 "$MESH_RECONSTRUCT" "$MAP_PCD" "$MESH_OUTPUT" \
    --voxel 0.01 \
    --orient sensor \
    --poisson-depth 8 \
    --keep-largest \
    --remove-outliers \
    --crop-to-input
)

echo ""
echo "[POST] af_reconstruct finished."

# ------------------------------------------------------------
# Check output
# ------------------------------------------------------------

if [[ ! -f "$MESH_OUTPUT" ]]; then
    echo ""
    echo "ERROR: Reconstruction finished but mesh file was not created."
    echo "Expected:"
    echo "  $MESH_OUTPUT"
    exit 1
fi

echo ""
echo "=========================================="
echo "[POST] Reconstruction successful!"
echo "=========================================="
echo ""
echo "OFF file:"
echo "  $MESH_OUTPUT"

# ------------------------------------------------------------
# Open MeshLab
# ------------------------------------------------------------

if command -v meshlab &> /dev/null; then

    echo ""
    echo "[POST] Opening MeshLab..."

    meshlab "$MESH_OUTPUT" &

else

    echo ""
    echo "WARNING: MeshLab was not found."
    echo "Install it with:"
    echo "  sudo apt install meshlab"

fi


# ------------------------------------------------------------
# Keep everything else alive
# ------------------------------------------------------------

echo ""
echo "=========================================="
echo "All processing complete."
echo "=========================================="
echo ""
echo "Livox / DLIO / RViz / map_pip are still running."
echo "Press Ctrl-C to shut everything down."

wait
