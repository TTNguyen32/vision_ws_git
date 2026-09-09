#!/bin/bash

# run_mid360_dlio_mappip.sh
#
# Launches:
#   1. Livox Mid-360 driver
#   2. DLIO
#   3. Static lidar -> livox_frame TF
#   4. RViz2
#   5. map_pip_mocap_multinormals
#   6. Poisson mesh reconstruction
#
# Source code remains in ~/vision_ws.
# All generated experiment outputs are stored in ~/vision_ws_outputs/
#
# Requires:
#   - livox_ros_driver2 in ~/livox_ws
#   - DLIO in ~/dlio_ws
#   - cloud_accumulator in ~/vision_ws
#   - xfer_format = 0 in msg_MID360_launch.py
#
# /livox/lidar must therefore publish sensor_msgs/PointCloud2
# with per-point timestamps.

set -e

# Prevent Conda libraries from interfering with ROS/CMake/runtime dependencies
unset PYTHONPATH
unset LD_LIBRARY_PATH

# ============================================================
# Configuration
# ============================================================

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

# --- Workspace ---

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# --- Output directories ---

OUTPUT_DIR="$HOME/vision_ws_outputs"

MAP_DIR="$OUTPUT_DIR/maps"
MESH_DIR="$OUTPUT_DIR/meshes"
NORMAL_DIR="$OUTPUT_DIR/normals"
LOG_DIR="$OUTPUT_DIR/logs"

mkdir -p "$MAP_DIR"
mkdir -p "$MESH_DIR"
mkdir -p "$NORMAL_DIR"
mkdir -p "$LOG_DIR"

# --- Timestamped outputs ---

MAP_PCD="$MAP_DIR/global_map_${TIMESTAMP}.pcd"
MESH_OUTPUT="$MESH_DIR/map_${TIMESTAMP}.stl"
NORMAL_OUTPUT="$NORMAL_DIR/selected_normals_${TIMESTAMP}.yaml"

# --- RViz configuration ---

RVIZ_CONFIG="$REPO_DIR/config/map_pip.rviz"

# --- DLIO log ---

DLIO_LOG="$LOG_DIR/dlio_${TIMESTAMP}.log"
: > "$DLIO_LOG"

# --- Mesh reconstruction script ---

MESH_RECONSTRUCT="$REPO_DIR/scripts/pcd_to_stl.py"

# ============================================================
# Source workspaces
# ============================================================

source /opt/ros/humble/setup.bash
source "$HOME/livox_ws/install/setup.bash"
source "$HOME/dlio_ws/install/setup.bash"
source "$REPO_DIR/install/setup.bash"

# ============================================================
# Cleanup
# ============================================================

DLIO_TAIL_PID=""

cleanup() {
    echo ""
    echo "Shutting down..."

    # Kill direct child processes
    kill $(jobs -p) 2>/dev/null || true

    # Kill DLIO log tail if one was started
    if [[ -n "$DLIO_TAIL_PID" ]]; then
        kill "$DLIO_TAIL_PID" 2>/dev/null || true
    fi

    wait 2>/dev/null || true
}

trap cleanup EXIT INT TERM

# ============================================================
# 1. Launch Livox Mid-360
# ============================================================

echo ""
echo "=========================================="
echo "[1/4] Starting Livox Mid-360 driver..."
echo "=========================================="

ros2 launch livox_ros_driver2 msg_MID360_launch.py &

# Give driver time to initialize
sleep 3

# ============================================================
# Check PointCloud2 output
# ============================================================

TOPIC_TYPE=$(ros2 topic type /livox/lidar 2>/dev/null || echo "unknown")

if [[ "$TOPIC_TYPE" != *"PointCloud2"* ]]; then
    echo ""
    echo "WARNING: /livox/lidar is reporting:"
    echo "  $TOPIC_TYPE"
    echo ""
    echo "Expected sensor_msgs/PointCloud2."
    echo "Check that xfer_format = 0 in:"
    echo "  msg_MID360_launch.py"
    echo ""
else
    echo "Confirmed: /livox/lidar -> PointCloud2"
fi

# ============================================================
# Previous normal
# ============================================================

LOCKED_TARGET_FILE="$NORMAL_DIR/locked_target.yaml"

USE_PREVIOUS_NORMAL=false

if [[ -f "$LOCKED_TARGET_FILE" ]]; then

    echo ""
    echo "========================================="
    echo "Previous normals found:"
    echo "$LOCKED_TARGET_FILE"
    echo "========================================="

    read -r -p \
        "(Assuming same setup) Display previous normals? [y/N]: " \
        DISPLAY_PREVIOUS_NORMAL

    if [[ "$DISPLAY_PREVIOUS_NORMAL" =~ ^[Yy]$ ]]; then
        USE_PREVIOUS_NORMAL=true
        echo "Using previous normals."
    else
        echo "Using newly estimated normals."
    fi

else

    echo ""
    echo "No previous normal found."
    echo "Running new normal estimation."

fi

# ============================================================
# 2. Launch DLIO
# ============================================================

echo ""
echo "=========================================="
echo "[2/4] Starting DLIO..."
echo "=========================================="

ros2 launch direct_lidar_inertial_odometry dlio.launch.py \
    rviz:=false \
    pointcloud_topic:=/livox/lidar \
    imu_topic:=/livox/imu \
    > "$DLIO_LOG" 2>&1 &

DLIO_PID=$!

echo "DLIO PID: $DLIO_PID"
echo "DLIO log:"
echo "  $DLIO_LOG"

# ============================================================
# Static TF
# ============================================================

echo "[2.5/4] Starting static TF..."

ros2 run tf2_ros static_transform_publisher \
    0 0 0 \
    0 0 0 \
    lidar livox_frame &

TF_PID=$!

# Give DLIO time to initialize
sleep 3

# ============================================================
# 3. Launch RViz
# ============================================================

echo ""
echo "=========================================="
echo "[3/4] Starting RViz2..."
echo "=========================================="

if [[ ! -f "$RVIZ_CONFIG" ]]; then

    echo "WARNING: RViz config not found:"
    echo "  $RVIZ_CONFIG"
    echo "Launching RViz2 with default configuration."

    ros2 run rviz2 rviz2 &

else

    echo "Using RViz config:"
    echo "  $RVIZ_CONFIG"

    ros2 run rviz2 rviz2 \
        -d "$RVIZ_CONFIG" &

fi

# ============================================================
# 4. Launch map pipeline
# ============================================================

echo ""
echo "=========================================="
echo "[4/4] Starting map_pip_mocap_multinormals"
echo "=========================================="

echo "Map output:"
echo "  $MAP_PCD"

echo "Mesh output:"
echo "  $MESH_OUTPUT"

echo "Normal output:"
echo "  $NORMAL_OUTPUT"

echo ""
echo "Previous normal:"
echo "  $USE_PREVIOUS_NORMAL"

# Remove files if somehow already present
rm -f "$MAP_PCD"
rm -f "$MESH_OUTPUT"
rm -f "$NORMAL_OUTPUT"

ros2 run cloud_pipeline map_pip_mocap_multinormals \
    --ros-args \
    -p save_path:="$MAP_PCD" \
    -p voxel_leaf_size:=0.007 \
    -p duplicate_distance:=0.005 \
    -p sor_mean_k:=2000 \
    -p collection_duration_sec:=20.0 \
    -p mesh_path:="$MESH_OUTPUT" \
    -p normals_save_path:="$NORMAL_OUTPUT" \
    -p use_previous_normal:="$USE_PREVIOUS_NORMAL" &

MAP_PIP_PID=$!

echo ""
echo "map_pip_mocap_multinormals PID: $MAP_PIP_PID"

# ============================================================
# Wait for map to be created
# ============================================================

echo ""
echo "=========================================="
echo "[POST] Waiting for map..."
echo "=========================================="

echo "Waiting for:"
echo "  $MAP_PCD"

while [[ ! -f "$MAP_PCD" ]]; do

    # Check whether map pipeline is still running
    if ! kill -0 "$MAP_PIP_PID" 2>/dev/null; then

        echo ""
        echo "ERROR: map_pip_mocap_multinormals has exited."
        echo "The expected map was never created:"
        echo "  $MAP_PCD"

        exit 1
    fi

    echo "[POST] Map not ready yet..."
    sleep 1

done

echo ""
echo "=========================================="
echo "[POST] Map created successfully!"
echo "=========================================="

echo "PCD:"
echo "  $MAP_PCD"

# ============================================================
# Mesh reconstruction
# ============================================================

echo ""
echo "=========================================="
echo "[POST] Mesh reconstruction"
echo "=========================================="

if [[ ! -f "$MESH_RECONSTRUCT" ]]; then

    echo "ERROR: Mesh reconstruction script not found:"
    echo "  $MESH_RECONSTRUCT"

    exit 1

fi

echo "Running:"
echo "  $MESH_RECONSTRUCT"

echo ""
echo "Input:"
echo "  $MAP_PCD"

echo "Output:"
echo "  $MESH_OUTPUT"

conda run -n lidar python "$MESH_RECONSTRUCT" \
    "$MAP_PCD" \
    "$MESH_OUTPUT" \
    --voxel 0.01 \
    --orient sensor \
    --poisson-depth 8 \
    --keep-largest \
    --remove-outliers \
    --crop-to-input

# ============================================================
# Check mesh output
# ============================================================

if [[ ! -f "$MESH_OUTPUT" ]]; then

    echo ""
    echo "ERROR: Reconstruction finished but mesh was not created."
    echo "Expected:"
    echo "  $MESH_OUTPUT"

    exit 1

fi

echo ""
echo "=========================================="
echo "[POST] Reconstruction successful!"
echo "=========================================="

echo ""
echo "Mesh:"
echo "  $MESH_OUTPUT"

# ============================================================
# Open MeshLab
# ============================================================

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

# ============================================================
# Normal selection
# ============================================================

echo ""
echo "=========================================="
echo "[POST] Normal selection"
echo "=========================================="

echo ""
echo "Select normals in RViz."
echo ""
echo "When you are finished:"
echo "  1. Return to this terminal"
echo "  2. Press ENTER"
echo ""

echo "Waiting for normal-selection service..."

while ! ros2 service list 2>/dev/null | \
      grep -q "/finish_normal_selection"; do

    if ! kill -0 "$MAP_PIP_PID" 2>/dev/null; then
        echo ""
        echo "ERROR: map_pip_mocap_multinormals has exited."
        exit 1
    fi

    sleep 1
done

echo ""
echo "Normal selection is ready."
echo ""

read -r -p "Press ENTER when you have finished selecting normals..."

# ============================================================
# Save selected normals
# ============================================================

echo ""
echo "=========================================="
echo "[POST] Saving normal selection"
echo "=========================================="

if ! ros2 service call \
    /finish_normal_selection \
    std_srvs/srv/Trigger "{}"; then

    echo ""
    echo "ERROR: Failed to finish normal selection."
    exit 1
fi

# Verify historical YAML was created

if [[ ! -f "$NORMAL_OUTPUT" ]]; then

    echo ""
    echo "ERROR: Normal selection service succeeded,"
    echo "but the expected YAML file was not created:"
    echo "  $NORMAL_OUTPUT"

    exit 1
fi

echo ""
echo "Historical normal selection saved:"
echo "  $NORMAL_OUTPUT"

# ============================================================
# Ask whether to update locked target
# ============================================================

echo ""
echo "=========================================="
echo "Update locked target?"
echo "=========================================="

read -r -p \
    "Overwrite locked target with these normals? [y/N]: " \
    OVERWRITE_LOCKED

if [[ "$OVERWRITE_LOCKED" =~ ^[Yy]$ ]]; then

    cp "$NORMAL_OUTPUT" "$LOCKED_TARGET_FILE"

    echo ""
    echo "Locked target updated successfully:"
    echo "  $LOCKED_TARGET_FILE"

else

    echo ""
    echo "Locked target unchanged."
    echo "Existing configuration has been preserved."

fi

# ============================================================
# Finished
# ============================================================

echo ""
echo "=========================================="
echo "All processing complete."
echo "=========================================="

echo ""
echo "Outputs for this run:"
echo ""
echo "Map:"
echo "  $MAP_PCD"
echo ""
echo "Mesh:"
echo "  $MESH_OUTPUT"
echo ""
echo "Normals:"
echo "  $NORMAL_OUTPUT"
echo ""
echo "Locked target:"
echo "  $LOCKED_TARGET_FILE"
echo ""
echo "DLIO log:"
echo "  $DLIO_LOG"
echo ""
echo "Livox / DLIO / RViz / map pipeline are"
echo "still running."
echo ""
echo "Press Ctrl-C to shut everything down."

wait
