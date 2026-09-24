#!/bin/bash

# ============================================================
# A*STAR Drone Vision Pipeline (OptiTrack mocap pose source)
#
# Pipeline:
#   Livox Mid-360 -> OptiTrack (lis-epfl optitrack_packages_ros2:
#   optitrack_wrapper_ros2 + optitrack_multiplexer_ros2) ->
#   map_pip_optitrack (buffers + lerp/slerp-interpolates the mocap
#   pose itself, composes T_world_lidar = T_world_body * T_body_lidar
#   directly -- no TF2 lookup in the pipeline's pose path) -> RViz2
#   -> map accumulation -> PCD -> Poisson mesh reconstruction ->
#   MeshLab -> previous OR new normal selection -> optional offset
#   spline.
#
# mocap_tf_broadcaster runs alongside map_pip_optitrack, NOT as part
# of its data path: it independently subscribes to the same
# multiplexer topic and republishes world -> body as a dynamic TF,
# purely so the drone's live mocap pose can be watched in RViz. If
# it were killed, map_pip_optitrack would keep working exactly as
# before -- it never consumes this TF.
#
# CHANGE vs. the DLIO version of this script (run_vision_pip):
#   DLIO is completely removed.
#     1. optitrack_wrapper_ros2 wraps NatNetSDK and talks to Motive;
#        optitrack_multiplexer_ros2 publishes the rigid body named
#        in rigid_body_names (its config yaml) on its own topic,
#        <node>/rigid_body/<name> -- confirmed live via
#        `ros2 topic echo ... --once` and `ros2 topic hz` this
#        session (~120 Hz, matching Motive's framerate).
#     2. map_pip_optitrack subscribes to that topic directly, buffers
#        poses, and interpolates to each LiDAR scan's own timestamp.
#        It also publishes body -> lidar as a STATIC TF itself
#        (publish_lidar_tf param, default true) from the
#        lidar_ext_* extrinsic below -- no separate
#        static_transform_publisher needed for that link any more.
#     3. mocap_tf_broadcaster (separate node, cloud_accumulator
#        package) independently republishes world -> body as a
#        DYNAMIC TF, for RViz visualization only.
#
# CONFIRMED THIS SESSION (do not re-guess these):
#   - Motive's actual rigid body name is "Tin_LIDAR" (id 5), NOT
#     "XFly" and NOT "drone" -- both of those were guesses from
#     earlier sessions that never matched what Motive itself calls
#     it. rigid_body_names in optitrack_multiplexer_config.yaml MUST
#     equal this exactly, or the multiplexer advertises a topic with
#     a live publisher but zero messages -- no error anywhere, it
#     just silently never finds a matching body. Confirmed via
#     `ros2 service call /optitrack_wrapper_node/get_data_descriptions
#     optitrack_wrapper_ros2_msgs/srv/GetDataDescriptions "{}"`.
#   - The real topic is /optitrack_multiplexer_node/rigid_body/Tin_LIDAR
#     (relative name, auto-prefixed by the node's own name) -- NOT
#     the flat /Tin_LIDAR an earlier script guessed by analogy with
#     "/drone". Both map_pip_optitrack.cpp and mocap_tf_broadcaster.cpp
#     already build this exact string themselves from
#     mocap_rigid_body, so neither needs an explicit topic override.
#   - server_address 192.168.50.141 confirmed correct in both
#     optitrack_wrapper_config.yaml and optitrack_wrapper.launch.py.
#     (If you ever see it connect to some OTHER IP again, that command
#     is not coming from either of those two files -- check
#     wrapper_and_multiplexer.launch.py, in optitrack_multiplexer_ros2,
#     for a generated --params-file overriding it. Seen this exact
#     failure mode once already this session.)
#
# REMINDER -- colcon does not read your source tree live. Any edit to
# a config/launch file under src/ needs a rebuild of that specific
# package before it takes effect:
#   cd ~/ros2_ws && colcon build --packages-select <package> && source install/setup.bash
# This bit us twice already (rigid_body_names and server_address both
# looked "fixed" in source while the installed copy was stale).
#
# Prerequisites before running this script:
#   - Motive: rigid body streaming enabled (Data Streaming Pane),
#     Up Axis confirmed.
#   - lis-epfl/optitrack_packages_ros2 built into $HOME/ros2_ws, with
#     rigid_body_names: "Tin_LIDAR" and world_frame: "world" in
#     optitrack_multiplexer_config.yaml (both already set).
#   - lidar_ext_* below MUST be replaced with your measured/CAD
#     offset between the Motive rigid-body origin and the Livox
#     optical center -- still placeholder zeros.
#   - mocap_tf_broadcaster registered as an executable in
#     cloud_accumulator's CMakeLists.txt -- NOT done yet as of this
#     script (see CMakeLists snippet from earlier in this session).
#
# Normal source for path:
#   Previous normals: ~/vision_ws_outputs/normals/locked_target.yaml
#   New normals:      ~/vision_ws_outputs/normals/selected_normals_TIMESTAMP.yaml
#
# Mesh source for path: newly generated map_TIMESTAMP.stl
# ============================================================

set -e

# Prevent Conda libraries from interfering with ROS/CMake/runtime deps
unset PYTHONPATH
unset LD_LIBRARY_PATH

## Kill stale processes
echo "Checking for stale ROS processes..."

STALE_PIDS=$(pgrep -f '/home/tin/vision_ws/install/cloud_accumulator/lib/cloud_accumulator/map_pip_optitrack' || true)

if [ -n "$STALE_PIDS" ]; then
    echo "Killing stale cloud_accumulator processes: $STALE_PIDS"
    kill -9 $STALE_PIDS 2>/dev/null || true
fi

sleep 1

echo "Stale process cleanup complete."

# ============================================================
# Configuration
# ============================================================

WORKSPACE="$HOME/vision_ws"

OUTPUT_DIR="$HOME/vision_ws_outputs"

MAP_DIR="$OUTPUT_DIR/maps"
MESH_DIR="$OUTPUT_DIR/meshes"
NORMAL_DIR="$OUTPUT_DIR/normals"
PATH_DIR="$OUTPUT_DIR/paths"
LOG_DIR="$OUTPUT_DIR/logs"

MESH_RECONSTRUCT="$HOME/lidar-tools/pcd_to_stl.py"
SPLINE_SCRIPT="$HOME/lidar-tools/offset_spline.py"

RVIZ_CONFIG="$WORKSPACE/src/cloud_accumulator/src/map_pip_optitrack.rviz"

LOCKED_TARGET_FILE="$NORMAL_DIR/locked_target.yaml"
# NOTE: map_pip_optitrack.cpp hardcodes this same path internally
# ($HOME/vision_ws_outputs/normals/locked_target.yaml) rather than
# taking it as a parameter -- LOCKED_TARGET_FILE above is used only
# by THIS SCRIPT (for the count_targets/previous-normal logic below),
# never passed to the node. If you ever change OUTPUT_DIR, this
# stops matching what the node actually reads/writes.

# ============================================================
# OptiTrack / mocap configuration
# ============================================================

# Workspace where lis-epfl/optitrack_packages_ros2 is built.
OPTITRACK_WS="$HOME/ros2_ws"

# Rigid body name EXACTLY as Motive reports it -- confirmed via
# get_data_descriptions this session. Must match rigid_body_names in
# optitrack_multiplexer_config.yaml. Passed to BOTH map_pip_optitrack
# and mocap_tf_broadcaster; both derive their own topic name
# (/optitrack_multiplexer_node/rigid_body/<this>) and TF body-frame
# name from it, so they always agree without a separate topic
# variable to keep in sync by hand.
MOCAP_RIGID_BODY="Tin_LIDAR"

# Topic this resolves to, used only for the readiness check below --
# not passed to either node (they build it themselves).
MOCAP_POSE_TOPIC="/optitrack_multiplexer_node/rigid_body/${MOCAP_RIGID_BODY}"

# World-fixed frame. Matches optitrack_multiplexer_config.yaml's own
# world_frame param and map_pip_optitrack.cpp's "world_frame"/
# "frame_id" defaults -- all three must agree.
MOCAP_WORLD_FRAME="world"

# CALIBRATED extrinsic: rigid-body origin -> Livox optical center.
# TODO: replace these zeros with your measured/CAD offset.
# Translation in metres, rotation in radians. Passed straight through
# to map_pip_optitrack's lidar_ext_* params, which it uses to build
# T_body_lidar_ internally (Z-Y-X / yaw-pitch-roll) -- same convention
# as before, just node-internal now instead of a separate
# static_transform_publisher call.
LIDAR_EXT_X=0.0
LIDAR_EXT_Y=0.0
LIDAR_EXT_Z=0.0
LIDAR_EXT_ROLL=0.0
LIDAR_EXT_PITCH=0.0
LIDAR_EXT_YAW=0.0

# Spline parameters
SPLINE_OFFSET=0.05
SPLINE_SPACING=0.01
SPLINE_ITERATIONS=60
SPLINE_SMOOTH_START=0.6
SPLINE_SMOOTH_END=0.02

# ============================================================
# Helper: count targets in a normals YAML file.
# Runs inside the 'lidar' conda env (same one used for mesh/
# spline scripts) because system python3 doesn't have pyyaml.
# ============================================================

count_targets() {
    local yaml_file="$1"
    local tmp_script
    tmp_script=$(mktemp /tmp/count_targets_XXXXXX.py)

    cat > "$tmp_script" <<'PY'
import sys
import yaml

with open(sys.argv[1], "r") as f:
    data = yaml.safe_load(f)

targets = data.get("targets", [])
print(len(targets) if isinstance(targets, list) else 0)
PY

    conda run -n lidar python "$tmp_script" "$yaml_file"
    rm -f "$tmp_script"
}

# ============================================================
# Timestamp / output paths
# ============================================================

TIMESTAMP=$(date +"%Y%m%d_%H%M%S")

MAP_OUTPUT="$MAP_DIR/global_map_${TIMESTAMP}.pcd"
MESH_OUTPUT="$MESH_DIR/map_${TIMESTAMP}.stl"
NORMAL_OUTPUT="$NORMAL_DIR/selected_normals_${TIMESTAMP}.yaml"

PATH_OUTPUT="$PATH_DIR/path_${TIMESTAMP}.yaml"
PATH_CSV="$PATH_DIR/path_${TIMESTAMP}.csv"

MOCAP_LOG="$LOG_DIR/optitrack_${TIMESTAMP}.log"
MOCAP_TF_LOG="$LOG_DIR/mocap_tf_broadcaster_${TIMESTAMP}.log"

mkdir -p "$MAP_DIR" "$MESH_DIR" "$NORMAL_DIR" "$PATH_DIR" "$LOG_DIR"
: > "$MOCAP_LOG"
: > "$MOCAP_TF_LOG"

# Remove any stale files at these exact paths
rm -f \
    "$MAP_OUTPUT" \
    "$MESH_OUTPUT" \
    "$NORMAL_OUTPUT" \
    "$PATH_OUTPUT" \
    "$PATH_CSV"

# ============================================================
# Cleanup
# ============================================================

cleanup() {

    # Disable all cleanup traps immediately so cleanup
    # cannot run twice.
    trap - EXIT INT TERM

    echo ""
    echo "=========================================="
    echo "Cleaning up"
    echo "=========================================="

    for pid_var in MAP_PID MOCAP_PID MOCAP_TF_PID RVIZ_PID LIVOX_PID; do

        pid="${!pid_var:-}"

        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then

            echo "Stopping $pid_var ($pid)..."

            kill "$pid" 2>/dev/null || true

        fi

    done

    # Don't wait indefinitely for ROS to shut down.
    sleep 1

    # Force any remaining processes to exit.
    for pid_var in MAP_PID MOCAP_PID MOCAP_TF_PID RVIZ_PID LIVOX_PID; do

        pid="${!pid_var:-}"

        if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then

            echo "Force stopping $pid_var ($pid)..."

            kill -9 "$pid" 2>/dev/null || true

        fi

    done

    echo "Cleanup complete."

}

trap cleanup EXIT INT TERM
# ============================================================
# Source workspaces
# ============================================================

echo ""
echo "=========================================="
echo "Initialising ROS 2 environment"
echo "=========================================="

source /opt/ros/humble/setup.bash

if [[ -f "$HOME/livox_ws/install/setup.bash" ]]; then
    source "$HOME/livox_ws/install/setup.bash"
else
    echo "WARNING: Livox workspace not found: $HOME/livox_ws/install/setup.bash"
fi

# optitrack_packages_ros2 (lis-epfl) is built from source -- no apt
# package for it.
if [[ -f "$OPTITRACK_WS/install/setup.bash" ]]; then
    source "$OPTITRACK_WS/install/setup.bash"
else
    echo "ERROR: OptiTrack workspace not found: $OPTITRACK_WS/install/setup.bash"
    exit 1
fi

if [[ -f "$WORKSPACE/install/setup.bash" ]]; then
    source "$WORKSPACE/install/setup.bash"
else
    echo "ERROR: Vision workspace not built: $WORKSPACE/install/setup.bash"
    exit 1
fi

# ============================================================
# Display configuration
# ============================================================

echo ""
echo "=========================================="
echo "A*STAR Drone Vision Pipeline"
echo "=========================================="
echo "Timestamp:       $TIMESTAMP"
echo "Rigid body:      $MOCAP_RIGID_BODY  (topic: $MOCAP_POSE_TOPIC)"
echo "World frame:     $MOCAP_WORLD_FRAME"
echo "Map output:      $MAP_OUTPUT"
echo "Mesh output:     $MESH_OUTPUT"
echo "Normals output:  $NORMAL_OUTPUT"
echo "Locked target:   $LOCKED_TARGET_FILE  (script-side only, see note above)"
echo "Path output:     $PATH_OUTPUT"

# ============================================================
# Ask whether to use previous normals
# ============================================================

USE_PREVIOUS_NORMAL=false

if [[ -f "$LOCKED_TARGET_FILE" ]]; then
    echo ""
    echo "Previous normal target found: $LOCKED_TARGET_FILE"
    read -r -p "Display previous normals? [y/N]: " PREVIOUS_NORMAL_REPLY

    if [[ "$PREVIOUS_NORMAL_REPLY" =~ ^[Yy]$ ]]; then
        USE_PREVIOUS_NORMAL=true
        echo "Using previous normals."
    else
        echo "Using newly estimated normals."
    fi
else
    echo ""
    echo "No previous normal target found. New normals will be estimated."
fi

# ============================================================
# Ask whether to generate a path
# ============================================================

GENERATE_PATH=false

read -r -p "Generate an offset spline path? [y/N]: " GENERATE_PATH_REPLY

if [[ "$GENERATE_PATH_REPLY" =~ ^[Yy]$ ]]; then
    GENERATE_PATH=true
    echo "Path generation enabled."
else
    echo "Path generation disabled."
fi

# ============================================================
# 1. Launch Livox Mid-360
# ============================================================

echo ""
echo "=========================================="
echo "[1/5] Starting Livox Mid-360 driver..."
echo "=========================================="

ros2 launch livox_ros_driver2 msg_MID360_launch.py &
LIVOX_PID=$!

sleep 3

TOPIC_TYPE=$(ros2 topic type /livox/lidar 2>/dev/null || echo "unknown")
if [[ "$TOPIC_TYPE" != *"PointCloud2"* ]]; then
    echo "WARNING: /livox/lidar reports '$TOPIC_TYPE', expected sensor_msgs/PointCloud2."
    echo "Check xfer_format = 0 in msg_MID360_launch.py"
else
    echo "Confirmed: /livox/lidar -> PointCloud2"
fi

# ============================================================
# 2. Launch OptiTrack (optitrack_wrapper_ros2 + optitrack_multiplexer_ros2)
#    + mocap_tf_broadcaster (RViz visualization only -- see header
#    comment. map_pip_optitrack does NOT depend on this node; it
#    subscribes to the same multiplexer topic independently and
#    computes its own pose composition without TF2.)
# ============================================================

echo ""
echo "=========================================="
echo "[2/5] Starting OptiTrack (wrapper + multiplexer)..."
echo "=========================================="

ros2 launch optitrack_multiplexer_ros2 wrapper_and_multiplexer.launch.py \
    > "$MOCAP_LOG" 2>&1 &

MOCAP_PID=$!
echo "optitrack wrapper+multiplexer PID: $MOCAP_PID"
echo "optitrack log: $MOCAP_LOG"

sleep 3

if ! ros2 topic list 2>/dev/null | grep -qF "$MOCAP_POSE_TOPIC"; then
    echo "WARNING: $MOCAP_POSE_TOPIC not found in 'ros2 topic list'."
    echo "Check rigid_body_names in optitrack_multiplexer_config.yaml matches"
    echo "Motive's actual rigid body name EXACTLY (confirm via"
    echo "  ros2 service call /optitrack_wrapper_node/get_data_descriptions \\"
    echo "    optitrack_wrapper_ros2_msgs/srv/GetDataDescriptions \"{}\""
    echo "), and that you rebuilt after editing it (colcon does not read"
    echo "source YAML/launch files live -- see header comment)."
else
    echo "Confirmed: $MOCAP_POSE_TOPIC is publishing"
fi

echo "[2.5/5] Starting mocap_tf_broadcaster (RViz visualization only)..."

ros2 run cloud_accumulator mocap_tf_broadcaster \
    --ros-args \
    -p mocap_rigid_body:="$MOCAP_RIGID_BODY" \
    -p world_frame:="$MOCAP_WORLD_FRAME" \
    > "$MOCAP_TF_LOG" 2>&1 &
MOCAP_TF_PID=$!
echo "mocap_tf_broadcaster PID: $MOCAP_TF_PID"

sleep 1

# ============================================================
# 3. Launch RViz2 with the project config
# ============================================================

echo ""
echo "=========================================="
echo "[3/5] Starting RViz2..."
echo "=========================================="

if [[ -f "$RVIZ_CONFIG" ]]; then
    echo "Using RViz config: $RVIZ_CONFIG"
    ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" &
else
    echo "WARNING: RViz config not found: $RVIZ_CONFIG"
    echo "Launching RViz2 with default configuration."
    ros2 run rviz2 rviz2 &
fi
RVIZ_PID=$!

# ============================================================
# 4. Launch map pipeline (map_pip_optitrack)
# ============================================================

echo ""
echo "=========================================="
echo "[4/5] Starting map_pip_optitrack"
echo "=========================================="

ros2 run cloud_accumulator map_pip_optitrack_slerp \
    --ros-args \
    -p save_path:="$MAP_OUTPUT" \
    -p frame_id:="$MOCAP_WORLD_FRAME" \
    -p world_frame:="$MOCAP_WORLD_FRAME" \
    -p mocap_rigid_body:="$MOCAP_RIGID_BODY" \
    -p lidar_ext_x:="$LIDAR_EXT_X" \
    -p lidar_ext_y:="$LIDAR_EXT_Y" \
    -p lidar_ext_z:="$LIDAR_EXT_Z" \
    -p lidar_ext_roll:="$LIDAR_EXT_ROLL" \
    -p lidar_ext_pitch:="$LIDAR_EXT_PITCH" \
    -p lidar_ext_yaw:="$LIDAR_EXT_YAW" \
    -p voxel_leaf_size:=0.007 \
    -p duplicate_distance:=0.005 \
    -p sor_mean_k:=2000 \
    -p collection_duration_sec:=10.0 \
    -p mesh_path:="$MESH_OUTPUT" \
    -p path_yaml:="$PATH_OUTPUT" \
    -p generate_path:="$GENERATE_PATH" \
    -p normals_save_path:="$NORMAL_OUTPUT" \
    -p use_previous_normal:="$USE_PREVIOUS_NORMAL" \
    > >(tee "$LOG_DIR/map_pip_${TIMESTAMP}.log") 2>&1 &

MAP_PID=$!
echo "map_pip_optitrack PID: $MAP_PID"

# ============================================================
# 5. Wait for the map file to appear (poll, don't block on
#    process exit -- the node stays alive for normal selection)
# ============================================================

echo ""
echo "=========================================="
echo "[5/5] Waiting for map..."
echo "=========================================="
echo "Waiting for: $MAP_OUTPUT"

while [[ ! -f "$MAP_OUTPUT" ]]; do
    if ! kill -0 "$MAP_PID" 2>/dev/null; then
        echo ""
        echo "ERROR: map_pip_optitrack exited before creating the map."
        echo "Check log: $LOG_DIR/map_pip_${TIMESTAMP}.log"
        exit 1
    fi
    echo "[wait] Map not ready yet..."
    sleep 1
done

echo ""
echo "Map successfully generated: $MAP_OUTPUT"

# ============================================================
# Mesh reconstruction
# ============================================================

echo ""
echo "=========================================="
echo "Generating mesh"
echo "=========================================="

if [[ ! -f "$MESH_RECONSTRUCT" ]]; then
    echo "ERROR: Mesh reconstruction script not found: $MESH_RECONSTRUCT"
    exit 1
fi

conda run -n lidar python "$MESH_RECONSTRUCT" \
    "$MAP_OUTPUT" \
    "$MESH_OUTPUT" \
    --voxel 0.01 \
    --orient sensor \
    --poisson-depth 8 \
    --keep-largest \
    --remove-outliers \
    --crop-to-input

if [[ ! -f "$MESH_OUTPUT" ]]; then
    echo "ERROR: Expected mesh was not generated: $MESH_OUTPUT"
    exit 1
fi

echo "Mesh successfully generated: $MESH_OUTPUT"

# ============================================================
# Open mesh in MeshLab
# ============================================================

if command -v meshlab >/dev/null 2>&1; then
    echo "Opening mesh in MeshLab..."
    meshlab "$MESH_OUTPUT" &
else
    echo "WARNING: MeshLab was not found. Install with: sudo apt install meshlab"
fi

# ============================================================
# PREVIOUS NORMAL BRANCH
# ============================================================

if [[ "$USE_PREVIOUS_NORMAL" == true ]]; then

    echo ""
    echo "=========================================="
    echo "Previous normals: $LOCKED_TARGET_FILE"
    echo "=========================================="

    if [[ "$GENERATE_PATH" == true ]]; then

	    TARGET_COUNT=$(count_targets "$LOCKED_TARGET_FILE")
	    echo "Previous normal count: $TARGET_COUNT"

	    if [[ "$TARGET_COUNT" -lt 2 ]]; then
		echo "A path requires at least 2 normals. Path generation skipped."
	    else
		if [[ ! -f "$SPLINE_SCRIPT" ]]; then
		    echo "ERROR: Spline script not found: $SPLINE_SCRIPT"
		    exit 1
		fi

		echo "Generating offset spline..."

		conda run -n lidar python "$SPLINE_SCRIPT" \
		    "$LOCKED_TARGET_FILE" \
		    "$MESH_OUTPUT" \
		    -o "$PATH_OUTPUT" \
		    --csv "$PATH_CSV" \
		    --offset "$SPLINE_OFFSET" \
		    --spacing "$SPLINE_SPACING" \
		    --iterations "$SPLINE_ITERATIONS" \
		    --smooth-start "$SPLINE_SMOOTH_START" \
		    --smooth-end "$SPLINE_SMOOTH_END" \
		    --order file \
		    --side outward \
		    --project-knots

		if [[ ! -f "$PATH_OUTPUT" ]]; then
		    echo "ERROR: Spline generation finished but no path was created: $PATH_OUTPUT"
		    exit 1
		fi

		echo "Path successfully generated: $PATH_OUTPUT"
		echo "CSV: $PATH_CSV"
		
		echo ""
		echo "Displaying generated path in RViz..."

		if ! ros2 service call /load_and_publish_path std_srvs/srv/Trigger "{}"; then
		    echo "WARNING: Failed to request path display."
		else
		    echo "Path display request sent to map pipeline."
		fi
	    fi

	else
	    echo "Path generation disabled. Skipping spline generation."
	fi

# ============================================================
# NEW NORMAL SELECTION BRANCH
# ============================================================

else

    echo ""
    echo "=========================================="
    echo "New normal selection"
    echo "=========================================="
    echo "Select normals in RViz, then return here and press ENTER."
    echo ""
    echo "Waiting for normal-selection service..."

    while ! ros2 service list 2>/dev/null | grep -q "/finish_normal_selection"; do
        if ! kill -0 "$MAP_PID" 2>/dev/null; then
            echo "ERROR: map_pip_optitrack has exited."
            exit 1
        fi
        sleep 1
    done

    echo
	echo "=============================================="
	echo "        NORMAL SELECTION CONTROLS"
	echo "=============================================="
	echo "Select normals in RViz."
	echo
	echo "  Backspace  = undo last normal"
	echo "  Enter      = finish normal selection"
	echo
	echo "When using Backspace or Enter, first click this"
	echo "terminal so that it has keyboard focus."
	echo "=============================================="
	echo

	while true; do
	    IFS= read -rsn1 key

	    # Enter
	    if [[ "$key" == "" ]]; then
		echo
		echo "Finishing normal selection..."

		ros2 service call \
		    /finish_normal_selection \
		    std_srvs/srv/Trigger "{}"

		break
	    fi

	    # Backspace
	    if [[ "$key" == $'\x7f' ]]; then
		echo
		echo "Undoing last normal..."

		ros2 service call \
		    /undo_normal_selection \
		    std_srvs/srv/Trigger "{}"

		echo
		echo "Backspace = undo | Enter = finish"
	    fi
	done

    echo "Saving selected normals..."
    if ! ros2 service call /finish_normal_selection std_srvs/srv/Trigger "{}"; then
        echo "ERROR: Failed to finish normal selection."
        exit 1
    fi

    if [[ ! -f "$NORMAL_OUTPUT" ]]; then
        echo "ERROR: Selected-normal file was not generated: $NORMAL_OUTPUT"
        exit 1
    fi

    echo "Selected normals saved: $NORMAL_OUTPUT"

    read -r -p "Overwrite locked target with these normals? [y/N]: " OVERWRITE_REPLY
    if [[ "$OVERWRITE_REPLY" =~ ^[Yy]$ ]]; then
        cp "$NORMAL_OUTPUT" "$LOCKED_TARGET_FILE"
        echo "Locked target updated: $LOCKED_TARGET_FILE"
    else
        echo "Locked target was not changed."
    fi

    TARGET_COUNT=$(count_targets "$NORMAL_OUTPUT")
    echo "Selected normal count: $TARGET_COUNT"

    if [[ "$TARGET_COUNT" -lt 2 ]]; then
        echo "Only $TARGET_COUNT normal(s) selected. A path requires at least 2. Path generation skipped."
    else
        if [[ "$GENERATE_PATH" == true ]]; then
            if [[ ! -f "$SPLINE_SCRIPT" ]]; then
                echo "ERROR: Spline script not found: $SPLINE_SCRIPT"
                exit 1
            fi

            echo "Generating offset spline..."
            conda run -n lidar python "$SPLINE_SCRIPT" \
                "$NORMAL_OUTPUT" \
                "$MESH_OUTPUT" \
                -o "$PATH_OUTPUT" \
                --csv "$PATH_CSV" \
                --offset "$SPLINE_OFFSET" \
                --spacing "$SPLINE_SPACING" \
                --iterations "$SPLINE_ITERATIONS" \
                --smooth-start "$SPLINE_SMOOTH_START" \
                --smooth-end "$SPLINE_SMOOTH_END" \
                --order file \
                --side outward \
                --project-knots

            if [[ ! -f "$PATH_OUTPUT" ]]; then
                echo "ERROR: Spline generation finished but no path was created: $PATH_OUTPUT"
                exit 1
            fi

            echo "Path successfully generated: $PATH_OUTPUT"
            echo "CSV: $PATH_CSV"
			    
		echo ""
		echo "Displaying generated path in RViz..."

		if ! ros2 service call /load_and_publish_path std_srvs/srv/Trigger "{}"; then
		    echo "WARNING: Failed to request path display."
		else
		    echo "Path display request sent to map pipeline."
		fi
        else
            echo "Path generation skipped."
        fi
    fi
fi

# ============================================================
# Final summary
# ============================================================

echo ""
echo "=========================================="
echo "Pipeline complete"
echo "=========================================="
echo "Timestamp: $TIMESTAMP"
echo "Map:       $MAP_OUTPUT"
echo "Mesh:      $MESH_OUTPUT"

if [[ "$USE_PREVIOUS_NORMAL" == true ]]; then
    echo "Normal source: $LOCKED_TARGET_FILE"
else
    echo "Normal source: $NORMAL_OUTPUT"
fi

if [[ -f "$PATH_OUTPUT" ]]; then
    echo "Generated path: $PATH_OUTPUT"
    echo "Generated path CSV: $PATH_CSV"
else
    echo "Generated path: None"
fi

echo ""
echo "Livox / OptiTrack / mocap_tf_broadcaster / RViz / map pipeline are still running."
echo "Press Ctrl-C to shut everything down."

wait
