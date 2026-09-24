#!/usr/bin/env bash
# ============================================================
# common.sh - shared config and helpers for the vision pipeline
#
# Source this from the pane scripts; do not execute it directly.
#   source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
#
# Assumes this file sits in the SAME directory as collect.sh,
# mesh.sh, select.sh, path.sh, run_vision_pip,
# load_params.py and pipeline_params.yaml.
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)" # Path setup: this file (common.sh) - make it safe to source anywhere

# Config variables
WORKSPACE="${WORKSPACE:-$HOME/vision_ws}"
TOOLS_DIR="${TOOLS_DIR:--$SCRIPT_DIR/lidar-tools}"

OUTPUT_DIR="$HOME/vision_ws_outputs"
LOCKED_TARGET_FILE="$OUTPUT_DIR/locked_target.yaml"

MESH_RECONSTRUCT="$TOOLS_DIR/pcd_to_stl.py"
SPLINE_SCRIPT="$TOOLS_DIR/offset_spline.py"
RVIZ_CONFIG="$SCRIPT_DIR/../config/map_pip.rviz"

CONDA_ENV="lidar"

# Marker file inside a run folder. Present == collection in progress.
# This is what lets the other panes tell "a run is live right now"
# from "the latest symlink still points at an old, finished run".
MARKER=".collecting"

# ---- TUNEABLE PARAMETERS ------------------------------------
# All tuning lives in pipeline_params.yaml. Point PARAMS_FILE at
# another file to run an experiment without editing the default.
# The launcher exports PARAMS_FILE as a per-boot snapshot, so every
# pane reads exactly the settings the node was started with.
PARAMS_FILE="${PARAMS_FILE:-$SCRIPT_DIR/pipeline_params.yaml}"
PARAMS_LOADER="$SCRIPT_DIR/load_params.py"

# ROS 2 pulls in python3-yaml, as system python normally has yaml (compared to conda)
if /usr/bin/python3 -c 'import yaml' 2>/dev/null; then
    PARAMS_PY=(/usr/bin/python3)
else
    PARAMS_PY=(conda run -n "$CONDA_ENV" python) # fall back to conda if system doesn't have yaml
fi

# Defines MESH_LIVE_ARGS, MESH_FINAL_ARGS, SPLINE_ARGS (bash arrays). 
#Runs load_params.py shell "$PARAMS_FILE", which prints three VAR=(...) bash array assignment lines #(MESH_LIVE_ARGS=(...), MESH_FINAL_ARGS=(...), SPLINE_ARGS=(...)) to stdout. That output is captured into _params_sh.
if ! _params_sh="$("${PARAMS_PY[@]}" "$PARAMS_LOADER" shell "$PARAMS_FILE")"; then
    echo "ERROR: could not load parameters from $PARAMS_FILE" >&2
    exit 1
fi
eval "$_params_sh"
unset _params_sh

# ============================================================
# Helpers
# ============================================================

log_line() { printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; } # consistently format every pane's log line [HH:MM:SS]

# Call a std_srvs/Trigger service and surface the result.
# Sets SERVICE_MESSAGE. Returns 0 on success=True, 1 otherwise.
# Every silent failure we hit during development came from piping
# these to /dev/null, so nothing here ever does.
call_service() {
    local svc="$1" resp
    resp=$(ros2 service call "$svc" std_srvs/srv/Trigger "{}" 2>&1)
    SERVICE_MESSAGE=$(grep -oP "message='\K[^']*" <<< "$resp")
    [[ -z "$SERVICE_MESSAGE" ]] && SERVICE_MESSAGE="$resp"
    grep -q "success=True" <<< "$resp"
}

wait_for_service() {
    local svc="$1"
    until ros2 service list 2>/dev/null | grep -q "^${svc}$"; do
        sleep 1
    done
}

# Echo the current run folder, or empty if there isn't one.
current_run() {
    local run
    run="$(readlink -f "$OUTPUT_DIR/latest" 2>/dev/null || true)"
    [[ -n "$run" && -d "$run" ]] && echo "$run"
}

# Block until a run is actively collecting, then echo its folder.
wait_for_active_run() {
    local run
    while true; do
        run="$(current_run)"
        if [[ -n "$run" && -f "$run/$MARKER" ]]; then
            echo "$run"
            return 0
        fi
        sleep 0.5
    done
}

# run_YYYYmmdd_HHMMSS -> YYYYmmdd_HHMMSS
run_stamp() {
    local id
    id="$(basename "$1")"
    echo "${id#run_}"
}

banner() {
    echo
    echo "=============================================="
    echo "  $1"
    echo "=============================================="
}

# Report which key was pressed, so no keypress is silent.
shout_key() {
    case "$1" in
        "")          log_line "key: Enter" ;;
        $'\x7f')     log_line "key: Backspace" ;;
        " ")         log_line "key: Space" ;;
        [[:print:]]) log_line "key: $1" ;;
        *)           log_line "key: $(printf '%q' "$1")" ;;
    esac
}
