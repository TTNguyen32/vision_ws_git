#!/usr/bin/env bash
# ============================================================
# mesh.sh - mesh generation pane
#
# No keyboard. Watches for an active run and meshes it:
#   while collecting : snapshot_NNNNN.pcd -> live_NNNNN.stl  (fast)
#   after collecting : global_map_TS.pcd  -> map_TS.stl      (quality)
#
# Both write to a dot-prefixed temp then rename, so the node never
# opens a partially written STL. Open3D picks its writer from the
# extension, so the temp name must still end in .stl.
# ============================================================

set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
trap 'exit 0' INT

banner "MESH GENERATION"
echo "Waiting for a run to start ..."
echo

mesh_one() {
    # mesh_one <input.pcd> <output.stl> <label> <pcd_to_stl args...>
    local in="$1" out="$2" label="$3"; shift 3
    local tmp start elapsed
    tmp="$(dirname "$out")/.tmp_$(basename "$out")"

    start=$(date +%s.%N)
    if conda run -n "$CONDA_ENV" python "$MESH_RECONSTRUCT" "$in" "$tmp" "$@" \
            >> "$LOG" 2>&1
    then
        mv -f "$tmp" "$out"
        elapsed=$(echo "$(date +%s.%N) - $start" | bc)
        log_line "$label $(basename "$out")  ${elapsed}s"
        return 0
    fi

    rm -f "$tmp"
    log_line "$label FAILED on $(basename "$in") - see $(basename "$LOG")"
    return 1
}

while true; do
    RUN_DIR="$(wait_for_active_run)"
    TS="$(run_stamp "$RUN_DIR")"
    LOG="$RUN_DIR/logs/mesh.log"

    log_line "run $(basename "$RUN_DIR")"

    # ---- live meshing, until the marker is retracted ----
    last=""
    while [[ -f "$RUN_DIR/$MARKER" ]]; do
        newest=$(ls -1 "$RUN_DIR/maps"/snapshot_*.pcd 2>/dev/null | sort | tail -n1)

        if [[ -n "$newest" && "$newest" != "$last" ]]; then
            num=$(basename "$newest" .pcd); num="${num#snapshot_}"
            mesh_one "$newest" "$RUN_DIR/meshes/live_${num}.stl" "live" \
                     "${MESH_LIVE_ARGS[@]}"
            # Record the input we just handled either way. On failure we
            # skip ahead rather than retry, so one bad snapshot can't
            # wedge the loop.
            last="$newest"
        fi
        sleep 1
    done

    # ---- collection ended: final mesh ----
    rm -f "$RUN_DIR/meshes/.tmp_live_"*.stl

    FINAL_PCD="$RUN_DIR/maps/global_map_${TS}.pcd"
    log_line "waiting for final cloud ..."

    waited=0
    while [[ ! -f "$FINAL_PCD" ]]; do
        if (( waited >= 120 )); then
            log_line "ERROR: no final cloud after 120s - check the node pane."
            break
        fi
        sleep 1
        ((waited++))
    done

    if [[ -f "$FINAL_PCD" ]]; then
           mesh_one "$FINAL_PCD" "$RUN_DIR/meshes/map_${TS}.stl" "FINAL" \
                 "${MESH_FINAL_ARGS[@]}"
    fi

    log_line "idle - waiting for the next run"
    echo
done
