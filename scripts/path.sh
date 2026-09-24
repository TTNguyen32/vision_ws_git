#!/usr/bin/env bash
# ============================================================
# path.sh - path generation pane
#
# Owns:  p = generate a spline from the current normals + mesh
#
# Resolves the run on each press rather than holding it, so it
# follows whatever run is current without restarting.
#
# During collection it uses the newest live mesh; afterwards the
# final one. /dump_normals enforces the 2-normal minimum node-side,
# so we never spawn conda for a run that cannot succeed.
# ============================================================

set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
trap 'exit 0' INT

banner "PATH GENERATION"
echo "Waiting for /dump_normals ..."
wait_for_service /dump_normals

echo
echo "  p = generate path from current normals"
echo "  x = clear generated path (keep file)"
echo
echo "Needs at least 2 normals and one mesh."
echo "Click this pane first so it has keyboard focus."
echo "=============================================="
echo
    
while true; do
    IFS= read -rsn1 key
    shout_key "$key"
    
    if [[ "$key" == "p" ]]; then

	    RUN_DIR="$(current_run)"
	    if [[ -z "$RUN_DIR" ]]; then
		log_line "no run yet"
		continue
	    fi

	    TS="$(run_stamp "$RUN_DIR")"
	    LOG="$RUN_DIR/logs/path.log"

	    if ! call_service /dump_normals; then
		log_line "$SERVICE_MESSAGE"
		continue
	    fi
	    NORMALS="$SERVICE_MESSAGE"

	    if [[ ! -f "$NORMALS" ]]; then
		log_line "ERROR: node reported $NORMALS but it does not exist"
		continue
	    fi

	    # Pick the mesh: final if collection has ended and it exists,
	    # otherwise the newest live one.
	    if [[ ! -f "$RUN_DIR/$MARKER" && -f "$RUN_DIR/meshes/map_${TS}.stl" ]]; then
		MESH="$RUN_DIR/meshes/map_${TS}.stl"
		OUT="$RUN_DIR/paths/path_${TS}.yaml"
		CSV="$RUN_DIR/paths/path_${TS}.csv"
		LABEL="final"
	    else
		MESH=$(ls -1 "$RUN_DIR/meshes"/live_*.stl 2>/dev/null | sort | tail -n1)
		NUM=$(basename "$NORMALS" .yaml); NUM="${NUM#live_}"
		OUT="$RUN_DIR/paths/live_${NUM}.yaml"
		CSV="$RUN_DIR/paths/live_${NUM}.csv"
		LABEL="live"
	    fi

	    if [[ -z "$MESH" || ! -f "$MESH" ]]; then
		log_line "no mesh available yet"
		continue
	    fi

	    TMP="$(dirname "$OUT")/.tmp_$(basename "$OUT")"

	        if conda run -n "$CONDA_ENV" python "$SPLINE_SCRIPT" \
		    "$NORMALS" "$MESH" \
		    -o "$TMP" \
		    --csv "$CSV" \
		    "${SPLINE_ARGS[@]}" \
		    >> "$LOG" 2>&1
	    then
		mv -f "$TMP" "$OUT"
		if call_service /load_and_publish_path; then
		    log_line "$LABEL path: $(basename "$OUT")  [$(basename "$MESH")]"
		else
		    log_line "written but not published: $SERVICE_MESSAGE"
		fi
	    else
		rm -f "$TMP"
		log_line "spline failed - see $(basename "$LOG")"
	    fi
    
    elif [[ "$key" == "x" ]]; then
        read -rsn1 -p "        clear path? [y/N] " confirm; echo
        if [[ "$confirm" == [yY] ]]; then
            if call_service /clear_path; then
                log_line "cleared: $SERVICE_MESSAGE"
            else
                log_line "clear failed: $SERVICE_MESSAGE"
            fi
        else
            log_line "  clear cancelled"
        fi
    fi
done
