#!/usr/bin/env bash
# ============================================================
# collect.sh - point cloud collection pane
#
# Owns:  s = start collection, e = end collection
# Emits: $RUN_DIR/.collecting  (created on start, removed on end)
#
# The marker is the only thing the other panes need from us.
# ============================================================

set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
trap 'rm -f "${RUN_DIR:-/nonexistent}/$MARKER"; exit 0' INT

banner "COLLECTION"
echo "Waiting for /start_collection ..."
wait_for_service /start_collection

echo
echo "  s = start accumulating"
echo "  e = end accumulation"
echo "  q = quit the whole pipeline"
echo
echo "Click this pane first so it has keyboard focus."
echo "=============================================="
echo

COLLECTING=false
RUN_DIR=""

while true; do
    IFS= read -rsn1 key
    shout_key "$key"

    if [[ "$key" == "s" && "$COLLECTING" == false ]]; then
        if ! call_service /start_collection; then
            log_line "start failed: $SERVICE_MESSAGE"
            continue
        fi

        RUN_DIR="$(current_run)"
        if [[ -z "$RUN_DIR" ]]; then
            log_line "ERROR: node reported success but no run directory appeared."
            continue
        fi

        # Publish the marker only after the folder exists, so no other
        # pane can observe a run that isn't ready yet.
        touch "$RUN_DIR/$MARKER"

        COLLECTING=true
        log_line "started: $(basename "$RUN_DIR")"
        echo "        press 'e' to end"

    elif [[ "$key" == "e" && "$COLLECTING" == true ]]; then
        # Retract the marker first. mesh.sh stops taking new snapshots
        # and moves on to the final mesh, which finalizeRun() is about
        # to start waiting for.
        rm -f "$RUN_DIR/$MARKER"

        if ! call_service /stop_collection; then
            log_line "stop failed: $SERVICE_MESSAGE"
            touch "$RUN_DIR/$MARKER"   # still collecting; put it back
            cp "$PARAMS_FILE" "$RUN_DIR/params.yaml"
            continue
        fi

        COLLECTING=false
        log_line "stopped. Finalising - watch the mesh pane."
        echo
        echo "  s = start another run"
        echo "  q = quit pipeline    "
        echo
        
         # q = quit the whole pipeline
    elif [[ "$key" == "q" ]]; then
        read -rsn1 -p "        quit the WHOLE pipeline? [y/N] " confirm; echo
        if [[ "$confirm" != [yY] ]]; then
            log_line "  quit cancelled"
            continue
        fi

        if [[ "$COLLECTING" == true ]]; then
            log_line "  collection was running - stopping without a final mesh"
            rm -f "$RUN_DIR/$MARKER"
        fi

        log_line "shutting down all panes ..."

        # Ctrl-C every other pane so their INT traps run and they exit cleanly.
        for pane in $(tmux list-panes -s -F '#{pane_id}'); do
            [[ "$pane" == "$TMUX_PANE" ]] || tmux send-keys -t "$pane" C-c
        done
        sleep 1

        # Ending the session returns run_vision_pip from 'attach', and its
        # cleanup() then stops the node, DLIO, RViz, Livox and tmux itself.
        tmux kill-session
    fi
done
