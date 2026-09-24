#!/usr/bin/env bash
# ============================================================
# select.sh - normal selection pane
#
# Owns:  Backspace = undo last normal
#        Enter     = save normals (+ optionally overwrite locked target)
#        l         = load locked target normals into the selection
#        x         = clear all selected normals (asks to confirm)
#
# Selection never locks: Enter can be pressed any number of times.
# Clearing only affects the node's selection, never the locked
# target file.
#
# Deliberately run-agnostic: every key is a service call that acts
# on whatever the node currently holds, so this pane needs no run
# folder and never has to be restarted between runs.
#
# Clicking itself happens in RViz, not here.
# ============================================================

set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
trap 'exit 0' INT

banner "NORMAL SELECTION"
echo "Waiting for /save_normals ..."
wait_for_service /save_normals

echo
echo "  Click faces in RViz to select normals."
echo
echo "  Backspace = undo last normal"
echo "  Enter     = save normals (+ optionally overwrite locked target)"
echo "  l         = load locked target normals"
echo "  x         = clear all normals (asks to confirm)"
echo
echo "Click this pane first so it has keyboard focus."
echo "=============================================="
echo

while true; do
    IFS= read -rsn1 key
    shout_key "$key"

    # l = load locked target
    if [[ "$key" == "l" ]]; then
        if call_service /load_locked_target; then
            log_line "locked: $SERVICE_MESSAGE"
        else
            log_line "load failed: $SERVICE_MESSAGE"
        fi

    # Backspace (DEL)
    elif [[ "$key" == $'\x7f' ]]; then
        if call_service /undo_normal_selection; then
            log_line "undo: $SERVICE_MESSAGE"
        else
            log_line "undo failed: $SERVICE_MESSAGE"
        fi

    # Enter = save (selection stays open)
    elif [[ "$key" == "" ]]; then
        if ! call_service /save_normals; then
            log_line "save failed: $SERVICE_MESSAGE"
            continue
        fi

        NORMAL_OUTPUT="$SERVICE_MESSAGE"
        if [[ ! -f "$NORMAL_OUTPUT" ]]; then
            log_line "ERROR: node reported $NORMAL_OUTPUT but it does not exist"
            continue
        fi

        log_line "saved: $NORMAL_OUTPUT"

        read -rsn1 -p "        overwrite locked target? [y/N] " confirm; echo
        if [[ "$confirm" == [yY] ]]; then
            tmp="$(dirname "$LOCKED_TARGET_FILE")/.tmp_locked_target.yaml"
            if cp "$NORMAL_OUTPUT" "$tmp" && mv -f "$tmp" "$LOCKED_TARGET_FILE"; then
                log_line "locked target updated: $LOCKED_TARGET_FILE"
            else
                rm -f "$tmp"
                log_line "ERROR: could not update $LOCKED_TARGET_FILE"
            fi
        else
            log_line "  locked target unchanged"
        fi

    # x = clear all
    elif [[ "$key" == "x" ]]; then
        read -rsn1 -p "        clear ALL normals? [y/N] " confirm; echo
        if [[ "$confirm" == [yY] ]]; then
            if call_service /clearNormals; then
                log_line "cleared: $SERVICE_MESSAGE"
            else
                log_line "clear failed: $SERVICE_MESSAGE"
            fi
        else
            log_line "  clear cancelled"
        fi

    else
        log_line "  no action"
    fi
done
