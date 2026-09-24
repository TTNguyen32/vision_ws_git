#!/usr/bin/env bash
# Standalone harness — INPUT/NORMALS/MESH paths below are hardcoded to a
# specific old run, for tuning parameters by hand. Not part of the pipeline.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
banner "PATH GENERATION (standalone test)"

# TOOLS_DIR / SPLINE_SCRIPT already come from common.sh - no need to
# redefine them here.

NORMALS="$HOME/vision_ws_outputs/run_20260916_171707/normals/selected_normals_20260916_171707.yaml"
MESH="$HOME/vision_ws_outputs/run_20260916_171707/meshes/map_20260916_171707.stl"
OUTPUT="$HOME/vision_ws_outputs/temp_path.yaml"
CSV="$HOME/vision_ws_outputs/temp_path.csv"

if [[ ! -f "$NORMALS" ]]; then
    echo "ERROR: normals file not found: $NORMALS" >&2
    exit 1
fi
if [[ ! -f "$MESH" ]]; then
    echo "ERROR: mesh not found: $MESH" >&2
    exit 1
fi

# ---- parameters (every offset_spline.py flag; defaults shown match the
#      script's own defaults except where noted "path:" for what
#      pipeline_params.yaml actually uses in production) ----

OFFSET=0.05                    # standoff distance from surface (m)                path: 0.05
SPACING=0.01                   # arc-length spacing between waypoints (m)          path: 0.01
ITERATIONS=60                  # project/smooth iterations                        path: 60
SMOOTH_START=0.6               # initial Laplacian smoothing weight, 0..1          path: 0.6
SMOOTH_END=0.02                # final Laplacian smoothing weight, 0..1            path: 0.02

MODE="geodesic"                # geodesic | elastic  (geodesic = shortest path along offset surface;
                                # elastic = older smoothed/regularized, not length-minimizing)
ORDER="file"                   # file | greedy   (file = click order)              path: file
CLOSED=false                   # close the loop back to the first target
THROUGH_CONTACT=false          # use raw contact points as knots instead of standoff points
NO_PIN=false                   # do not re-anchor the path to targets each iteration
PROJECT_KNOTS=true             # slide each standoff along its own clicked normal to exactly
                                # --offset from the surface (keeps approach direction)       path: true
SIDE="outward"                 # outward | auto   (outward = always stay outside the trunk)  path: outward
NO_ORIENT_CHECK=false          # skip validating mesh normal orientation against the YAML normals
QUIET=false

ARGS=(
    -o "$OUTPUT"
    --csv "$CSV"
    --offset "$OFFSET"
    --spacing "$SPACING"
    --iterations "$ITERATIONS"
    --smooth-start "$SMOOTH_START"
    --smooth-end "$SMOOTH_END"
    --mode "$MODE"
    --order "$ORDER"
    --side "$SIDE"
)
[[ "$CLOSED" == true ]] && ARGS+=(--closed)
[[ "$THROUGH_CONTACT" == true ]] && ARGS+=(--through-contact)
[[ "$NO_PIN" == true ]] && ARGS+=(--no-pin)
[[ "$PROJECT_KNOTS" == true ]] && ARGS+=(--project-knots)
[[ "$NO_ORIENT_CHECK" == true ]] && ARGS+=(--no-orient-check)
[[ "$QUIET" == true ]] && ARGS+=(-q)

conda run -n "$CONDA_ENV" python "$SPLINE_SCRIPT" "$NORMALS" "$MESH" "${ARGS[@]}"

echo "wrote: $OUTPUT"
echo "wrote: $CSV"

rm -f "$OUTPUT" "$CSV"
