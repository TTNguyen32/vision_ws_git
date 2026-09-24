#!/usr/bin/env bash
# Standalone harness — INPUT/NORMALS/MESH paths below are hardcoded to a
# specific old run, for tuning parameters by hand. Not part of the pipeline.
set -uo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"
banner "MESH GENERATION (standalone test)"

# TOOLS_DIR / MESH_RECONSTRUCT already come from common.sh 

INPUT="$HOME/vision_ws_outputs/run_20260917_111733/maps/global_map_20260917_111733.pcd"
OUTPUT="$HOME/vision_ws_outputs/temp_mesh.stl"     # extension matters: Open3D picks its writer from it

if [[ ! -f "$INPUT" ]]; then
    echo "ERROR: input cloud not found: $INPUT" >&2
    exit 1
fi

# ---- parameters (every pcd_to_stl.py flag; defaults shown match the
#      script's own defaults except where noted "mesh.final" for what
#      pipeline_params.yaml actually uses in production) ----

# method
METHOD="poisson"              # poisson | bpa | alpha | delaunay

# preprocessing
VOXEL=0.015                     # voxel downsample size (m); 0 = off
REMOVE_OUTLIERS=true         # statistical outlier removal before meshing
OUTLIER_NEIGHBORS=20
OUTLIER_STD=2.0

# normals
NORMAL_RADIUS=0.0             # 0 = auto from point spacing
NORMAL_MAX_NN=30
ORIENT="radial"                # consistent | sensor | radial | none   (mesh.final: radial)
AXIS_DIR=""                    # "X Y Z", e.g. "0 0 1"; empty = auto (longest principal axis)
SENSOR_ORIGIN="0 0 0"          # "X Y Z"; used by --orient sensor / --project sensor
ORIENT_K=15                    # neighbours for --orient consistent

# poisson
POISSON_DEPTH=9                 # mesh.final: 9
POISSON_LINEAR_FIT=false
DENSITY_QUANTILE=0.50            # mesh.final: 0.10
CROP_TO_INPUT=true                # mesh.final: true

# ball pivoting (only used if METHOD=bpa)
BPA_RADII=""                    # "R1 R2 ..."; empty = auto from spacing

# alpha shape (only used if METHOD=alpha)
ALPHA=0.0                       # 0 = auto

# 2.5D delaunay (only used if METHOD=delaunay)
PROJECT="pca"                   # pca | xy | xz | yz | sensor
MAX_EDGE=0.0                    # 0 = auto

# mesh cleanup
KEEP_LARGEST=true                # mesh.final: true
MIN_CLUSTER_TRIANGLES=0
SMOOTH_ITERATIONS=0
TARGET_TRIANGLES=0

# output format
WRITE_ASCII=false

ARGS=(
    --method "$METHOD"
    --voxel "$VOXEL"
    --outlier-neighbors "$OUTLIER_NEIGHBORS"
    --outlier-std "$OUTLIER_STD"
    --normal-radius "$NORMAL_RADIUS"
    --normal-max-nn "$NORMAL_MAX_NN"
    --orient "$ORIENT"
    --sensor-origin $SENSOR_ORIGIN
    --orient-k "$ORIENT_K"
    --poisson-depth "$POISSON_DEPTH"
    --density-quantile "$DENSITY_QUANTILE"
    --alpha "$ALPHA"
    --project "$PROJECT"
    --max-edge "$MAX_EDGE"
    --min-cluster-triangles "$MIN_CLUSTER_TRIANGLES"
    --smooth-iterations "$SMOOTH_ITERATIONS"
    --target-triangles "$TARGET_TRIANGLES"
)
[[ -n "$AXIS_DIR" ]] && ARGS+=(--axis-dir $AXIS_DIR)
[[ -n "$BPA_RADII" ]] && ARGS+=(--bpa-radii $BPA_RADII)
[[ "$REMOVE_OUTLIERS" == true ]] && ARGS+=(--remove-outliers)
[[ "$POISSON_LINEAR_FIT" == true ]] && ARGS+=(--poisson-linear-fit)
[[ "$CROP_TO_INPUT" == true ]] && ARGS+=(--crop-to-input)
[[ "$KEEP_LARGEST" == true ]] && ARGS+=(--keep-largest)
[[ "$WRITE_ASCII" == true ]] && ARGS+=(--ascii)

conda run -n "$CONDA_ENV" python "$MESH_RECONSTRUCT" "$INPUT" "$OUTPUT" "${ARGS[@]}"

meshlab "$OUTPUT"

rm -f "$OUTPUT"
