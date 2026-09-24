#!/usr/bin/env bash
# One-time environment setup for pcd_to_stl.py
# Creates a conda env called "lidar" with open3d + numpy.
set -euo pipefail

ENV_NAME="${1:-lidar}"

if conda env list | awk '{print $1}' | grep -qx "$ENV_NAME"; then
    echo "conda env '$ENV_NAME' already exists"
else
    conda create -y -n "$ENV_NAME" python=3.12
fi

conda run -n "$ENV_NAME" python -m pip install --upgrade open3d numpy
conda run -n "$ENV_NAME" python -c "import open3d, numpy; print('open3d', open3d.__version__, '| numpy', numpy.__version__)"

echo
echo "Done. Run the converter with:"
echo "  conda run -n $ENV_NAME python $(dirname "$0")/pcd_to_stl.py input.pcd output.stl"
