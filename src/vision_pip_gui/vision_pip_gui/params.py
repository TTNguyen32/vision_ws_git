"""The pipeline_params.yaml values the panel can change at runtime.

Two kinds, and they behave differently:

  fov fields   node parameters for the per-scan crop sector. The node
               reads them in startCollection(), so a change applies
               from the next Start, not to the run in progress.
  tool fields  flags for pcd_to_stl.py / offset_spline.py. The panel
               passes them to the next mesh or path job, so a change
               applies as soon as the current job finishes.

Changes live in memory only. pipeline_params.yaml on disk is never
rewritten, so the next launch starts from the file again; what each
run actually used is recorded in its own params files.
"""
import os
import time

import yaml

# Per-scan crop sector, in LiDAR-frame spherical coordinates. Azimuth
# runs from the +x boresight and wraps at +-180, so a minimum above the
# maximum is legal and means a wedge straddling the back of the sensor.
# The ranges here are the panel's guard rails only; the node clamps
# again in updateFovDerived(), because the yaml can be edited without
# the panel.
#
# (dotted key, label, is_int, minimum, maximum, step, decimals)
FOV_FIELDS = [
    ('node.fov_min_range',         'min range (m)',  False,    0.0,  50.0, 0.01, 3),
    ('node.fov_max_range',         'max range (m)',  False,    0.0,  50.0, 0.01, 3),
    ('node.fov_min_azimuth_deg',   'min azim (deg)', False, -180.0, 180.0, 1.0,  1),
    ('node.fov_max_azimuth_deg',   'max azim (deg)', False, -180.0, 180.0, 1.0,  1),
    ('node.fov_min_elevation_deg', 'min elev (deg)', False,  -90.0,  90.0, 1.0,  1),
    ('node.fov_max_elevation_deg', 'max elev (deg)', False,  -90.0,  90.0, 1.0,  1),
]
TOOL_FIELDS = [
    ('mesh.live.poisson_depth', 'live mesh depth', True, 1, 14, 1, 0),
    ('mesh.final.poisson_depth', 'final mesh depth', True, 1, 14, 1, 0),
    ('path.offset', 'path offset (m)', False, 0.0, 2.0, 0.005, 3),
]
FIELDS = FOV_FIELDS + TOOL_FIELDS


def load(path):
    """Parsed pipeline_params.yaml. Raises OSError/yaml.YAMLError."""
    with open(path) as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f'{path}: top level must be a mapping')
    return data


def get(data, dotted):
    node = data
    for part in dotted.split('.'):
        node = node[part]
    return node


def set_value(data, dotted, value):
    parts = dotted.split('.')
    node = data
    for part in parts[:-1]:
        node = node[part]
    node[parts[-1]] = value


def has_all_fields(data):
    try:
        for key, *_ in FIELDS:
            get(data, key)
    except (KeyError, TypeError):
        return False
    return True


def write(data, path, note=''):
    """Write params to path via a temp file. Returns path."""
    tmp = os.path.join(os.path.dirname(path), '.tmp_' + os.path.basename(path))
    header = f'# Effective parameters {time.strftime("%Y-%m-%d %H:%M:%S")}'
    if note:
        header += f' - {note}'
    with open(tmp, 'w') as f:
        f.write(header + '\n')
        yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
    os.replace(tmp, path)
    return path


def next_snapshot(run):
    """run/params_002.yaml, 003, ... alongside the params.yaml from Start."""
    highest = 1
    try:
        names = os.listdir(run)
    except OSError:
        names = []
    for name in names:
        stem, ext = os.path.splitext(name)
        if ext == '.yaml' and stem.startswith('params_'):
            try:
                highest = max(highest, int(stem[len('params_'):]))
            except ValueError:
                pass
    return os.path.join(run, f'params_{highest + 1:03d}.yaml')
