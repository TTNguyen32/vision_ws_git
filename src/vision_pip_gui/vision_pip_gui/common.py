"""Paths and file helpers - the Python side of common.sh.

run_vision_pip passes the values from common.sh in the environment.
The defaults below match common.sh so the plugin also opens when rqt
is started by hand (mesh and path then need PARAMS_FILE and
PARAMS_LOADER set to work).
"""
import os
import re
import shlex
import shutil
import subprocess
import sys

HOME = os.path.expanduser('~')

OUTPUT_DIR = os.environ.get('OUTPUT_DIR', os.path.join(HOME, 'vision_ws_outputs'))
LOCKED_TARGET_FILE = os.environ.get(
    'LOCKED_TARGET_FILE', os.path.join(OUTPUT_DIR, 'locked_target.yaml'))

TOOLS_DIR = os.environ.get('TOOLS_DIR', os.path.join(HOME, 'lidar-tools'))
MESH_RECONSTRUCT = os.environ.get(
    'MESH_RECONSTRUCT', os.path.join(TOOLS_DIR, 'pcd_to_stl.py'))
SPLINE_SCRIPT = os.environ.get(
    'SPLINE_SCRIPT', os.path.join(TOOLS_DIR, 'offset_spline.py'))
CONDA_ENV = os.environ.get('CONDA_ENV', 'lidar')

PARAMS_LOADER = os.environ.get('PARAMS_LOADER', '')

# Present in a run folder == collection in progress.
MARKER = '.collecting'

TOOL_ARRAYS = ('MESH_LIVE_ARGS', 'MESH_FINAL_ARGS', 'SPLINE_ARGS')


# ------------------------------------------------------------ runs

def current_run():
    """Folder that 'latest' points at, or None."""
    link = os.path.join(OUTPUT_DIR, 'latest')
    if not os.path.islink(link):
        return None
    run = os.path.realpath(link)
    return run if os.path.isdir(run) else None


def run_stamp(run):
    """run_YYYYmmdd_HHMMSS -> YYYYmmdd_HHMMSS"""
    name = os.path.basename(run)
    return name[len('run_'):] if name.startswith('run_') else name


def is_collecting(run):
    return run is not None and os.path.isfile(os.path.join(run, MARKER))


def touch_marker(run):
    with open(os.path.join(run, MARKER), 'a'):
        pass


def remove_marker(run):
    try:
        os.remove(os.path.join(run, MARKER))
    except FileNotFoundError:
        pass


def sorted_files(folder, prefix, suffix):
    """Full paths of folder/prefix*suffix, sorted by name."""
    try:
        names = os.listdir(folder)
    except FileNotFoundError:
        return []
    return [os.path.join(folder, n) for n in sorted(names)
            if n.startswith(prefix) and n.endswith(suffix)]


def newest_file(folder, prefix, suffix):
    files = sorted_files(folder, prefix, suffix)
    return files[-1] if files else None


def tmp_name(out):
    """Dot-prefixed sibling; keeps the extension so Open3D picks its writer."""
    return os.path.join(os.path.dirname(out), '.tmp_' + os.path.basename(out))


def copy_atomic(src, dst):
    tmp = tmp_name(dst)
    try:
        shutil.copyfile(src, tmp)
        os.replace(tmp, dst)
    except OSError:
        try:
            os.remove(tmp)
        except OSError:
            pass
        raise


# ------------------------------------------------------ parameters

def params_file():
    """Per-boot params snapshot exported by run_vision_pip, or None."""
    fn = os.environ.get('PARAMS_FILE', '')
    return fn if fn and os.path.isfile(fn) else None


def copy_params(run):
    """Record the settings this run used. Returns the destination path."""
    src = params_file()
    if src is None:
        raise FileNotFoundError('PARAMS_FILE is not set or does not exist')
    dst = os.path.join(run, 'params.yaml')
    shutil.copyfile(src, dst)
    return dst


def load_tool_args():
    """Run load_params.py exactly as common.sh does and return its arrays.

    {'MESH_LIVE_ARGS': [...], 'MESH_FINAL_ARGS': [...], 'SPLINE_ARGS': [...]}
    Raises RuntimeError with a readable reason.
    """
    params = params_file()
    if params is None:
        raise RuntimeError('PARAMS_FILE is not set or does not exist')
    if not PARAMS_LOADER or not os.path.isfile(PARAMS_LOADER):
        raise RuntimeError(f'PARAMS_LOADER not found: "{PARAMS_LOADER}"')

    # rqt runs on /usr/bin/python3, which has PyYAML from ROS.
    res = subprocess.run([sys.executable, PARAMS_LOADER, 'shell', params],
                         capture_output=True, text=True, timeout=30)
    if res.returncode != 0:
        raise RuntimeError(res.stderr.strip() or 'load_params.py failed')

    arrays = {}
    for line in res.stdout.splitlines():
        m = re.fullmatch(r'([A-Z_]+)=\((.*)\)', line.strip())
        if m:
            # load_params.py quotes with shlex.quote; shlex.split undoes it.
            arrays[m.group(1)] = shlex.split(m.group(2))
    missing = [k for k in TOOL_ARRAYS if k not in arrays]
    if missing:
        raise RuntimeError(f'load_params.py did not define {missing}')
    return arrays


def conda_exe():
    return os.environ.get('CONDA_EXE') or shutil.which('conda')
