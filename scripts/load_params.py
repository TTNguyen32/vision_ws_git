#!/usr/bin/env python3
"""Read pipeline_params.yaml for the shell scripts and the node.

  load_params.py shell <params.yaml>
      Print bash array assignments (MESH_LIVE_ARGS, MESH_FINAL_ARGS,
      SPLINE_ARGS) for common.sh to eval.

  load_params.py node <params.yaml> <out.yaml>
      Write the 'node' section as a ROS 2 --params-file.

Exits non-zero with a readable message on any problem, so a typo in
the YAML stops the launch instead of silently using defaults.
"""
import shlex
import sys

import yaml

SECTIONS = ("node", "mesh", "path")
# Supplied by path.sh itself; setting them in the YAML would clash.
PATH_RESERVED = {"o", "output", "csv"}


def fail(msg):
    print(f"load_params: {msg}", file=sys.stderr)
    sys.exit(1)


def load(fn):
    try:
        with open(fn) as f:
            data = yaml.safe_load(f)
    except FileNotFoundError:
        fail(f"file not found: {fn}")
    except yaml.YAMLError as e:
        fail(f"invalid YAML in {fn}:\n{e}")

    if not isinstance(data, dict):
        fail(f"{fn}: top level must be a mapping")
    unknown = set(data) - set(SECTIONS)
    if unknown:
        fail(f"{fn}: unknown section(s) {sorted(unknown)}; expected {list(SECTIONS)}")
    for s in SECTIONS:
        if not isinstance(data.get(s), dict):
            fail(f"{fn}: section '{s}' is missing or not a mapping")
    for s in ("live", "final"):
        if not isinstance(data["mesh"].get(s), dict):
            fail(f"{fn}: 'mesh.{s}' is missing or not a mapping")
    return data


def to_flags(section, name):
    """{'poisson_depth': 7, 'keep_largest': True} -> ['--poisson-depth', '7', '--keep-largest']"""
    out = []
    for key, val in section.items():
        flag = "--" + str(key).replace("_", "-")
        if val is True:
            out.append(flag)
        elif val is False or val is None:
            continue
        elif isinstance(val, list):
            out += [flag] + [str(v) for v in val]
        elif isinstance(val, dict):
            fail(f"{name}.{key}: nested mappings are not allowed")
        else:
            out += [flag, str(val)]
    return out


def bash_array(var, items):
    return f"{var}=({' '.join(shlex.quote(i) for i in items)})"


def cmd_shell(fn):
    data = load(fn)
    clash = PATH_RESERVED & set(data["path"])
    if clash:
        fail(f"path.{sorted(clash)[0]} is set by path.sh; remove it from {fn}")
    print(bash_array("MESH_LIVE_ARGS", to_flags(data["mesh"]["live"], "mesh.live")))
    print(bash_array("MESH_FINAL_ARGS", to_flags(data["mesh"]["final"], "mesh.final")))
    print(bash_array("SPLINE_ARGS", to_flags(data["path"], "path")))


def cmd_node(fn, out):
    node = load(fn)["node"]
    for key, val in node.items():
        if isinstance(val, (dict, list)) or val is None:
            fail(f"node.{key}: must be a single value")
    with open(out, "w") as f:
        f.write(f"# Generated from {fn} - edit that file, not this one.\n")
        yaml.safe_dump({"/**": {"ros__parameters": node}}, f,
                       default_flow_style=False, sort_keys=False)


if __name__ == "__main__":
    if len(sys.argv) == 3 and sys.argv[1] == "shell":
        cmd_shell(sys.argv[2])
    elif len(sys.argv) == 4 and sys.argv[1] == "node":
        cmd_node(sys.argv[2], sys.argv[3])
    else:
        fail(__doc__)
