# OptiTrack setup

## Network

OptiTrack (192.168.50.x) and the Livox Mid-360 (192.168.1.x, its expected
default) sit on different subnets and conflict on a single NIC. Both run
simultaneously via two separate Ethernet adapters, one static-IP'd per
subnet.

## Rigid body naming

`rigid_body_names` in `optitrack_multiplexer_config.yaml` must match the
name **Motive** has the rigid body registered under, exactly. A mismatch
produces a live topic with a connected publisher but zero messages —
no error anywhere. Confirm the real name via a data-descriptions service
call rather than assuming a config edit took effect.

Current value: `Tin_LIDAR`.

## world_frame convention

All OptiTrack config and every C++ node use `world`, not `odom`, for the
global frame — `optitrack_multiplexer_config.yaml`'s `world_frame`,
`mocap_tf_broadcaster.cpp`'s default `world_frame` param, and
`pipeline_params.yaml`'s `node.world_frame` all need to agree.

## Fork + submodule structure

`src/optitrack_packages_ros2` is a git submodule pointing at
`TTNguyen32/optitrack_packages_ros2` — a fork of
[`lis-epfl/optitrack_packages_ros2`](https://github.com/lis-epfl/optitrack_packages_ros2),
not the upstream repo directly. The project-specific config
(`Tin_LIDAR`, `world` frame, `192.168.50.141` server address) is
committed inside the fork itself, not layered on top — `config/optitrack/`
in this repo holds **reference copies only**; the live files are inside
the submodule.

To update the OptiTrack config:

```bash
cd src/optitrack_packages_ros2
# edit optitrack_multiplexer_ros2/config/optitrack_multiplexer_config.yaml
# edit optitrack_wrapper_ros2/config/optitrack_wrapper_config.yaml
git add -A && git commit -m "..."
git push origin HEAD      # not 'main' — the fork's default branch is 'master'
cd ../..
git add src/optitrack_packages_ros2
git commit -m "Pin optitrack_packages_ros2 submodule to <what changed>"
```

Cloning this repo fresh needs `--recurse-submodules`, or the submodule
directory will exist but be empty:

```bash
git clone --recurse-submodules <this repo>
```

## Building

`cloud_pipeline` depends on message packages defined inside the
submodule. Use `--packages-up-to`, not `--packages-select` — it walks
the full dependency graph from `package.xml`, so it doesn't matter how
many submodule packages `cloud_pipeline` ends up needing:

```bash
colcon build --packages-up-to cloud_pipeline
source install/setup.bash
```

## `pipeline_params.yaml`

All node/mesh/path tuning lives in `scripts/pipeline_params.yaml`, read
once at launch. To try alternative settings without editing the default:

```bash
PARAMS_FILE=~/experiments/deep_mesh.yaml ./run_vision_pip_optitrack_tf
```

## Debugging notes from the DLIO → OptiTrack/rqt migration

- **`load_params.py` run with plain `python3` under `(base)` conda**
  fails with `ModuleNotFoundError: No module named 'yaml'`. `common.sh`
  specifically probes `/usr/bin/python3` (system Python, has PyYAML from
  ROS's apt deps) first and falls back to `conda run -n lidar python`
  only if that fails. Test with `/usr/bin/python3 scripts/load_params.py ...`
  to match what `common.sh` actually runs, not plain `python3`.

- **`project(cloud_accumulator)` in `CMakeLists.txt`** controls the
  `lib/${PROJECT_NAME}` install path *independently* of `package.xml`'s
  `<name>` tag. A mismatch between the two means executables install
  under a different `lib/` subfolder than `ros2 run <pkg>` expects —
  `ros2 pkg executables <pkg>` and `ls install/<pkg>/lib/<pkg>/` are
  the actual verification; a successful `colcon build` alone doesn't
  prove the install path is right.

- **`colcon build --packages-select cloud_pipeline`** fails one
  submodule dependency at a time (`optitrack_multiplexer_ros2_msgs`,
  then `optitrack_wrapper_ros2_msgs`, ...) because it only builds the
  named package, not its dependencies. Use `--packages-up-to
  cloud_pipeline` instead — it resolves the full graph from every
  `package.xml` in one pass.

- **`git push origin main`** fails with `src refspec main does not
  match any` if the target branch is actually `master` (true for the
  OptiTrack fork, inherited from upstream). `git push origin HEAD`
  pushes whatever branch you're actually on, regardless of its name —
  avoids this class of error entirely.

- **A commit succeeding locally doesn't mean it reached GitHub.**
  `git branch -vv` shows whether the current branch is ahead of its
  remote tracking branch. Worth checking in *both* the submodule and
  the outer repo before considering a stage done — a gitlink commit in
  the outer repo can be perfectly correct while pointing at a submodule
  commit that only exists locally.
