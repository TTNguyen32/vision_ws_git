# Cluster removal and the per-scan crop volume

Two independent pieces of work, plus one bug fix that fell out of the
second.

- [Cluster removal](#1-cluster-removal)
- [Crop volume: box → cone → spherical sector](#2-crop-volume-box--cone--spherical-sector)
- [Node name fix](#3-node-name-fix)
- [Known gaps](#4-known-gaps)
- [Test checklist](#5-test-checklist)

All paths below are relative to the repo root. The node is the
`map_pip_optitrack_tf` executable in package `cloud_pipeline`; its
**node name** is `map_pip_tf` (see §3 — the distinction matters).

---

## 1. Cluster removal

### Why

A trunk scan should end up as one connected cluster. Anything else in
the crop volume — a pot, a wall, a bench leg — is a separate cluster,
and deleting it after the fact is cheaper than re-scanning with a
tighter crop. The feature only arms when more than one cluster is
found, so on a clean scan the button tells you so and changes nothing.

### Interface

Three `std_srvs/srv/Trigger` services. No custom `.srv`, no new
interface package, nothing to add to `CMakeLists.txt`.

| Service | Effect |
| --- | --- |
| `/cluster_map` | Cluster the map. Refuses with `success=false` when ≤ 1 cluster. Arms a selection. |
| `/confirm_cluster_removal` | Delete the pending cluster from `global_map_`. |
| `/cancel_cluster_removal` | Drop the selection. Changes nothing. |

Two topics, both `QoS(1).transient_local()` so a late subscriber still
sees the current state:

| Topic | Type | Purpose |
| --- | --- | --- |
| `/processed/clusters` | `sensor_msgs/PointCloud2` (XYZRGB) | Colour-coded clusters for RViz; pending one in red, unclustered points dark grey |
| `/processed/cluster_status` | `std_msgs/String` | One machine-readable line for the panel |

Status line grammar:

```
idle
armed|<n_clusters>
pending|<index>|<n_clusters>|<points>|<x>|<y>|<z>
```

### Flow

```
Remove cluster ──► /cluster_map            cluster, refuse if ≤ 1
click in RViz  ──► /clicked_point          mark pending, publish "pending|…"
                                            → panel pops the dialog
Yes            ──► /confirm_cluster_removal points deleted
No             ──► /cancel_cluster_removal  nothing changes
```

Nothing is deleted until confirm arrives. That is what allows a dialog
to sit between the click and the deletion.

### Parameters

| Name | Default (C++) | `pipeline_params.yaml` | Notes |
| --- | --- | --- | --- |
| `cluster_voxel` | 0.01 | 0.01 | leaf size of the copy that gets clustered |
| `cluster_tolerance` | 0.05 | 0.03 | join radius; must exceed `cluster_voxel` |
| `cluster_min_points` | 50 | 10 | counted in **voxels**, not raw points |
| `cluster_max_points` | 10000000 | 10000000 | |
| `cluster_click_radius` | 0.10 | 0.10 | a click further than this from any clustered point is ignored |
| `clusters_topic` | `/processed/clusters` | same | |
| `cluster_status_topic` | `/processed/cluster_status` | same | |

### Two decisions worth remembering

**Clustering runs on a voxelised copy, deletion runs on the real map.**
Accumulation dedups at `duplicate_distance = 5 mm`, so `global_map_` is
far too dense for Euclidean clustering inside a service callback. The
copy is taken under `map_mutex_` and the lock released immediately, so
`cloudCallback()` keeps accumulating while clustering runs.

**Deletion cannot work by index.** The cluster holds *voxel centres*,
not real map points. So confirm builds a `KdTreeFLANN` on `global_map_`
and drops every real point within one voxel of a picked centre. A
voxel's half-diagonal is 0.866 × leaf, so a radius of one leaf covers
the whole cell with margin.

```cpp
std::vector<char> drop(before, 0);   // not vector<bool> — that's a bitfield
for (int i : picked)
    if (tree.radiusSearch(cluster_cloud_->points[i], radius, nb, nd) > 0)
        for (int j : nb) drop[j] = 1;
```

The kept points go into a fresh cloud rather than being erased in
place — erasing scattered indices from a vector is O(n) per erase,
whereas one filtered copy is a single pass. It replaces `global_map_`
under the mutex, so `cloudCallback()` never sees a half-modified map.

### Non-obvious places that had to be touched

- `onClickedPoint()` gained an early branch: while armed, a click picks
  a cluster instead of a normal. One RViz tool, two meanings, chosen by
  mode.
- `startCollection()` and `clearMapService()` both call
  `resetClusterSelection()`. `cluster_indices_` indexes into
  `cluster_cloud_`; a stale pair would delete the wrong points.
- After removal, `last_processed_msg_` is updated as well as
  republished — the 1 Hz keepalive timer would otherwise put the old
  map straight back.
- `clearRVizVisualizations()` publishes an empty cloud on
  `clusters_pub_` so the overlay clears with everything else.

### Mesh rebuild

The cloud on disk changes but nothing in the normal flow notices,
because `MeshWatcher` reacts to the `.collecting` marker and to **new**
snapshot filenames — and a post-collection removal rewrites
`global_map_TS.pcd` in place, same name.

- **While collecting**: the node writes a fresh snapshot immediately, so
  the LIVE branch of `_tick()` picks it up on its own.
- **After collecting**: the node rewrites `save_path_`, and
  `MeshWatcher.remesh()` starts a FINAL job on it.

`remesh()` is the only entry point into the watcher that isn't the
timer. It refuses while a mesh is running, and returns
`(handled, message)` rather than silently doing nothing. The panel asks
before calling it — a final mesh is the expensive job here, and after
removing the first of three clusters you usually want to remove the
other two first.

### Files

| File | Change |
| --- | --- |
| `src/cloud_pipeline/src/map_pip_optitrack_tf.cpp` | 3 services, 2 publishers, 7 methods, click branch, resets |
| `src/vision_pip_gui/vision_pip_gui/cluster.py` | **new** — `ClusterStatus`, `ClusterSelection` |
| `src/vision_pip_gui/vision_pip_gui/pipeline_panel.py` | button, dialog, service constants |
| `src/vision_pip_gui/vision_pip_gui/mesh_watcher.py` | `remesh()` |
| `scripts/pipeline_params.yaml` | cluster params |

### Thread note

`ClusterStatus` exists for one reason: rclpy delivers the status on the
executor thread, and `QMessageBox` may only open on the GUI thread.
Emitting a Qt signal across thread affinity gives a queued connection,
so the panel's slot runs on the GUI thread. The dialog then blocks the
500 ms poll timer, which is wanted — the node holds the selection until
it hears confirm or cancel.

`_cluster_armed` guards against the latched topic replaying a stale
`pending` line after a panel restart.

### RViz

Needs a `PointCloud2` display on `/processed/clusters` with
**Color Transformer: RGB8**. Not yet in the committed `.rviz` config.

---

## 2. Crop volume: box → cone → spherical sector

The per-scan crop went through two changes in one session. Only the
final state matters for the code, but the intermediate one explains the
parameter names in any commits between them.

### Where it is applied

Two places only — the deskew loop and the fallback loop in
`cloudCallback()`, both via `inFov()`. The **world-frame** `final_*`
crop (`cropToFinalBounds()`) is a different thing and was not touched;
it is still an axis-aligned box.

### Final parameters

Spherical coordinates about the sensor origin, in the LiDAR frame.

| Name | C++ default | `pipeline_params.yaml` | |
| --- | --- | --- | --- |
| `fov_min_range` | 0.10 | 0.1 | true radial distance, not axial |
| `fov_max_range` | 0.60 | 0.6 | |
| `fov_min_azimuth_deg` | −20.0 | −20.0 | `atan2(y, x)`; 0° = +x boresight, +90° = +y |
| `fov_max_azimuth_deg` | 20.0 | 20.0 | |
| `fov_min_elevation_deg` | −20.0 | **20.0** | angle above the xy-plane; +90° = +z |
| `fov_max_elevation_deg` | 20.0 | **50.0** | |

The elevation window in `pipeline_params.yaml` is deliberately
asymmetric and upward-looking, unlike the symmetric C++ default — it
sits inside the Mid-360's real −7°/+52° range. So the start-of-run log
reads `elevation 20 to 50 deg`, not the C++ default.

Mid-360 hardware limits, for choosing these: azimuth 360°, elevation
−7° to +52°, close-proximity blind zone 0.1 m. Asking for more than the
sensor provides is harmless; the limit just never binds.

**`min_range` was removed.** `fov_min_range` does the same job — any
positive inner radius drops the `(0,0,0)` no-return beams — and two
overlapping range floors are dead weight.

**Removed parameters**, for anyone diffing an old `pipeline_params.yaml`:
`min_x`, `max_x`, `min_y`, `max_y`, `min_z`, `max_z`, `min_range`, and
the short-lived `cone_min_x`, `cone_max_x`, `cone_half_angle_deg`,
`cone_near_radius`.

### The test

```cpp
inline bool inFov(float x, float y, float z) const
{
    const float rho2 = x*x + y*y;
    const float r2   = rho2 + z*z;
    if (r2 < fov_min_range_sq_ || r2 > fov_max_range_sq_) return false;

    if (!fov_el_full_) {
        const float rho = std::sqrt(rho2);
        if (z < fov_tan_el_min_ * rho) return false;
        if (z > fov_tan_el_max_ * rho) return false;
    }
    if (fov_az_full_) return true;

    float d = std::fmod(std::atan2(y, x) * RAD2DEG - fov_az_min_f_, 360.0f);
    if (d < 0.0f) d += 360.0f;
    return d <= fov_az_span_;
}
```

Ordered cheapest first — ~20k points per scan at 10 Hz, over two code
paths. Range costs only multiplies. Elevation compares `z` against
`tan(el)·ρ` rather than calling `asin` per point: ρ ≥ 0 and `tan` is
monotonic on (−90, 90), so it is the same ordering as comparing the
angles. Azimuth is the only `atan2`, and it is skipped when the full
circle is wanted.

### Azimuth wraps, elevation does not

The subtle part, and the part to not "simplify" later.

Azimuth min **above** max is legal and meaningful: a wedge straddling
±180. The span is measured counter-clockwise from min to max.

| Setting | Means |
| --- | --- |
| `-170 → 170` | the wide 340° wedge |
| `170 → -170` | the narrow 20° wedge |
| `-180 → 180` | the whole circle |

The last case must be caught **before** the `fmod`, which folds 360 to 0
and would pass nothing. `updateFovDerived()` checks
`std::abs(raw) >= 360.0` first.

Elevation cannot wrap, so min above max is simply an error — the node
swaps and warns. And `tan` is negative past 90°, which would silently
invert the test into "keep everything outside the sector", so a request
reaching either pole becomes `fov_el_full_` (no elevation limit) rather
than being clamped into nonsense.

Verified numerically against a Python mirror: default sector, the ±180
wedge, full circle, the Mid-360's real −7°/+52° window, and `(0,0,0)`
rejection.

### Panel layout

`params.py`: `BOX_FIELDS` → `CONE_FIELDS` → `FOV_FIELDS`.

The parameter grid used to index with `divmod(i, 3)`, which only lined
up because both field groups happened to have exactly three entries.
Each group now flows down its own column, wrapping every `GRID_ROWS`
fields, with the next group always starting on a fresh column. Six
sector fields and three tool fields land as 3 + 3 + 3, and neither list
is pinned to a length any more.

### Files

| File | Change |
| --- | --- |
| `src/cloud_pipeline/src/map_pip_optitrack_tf.cpp` | `inFov()`, `updateFovDerived()`, params, both loops, start-of-run log |
| `src/vision_pip_gui/vision_pip_gui/params.py` | `FOV_FIELDS` |
| `src/vision_pip_gui/vision_pip_gui/pipeline_panel.py` | grid layout, `_fov_set_done` |
| `scripts/pipeline_params.yaml` | `fov_*` block replacing the box |

### Behaviour change from the original box

The old box had `z ∈ [0, 0.2]` — asymmetric, keeping nothing below the
sensor plane. The sector is symmetric unless you set the elevation
limits asymmetrically, which `pipeline_params.yaml` does (20 → 50).

Also: the sensor sits at the sector's apex, so the wedge sweeps with the
drone. At ±20° azimuth you admit a 42 cm arc at 0.6 m but only 7 cm at
0.1 m. If the trunk fills the frame close in, there is no single
azimuth that is right at both ends — a cone could express that, a
sector cannot. Watch for it.

---

## 3. Node name fix

`pipeline_panel.py` defaulted `NODE_NAME` to `cloud_accumulator`, which
was the **package** name (now `cloud_pipeline`). The node is
`map_pip_tf`. Apply failed with:

```
/cloud_accumulator/set_parameters is not available
```

It hid for a long time because parameter services are the only ones
namespaced by node name. Every Trigger service here is declared
relative:

```cpp
create_service<std_srvs::srv::Trigger>("start_collection", ...)  → /start_collection
```

Relative names take the node's *namespace* (`/`), not its name, so
Start, End, Clear map and the cluster services resolve identically
whatever the panel thinks the node is called. `ParamSetter` is the only
consumer of `NODE_NAME`.

Fixed in two places:

- `pipeline_panel.py` — default is now `map_pip_tf`
- `scripts/run_vision_pip_optitrack_tf` — `MAP_NODE_NAME="map_pip_tf"`
  plus `export VISION_NODE_NAME="$MAP_NODE_NAME"`, and the existing
  params-file key check reads the variable instead of repeating the
  literal

Note there is no `add_on_set_parameters_callback` in the node and none
is needed — rclcpp creates `/<node>/set_parameters` automatically for
declared parameters. See the gaps below for why one might still be
worth adding.

---

## 4. Known gaps

Ordered roughly by how likely they are to bite. Status as of the
current `map_pip_optitrack_tf.cpp`.

1. **The node does not reload the mesh after a post-collection
   rebuild.** `mesh_centroids_`, `mesh_tree_` and `mesh_face_normals_`
   are only loaded by `finalizeRun()`, and `checkForNewMesh()` watches
   `live_*.stl` and only while collecting. So after a remesh following
   cluster removal, click-to-select and the cylinder fit read a stale
   mesh. Path generation is unaffected — it passes the mesh *file* to
   `offset_spline.py`. Proposed fix: a `/reload_mesh` Trigger calling
   `loadAndPublishMesh(mesh_path_)` and refitting, called by the panel
   when `MeshWatcher` reports a successful FINAL.
   **Status: open.** No `/reload_mesh` service exists in the node.

2. **`writeSnapshot()`'s point-count floor is too low for Poisson.**
   The `size() < 100` check runs **before** voxelisation, so a cloud
   that voxelises down to a handful of points still reaches the mesher.
   A `min_snapshot_points` parameter checked *after* voxelisation
   (~2000) would close the remaining crash surface.
   **Status: open.** Still `< 100`, still pre-voxelisation.

   The related `fitAndPublishCylinder()` guard **is** applied:

   ```cpp
   if (!cloud || !normals || cloud->size() < 10 ||
       cloud->size() != normals->size()) { ...; has_cylinder_ = false; return; }
   ```

   That was the suspected source of two `SIGFPE` crashes — a degenerate
   mesh from a near-empty map, which cluster removal makes easier to
   produce because a shrunken map survives into the next run
   (`startCollection()` deliberately does not clear `global_map_`).
   **Status: fixed.**

3. **No parameter validation callback.** A bad value is accepted, the
   panel logs success, and nothing complains until Start — where
   `updateFovDerived()` clamps it and warns into the *node* log, which
   the panel does not show. An `add_on_set_parameters_callback` doing
   per-field sanity would surface it at Apply. Cross-field checks
   (`fov_max_range > fov_min_range`) are only sound if `ParamSetter`
   uses `set_parameters_atomically`; with plain `set_parameters` rclcpp
   calls the callback once per parameter and a valid final state can be
   rejected mid-sequence. `services.py` currently uses plain
   `call_async` on `SetParameters`, so cross-field checks are **not**
   safe to add as-is.

4. **No RViz visualisation of the crop sector.** Aiming is currently
   done by watching the point count. A `Marker` of the sector on the
   live `world → lidar` TF would make it visible.

5. **The params snapshot is written before the node confirms.** In the
   failure above, `params_002.yaml` recorded ±90° as applied while the
   node never received it. Reordering `_on_apply_params()` to write
   after the service returns would keep the record honest.

---

## 5. Test checklist

Build both packages — the yaml and `params.py` are read from
`install/.../share/`, so editing source copies and relaunching silently
re-reads the stale installed ones.

```bash
conda deactivate                 # colcon needs system python
colcon build --packages-up-to cloud_pipeline vision_pip_gui
source install/setup.bash
```

**Node name**

```bash
ros2 node list                        # expect /map_pip_tf
ros2 service list | grep set_parameters
```

**FOV sector** — on Start, the log should read:

```
FOV sector: range [0.100, 0.600] m, azimuth -20 to 20 deg, elevation 20 to 50 deg
```

with `full 360 deg` / `unlimited` substituted when a limit is not
binding, so the line tells you which tests are actually running. Then
Apply a change and confirm with
`ros2 param get /map_pip_tf fov_max_azimuth_deg`.

**Cluster removal** — node first, panel after:

1. `ros2 service call /cluster_map std_srvs/srv/Trigger "{}"` on a
   finished run. A clean single-trunk scan should **refuse**; that is
   the path most likely to be wrong.
2. Add the RViz display, arm, click, check the log names the cluster
   and the face count.
3. Confirm, and check the saved PCD actually shrinks.
4. Only then exercise the panel button and dialog.
