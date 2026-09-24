# Mesh & path generation — parameter reference

Reference for every tunable flag in `scripts/lidar-tools/offset_spline.py`
(path generation) and `scripts/lidar-tools/pcd_to_stl.py` (mesh
reconstruction). Measurements below are from direct testing on a real
run, not derived from the source alone — where a "Measured" column is
present, it reflects actual observed behaviour, not just documented
intent.

In production, these are set via `mesh.live` / `mesh.final` / `path` in
`scripts/pipeline_params.yaml`, not passed on the command line directly
— see that file's own comments for the flag-name-to-key mapping
(`poisson_depth` → `--poisson-depth`, etc.).

---

## Path generation (`offset_spline.py`)

### Shape of the path

| Parameter | Default | Effect | Measured |
| --- | --- | --- | --- |
| `--offset` | 0.05 | Standoff distance from the surface. Larger = the path sits further out, so it wraps a bigger effective radius and gets longer. Also *smooths* the geometry, because a 10 cm offset surface barely feels bark bumps. | 0.02→0.05→0.10 m: length 1.370→1.491→1.698 m; max offset error 1.04→1.02→0.28 mm |
| `--order` | `file` | Visiting order. `file` = click order, `greedy` = nearest-neighbour + 2-opt. The single biggest lever on path quality. | length 1.491→1.341 m, worst turn 174.8°→137.6° |
| `--mode` | `geodesic` | `geodesic` runs curve-shortening to convergence (true shortest surface path). `elastic` is the older annealed smoothing — stops on a schedule, not at a length minimum. | \|κ_g\| 0.153 vs 0.307 1/m; max offset error 0.24 vs 0.45 mm |
| `--closed` | off | Returns to the first target, making a loop. | 137→236 waypoints, 1.341→2.336 m |

### Where the path is anchored

| Parameter | Default | Effect | Measured |
| --- | --- | --- | --- |
| `--project-knots` | off | Slides each standoff outward along its *own clicked normal* until truly `--offset` from the surface. Fixes the few-mm error where a neighbouring bump sits closer than the contact point's tangent plane. Approach direction unchanged. **Recommend turning on.** | max offset error 3.87→0.24 mm |
| `--no-pin` | off | Stops forcing the path through the targets. Slightly better offset conformance, but the drone no longer provably reaches each contact pose. | max err 0.34 mm but \|κ_g\| worsens 0.153→0.603 |
| `--through-contact` | off | Uses raw contact points (on the surface) as knots instead of standoff points. Only sensible with `--offset 0`. | with offset 0.05: 50 mm error — the knots are 50 mm from where they should be, as expected |

### Resolution and cost

| Parameter | Default | Effect | Measured |
| --- | --- | --- | --- |
| `--spacing` | 0.01 | Arc-length gap between output waypoints. Linear in waypoint count and runtime. | 0.02→0.01→0.005 m: 77→151→301 waypoints |
| `--iterations` | 60 | **Effectively ignored in geodesic mode** — it's passed as `max(iterations, 400)`, so anything ≤400 does nothing. `--iterations 10` still converged after 127. Only matters in `--mode elastic`, where it's the fixed schedule length. | 10 / 50 / 400 → identical output |
| `--smooth-start`, `--smooth-end` | 0.6, 0.02 | **`elastic` mode only.** Annealing endpoints for the smoothing weight. No effect in the default geodesic mode. | — |

> **Caveat on `--iterations`:** the floor of 400 may be deliberate (geodesic
> convergence genuinely needs more than the elastic default of 60), but as
> written the flag silently does nothing in the default mode, which is
> misleading. Fix if desired: either drop the floor, or rename it
> `--max-iterations` and document the geodesic minimum.

### Safety / correctness

| Parameter | Default | Effect |
| --- | --- | --- |
| `--side` | `outward` | `outward` always offsets along the mesh's outward normal, guaranteeing the drone stays outside the trunk. `auto` uses whichever side each sample is on — only for meshes whose winding you can't trust, and it lets the path cut through the trunk. Made no difference on a correctly-wound mesh; on a winding-inverted test mesh it was the difference between 0.02 mm and ~50 mm error. |
| `--no-orient-check` | off (check on) | Disables validating mesh normals against your YAML normals. Leave the check on — on an inverted-winding STL it caught the flip and kept error at 0.04 mm; without it, the whole path lands *inside* the trunk. |

**Suggested starting point:** `--offset 0.05 --spacing 0.01 --project-knots --order greedy`

---

## Mesh generation (`pcd_to_stl.py`)

### Reconstruction methods

| Method | Needs normals? | Speed | Output character | Best for | Weaknesses |
| --- | --- | --- | --- | --- | --- |
| `poisson` (default) | Yes | Slowest | Watertight, smooth, "solid" | Whole scenes/objects you want closed | Invents surface in gaps; blobby if under-tuned; memory-heavy |
| `bpa` | Yes | Medium | Hugs raw points, holes in sparse spots | Faithful surface from dense, even sampling | Leaves holes; sensitive to radius choice |
| `alpha` | **No** | Fast | Hugs points, can be non-manifold | Normal-free fallback for arbitrary 3D shape | Tetrahedralises every point → downsample first |
| `delaunay` | **No** | Fastest (O(n log n)) | 2.5D sheet from one projection | Single-viewpoint scans, terrain, walls | One projection only; can't wrap fully around an object |

### Key parameters by stage

| Parameter | Applies to | Default | What it does / when to change |
| --- | --- | --- | --- |
| `--voxel M` | all | `0` (off) | Downsample to M-metre grid. Biggest speed lever. Try `0.01`–`0.03`. Required in practice for `alpha`. |
| `--remove-outliers` | all | off | Statistical outlier removal. Add `--outlier-neighbors` (20), `--outlier-std` (2.0). |
| `--orient` | `poisson`, `bpa` | `consistent` | `consistent` = accurate but slowest step; `sensor` = fast, correct for single-viewpoint lidar; `none` = skip. |
| `--sensor-origin X Y Z` | `--orient sensor`, `--project sensor` | `0 0 0` | Scanner position in the cloud's frame. |
| `--orient-k N` | `--orient consistent` | `15` | Neighbours for orientation graph; lower = faster. |
| `--normal-radius M` / `--normal-max-nn N` | `poisson`, `bpa` | auto / `30` | Normal search size; auto radius = 4× point spacing. |
| `--poisson-depth N` | `poisson` | `9` | Octree depth. Each −1 ≈ 2× faster, less memory. `7`–`8` for a pipeline. |
| `--density-quantile Q` | `poisson` | `0.02` | Trim low-confidence vertices. `0` = keep all (fully watertight). |
| `--poisson-linear-fit` / `--crop-to-input` | `poisson` | off | Slightly better boundaries / clip to input bbox. |
| `--bpa-radii M [M ...]` | `bpa` | auto | Explicit ball radii; auto = `[1, 2, 4] ×` spacing. |
| `--alpha M` | `alpha` | auto | Alpha radius; auto = 5× spacing. Smaller → more holes; larger → blobbier. |
| `--project MODE` | `delaunay` | `pca` | `sensor` (azimuth/elev from origin — full scan), `pca` (drop thinnest axis — sheet), `xy`/`xz`/`yz` (raw plane). |
| `--max-edge M` | `delaunay` | auto | Cull triangles stretched across gaps; auto = 5× spacing. Lower → open holes at depth jumps. |

### Mesh cleanup (all methods)

| Parameter | Default | What it does |
| --- | --- | --- |
| `--keep-largest` | off | Keep only the largest connected component (drops floating junk). |
| `--min-cluster-triangles N` | `0` | Drop components smaller than N triangles. |
| `--smooth-iterations N` | `0` | Taubin smoothing passes. |
| `--target-triangles N` | `0` | Decimate down to ~N triangles. |
| `--ascii` | off | Write ASCII STL instead of binary. |

### Suggested starting commands

```bash
# fastest, no normals, single scan
pcd_to_stl.py in.pcd out.stl --method delaunay --project sensor --sensor-origin 0 0 0

# no normals, arbitrary shape
pcd_to_stl.py in.pcd out.stl --method alpha --voxel 0.02

# solid watertight, tuned for speed
pcd_to_stl.py in.pcd out.stl --method poisson --orient sensor --poisson-depth 8 --voxel 0.01 --keep-largest
```
