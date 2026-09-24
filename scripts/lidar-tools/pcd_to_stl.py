#!/home/tin/miniconda3/envs/lidar/bin/python
"""Convert a PointCloud2 .pcd file into a triangulated .stl mesh.

Standalone CLI. Typical use:

    python pcd_to_stl.py scan.pcd scan.stl

The pipeline is: load -> (optional) downsample -> (optional) outlier
removal -> estimate + orient normals -> surface reconstruction -> clean
-> (optional) smooth -> write STL.

Reconstruction methods (--method):

  * poisson (default) - watertight, smooth. Needs normals. Slowest.
  * bpa (ball pivoting) - hugs the raw points, leaves holes in sparse
    areas. Needs normals.
  * alpha - alpha-shape. NO normals. One radius parameter. Downsample
    first (it tetrahedralises every point).
  * delaunay - 2.5D: project to a plane, triangulate in 2D, lift back.
    NO normals. Fastest by far (O(n log n)). Best for single-viewpoint
    scans / sheet-like surfaces; use --project sensor for a full scan.

If your pipeline already has normals, or you just want speed, use
--method delaunay (or alpha) and normal estimation is skipped entirely.
"""

from __future__ import annotations

import argparse
import sys

import numpy as np

try:
    import open3d as o3d
except ImportError:
    sys.exit(
        "open3d is not installed in this Python.\n"
        "  pip install open3d numpy\n"
        "(needs CPython 3.8-3.12; see setup.sh next to this script)"
    )


def log(msg: str) -> None:
    print(f"[pcd_to_stl] {msg}", file=sys.stderr)


def load_cloud(path: str) -> "o3d.geometry.PointCloud":
    pcd = o3d.io.read_point_cloud(path)
    if len(pcd.points) == 0:
        sys.exit(f"no points read from {path!r} - is it a valid PCD?")
    log(f"loaded {len(pcd.points):,} points from {path}")
    return pcd


def avg_neighbour_distance(pcd: "o3d.geometry.PointCloud") -> float:
    d = np.asarray(pcd.compute_nearest_neighbor_distance())
    d = d[np.isfinite(d) & (d > 0)]
    return float(np.mean(d)) if d.size else 0.0
    
def fit_trunk_axis(pts, axis_dir=None):
    """Return (point_on_axis, unit_axis, radius) for a roughly cylindrical cloud.

    Axis direction: given, or the cloud's longest principal axis.
    Axis position: least-squares circle fit in the plane perpendicular to
    the axis. Unlike the centroid, this stays on the true axis even when
    only part of the trunk has been scanned.
    """
    c = pts.mean(axis=0)
    q = pts - c
    if axis_dir is None:
        _, _, vh = np.linalg.svd(q, full_matrices=False)
        a = vh[0]
    else:
        a = np.asarray(axis_dir, dtype=float)
    a = a / np.linalg.norm(a)

    # Orthonormal basis (u, v) of the plane perpendicular to the axis.
    helper = np.array([1.0, 0.0, 0.0]) if abs(a[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    u = np.cross(a, helper); u /= np.linalg.norm(u)
    v = np.cross(a, u)
    x, y = q @ u, q @ v

    # Kasa circle fit: x^2 + y^2 + D x + E y + F = 0
    A = np.column_stack([x, y, np.ones_like(x)])
    (D, E, F), *_ = np.linalg.lstsq(A, -(x**2 + y**2), rcond=None)
    cx, cy = -D / 2.0, -E / 2.0
    r2 = cx**2 + cy**2 - F
    extent = float(np.ptp(x) + np.ptp(y))
    if not np.isfinite(r2) or r2 <= 0 or np.sqrt(r2) > 5.0 * max(extent, 1e-6):
        return c, a, float("nan")          # fit failed: fall back to centroid
    return c + cx * u + cy * v, a, float(np.sqrt(r2))


def orient_radial(points, normals, axis_dir=None):
    """Flip normals so each points away from the trunk axis."""
    centre, a, r = fit_trunk_axis(points, axis_dir)
    rel = points - centre
    radial = rel - np.outer(rel @ a, a)
    flip = np.einsum("ij,ij->i", normals, radial) < 0
    out = normals.copy()
    out[flip] *= -1
    return out, centre, a, r, int(flip.sum())


def preprocess(pcd, args):
    if args.voxel and args.voxel > 0:
        before = len(pcd.points)
        pcd = pcd.voxel_down_sample(voxel_size=args.voxel)
        log(f"voxel downsample @ {args.voxel} m: {before:,} -> {len(pcd.points):,}")

    if args.remove_outliers:
        before = len(pcd.points)
        pcd, _ = pcd.remove_statistical_outlier(
            nb_neighbors=args.outlier_neighbors, std_ratio=args.outlier_std
        )
        log(f"outlier removal: {before:,} -> {len(pcd.points):,}")

    return pcd


def estimate_normals(pcd, args, spacing):
    radius = args.normal_radius if args.normal_radius > 0 else max(spacing * 4.0, 1e-6)
    log(f"normal search radius {radius:.4f} m")

    pcd.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=radius, max_nn=args.normal_max_nn
        )
    )

    if args.orient == "sensor":
        # Cheap and correct for a single-viewpoint lidar: every surface
        # was seen from the scanner, so normals should face the origin.
        origin = np.asarray(args.sensor_origin, dtype=float)
        pcd.orient_normals_towards_camera_location(origin)
        log(f"oriented normals towards sensor origin {tuple(origin)}")
    elif args.orient == "radial":
        # Outward from the trunk axis. Independent of where the sensor
        # was, so scans from several sides of the tree agree.
        nrm, centre, axis, r, nflip = orient_radial(
            np.asarray(pcd.points), np.asarray(pcd.normals), args.axis_dir)
        pcd.normals = o3d.utility.Vector3dVector(nrm)
        log(f"radial orientation: axis point {np.round(centre, 3)}, "
            f"dir {np.round(axis, 3)}, radius {r:.3f} m, flipped {nflip:,}")
    elif args.orient == "none":
        log("skipping normal orientation")
    else:
        # Consistent orientation matters a lot for Poisson but is the
        # slowest step (builds a Riemannian graph + MST over all points).
        try:
            pcd.orient_normals_consistent_tangent_plane(k=args.orient_k)
            log(f"oriented normals via consistent tangent plane (k={args.orient_k})")
        except Exception as exc:  # noqa: BLE001 - fall back to a cheap heuristic
            log(f"consistent orientation failed ({exc}); falling back to sensor origin")
            pcd.orient_normals_towards_camera_location(
                np.asarray(args.sensor_origin, dtype=float)
            )
    return pcd


def reconstruct_poisson(pcd, args, spacing):
    log(f"Poisson reconstruction (depth={args.poisson_depth})")
    mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
        pcd, depth=args.poisson_depth, linear_fit=args.poisson_linear_fit
    )
    densities = np.asarray(densities)
    if args.density_quantile > 0:
        thresh = np.quantile(densities, args.density_quantile)
        keep = densities >= thresh
        mesh.remove_vertices_by_mask(~keep)
        log(
            f"trimmed low-density vertices below q{args.density_quantile:.2f} "
            f"({(~keep).sum():,} removed)"
        )
    if args.crop_to_input:
        mesh = mesh.crop(pcd.get_axis_aligned_bounding_box())
    return mesh


def reconstruct_bpa(pcd, args, spacing):
    if args.bpa_radii:
        radii = list(args.bpa_radii)
    else:
        base = spacing if spacing > 0 else 0.01
        radii = [base * m for m in (1.0, 2.0, 4.0)]
    log(f"ball-pivoting reconstruction, radii = {[round(r, 4) for r in radii]}")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
        pcd, o3d.utility.DoubleVector(radii)
    )
    return mesh


def reconstruct_alpha(pcd, args, spacing):
    alpha = args.alpha if args.alpha > 0 else max(spacing * 5.0, 1e-6)
    log(f"alpha-shape reconstruction, alpha = {alpha:.4f} m (no normals used)")
    mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_alpha_shape(pcd, alpha)
    return mesh


def reconstruct_delaunay(pcd, args, spacing):
    """2.5D Delaunay: project to a plane, triangulate in 2D, lift back.

    Needs no normals and is the cheapest option (O(n log n)). Works when
    the cloud is roughly a single sheet / single viewpoint. Triangles
    that come out stretched across gaps are culled by --max-edge.
    """
    from scipy.spatial import Delaunay

    pts = np.asarray(pcd.points)
    proj = args.project

    if proj == "sensor":
        rel = pts - np.asarray(args.sensor_origin, dtype=float)
        azimuth = np.arctan2(rel[:, 1], rel[:, 0])
        elevation = np.arctan2(rel[:, 2], np.hypot(rel[:, 0], rel[:, 1]))
        uv = np.column_stack([azimuth, elevation])
    elif proj in ("xy", "xz", "yz"):
        idx = {"xy": (0, 1), "xz": (0, 2), "yz": (1, 2)}[proj]
        uv = pts[:, idx]
    else:  # pca - drop the thinnest axis
        centered = pts - pts.mean(axis=0)
        _, _, vh = np.linalg.svd(centered, full_matrices=False)
        uv = centered @ vh[:2].T

    log(f"2.5D Delaunay, projection = {proj} (no normals used)")
    tri = Delaunay(uv)
    faces = tri.simplices

    max_edge = args.max_edge if args.max_edge > 0 else max(spacing * 5.0, 1e-6)
    v = pts[faces]
    e = np.stack([
        np.linalg.norm(v[:, 0] - v[:, 1], axis=1),
        np.linalg.norm(v[:, 1] - v[:, 2], axis=1),
        np.linalg.norm(v[:, 2] - v[:, 0], axis=1),
    ], axis=1)
    keep = e.max(axis=1) <= max_edge
    log(f"culled {(~keep).sum():,}/{len(faces):,} stretched triangles "
        f"(edge > {max_edge:.4f} m)")

    mesh = o3d.geometry.TriangleMesh(
        o3d.utility.Vector3dVector(pts),
        o3d.utility.Vector3iVector(faces[keep]),
    )
    return mesh


def clean_mesh(mesh, args):
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    mesh.remove_unreferenced_vertices()

    if args.keep_largest or args.min_cluster_triangles > 0:
        tri_ids, counts, _ = mesh.cluster_connected_triangles()
        tri_ids = np.asarray(tri_ids)
        counts = np.asarray(counts)
        if args.keep_largest:
            biggest = int(counts.argmax())
            remove = tri_ids != biggest
            log(f"keeping largest component ({counts[biggest]:,} triangles)")
        else:
            small = set(np.where(counts < args.min_cluster_triangles)[0].tolist())
            remove = np.array([t in small for t in tri_ids])
            log(f"dropping {len(small)} components under {args.min_cluster_triangles} tris")
        mesh.remove_triangles_by_mask(remove)
        mesh.remove_unreferenced_vertices()

    if args.smooth_iterations > 0:
        mesh = mesh.filter_smooth_taubin(number_of_iterations=args.smooth_iterations)
        log(f"Taubin smoothing x{args.smooth_iterations}")

    if args.target_triangles > 0 and len(mesh.triangles) > args.target_triangles:
        before = len(mesh.triangles)
        mesh = mesh.simplify_quadric_decimation(int(args.target_triangles))
        log(f"decimated {before:,} -> {len(mesh.triangles):,} triangles")

    mesh.compute_vertex_normals()
    mesh.compute_triangle_normals()  # STL stores per-facet normals
    return mesh


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Convert a .pcd point cloud into a .stl mesh.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument("input", help="input point cloud (.pcd, .ply, .xyz, ...)")
    p.add_argument("output", help="output mesh (.stl)")

    p.add_argument("--method",
                   choices=("poisson", "bpa", "alpha", "delaunay"),
                   default="poisson",
                   help="reconstruction method. poisson/bpa need normals; "
                        "alpha/delaunay do not. delaunay is fastest.")

    # preprocessing
    p.add_argument("--voxel", type=float, default=0.0,
                   help="voxel downsample size in metres (0 = off)")
    p.add_argument("--remove-outliers", action="store_true",
                   help="statistical outlier removal before meshing")
    p.add_argument("--outlier-neighbors", type=int, default=20)
    p.add_argument("--outlier-std", type=float, default=2.0)

    # normals
    p.add_argument("--normal-radius", type=float, default=0.0,
                   help="normal search radius in m (0 = auto from point spacing)")
    p.add_argument("--normal-max-nn", type=int, default=30)
    p.add_argument("--orient", choices=("consistent", "sensor", "radial", "none"),
                   default="consistent",
                   help="normal orientation: 'consistent' (accurate, slow), "
                        "'sensor' (fast, single viewpoint), "
                        "'radial' (outward from a fitted trunk axis; multi-view), 'none'")
    p.add_argument("--axis-dir", type=float, nargs=3, default=None,
                   metavar=("X", "Y", "Z"),
                   help="trunk axis direction for --orient radial "
                        "(default: longest principal axis of the cloud)")
    p.add_argument("--sensor-origin", type=float, nargs=3, default=(0.0, 0.0, 0.0),
                   metavar=("X", "Y", "Z"),
                   help="scanner position in the cloud's frame, for --orient sensor")
    p.add_argument("--orient-k", type=int, default=15,
                   help="neighbours for --orient consistent (lower = faster)")

    # poisson
    p.add_argument("--poisson-depth", type=int, default=9,
                   help="octree depth; higher = more detail + more memory")
    p.add_argument("--poisson-linear-fit", action="store_true")
    p.add_argument("--density-quantile", type=float, default=0.02,
                   help="trim vertices below this density quantile (0 = keep all)")
    p.add_argument("--crop-to-input", action="store_true",
                   help="clip the mesh to the input cloud's bounding box")

    # ball pivoting
    p.add_argument("--bpa-radii", type=float, nargs="+", default=None,
                   help="explicit ball radii in m (default: auto from spacing)")

    # alpha shape (no normals)
    p.add_argument("--alpha", type=float, default=0.0,
                   help="alpha radius in m for --method alpha (0 = auto). "
                        "Downsample first: it tetrahedralises every point.")

    # 2.5D delaunay (no normals)
    p.add_argument("--project", choices=("pca", "xy", "xz", "yz", "sensor"),
                   default="pca",
                   help="projection for --method delaunay. 'sensor' = "
                        "azimuth/elevation from --sensor-origin (single scan)")
    p.add_argument("--max-edge", type=float, default=0.0,
                   help="cull delaunay triangles with an edge longer than this "
                        "in m (0 = auto from point spacing)")

    # mesh cleanup
    p.add_argument("--keep-largest", action="store_true",
                   help="keep only the largest connected component")
    p.add_argument("--min-cluster-triangles", type=int, default=0,
                   help="drop connected components smaller than this")
    p.add_argument("--smooth-iterations", type=int, default=0,
                   help="Taubin smoothing passes (0 = off)")
    p.add_argument("--target-triangles", type=int, default=0,
                   help="decimate down to roughly this many triangles (0 = off)")

    p.add_argument("--ascii", action="store_true",
                   help="write ASCII STL instead of binary")
    return p


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)

    if not args.output.lower().endswith(".stl"):
        log(f"warning: output {args.output!r} does not end in .stl")

    pcd = load_cloud(args.input)
    pcd = preprocess(pcd, args)
    if len(pcd.points) < 4:
        sys.exit("too few points left after preprocessing to build a mesh")

    spacing = avg_neighbour_distance(pcd)
    log(f"mean point spacing ~ {spacing:.4f} m")

    needs_normals = args.method in ("poisson", "bpa")
    if needs_normals:
        pcd = estimate_normals(pcd, args, spacing)

    recon = {
        "poisson": reconstruct_poisson,
        "bpa": reconstruct_bpa,
        "alpha": reconstruct_alpha,
        "delaunay": reconstruct_delaunay,
    }[args.method]
    mesh = recon(pcd, args, spacing)

    if len(mesh.triangles) == 0:
        sys.exit("reconstruction produced no triangles - try another --method, "
                 "adjust --poisson-depth / --alpha / --max-edge, or --voxel "
                 "to regularise the cloud")

    mesh = clean_mesh(mesh, args)

    ok = o3d.io.write_triangle_mesh(
        args.output, mesh, write_ascii=args.ascii, print_progress=False
    )
    if not ok:
        sys.exit(f"failed to write {args.output}")

    log(f"wrote {args.output}: {len(mesh.vertices):,} vertices, "
        f"{len(mesh.triangles):,} triangles")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
