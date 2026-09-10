#!/usr/bin/env python3
"""
offset_spline.py

Generate a smooth, continuous spline that links a set of selected normal
targets while holding a fixed standoff distance from a reconstructed mesh
surface.

    selected_normals_*.yaml  (contact points + orientations)
                +
    map_*.stl                (reconstructed surface)
                |
                v
    waypoints.yaml / .csv    (dense, smooth, constant-offset path)

The problem this solves
-----------------------
A plain spline through the target points cuts *through* the trunk wherever
the surface is convex, and drifts away from it wherever the surface is
concave. A pure per-sample projection onto an offset surface gives the right
distance but is not smooth -- it inherits every facet-level wrinkle in the
mesh. This script alternates the two:

    for each iteration:
        1. PROJECT  push every sample to exactly `offset` from the surface
        2. SMOOTH   re-fit a spline through the corrected samples

Repeating this converges to a curve that is both C2-smooth and close to
constant offset. It is the same idea as an elastic band / active contour:
the projection step supplies the constraint, the spline supplies the
regularizer, and the smoothing weight is annealed downward so the early
iterations move a lot and the later ones only polish.

Note on where the spline passes
-------------------------------
The contact points in the YAML lie *on* the surface (distance 0), so a path
through them cannot also hold a nonzero standoff. By default the spline is
built through each target's STANDOFF point (contact + outward_normal *
offset), which is exactly `offset` away by construction. Use
--through-contact if you instead want the raw contact points used as knots
(only sensible with --offset 0).

Dependencies: numpy, scipy, PyYAML. STL parsing is built in (binary +
ASCII), so no trimesh/open3d/VTK needed.

Author: generated for the APhI cable-driven gripper drone pipeline
"""

import argparse
import os
import struct
import sys

import numpy as np
import yaml
from scipy.interpolate import CubicSpline
from scipy.spatial import cKDTree


# =====================================================================
# STL loading (binary + ASCII, no external mesh library)
# =====================================================================

def load_stl(path):
    """Load an STL file. Returns (V, F) with V (n,3) float64 vertices and
    F (m,3) int triangle indices. Duplicate vertices are welded so that
    per-vertex normals can be averaged across adjacent facets."""
    if not os.path.isfile(path):
        raise FileNotFoundError(f"STL not found: {path}")

    with open(path, "rb") as f:
        header = f.read(84)
        if len(header) < 84:
            raise ValueError(f"STL too short to be valid: {path}")
        n_tri = struct.unpack("<I", header[80:84])[0]
        rest = f.read()

    # A binary STL body is exactly 50 bytes per triangle. If that matches,
    # trust it; otherwise fall back to the ASCII parser. (Checking the
    # "solid" prefix alone is unreliable -- some binary writers emit it.)
    if len(rest) == n_tri * 50 and n_tri > 0:
        tris = _parse_binary_stl(rest, n_tri)
    else:
        tris = _parse_ascii_stl(path)

    if len(tris) == 0:
        raise ValueError(f"No triangles parsed from {path}")

    return _weld(np.asarray(tris, dtype=np.float64))


def _parse_binary_stl(body, n_tri):
    # Each record: 12 floats (normal + 3 verts) then a 2-byte attribute count.
    rec = np.frombuffer(body, dtype=np.uint8).reshape(n_tri, 50)
    floats = rec[:, :48].copy().view(np.float32).reshape(n_tri, 12)
    return floats[:, 3:12].reshape(n_tri, 3, 3).astype(np.float64)


def _parse_ascii_stl(path):
    verts = []
    with open(path, "r", errors="ignore") as f:
        for line in f:
            s = line.strip()
            if s.startswith("vertex"):
                parts = s.split()
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
    v = np.asarray(verts, dtype=np.float64)
    if len(v) % 3 != 0:
        raise ValueError("ASCII STL vertex count is not a multiple of 3")
    return v.reshape(-1, 3, 3)


def _weld(tris, decimals=9):
    """Merge coincident vertices so adjacent facets share indices."""
    flat = tris.reshape(-1, 3)
    keys = np.round(flat, decimals)
    _, first_idx, inverse = np.unique(
        keys, axis=0, return_index=True, return_inverse=True
    )
    V = flat[first_idx]
    F = inverse.reshape(-1, 3)
    # Drop degenerate triangles (two or more identical corners)
    ok = (F[:, 0] != F[:, 1]) & (F[:, 1] != F[:, 2]) & (F[:, 0] != F[:, 2])
    return V, F[ok]


# =====================================================================
# Exact point-to-mesh distance
# =====================================================================

class MeshDistance:
    """Exact closest-point-on-mesh queries.

    A cKDTree over triangle centroids selects the k nearest candidate
    triangles per query point, then an exact vectorized point-to-triangle
    test picks the true closest among them. k is grown automatically if the
    result looks unsafe (a candidate set whose centroid radius does not
    provably bound the answer), so this stays exact on elongated facets.
    """

    def __init__(self, V, F):
        self.V = V
        self.F = F
        self.A = V[F[:, 0]]
        self.B = V[F[:, 1]]
        self.C = V[F[:, 2]]

        fn = np.cross(self.B - self.A, self.C - self.A)
        area2 = np.linalg.norm(fn, axis=1, keepdims=True)
        self.face_normals = fn / np.maximum(area2, 1e-20)
        self.face_area2 = area2.ravel()

        self.centroids = (self.A + self.B + self.C) / 3.0
        self.tree = cKDTree(self.centroids)

        # Largest centroid->corner distance, used as the safety margin that
        # makes the k-nearest-centroid prefilter provably conservative.
        r = np.maximum.reduce([
            np.linalg.norm(self.A - self.centroids, axis=1),
            np.linalg.norm(self.B - self.centroids, axis=1),
            np.linalg.norm(self.C - self.centroids, axis=1),
        ])
        self.max_radius = float(r.max())

        # Area-weighted vertex normals, used to give the offset direction a
        # smooth field to follow (face normals alone are piecewise constant
        # and make the path visibly facet-stepped).
        acc = np.zeros_like(V)
        w = fn  # unnormalized => magnitude is 2*area, i.e. area weighting
        for c in range(3):
            np.add.at(acc, F[:, c], w)
        nrm = np.linalg.norm(acc, axis=1, keepdims=True)
        self.vertex_normals = np.where(nrm > 1e-20, acc / np.maximum(nrm, 1e-20),
                                       np.array([0.0, 0.0, 1.0]))
        self.vtree = cKDTree(V)

    def query(self, P, k=12, max_k=192):
        """Return (closest_point, distance, face_index) for each row of P."""
        P = np.atleast_2d(np.asarray(P, dtype=np.float64))
        n = len(P)
        best_d = np.full(n, np.inf)
        best_p = np.zeros((n, 3))
        best_f = np.zeros(n, dtype=np.int64)
        todo = np.arange(n)

        while len(todo) and k <= max_k:
            k_eff = min(k, len(self.F))
            _, idx = self.tree.query(P[todo], k=k_eff)
            idx = np.atleast_2d(idx)

            cp, d = self._point_tri(P[todo], idx)
            arg = np.argmin(d, axis=1)
            rows = np.arange(len(todo))
            best_d[todo] = d[rows, arg]
            best_p[todo] = cp[rows, arg]
            best_f[todo] = idx[rows, arg]

            if k_eff >= len(self.F):
                break

            # A candidate set is only trustworthy if the true answer cannot
            # lie outside the ball we searched. Re-query the rest with more
            # candidates rather than silently returning a near-miss.
            centroid_d, _ = self.tree.query(P[todo], k=k_eff)
            shell = np.atleast_2d(centroid_d)[:, -1]
            unsafe = best_d[todo] > shell - self.max_radius
            todo = todo[unsafe]
            k *= 2

        return best_p, best_d, best_f

    def _point_tri(self, P, idx):
        """Vectorized exact squared distance from points to candidate
        triangles, via barycentric region classification with clamping."""
        p = P[:, None, :]                    # (n,1,3)
        a = self.A[idx]                      # (n,k,3)
        b = self.B[idx]
        c = self.C[idx]

        ab = b - a
        ac = c - a
        ap = p - a

        d1 = np.einsum("nkj,nkj->nk", ab, ap)
        d2 = np.einsum("nkj,nkj->nk", ac, ap)
        bp = p - b
        d3 = np.einsum("nkj,nkj->nk", ab, bp)
        d4 = np.einsum("nkj,nkj->nk", ac, bp)
        cp_ = p - c
        d5 = np.einsum("nkj,nkj->nk", ab, cp_)
        d6 = np.einsum("nkj,nkj->nk", ac, cp_)

        vc = d1 * d4 - d3 * d2
        vb = d5 * d2 - d1 * d6
        va = d3 * d6 - d5 * d4
        denom = va + vb + vc

        # Default: projection lies inside the face
        with np.errstate(divide="ignore", invalid="ignore"):
            inv = 1.0 / np.where(np.abs(denom) < 1e-30, 1e-30, denom)
            v_in = vb * inv
            w_in = vc * inv
        closest = a + ab * v_in[..., None] + ac * w_in[..., None]

        # Vertex regions
        m = (d1 <= 0) & (d2 <= 0)
        closest = np.where(m[..., None], a, closest)
        m = (d3 >= 0) & (d4 <= d3)
        closest = np.where(m[..., None], b, closest)
        m = (d6 >= 0) & (d5 <= d6)
        closest = np.where(m[..., None], c, closest)

        # Edge regions
        m = (vc <= 0) & (d1 >= 0) & (d3 <= 0)
        t = np.where(m, d1 / np.where((d1 - d3) == 0, 1e-30, d1 - d3), 0.0)
        closest = np.where(m[..., None], a + ab * t[..., None], closest)

        m = (vb <= 0) & (d2 >= 0) & (d6 <= 0)
        t = np.where(m, d2 / np.where((d2 - d6) == 0, 1e-30, d2 - d6), 0.0)
        closest = np.where(m[..., None], a + ac * t[..., None], closest)

        m = (va <= 0) & ((d4 - d3) >= 0) & ((d5 - d6) >= 0)
        den = (d4 - d3) + (d5 - d6)
        t = np.where(m, (d4 - d3) / np.where(den == 0, 1e-30, den), 0.0)
        closest = np.where(m[..., None], b + (c - b) * t[..., None], closest)

        return closest, np.linalg.norm(p - closest, axis=2)

    def orient_to_reference(self, ref_points, ref_normals):
        """Check the mesh's normal orientation against known-good reference
        normals (the ones estimated at the selected contact points) and flip
        the whole mesh's normals if they disagree.

        STL stores orientation only implicitly, via triangle winding, and
        reconstruction tools do not always get it right. Getting this
        backwards would put the entire path *inside* the trunk, so it is
        worth checking rather than assuming. Returns True if flipped."""
        n = self.smooth_normal(np.atleast_2d(ref_points))
        agree = np.sum(n * np.atleast_2d(ref_normals), axis=1)
        if np.mean(agree) < 0.0:
            self.vertex_normals = -self.vertex_normals
            self.face_normals = -self.face_normals
            return True
        return False

    def smooth_normal(self, P, k=8):
        """Interpolated outward direction at P, from inverse-distance
        weighted vertex normals. Smoother than the hit face's normal."""
        P = np.atleast_2d(P)
        k_eff = min(k, len(self.V))
        d, idx = self.vtree.query(P, k=k_eff)
        d = np.atleast_2d(d)
        idx = np.atleast_2d(idx)
        w = 1.0 / np.maximum(d, 1e-9)
        n = np.einsum("nk,nkj->nj", w, self.vertex_normals[idx])
        nrm = np.linalg.norm(n, axis=1, keepdims=True)
        return n / np.maximum(nrm, 1e-20)


# =====================================================================
# YAML target loading
# =====================================================================

def quat_to_axis(q, axis=(1.0, 0.0, 0.0)):
    """Rotate `axis` by quaternion q=(x,y,z,w). The pipeline's convention is
    that local +X points along the outward surface normal."""
    x, y, z, w = q
    n = np.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-12:
        return np.array(axis, dtype=np.float64)
    x, y, z, w = x / n, y / n, z / n, w / n
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return R @ np.asarray(axis, dtype=np.float64)


def axis_to_quat(v, axis=(1.0, 0.0, 0.0)):
    """Shortest-arc quaternion (x,y,z,w) taking `axis` to direction v."""
    a = np.asarray(axis, dtype=np.float64)
    a = a / max(np.linalg.norm(a), 1e-20)
    b = np.asarray(v, dtype=np.float64)
    nb = np.linalg.norm(b)
    if nb < 1e-20:
        return np.array([0.0, 0.0, 0.0, 1.0])
    b = b / nb

    d = float(np.dot(a, b))
    if d < -1.0 + 1e-9:  # antiparallel: any perpendicular axis works
        perp = np.array([1.0, 0.0, 0.0])
        if abs(a[0]) > 0.9:
            perp = np.array([0.0, 1.0, 0.0])
        ax = np.cross(a, perp)
        ax /= max(np.linalg.norm(ax), 1e-20)
        return np.array([ax[0], ax[1], ax[2], 0.0])

    ax = np.cross(a, b)
    q = np.array([ax[0], ax[1], ax[2], 1.0 + d])
    return q / max(np.linalg.norm(q), 1e-20)


def load_targets(path):
    """Read a normals YAML. Accepts both the multi-target schema
    (`targets:` list) and the legacy single-target schema
    (top-level `position:`/`orientation:`)."""
    with open(path, "r") as f:
        doc = yaml.safe_load(f)
    if doc is None:
        raise ValueError(f"Empty YAML: {path}")

    frame = doc.get("frame_id", "odom")
    entries = doc.get("targets")
    if entries is None:
        if "position" not in doc:
            raise ValueError(f"No 'targets' list and no 'position' block in {path}")
        entries = [doc]

    pts, quats = [], []
    for e in entries:
        p = e["position"]
        o = e.get("orientation", {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0})
        pts.append([float(p["x"]), float(p["y"]), float(p["z"])])
        quats.append([float(o["x"]), float(o["y"]), float(o["z"]), float(o["w"])])

    return np.asarray(pts), np.asarray(quats), frame


# =====================================================================
# Path construction
# =====================================================================

def order_targets(pts, mode="file", start=0):
    """Return a visiting order for the targets."""
    if mode == "file":
        return np.arange(len(pts))
    if mode != "greedy":
        raise ValueError(f"unknown order mode: {mode}")

    # Nearest-neighbour tour, then a 2-opt pass to remove crossings.
    n = len(pts)
    D = np.linalg.norm(pts[:, None, :] - pts[None, :, :], axis=2)
    unvisited = set(range(n))
    cur = start
    unvisited.discard(cur)
    tour = [cur]
    while unvisited:
        nxt = min(unvisited, key=lambda j: D[cur, j])
        unvisited.discard(nxt)
        tour.append(nxt)
        cur = nxt

    improved = True
    while improved:
        improved = False
        for i in range(1, n - 1):
            for j in range(i + 1, n):
                a, b = tour[i - 1], tour[i]
                c = tour[j]
                d = tour[j + 1] if j + 1 < n else None
                if d is None:
                    delta = D[a, c] - D[a, b]
                else:
                    delta = (D[a, c] + D[b, d]) - (D[a, b] + D[c, d])
                if delta < -1e-12:
                    tour[i:j + 1] = tour[i:j + 1][::-1]
                    improved = True
    return np.asarray(tour)


def solve_ray_offset(contacts, normals, md, offset, hi_scale=4.0, iters=40):
    """Slide each standoff point along its own clicked normal ray until it is
    truly `offset` from the surface.

    Naively projecting the standoff point onto the offset surface also moves
    it *sideways* (its closest mesh point is often a neighbouring bump, not
    the contact point it came from) -- measured 32-40 mm of lateral drift on
    bark-like geometry, which would dock the drone in the wrong place. Moving
    only along the ray keeps the clicked approach direction exactly and
    corrects the distance alone.

    d(t) = dist(contact + n*t, mesh) is nondecreasing in t over the range we
    care about, so plain bisection is sufficient and robust."""
    contacts = np.atleast_2d(contacts)
    normals = np.atleast_2d(normals)

    lo = np.full(len(contacts), float(offset))
    hi = np.full(len(contacts), float(offset) * hi_scale)

    for _ in range(iters):
        mid = 0.5 * (lo + hi)
        _, d, _ = md.query(contacts + normals * mid[:, None])
        too_close = d < offset
        lo = np.where(too_close, mid, lo)
        hi = np.where(too_close, hi, mid)

    t = 0.5 * (lo + hi)
    return contacts + normals * t[:, None]


def project_to_offset(P, md, offset, use_smooth_normal=True, side="outward",
                      relax=1.0, max_step=None):
    """Correct each sample's *distance* to the surface, moving it only along
    the local surface normal.

    The obvious implementation -- snap each sample to closest_point +
    n*offset -- is wrong here. That teleports a sample sideways onto its
    nearest surface feature (measured 32-40 mm of lateral drift on bark-like
    geometry), so samples leapfrog one another and the ordering along the
    curve breaks. The visible symptom is the path running past a target and
    then doubling back through a ~180 deg cusp to reach it.

    Displacing by (offset - d) along the normal fixes the distance without
    moving the sample along the surface, so the ordering is preserved. It is
    a gradient step on the distance constraint rather than a projection, so
    it needs the iteration it already sits inside to converge.
    """
    cp, d, fidx = md.query(P)

    n = md.smooth_normal(cp) if use_smooth_normal else md.face_normals[fidx]

    # Which side of the surface is the sample on? md.query returns an
    # unsigned distance, and using it directly is a divergence bug: a sample
    # that starts *inside* the trunk with d > offset gets pushed further
    # inward and escapes to infinity. The signed distance sends it outward.
    v = P - cp
    vn = np.linalg.norm(v, axis=1, keepdims=True)
    cur = np.where(vn > 1e-9, v / np.maximum(vn, 1e-20), n)
    inside = np.sum(cur * n, axis=1) < 0.0

    if side == "auto":
        n = np.where(inside[:, None], -n, n)
        signed = d
    else:
        signed = np.where(inside, -d, d)

    step = relax * (offset - signed)
    # Cap a single step so a wildly misplaced sample cannot overshoot and
    # oscillate; the surrounding iteration converges it over several passes.
    # The cap must not be derived from `offset` alone -- with --offset 0 that
    # collapses to zero and disables the correction entirely.
    cap = 2.0 * max(offset, max_step or 0.0, 1e-3)
    step = np.clip(step, -cap, cap)

    return P + step[:, None] * n, d


def surface_normals_at(P, md, side="outward", use_smooth_normal=True):
    """Outward unit normal of the offset surface at each sample."""
    cp, d, fidx = md.query(P)
    n = md.smooth_normal(cp) if use_smooth_normal else md.face_normals[fidx]
    v = P - cp
    vn = np.linalg.norm(v, axis=1, keepdims=True)
    cur = np.where(vn > 1e-9, v / np.maximum(vn, 1e-20), n)
    inside = np.sum(cur * n, axis=1) < 0.0
    if side == "auto":
        n = np.where(inside[:, None], -n, n)
    return n, np.where(inside, -d, d)


def tangential_smooth(P, md, beta, pinned=None, sweeps=1, side="outward"):
    """Laplacian smoothing with the displacement projected onto the surface's
    tangent plane.

    This is the difference between an elastic band and a geodesic. A plain
    3D Laplacian step pulls the curve off the offset surface (mostly inward,
    since it cuts corners), and the distance correction then has to undo
    that -- the two fight, and the result is a curve that is smooth but not
    shortest. Removing the normal component first means the step only ever
    slides the curve *along* the surface, so it is a genuine constrained
    gradient step on curve length: discrete curve-shortening flow, whose
    fixed points are geodesics."""
    Q = P.copy()
    n_pts = len(Q)
    if n_pts < 3 or beta <= 0.0:
        return Q

    fixed = np.zeros(n_pts, dtype=bool)
    if pinned is not None:
        fixed[pinned] = True
    fixed[0] = fixed[-1] = True

    for _ in range(sweeps):
        nrm, _ = surface_normals_at(Q, md, side=side)
        prev = np.roll(Q, 1, axis=0)
        nxt = np.roll(Q, -1, axis=0)
        delta = 0.5 * (prev + nxt) - Q
        delta[0] = delta[-1] = 0.0
        # keep only the component lying in the tangent plane
        delta = delta - np.sum(delta * nrm, axis=1)[:, None] * nrm
        Q = np.where(fixed[:, None], Q, Q + beta * delta)
    return Q


def geodesic_relax(P, knot_idx, md, offset, spacing, side="outward",
                   beta=0.5, max_iter=400, tol=1e-7, pin_targets=True,
                   log=None):
    """Relax the path to a piecewise geodesic on the offset surface.

    Alternates a tangential smoothing step (shortens the curve while sliding
    along the surface) with a normal correction (restores the exact offset),
    resampling within each knot interval so samples stay evenly spaced and
    the targets never move. Runs to convergence rather than on a fixed
    schedule -- a geodesic is a fixed point, so stopping early just leaves a
    longer curve."""
    prev_length = None
    for it in range(max_iter):

        fixed = np.zeros(len(P), dtype=bool)
        if pin_targets:
            fixed[knot_idx] = True

        P = tangential_smooth(P, md, beta, pinned=np.flatnonzero(fixed),
                              sweeps=2, side=side)

        Pp, _ = project_to_offset(P, md, offset, side=side, max_step=spacing)
        P = np.where(fixed[:, None], P, Pp)

        P, knot_idx = resample_preserving(P, knot_idx, spacing)

        # Converge on curve length, not on per-sample movement. Resampling
        # re-interpolates every iteration and leaves a ~1e-5 m jitter floor
        # that a displacement tolerance can never get under, so the loop
        # would always run to max_iter even after the length had plateaued.
        length = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
        if prev_length is not None and length > 0:
            if abs(length - prev_length) / length < tol:
                if log:
                    log(f"Geodesic converged after {it + 1} iterations "
                        f"(length {length:.4f} m)")
                return P, knot_idx, True
        prev_length = length

    if log:
        log(f"Geodesic did not fully converge in {max_iter} iterations "
            f"(path is still valid, just not provably shortest)")
    return P, knot_idx, False


def laplacian_smooth(P, beta, pinned=None, closed=False, sweeps=1):
    """Move each sample toward the midpoint of its neighbours.

    This is the regularizer in the project/smooth loop. Unlike a spline
    approximation weight, `beta` is a direct, bounded (0..1) statement of
    'how far toward my neighbours do I move', which makes the annealing
    schedule predictable. Pinned samples and (for open paths) the endpoints
    never move."""
    Q = P.copy()
    n = len(Q)
    if n < 3 or beta <= 0.0:
        return Q

    fixed = np.zeros(n, dtype=bool)
    if pinned is not None:
        fixed[pinned] = True
    if not closed:
        fixed[0] = fixed[-1] = True

    for _ in range(sweeps):
        prev = np.roll(Q, 1, axis=0)
        nxt = np.roll(Q, -1, axis=0)
        mid = 0.5 * (prev + nxt)
        if not closed:
            mid[0] = Q[0]
            mid[-1] = Q[-1]
        Q = np.where(fixed[:, None], Q, Q + beta * (mid - Q))
    return Q


def build_offset_spline(
    targets_xyz,
    targets_quat,
    md,
    offset=0.05,
    spacing=0.01,
    iterations=60,
    smooth_start=0.6,
    smooth_end=0.02,
    closed=False,
    through_contact=False,
    pin_targets=True,
    side="outward",
    project_knots=False,
    mode="geodesic",
    log=None,
):
    """Alternate projection (hold the offset) and smoothing (hold
    continuity) until the path is both, then fit an interpolating spline
    through the converged samples for a true C2 curve."""
    normals = np.array([quat_to_axis(q) for q in targets_quat])
    nrm = np.linalg.norm(normals, axis=1, keepdims=True)
    normals = normals / np.maximum(nrm, 1e-20)

    knots = targets_xyz if through_contact else targets_xyz + normals * offset

    # On a bumpy surface, contact + normal*offset can land noticeably nearer
    # than `offset`, because a neighbouring bump is closer than the contact
    # point's own tangent plane. Measured up to ~3.9 mm on a 50 mm offset
    # over bark-like geometry. Pinning bakes that error into the path.
    #
    # --project-knots slides each standoff further out along its own clicked
    # normal ray until the distance is exactly `offset`. The approach
    # direction is preserved exactly; only the standoff distance grows.
    if project_knots and not through_contact:
        knots = solve_ray_offset(targets_xyz, normals, md, offset)

    # Seed: straight segments between consecutive knots, subdivided to the
    # requested spacing, with each knot at a known sample index.
    #
    # A cubic interpolant through the knots is NOT safe here. With knots in
    # click order the spacing is very uneven, and cubic interpolation
    # overshoots enormously -- a seed swinging 1.1 m off a 0.15 m radius
    # trunk was measured, which the correction step then cannot reel back
    # before the path length runs away. A linear seed cannot overshoot; the
    # projection and smoothing supply the curvature.
    seed_knots = np.vstack([knots, knots[0]]) if closed else knots
    P, knot_idx = resample_preserving(
        seed_knots, np.arange(len(seed_knots)), spacing)

    # Annealed smoothing: strong early (pull the curve into a sane shape
    # while it is still far from the offset surface), weak late (so the
    # projection wins and the final offset error is small).
    if mode == "geodesic":
        # Shortest path on the offset surface between consecutive targets.
        P, knot_idx, _converged = geodesic_relax(
            P, knot_idx, md, offset, spacing, side=side,
            max_iter=max(iterations, 400), pin_targets=pin_targets, log=log)
        return _finish(P, knot_idx, knots, md, spacing, closed, through_contact)

    schedule = np.geomspace(max(smooth_start, 1e-6), max(smooth_end, 1e-9), iterations)

    def knot_mask(n, idx):
        m = np.zeros(n, dtype=bool)
        if pin_targets:
            m[idx] = True
        return m

    fixed = knot_mask(len(P), knot_idx)

    for beta in schedule:
        # 1. PROJECT the interior samples to exact offset. The knots are left
        #    alone entirely -- projecting them and snapping them back is what
        #    used to put a kink at every target on every iteration.
        Pp, _ = project_to_offset(P, md, offset, side=side, max_step=spacing)
        P = np.where(fixed[:, None], P, Pp)

        # 2. SMOOTH, holding the knots fixed
        P = laplacian_smooth(P, float(beta), pinned=np.flatnonzero(fixed),
                             closed=False, sweeps=2)

        # 3. Resample within each knot interval, so spacing stays uniform
        #    without ever displacing a knot
        P, knot_idx = resample_preserving(P, knot_idx, spacing)
        fixed = knot_mask(len(P), knot_idx)

    # Final projection, so the reported error is the true post-hoc one.
    Pp, _ = project_to_offset(P, md, offset, side=side, max_step=spacing)
    P = np.where(fixed[:, None], P, Pp)
    P = laplacian_smooth(P, float(smooth_end), pinned=np.flatnonzero(fixed),
                         closed=False, sweeps=1)
    P, knot_idx = resample_preserving(P, knot_idx, spacing)

    return _finish(P, knot_idx, knots, md, spacing, closed, through_contact)


def resample_preserving(P, knot_idx, spacing):
    """Resample to uniform arc-length spacing *within each knot interval*,
    leaving the knots themselves untouched.

    Resampling the whole path at once interpolates the knots away, and
    re-snapping the nearest sample back onto each knot afterwards puts a
    kink at every target. Subdividing per interval avoids both."""
    pieces, new_idx = [], [0]

    for a, b in zip(knot_idx[:-1], knot_idx[1:]):
        seg = P[a:b + 1]
        d = np.linalg.norm(np.diff(seg, axis=0), axis=1)
        s = np.concatenate([[0.0], np.cumsum(d)])
        total = s[-1]
        if total < 1e-12:
            pieces.append(seg[:-1])
            new_idx.append(new_idx[-1] + max(len(seg) - 1, 1))
            continue
        n = max(2, int(np.ceil(total / max(spacing, 1e-6))) + 1)
        su = np.linspace(0.0, total, n)
        out = np.column_stack([np.interp(su, s, seg[:, i]) for i in range(3)])
        pieces.append(out[:-1])
        new_idx.append(new_idx[-1] + len(out) - 1)

    pieces.append(P[knot_idx[-1]][None, :])
    return np.vstack(pieces), np.asarray(new_idx)

def make_arclength_spline(P, closed=False):
    """C2 cubic spline through P, parameterized by cumulative arc length."""
    Q = np.vstack([P, P[0]]) if closed else P
    seg = np.linalg.norm(np.diff(Q, axis=0), axis=1)

    # Drop segments that are negligible relative to the path as a whole.
    # An absolute epsilon is not enough: a segment of, say, 1e-6 m is far
    # above 1e-12 but still small enough to blow up the end conditions.
    total = float(seg.sum())
    min_seg = max(total * 1e-6, 1e-12)
    keep = np.concatenate([[True], seg > min_seg])
    Q = Q[keep]
    if len(Q) < 4:
        Q = P  # too aggressive for a very short path; use it as-is

    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))])
    bc = "periodic" if closed else "not-a-knot"
    if closed:
        Q = Q.copy()
        Q[-1] = Q[0]  # exact closure is required by the periodic BC
    return CubicSpline(s, Q, axis=0, bc_type=bc)


def eval_spline_uniform(spline, spacing, closed=False, include=None):
    """Sample a spline at approximately uniform arc-length spacing.

    If `include` is given, the arc-length parameter closest to each of those
    points is added to the sample set, guaranteeing a waypoint lands on it."""
    s0, s1 = float(spline.x[0]), float(spline.x[-1])
    n = max(3, int(np.ceil((s1 - s0) / max(spacing, 1e-6))) + 1)
    su = np.linspace(s0, s1, n)

    if include is not None and len(include):
        # Locate each requested point on the curve by dense search, then
        # refine locally so the inserted sample is on the curve, not merely
        # near it.
        dense_s = np.linspace(s0, s1, max(2000, n * 10))
        dense_p = spline(dense_s)
        tree = cKDTree(dense_p)
        _, idx = tree.query(np.atleast_2d(include), k=1)
        step = dense_s[1] - dense_s[0]
        extra = []
        for m, i in enumerate(np.atleast_1d(idx)):
            lo = max(s0, dense_s[i] - step)
            hi = min(s1, dense_s[i] + step)
            fine = np.linspace(lo, hi, 64)
            j = int(np.argmin(np.linalg.norm(spline(fine) - include[m], axis=1)))
            extra.append(fine[j])
        # Merge the knot parameters into the uniform ones, dropping any
        # uniform sample that lands almost on top of a knot. Without this the
        # two nearly coincide and produce a sub-millimetre segment whose
        # direction is numerical noise -- which reads downstream as a
        # spurious ~180 deg turn in an otherwise smooth path.
        extra = np.asarray(extra)
        tol = 0.4 * spacing
        keep = np.ones(len(su), dtype=bool)
        for e in extra:
            keep &= np.abs(su - e) > tol
        keep[0] = keep[-1] = True          # always retain the true endpoints
        su = np.unique(np.concatenate([su[keep], extra, su[[0, -1]]]))

    P = spline(su)
    return P[:-1] if closed else P


# =====================================================================
# Output
# =====================================================================

def write_yaml(path, P, outward, frame, offset, stats):
    poses = []
    for p, n in zip(P, outward):
        q = axis_to_quat(n)
        poses.append({
            "position": {"x": float(p[0]), "y": float(p[1]), "z": float(p[2])},
            "orientation": {"x": float(q[0]), "y": float(q[1]),
                            "z": float(q[2]), "w": float(q[3])},
        })

    doc = {
        "frame_id": frame,
        "offset": float(offset),
        "count": len(poses),
        "path_length": float(stats["length"]),
        "offset_error": {
            "mean": float(stats["mean_err"]),
            "max": float(stats["max_err"]),
            "rms": float(stats["rms_err"]),
        },
        "continuity": {
            "turn_deg_mean": float(stats["cont"]["turn_mean"]),
            "turn_deg_max": float(stats["cont"]["turn_max"]),
            "reversals": int(stats["cont"]["reversals"]),
            "peak_curvature_inv_m": float(stats["cont"]["kmax"]),
            "geodesic_curvature_median": float(stats["cont"]["kg_median"]),
        },
        "waypoints": poses,
    }
    with open(path, "w") as f:
        yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=False)


def write_csv(path, P, outward):
    with open(path, "w") as f:
        f.write("x,y,z,qx,qy,qz,qw\n")
        for p, n in zip(P, outward):
            q = axis_to_quat(n)
            f.write(f"{p[0]:.9f},{p[1]:.9f},{p[2]:.9f},"
                    f"{q[0]:.9f},{q[1]:.9f},{q[2]:.9f},{q[3]:.9f}\n")


# =====================================================================
# CLI
# =====================================================================


def _finish(P, knot_idx, knots, md, spacing, closed, through_contact):
    """Shared tail for both modes: fit the continuous C2 representation,
    resample it, and derive the per-waypoint outward direction.

    Both modes go through this so the output is byte-for-byte the same shape
    regardless of how the polyline was produced.

    NOTE: do *not* use splprep(s=0) here. With a dense sample set it puts a
    knot at every point and rings badly between them -- on a 5 cm offset it
    inflated the worst-case offset error from ~4 mm to ~41 mm in testing. A
    cubic spline parameterized by cumulative arc length is tridiagonal,
    cannot ring like that, and passes exactly through every input sample.
    """
    spline = make_arclength_spline(P, closed=closed)

    # The targets are already exact nodes of P, so they are exact nodes of
    # the spline; sampling it at those parameters reproduces them exactly.
    P = eval_spline_uniform(spline, spacing, closed=closed,
                            include=None if through_contact else knots)

    cp, d, _ = md.query(P)
    n = md.smooth_normal(cp)
    outward = P - cp
    on = np.linalg.norm(outward, axis=1, keepdims=True)
    outward = np.where(on > 1e-9, outward / np.maximum(on, 1e-20), n)

    return P, d, outward, spline


def path_continuity(P):
    """Measure how flyable the path is: turn angle between consecutive
    segments, and peak discrete curvature."""
    seg = np.diff(P, axis=0)
    L = np.linalg.norm(seg, axis=1)
    ok = L > 1e-12
    T = seg[ok] / L[ok][:, None]
    if len(T) < 2:
        return {"turn_mean": 0.0, "turn_max": 0.0, "reversals": 0, "kmax": 0.0}
    cos = np.clip(np.sum(T[:-1] * T[1:], axis=1), -1.0, 1.0)
    ang = np.degrees(np.arccos(cos))
    d1 = np.gradient(P, axis=0)
    d2 = np.gradient(d1, axis=0)
    k = np.linalg.norm(np.cross(d1, d2), axis=1) / np.maximum(
        np.linalg.norm(d1, axis=1) ** 3, 1e-20)
    return {
        "turn_mean": float(ang.mean()),
        "turn_max": float(ang.max()),
        "reversals": int((ang > 90.0).sum()),
        "kmax": float(k.max()),
    }


def geodesic_curvature(P, md):
    """|kappa_g|: the in-surface (tangential) part of the curvature vector.

    This is the quantity that vanishes on a true geodesic -- a curve on a
    surface can be strongly curved in 3D while still being geodesic, as long
    as all of that curvature points along the surface normal (think of a
    great circle on a sphere). So it, not the 3D curvature, is the right
    check on whether the path is actually taking the straightest route
    across the surface."""
    cp, _, _ = md.query(P)
    N = md.smooth_normal(cp)
    d1 = np.gradient(P, axis=0)
    d2 = np.gradient(d1, axis=0)
    sp = np.linalg.norm(d1, axis=1)
    kvec = np.cross(d1, np.cross(d2, d1)) / np.maximum(sp ** 4, 1e-20)[:, None]
    kg = kvec - np.sum(kvec * N, axis=1)[:, None] * N
    return np.linalg.norm(kg, axis=1)


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Generate a smooth constant-offset spline through "
                    "selected normal targets over an STL surface.")
    ap.add_argument("normals_yaml", help="selected_normals_*.yaml or locked_target.yaml")
    ap.add_argument("mesh_stl", help="reconstructed surface mesh (.stl)")
    ap.add_argument("-o", "--output", default="path.yaml",
                    help="output YAML path (default: path.yaml)")
    ap.add_argument("--csv", default=None, help="also write waypoints to this CSV")

    ap.add_argument("--offset", type=float, default=0.05,
                    help="standoff distance from the surface, metres (default: 0.05)")
    ap.add_argument("--spacing", type=float, default=0.01,
                    help="arc-length spacing between waypoints, metres (default: 0.01)")
    ap.add_argument("--iterations", type=int, default=60,
                    help="project/smooth iterations (default: 60)")
    ap.add_argument("--smooth-start", type=float, default=0.6,
                    help="initial Laplacian smoothing weight, 0..1 (default: 0.6)")
    ap.add_argument("--smooth-end", type=float, default=0.02,
                    help="final Laplacian smoothing weight, 0..1 (default: 0.02)")

    ap.add_argument("--mode", choices=["geodesic", "elastic"], default="geodesic",
                    help="geodesic: shortest path along the offset surface "
                         "between consecutive targets (default). elastic: the "
                         "older smoothed/regularized path, which is not "
                         "length-minimizing")
    ap.add_argument("--order", choices=["file", "greedy"], default="file",
                    help="target visiting order (default: file = click order)")
    ap.add_argument("--closed", action="store_true",
                    help="close the loop back to the first target")
    ap.add_argument("--through-contact", action="store_true",
                    help="use raw contact points as knots instead of standoff points")
    ap.add_argument("--no-pin", action="store_true",
                    help="do not re-anchor the path to the targets each iteration")
    ap.add_argument("--project-knots", action="store_true",
                    help="slide each target's standoff outward along its own "
                         "clicked normal until it is exactly --offset from the "
                         "surface (keeps the approach direction, only changes "
                         "the standoff distance)")
    ap.add_argument("--side", choices=["outward", "auto"], default="outward",
                    help="which side of the surface to offset to "
                         "(default: outward = always stay outside the trunk)")
    ap.add_argument("--no-orient-check", action="store_true",
                    help="skip validating mesh normal orientation against the YAML normals")
    ap.add_argument("-q", "--quiet", action="store_true")

    args = ap.parse_args(argv)

    def log(*a):
        if not args.quiet:
            print(*a, file=sys.stderr)

    pts, quats, frame = load_targets(args.normals_yaml)
    log(f"Loaded {len(pts)} target(s) from {args.normals_yaml} (frame: {frame})")

    if len(pts) < 2:
        print(f"error: a path needs at least 2 targets, but {args.normals_yaml} "
              f"has {len(pts)}. A single-target file (e.g. a one-normal "
              f"locked_target.yaml) defines a docking pose, not a path.",
              file=sys.stderr)
        return 2

    V, F = load_stl(args.mesh_stl)
    log(f"Loaded mesh: {len(V)} vertices, {len(F)} faces from {args.mesh_stl}")

    order = order_targets(pts, args.order)
    pts, quats = pts[order], quats[order]
    if args.order != "file":
        log(f"Visiting order ({args.order}): {list(order)}")

    if args.closed and len(pts) > 2:
        pts = np.vstack([pts, pts[0]])
        quats = np.vstack([quats, quats[0]])

    md = MeshDistance(V, F)

    if not args.no_orient_check:
        ref_n = np.array([quat_to_axis(q) for q in quats])
        if md.orient_to_reference(pts, ref_n):
            log("Mesh normals disagreed with the YAML normals -- flipped mesh "
                "orientation (STL winding was inverted).")

    P, d, outward, _spline = build_offset_spline(
        pts, quats, md,
        offset=args.offset,
        spacing=args.spacing,
        iterations=args.iterations,
        smooth_start=args.smooth_start,
        smooth_end=args.smooth_end,
        closed=args.closed,
        through_contact=args.through_contact,
        pin_targets=not args.no_pin,
        side=args.side,
        project_knots=args.project_knots,
        mode=args.mode,
        log=log,
    )

    err = np.abs(d - args.offset)
    length = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
    stats = {
        "length": length,
        "mean_err": float(err.mean()),
        "max_err": float(err.max()),
        "rms_err": float(np.sqrt((err ** 2).mean())),
    }

    log(f"Path: {len(P)} waypoints, length {length:.3f} m")
    log(f"Offset {args.offset:.3f} m -> actual {d.mean():.4f} m mean, "
        f"[{d.min():.4f}, {d.max():.4f}] m")
    log(f"Offset error: mean {stats['mean_err']*1000:.2f} mm, "
        f"max {stats['max_err']*1000:.2f} mm, "
        f"rms {stats['rms_err']*1000:.2f} mm")

    cont = path_continuity(P)
    # |kappa_g| is the real check on "is this a geodesic": it vanishes on a
    # true geodesic regardless of how curved the path looks in 3D.
    kg = geodesic_curvature(P, md)
    cont["kg_median"] = float(np.median(kg))
    stats["cont"] = cont
    log(f"Continuity: turn/waypoint mean {cont['turn_mean']:.2f} deg, "
        f"max {cont['turn_max']:.2f} deg, peak curvature {cont['kmax']:.0f} 1/m")
    log(f"Geodesic curvature |kappa_g| median {cont['kg_median']:.3f} 1/m "
        f"(0 = exactly geodesic)")
    if cont["reversals"]:
        log(f"WARNING: {cont['reversals']} point(s) where the path reverses "
            f"(>90 deg turn). The curve is still C2, but the drone has to stop "
            f"and turn around there. This is caused by the order the targets "
            f"are visited, not by the smoothing"
            + ("; try --order greedy." if args.order == "file"
               else " -- even the shortest tour has to double back on these "
                    "targets."))

    write_yaml(args.output, P, outward, frame, args.offset, stats)
    log(f"Wrote {args.output}")

    if args.csv:
        write_csv(args.csv, P, outward)
        log(f"Wrote {args.csv}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
