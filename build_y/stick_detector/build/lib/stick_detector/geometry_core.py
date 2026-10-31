"""Pure-numpy helpers: connected components, plane RANSAC, stick features.

No scipy / cv2 dependency (the system numpy 2.x breaks both).
"""
import numpy as np


# ----------------------------------------------------------------------
# Connected components (run-based union-find, 8-connectivity + 1px tolerance)
# ----------------------------------------------------------------------
def _label_runs(mask):
    """Extract horizontal runs from a 2D boolean mask."""
    h, w = mask.shape
    padded = np.zeros((h, w + 1), dtype=np.int8)
    padded[:, :w] = mask
    diff = np.diff(padded.ravel())
    starts = np.flatnonzero(diff == 1) + 1
    ends = np.flatnonzero(diff == -1) + 1          # exclusive
    stride = w + 1
    rows = starts // stride
    c0 = starts - rows * stride
    c1 = ends - rows * stride
    return rows, c0, c1, len(starts)


def connected_components(mask):
    """Label 8-connected components of a 2D bool mask.

    Returns (label_image int32 with 0=background, num_labels).
    Runs are linked with an inlined union-find (no per-call overhead) and
    roots are resolved by vectorised pointer jumping.
    """
    h, w = mask.shape
    lab = np.zeros((h, w), dtype=np.int32)
    rows, c0, c1, nruns = _label_runs(mask)
    if nruns == 0:
        return lab, 0

    parent = np.arange(nruns, dtype=np.int32)
    n_rows = int(rows[-1]) + 1
    row_bounds = np.searchsorted(rows, np.arange(n_rows + 1), side='left')

    for r in range(1, n_rows):
        j0, j1 = row_bounds[r - 1], row_bounds[r]
        k0, k1 = row_bounds[r], row_bounds[r + 1]
        j = j0
        for k in range(k0, k1):
            ck0 = c0[k]
            ck1 = c1[k]
            while j < j1 and c1[j] < ck0 - 1:
                j += 1
            jj = j
            while jj < j1 and c0[jj] <= ck1 + 1:
                if c1[jj] >= ck0 - 1:
                    # inline union-find with path halving
                    a = k
                    while parent[a] != a:
                        parent[a] = parent[parent[a]]
                        a = parent[a]
                    b = jj
                    while parent[b] != b:
                        parent[b] = parent[parent[b]]
                        b = parent[b]
                    if a != b:
                        parent[a] = b
                jj += 1

    # resolve roots by pointer jumping (log steps, fully vectorised)
    root = parent
    while True:
        nxt = root[root]
        if np.array_equal(nxt, root):
            break
        root = nxt

    uniq, inv = np.unique(root, return_inverse=True)
    run_lab = (inv + 1).astype(np.int32)
    K = len(uniq)

    counts = c1 - c0
    total = int(counts.sum())
    if total:
        offsets = np.concatenate(([0], np.cumsum(counts[:-1])))
        run_exp = np.repeat(np.arange(nruns), counts)
        cols = c0[run_exp] + (np.arange(total) - np.repeat(offsets, counts))
        rows_exp = rows[run_exp]
        lab[rows_exp, cols] = run_lab[run_exp]
    return lab, K


# ----------------------------------------------------------------------
# Morphology (shift-based, no cv2/scipy)
# ----------------------------------------------------------------------
def erode3(mask):
    """3x3 (8-neighbourhood) erosion: breaks 1-2 px noise bridges."""
    h, w = mask.shape
    p = np.pad(mask, 1, constant_values=False)
    out = mask.copy()
    for dy in range(3):
        for dx in range(3):
            out &= p[dy:dy + h, dx:dx + w]
    return out


def dilate3(mask):
    """3x3 (8-neighbourhood) dilation: re-joins chunks split by pooling."""
    h, w = mask.shape
    p = np.pad(mask, 1, constant_values=False)
    out = mask.copy()
    for dy in range(3):
        for dx in range(3):
            out |= p[dy:dy + h, dx:dx + w]
    return out


# ----------------------------------------------------------------------
# Plane RANSAC
# ----------------------------------------------------------------------
def ransac_plane(P, thresh=0.012, iters=60, sample=6000, rng=None,
                 early_ratio=0.85):
    """Fit the dominant plane of P (N,3). Returns (normal, centroid, inlier_mask)."""
    rng = rng or np.random.default_rng()
    n = len(P)
    if n < 100:
        return None
    if n > sample:
        S = P[rng.choice(n, sample, replace=False)]
    else:
        S = P

    m = len(S)
    stop_cnt = int(early_ratio * m)
    best_cnt, best_inliers, best_nvec = -1, None, None
    for _ in range(iters):
        idx = rng.integers(0, m, 3)
        p = S[idx]
        nvec = np.cross(p[1] - p[0], p[2] - p[0])
        norm = np.linalg.norm(nvec)
        if norm < 1e-9:
            continue
        nvec = nvec / norm
        d = float(np.dot(nvec, p[0]))
        inl = np.abs(S @ nvec - d) < thresh
        cnt = int(inl.sum())
        if cnt > best_cnt:
            best_cnt, best_nvec, best_inliers = cnt, nvec, inl
            if cnt >= stop_cnt:      # confident enough, stop early
                break
    if best_nvec is None:
        return None

    # least-squares refine on sampled inliers: eigenvectors of the 3x3
    # covariance (NOT svd full_matrices=True, which builds an NxN U matrix
    # and cost ~360 ms per call on real data)
    inliers_S = S[best_inliers]
    if len(inliers_S) < 50:
        return None
    c = inliers_S.mean(axis=0)
    Q = inliers_S - c
    evals, evecs = np.linalg.eigh(Q.T @ Q)
    nvec = evecs[:, 0]                       # smallest eigenvalue -> normal
    inlier_mask = np.abs((P - c) @ nvec) < thresh
    return nvec, c, inlier_mask


# ----------------------------------------------------------------------
# Stick features via PCA
# ----------------------------------------------------------------------
def stick_features(P):
    """Compute (center, axis, length_m, radius_m, elongation) of a point set.

    - length: robust extent (p1..p99) along the principal axis
    - radius: RMS distance of points to the principal axis
    - elongation: sqrt(lambda1 / lambda2) of the 3D covariance
    """
    c = P.mean(axis=0)
    Q = P - c
    cov = (Q.T @ Q) / max(len(Q), 1)
    evals, evecs = np.linalg.eigh(cov)          # ascending
    l1, l2 = evals[2], max(evals[1], 1e-12)
    axis = evecs[:, 2]
    elongation = float(np.sqrt(l1 / l2))

    t = Q @ axis
    p_lo, p_hi = np.percentile(t, [1.0, 99.0])
    length = float(p_hi - p_lo)

    radial = np.linalg.norm(Q - np.outer(t, axis), axis=1)
    radius = float(np.sqrt((radial ** 2).mean()))

    mid = 0.5 * (p_lo + p_hi)
    center = c + axis * mid
    tip_a = c + axis * p_lo
    tip_b = c + axis * p_hi
    return dict(center=center, axis=axis, length=length, radius=radius,
                elongation=elongation, tip_a=tip_a, tip_b=tip_b)


def quat_x_to(v):
    """Quaternion rotating unit +x onto vector v (for Pose orientation)."""
    v = np.asarray(v, dtype=float)
    v = v / (np.linalg.norm(v) + 1e-12)
    d = float(np.dot(np.array([1.0, 0.0, 0.0]), v))
    if d < -0.999999:
        return (0.0, 0.0, 1.0, 0.0)            # antiparallel: 180 deg around z
    c = np.cross(np.array([1.0, 0.0, 0.0]), v)
    q = np.array([c[0], c[1], c[2], 1.0 + d])
    q /= np.linalg.norm(q)
    return tuple(q)


def quat_z_to(v):
    """Quaternion rotating unit +z onto vector v (for CYLINDER markers)."""
    v = np.asarray(v, dtype=float)
    v = v / (np.linalg.norm(v) + 1e-12)
    d = float(np.dot(np.array([0.0, 0.0, 1.0]), v))
    if d < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)            # antiparallel: 180 deg around x
    c = np.cross(np.array([0.0, 0.0, 1.0]), v)
    q = np.array([c[0], c[1], c[2], 1.0 + d])
    q /= np.linalg.norm(q)
    return tuple(q)
