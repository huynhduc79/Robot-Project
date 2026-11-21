# -*- coding: utf-8 -*-
"""
pgslam_core.py  -  Lõi thuật toán LiDAR Pose-Graph SLAM 2D (tự cài đặt, chỉ dùng numpy)

Bám theo:
  * Slide Chương 4 (part 2) / Corke et al. (2023) mục 6.1: Dead reckoning bằng odometry,
    lan truyền hiệp phương sai  P(k+1) = Fx P Fx^T + Fv V Fv^T, biên ±2σ (95 %).
  * Corke et al. (2023) mục 6.4 + Grisetti et al. (2010): Pose-graph SLAM, hàm sai số cạnh
    e_ij, Jacobian A_ij, B_ij, hệ H Δx = -b giải lặp Gauss-Newton, neo nút đầu tiên.
  * Hess et al. (2016) / Elfes (1989): bản đồ lưới chiếm chỗ, cập nhật hits / misses (log-odds).
  * Kümmerle et al. (2009): ATE, sai số tương đối (RPE).

Quy ước tư thế: x = [x, y, theta]  (m, m, rad), hệ tọa độ thế giới (nhà xưởng).
"""
import numpy as np

# ============================================================================
# 0. TIỆN ÍCH SE(2)
# ============================================================================

def wrap(a):
    """Chuẩn hóa góc về (-pi, pi]."""
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def oplus(a, b):
    """Phép hợp thành a ⊕ b (tư thế b biểu diễn trong khung a)."""
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([a[0] + c * b[0] - s * b[1],
                     a[1] + s * b[0] + c * b[1],
                     wrap(a[2] + b[2])])


def relpose(a, b):
    """Tư thế tương đối của b nhìn từ a:  a^{-1} ⊕ b   (toán tử ⊖ trong Kümmerle 2009)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = np.cos(a[2]), np.sin(a[2])
    return np.array([c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2])])


def rot(t):
    c, s = np.cos(t), np.sin(t)
    return np.array([[c, -s], [s, c]])


# ============================================================================
# 1. MÔI TRƯỜNG NHÀ XƯỞNG CHUẨN (GROUND TRUTH)
# ============================================================================
MAP_W, MAP_H, RES = 20.0, 20.0, 0.05          # 20 m x 20 m, ô 5 cm  -> 400 x 400 ô

# [xmin, xmax, ymin, ymax] (m) - giống hệt bố trí trong báo cáo
OBSTACLES = np.array([
    [0.00, 20.00, 0.00, 0.25],    # Tường Nam
    [0.00, 20.00, 19.75, 20.00],  # Tường Bắc
    [0.00, 0.25, 0.00, 20.00],    # Tường Tây
    [19.75, 20.00, 0.00, 20.00],  # Tường Đông
    [4.80, 8.20, 4.80, 8.00],     # Kệ khu A
    [4.80, 6.20, 4.00, 4.80],     # Nhánh phụ kệ A (phá đối xứng)
    [11.80, 15.20, 4.80, 8.20],   # Kệ khu B
    [13.80, 15.20, 8.20, 8.80],   # Nhánh phụ kệ B
    [4.80, 8.20, 12.00, 15.20],   # Kệ khu C
    [7.00, 8.20, 15.20, 16.00],   # Nhánh phụ kệ C
    [12.00, 15.20, 12.00, 15.20], # Khu máy CNC D
    [9.40, 10.60, 0.25, 1.15],    # Trụ tường Nam
    [9.40, 10.60, 18.85, 19.75],  # Trụ tường Bắc
    [0.25, 1.15, 9.40, 10.60],    # Trụ tường Tây
    [18.85, 19.75, 9.40, 10.60],  # Trụ tường Đông
])

WAYPOINTS = np.array([
    [2.5, 2.5], [10.0, 2.5], [17.5, 2.5], [17.5, 10.0], [17.5, 17.5],
    [10.0, 17.5], [10.0, 10.0], [2.5, 10.0], [2.5, 17.5], [2.5, 10.0], [2.5, 2.5]])


def make_gt_map():
    """Ma trận chiếm chỗ nhị phân M_true[iy, ix] (iy tính từ dưới lên)."""
    n = int(round(MAP_W / RES))
    g = np.zeros((n, n), dtype=bool)
    for x0, x1, y0, y1 in OBSTACLES:
        ix0, ix1 = int(np.floor(x0 / RES + 1e-9)), int(np.ceil(x1 / RES - 1e-9))
        iy0, iy1 = int(np.floor(y0 / RES + 1e-9)), int(np.ceil(y1 / RES - 1e-9))
        g[iy0:iy1, ix0:ix1] = True
    return g


# ============================================================================
# 2. QUỸ ĐẠO CHUẨN + MÔ HÌNH ODOMETRY (slide 4.5.2.1)
# ============================================================================

def make_trajectory(waypoints=WAYPOINTS, step=0.45, turn_step=np.deg2rad(18)):
    """Sinh chuỗi lệnh odometry thực  δ_k = (δd, δθ)  và quỹ đạo thực X_true.
    Mô hình (Corke 2023, slide 4.5.2.1): tiến δd theo trục x thân xe rồi quay δθ
        x_{k+1} = x_k + δd cosθ_k ,  y_{k+1} = y_k + δd sinθ_k ,  θ_{k+1} = θ_k + δθ
    """
    p0 = waypoints[0]
    th0 = np.arctan2(*(waypoints[1] - p0)[::-1])
    poses = [np.array([p0[0], p0[1], th0])]
    ctrl = []
    th = th0                                       # hướng hiện tại của robot (cập nhật sau mỗi lần quay)
    for i in range(len(waypoints) - 1):
        a, b = waypoints[i], waypoints[i + 1]
        tgt = np.arctan2(b[1] - a[1], b[0] - a[0])
        dth = wrap(tgt - th)
        th = tgt
        nt = int(np.ceil(abs(dth) / turn_step - 1e-9))
        for _ in range(nt):                       # quay tại chỗ
            ctrl.append((0.0, dth / nt))
        dist = np.linalg.norm(b - a)
        ns = max(1, int(round(dist / step)))
        for _ in range(ns):                       # đi thẳng
            ctrl.append((dist / ns, 0.0))
    ctrl = np.array(ctrl)
    # Tích phân: quay được áp dụng SAU khi tiến (đúng mô hình slide)
    x = poses[0].copy()
    for dd, dt in ctrl:
        x = np.array([x[0] + dd * np.cos(x[2]), x[1] + dd * np.sin(x[2]), wrap(x[2] + dt)])
        poses.append(x)
    return np.array(poses), ctrl


def odo_to_rel(dd, dt):
    """Phép đo odometry (δd, δθ) -> tư thế tương đối z = (δd, 0, δθ) trong SE(2)."""
    return np.array([dd, 0.0, dt])


def dead_reckoning(x0, odo, V, P0):
    """Dead reckoning + lan truyền hiệp phương sai (bước dự đoán EKF, slide 4.5.2.2):
        x̂(k+1) = f(x̂(k), δ̂k)
        P(k+1) = Fx P Fx^T + Fv V Fv^T
    với Fx = ∂f/∂x, Fv = ∂f/∂v (Jacobian theo nhiễu odometry)."""
    n = len(odo) + 1
    X = np.zeros((n, 3)); P = np.zeros((n, 3, 3))
    X[0], P[0] = x0, P0
    for k, (dd, dt) in enumerate(odo):
        th = X[k, 2]
        c, s = np.cos(th), np.sin(th)
        X[k + 1] = [X[k, 0] + dd * c, X[k, 1] + dd * s, wrap(th + dt)]
        Fx = np.array([[1, 0, -dd * s], [0, 1, dd * c], [0, 0, 1]])
        Fv = np.array([[c, 0], [s, 0], [0, 1]])
        P[k + 1] = Fx @ P[k] @ Fx.T + Fv @ V @ Fv.T
    return X, P


# ============================================================================
# 3. CẢM BIẾN LIDAR 2D CÓ NHIỄU (ray casting trên lưới chuẩn)
# ============================================================================

class Lidar:
    def __init__(self, max_range=12.0, ang_res_deg=1.5, sigma_r=0.03, sigma_b_deg=0.2):
        self.max_range = max_range
        self.angles = np.deg2rad(np.arange(-180.0, 180.0, ang_res_deg))
        self.sigma_r = sigma_r
        self.sigma_b = np.deg2rad(sigma_b_deg)

    def scan(self, gt, pose, rng):
        """Trả về ranges (NaN = không phản xạ trong tầm đo) tại các góc danh định self.angles.
        Mô hình: r̂ = r(β + w_β) + w_r ,  w_r ~ N(0, σ_r²), w_β ~ N(0, σ_β²)   (slide 4.5.3.1)."""
        A = self.angles + rng.normal(0, self.sigma_b, self.angles.size) + pose[2]
        r = ray_cast(gt, pose[:2], A, self.max_range)
        r = r + rng.normal(0, self.sigma_r, r.size)
        return r


def _occ(gt, x, y):
    n = gt.shape[0]
    ix = np.floor(x / RES).astype(int); iy = np.floor(y / RES).astype(int)
    out = (ix < 0) | (iy < 0) | (ix >= n) | (iy >= n)
    ixc, iyc = np.clip(ix, 0, n - 1), np.clip(iy, 0, n - 1)
    return gt[iyc, ixc] | out


def ray_cast(gt, p, A, rmax, step=RES / 2):
    r = np.arange(step, rmax + step, step)
    ca, sa = np.cos(A)[:, None], np.sin(A)[:, None]
    hit = _occ(gt, p[0] + ca * r, p[1] + sa * r)
    has = hit.any(axis=1)
    first = np.argmax(hit, axis=1)
    hi = r[first]; lo = np.maximum(hi - step, 0.0)
    for _ in range(6):                      # chia đôi -> độ chính xác < 1 mm
        mid = 0.5 * (lo + hi)
        h = _occ(gt, p[0] + ca[:, 0] * mid, p[1] + sa[:, 0] * mid)
        hi = np.where(h, mid, hi); lo = np.where(h, lo, mid)
    out = hi.copy(); out[~has] = np.nan; out[out > rmax] = np.nan
    return out


def scan_points(ranges, angles):
    m = np.isfinite(ranges)
    return np.c_[ranges[m] * np.cos(angles[m]), ranges[m] * np.sin(angles[m])]


def scan_normals(pts, max_gap=0.35):
    """Pháp tuyến bề mặt từ 2 điểm lân cận (điểm quét được sắp theo góc)."""
    n = len(pts)
    N = np.zeros_like(pts); ok = np.zeros(n, bool)
    if n < 3:
        return N, ok
    prv, nxt = np.roll(pts, 1, 0), np.roll(pts, -1, 0)
    t = nxt - prv
    good = (np.linalg.norm(pts - prv, axis=1) < max_gap) & (np.linalg.norm(nxt - pts, axis=1) < max_gap)
    L = np.linalg.norm(t, axis=1); good &= L > 1e-6
    N[good] = np.c_[-t[good, 1], t[good, 0]] / L[good, None]
    return N, good


# ============================================================================
# 4. FRONT-END: SO KHỚP QUÉT ICP ĐIỂM - ĐƯỜNG (point-to-line) + HIỆP PHƯƠNG SAI
# ============================================================================

def icp_pl(src, dst, dstN, dstOK, x0, iters=30, huber=0.05):
    """Tìm z = tư thế của quét 'src' trong khung quét 'dst' (z: dst -> src).
    Cực tiểu  Σ ρ( n_q^T (R(z) p + t(z) - q) )  bằng Gauss-Newton có trọng số Huber.
    Hiệp phương sai (xấp xỉ Laplace):  Σ_z ≈ s² (Jᵀ W J)^{-1}."""
    x = np.array(x0, float)
    H = None
    for it in range(iters):
        R = rot(x[2]); P = src @ R.T + x[:2]
        D2 = ((P[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
        nn = D2.argmin(1); d = np.sqrt(D2[np.arange(len(P)), nn])
        thr = max(0.25, 1.2 * 0.8 ** it)
        m = (d < thr) & dstOK[nn]
        if m.sum() < 30:
            return None
        q, n, p, ps = dst[nn[m]], dstN[nn[m]], P[m], src[m]
        r = ((p - q) * n).sum(1)
        c, s = np.cos(x[2]), np.sin(x[2])
        dPdt = np.c_[-s * ps[:, 0] - c * ps[:, 1], c * ps[:, 0] - s * ps[:, 1]]
        J = np.c_[n[:, 0], n[:, 1], (n * dPdt).sum(1)]
        w = np.where(np.abs(r) <= huber, 1.0, huber / np.maximum(np.abs(r), 1e-12))
        H = J.T @ (J * w[:, None]); g = J.T @ (w * r)
        try:
            dx = -np.linalg.solve(H + 1e-9 * np.eye(3), g)
        except np.linalg.LinAlgError:
            return None
        x = x + dx; x[2] = wrap(x[2])
        if np.linalg.norm(dx[:2]) < 1e-5 and abs(dx[2]) < 1e-6 and thr <= 0.25:
            break
    # Đánh giá chất lượng khớp
    R = rot(x[2]); P = src @ R.T + x[:2]
    D2 = ((P[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
    nn = D2.argmin(1); d = np.sqrt(D2[np.arange(len(P)), nn])
    m = (d < 0.15) & dstOK[nn]
    if m.sum() < 30:
        return None
    r = ((P[m] - dst[nn[m]]) * dstN[nn[m]]).sum(1)
    s2 = max((r ** 2).sum() / max(m.sum() - 3, 1), 1e-6)
    cov = s2 * np.linalg.inv(H + 1e-9 * np.eye(3))
    return dict(z=x, cov=cov, inlier=m.mean(), rms=float(np.sqrt((r ** 2).mean())), n=int(m.sum()))


# ============================================================================
# 5. BACK-END: TỐI ƯU ĐỒ THỊ TƯ THẾ (Gauss-Newton, Grisetti 2010 / Corke 6.4)
# ============================================================================

def edge_errors_jac(X, I, J, Z):
    """Sai số và Jacobian cho mọi cạnh (vector hóa).
        e_ij = [ R_z^T ( R_i^T (t_j - t_i) - t_z ) ;  wrap(θ_j - θ_i - θ_z) ]
        A_ij = [ -R_z^T R_i^T ,  R_z^T dR_i^T/dθ_i (t_j - t_i) ; 0 0 -1 ]
        B_ij = [  R_z^T R_i^T ,  0 ; 0 0 1 ]"""
    xi, xj = X[I], X[J]
    ci, si = np.cos(xi[:, 2]), np.sin(xi[:, 2])
    cz, sz = np.cos(Z[:, 2]), np.sin(Z[:, 2])
    dt = xj[:, :2] - xi[:, :2]
    # R_i^T (t_j - t_i)
    u = np.c_[ci * dt[:, 0] + si * dt[:, 1], -si * dt[:, 0] + ci * dt[:, 1]]
    v = u - Z[:, :2]
    e = np.c_[cz * v[:, 0] + sz * v[:, 1], -sz * v[:, 0] + cz * v[:, 1],
              wrap(xj[:, 2] - xi[:, 2] - Z[:, 2])]
    m = len(I)
    # R_z^T R_i^T = R(-(θz+θi))
    cc, ss = np.cos(Z[:, 2] + xi[:, 2]), np.sin(Z[:, 2] + xi[:, 2])
    RzRi = np.zeros((m, 2, 2)); RzRi[:, 0, 0] = cc; RzRi[:, 0, 1] = ss; RzRi[:, 1, 0] = -ss; RzRi[:, 1, 1] = cc
    # dR_i^T/dθ (t_j - t_i)
    du = np.c_[-si * dt[:, 0] + ci * dt[:, 1], -ci * dt[:, 0] - si * dt[:, 1]]
    dth = np.c_[cz * du[:, 0] + sz * du[:, 1], -sz * du[:, 0] + cz * du[:, 1]]
    A = np.zeros((m, 3, 3)); B = np.zeros((m, 3, 3))
    A[:, :2, :2] = -RzRi; A[:, :2, 2] = dth; A[:, 2, 2] = -1
    B[:, :2, :2] = RzRi; B[:, 2, 2] = 1
    return e, A, B


def build_system(X, I, J, Z, Om, anchor_info):
    n = len(X)
    e, A, B = edge_errors_jac(X, I, J, Z)
    OA = np.einsum('mab,mbc->mac', Om, A); OB = np.einsum('mab,mbc->mac', Om, B)
    Hii = np.einsum('mba,mbc->mac', A, OA); Hij = np.einsum('mba,mbc->mac', A, OB)
    Hjj = np.einsum('mba,mbc->mac', B, OB)
    bi = np.einsum('mba,mb->ma', OA, e); bj = np.einsum('mba,mb->ma', OB, e)
    H = np.zeros((3 * n, 3 * n)); b = np.zeros(3 * n)
    a3 = np.arange(3)
    def add(Hblk, R_, C_):
        rr = (3 * R_)[:, None, None] + a3[None, :, None]
        cc = (3 * C_)[:, None, None] + a3[None, None, :]
        np.add.at(H, (np.broadcast_to(rr, Hblk.shape), np.broadcast_to(cc, Hblk.shape)), Hblk)
    add(Hii, I, I); add(Hij, I, J); add(np.transpose(Hij, (0, 2, 1)), J, I); add(Hjj, J, J)
    np.add.at(b, (3 * I)[:, None] + a3, bi); np.add.at(b, (3 * J)[:, None] + a3, bj)
    H[:3, :3] += anchor_info                    # neo nút đầu (gauge freedom)
    chi2 = float(np.einsum('ma,mab,mb->', e, Om, e))
    return H, b, chi2, e


def optimize_pose_graph(X0, I, J, Z, Om, x_anchor, anchor_info, iters=15, tol=1e-6):
    X = X0.copy()
    hist = []
    for _ in range(iters):
        H, b, chi2, _ = build_system(X, I, J, Z, Om, anchor_info)
        # ràng buộc mềm của nút neo: kéo nút 0 về x_anchor
        r0 = X[0] - x_anchor; r0[2] = wrap(r0[2])
        b[:3] += anchor_info @ r0
        hist.append(chi2)
        dx = np.linalg.solve(H, -b).reshape(-1, 3)
        X = X + dx; X[:, 2] = wrap(X[:, 2])
        if np.abs(dx).max() < tol:
            break
    H, b, chi2, e = build_system(X, I, J, Z, Om, anchor_info)
    hist.append(chi2)
    return X, H, hist, e


def marginal_covs(H):
    """Hiệp phương sai biên từng nút: Σ = H^{-1} (xấp xỉ Laplace quanh nghiệm MAP)."""
    S = np.linalg.inv(H)
    n = H.shape[0] // 3
    return np.stack([S[3 * k:3 * k + 3, 3 * k:3 * k + 3] for k in range(n)]), S


# ============================================================================
# 6. BẢN ĐỒ LƯỚI CHIẾM CHỖ (log-odds, hits / misses - Elfes 1989, Hess 2016)
# ============================================================================
L_HIT, L_MISS, L_MIN, L_MAX = np.log(0.7 / 0.3), np.log(0.4 / 0.6), -4.0, 4.0


def build_occupancy(poses, scans, angles, max_range, n=400):
    L = np.zeros((n, n))
    for pose, rg in zip(poses, scans):
        insert_scan(L, pose, rg, angles, max_range)
    return L


def insert_scan(L, pose, rg, angles, max_range):
    n = L.shape[0]
    A = angles + pose[2]
    valid = np.isfinite(rg)
    rr = np.where(valid, rg, max_range)
    step = RES / 2
    smax = int(np.ceil(max_range / step))
    s = np.arange(smax) * step
    ca, sa = np.cos(A)[:, None], np.sin(A)[:, None]
    mask = s[None, :] < (rr[:, None] - RES * 0.5)
    xs = pose[0] + ca * s; ys = pose[1] + sa * s
    ix = np.floor(xs / RES).astype(int); iy = np.floor(ys / RES).astype(int)
    ins = mask & (ix >= 0) & (iy >= 0) & (ix < n) & (iy < n)
    miss = np.unique(iy[ins] * n + ix[ins])
    hx = pose[0] + np.cos(A[valid]) * rg[valid]; hy = pose[1] + np.sin(A[valid]) * rg[valid]
    hix, hiy = np.floor(hx / RES).astype(int), np.floor(hy / RES).astype(int)
    hin = (hix >= 0) & (hiy >= 0) & (hix < n) & (hiy < n)
    hit = np.unique(hiy[hin] * n + hix[hin])
    miss = np.setdiff1d(miss, hit, assume_unique=True)
    Lf = L.reshape(-1)
    Lf[miss] = np.clip(Lf[miss] + L_MISS, L_MIN, L_MAX)
    Lf[hit] = np.clip(Lf[hit] + L_HIT, L_MIN, L_MAX)


def map_metrics(L, gt, p_occ=0.65, unknown_band=0.05):
    """Map Accuracy và Obstacle IoU trên các ô đã quan sát (theo công thức báo cáo)."""
    P = 1.0 / (1.0 + np.exp(-L))
    known = np.abs(P - 0.5) > unknown_band
    occ = P >= p_occ
    TP = np.sum(occ & gt & known); TN = np.sum(~occ & ~gt & known)
    FP = np.sum(occ & ~gt & known); FN = np.sum(~occ & gt & known)
    acc = 100.0 * (TP + TN) / max(TP + TN + FP + FN, 1)
    iou = 100.0 * TP / max(TP + FP + FN, 1)
    return dict(acc=acc, iou=iou, TP=int(TP), TN=int(TN), FP=int(FP), FN=int(FN), P=P, known=known, occ=occ)


# ============================================================================
# 7. CHỈ SỐ ĐÁNH GIÁ (Kümmerle 2009) + KIỂM ĐỊNH NHẤT QUÁN 95 %
# ============================================================================

def pose_errors(Xest, Xtrue):
    e = Xest - Xtrue; e[:, 2] = wrap(e[:, 2]); return e


def ate_rmse(Xest, Xtrue):
    e = pose_errors(Xest, Xtrue)
    return float(np.sqrt(np.mean(e[:, 0] ** 2 + e[:, 1] ** 2)))


def rpe(Xest, Xtrue, delta=1):
    """Sai số tương đối (Kümmerle 2009): δ_ij = x_j ⊖ x_i so với δ*_ij."""
    et, er = [], []
    for i in range(len(Xest) - delta):
        d = relpose(Xest[i], Xest[i + delta]); ds = relpose(Xtrue[i], Xtrue[i + delta])
        err = relpose(ds, d)
        et.append(np.hypot(err[0], err[1])); er.append(abs(err[2]))
    return float(np.sqrt(np.mean(np.square(et)))), float(np.rad2deg(np.sqrt(np.mean(np.square(er)))))


def consistency(Xest, P, Xtrue):
    """% thời điểm sai số nằm trong biên ±2σ và NEES vị trí (2 bậc tự do)."""
    e = pose_errors(Xest, Xtrue)
    sig = np.sqrt(np.maximum(np.stack([P[:, 0, 0], P[:, 1, 1], P[:, 2, 2]], 1), 1e-18))
    inside = np.abs(e) <= 2 * sig
    nees = np.array([e[k, :2] @ np.linalg.solve(P[k, :2, :2] + 1e-12 * np.eye(2), e[k, :2])
                     for k in range(1, len(e))])
    return dict(in_x=100 * inside[1:, 0].mean(), in_y=100 * inside[1:, 1].mean(),
                in_t=100 * inside[1:, 2].mean(), nees=nees, anees=float(nees.mean()),
                in_ell=100 * np.mean(nees <= 5.991))


def ellipse_pts(mu, C, conf_chi2=5.991, n=60):
    """Đường elip tin cậy 95 % (χ²₂ = 5.991) của phân bố N(mu, C)."""
    w, V = np.linalg.eigh(C)
    w = np.maximum(w, 0)
    t = np.linspace(0, 2 * np.pi, n)
    E = V @ np.diag(np.sqrt(conf_chi2 * w)) @ np.vstack([np.cos(t), np.sin(t)])
    return mu[0] + E[0], mu[1] + E[1]


# ============================================================================
# 8. CHẠY MỘT LẦN MÔ PHỎNG ĐẦY ĐỦ
# ============================================================================
DEFAULT = dict(sigma_d=0.02, sigma_th_deg=0.5,           # nhiễu odometry (giống Corke: V = diag(0.02², 0.5°²))
               sigma_r=0.03, sigma_b_deg=0.2,            # nhiễu LiDAR
               max_range=12.0, ang_res_deg=1.5,
               use_seq_icp=True,                          # cạnh so khớp quét liên tiếp (LiDAR odometry)
               lc_radius=3.0, lc_min_gap=20, lc_every=2,  # tham số phát hiện đóng vòng
               lc_min_inlier=0.6, lc_max_rms=0.06,
               cov_inflate=1.0,                           # hệ số thổi phồng hiệp phương sai ICP (κ), 1 = không thổi phồng
               lat_sigma=0.002)                           # σ ngang cho cạnh odometry (≈ ràng buộc không trượt)


def run_simulation(cfg=None, seed=0, keep_history=False, verbose=False):
    c = dict(DEFAULT); c.update(cfg or {})
    rng = np.random.default_rng(seed)
    gt = make_gt_map()
    Xtrue, ctrl = make_trajectory()
    N = len(Xtrue)
    V = np.diag([c['sigma_d'] ** 2, np.deg2rad(c['sigma_th_deg']) ** 2])
    odo = ctrl + rng.multivariate_normal([0, 0], V, len(ctrl))          # odometry đo được (có nhiễu)
    P0 = np.diag([1e-6, 1e-6, 1e-8])

    # --- (A) Dead reckoning + EKF prediction (slide)
    Xdr, Pdr = dead_reckoning(Xtrue[0], odo, V, P0)

    # --- LiDAR
    lid = Lidar(c['max_range'], c['ang_res_deg'], c['sigma_r'], c['sigma_b_deg'])
    scans = [lid.scan(gt, Xtrue[k], rng) for k in range(N)]
    pts = [scan_points(s, lid.angles) for s in scans]
    nrm = [scan_normals(p) for p in pts]

    # Hiệp phương sai cạnh odometry trong SE(2): z = (δd, 0, δθ)
    Sodo = np.diag([c['sigma_d'] ** 2, c['lat_sigma'] ** 2, np.deg2rad(c['sigma_th_deg']) ** 2])
    Oodo = np.linalg.inv(Sodo)
    anchor_info = np.linalg.inv(P0)

    I, J, Z, OM, KIND = [], [], [], [], []        # KIND: 0 odo, 1 seq-icp, 2 loop closure
    def add_edge(i, j, z, Om, kind):
        I.append(i); J.append(j); Z.append(np.asarray(z, float)); OM.append(Om); KIND.append(kind)

    X = np.zeros((N, 3)); X[0] = Xtrue[0]
    Pcur = P0.copy()
    hist = dict(Xonline=[], Pnode=[], lc_events=[], snapshots={}, chi2=[]) if keep_history else None
    lc_count = 0; n_lc_try = 0
    Pnodes = np.zeros((N, 3, 3)); Pnodes[0] = P0

    for k in range(1, N):
        dd, dt = odo[k - 1]
        zodo = odo_to_rel(dd, dt)
        X[k] = oplus(X[k - 1], zodo)
        add_edge(k - 1, k, zodo, Oodo, 0)
        Sz = Sodo.copy()
        if c['use_seq_icp']:
            src, dst = pts[k], pts[k - 1]
            res = icp_pl(src, dst, nrm[k - 1][0], nrm[k - 1][1], zodo)
            if res is not None and res['inlier'] > 0.7 and res['rms'] < c['lc_max_rms']:
                Sicp = c['cov_inflate'] * res['cov'] + np.diag([1e-6, 1e-6, 1e-8])
                add_edge(k - 1, k, res['z'], np.linalg.inv(Sicp), 1)
                # dự đoán theo trung bình có trọng số (thông tin) của 2 cạnh
                Om_f = Oodo + np.linalg.inv(Sicp)
                dz = wrap(res['z'] - zodo); dz[:2] = res['z'][:2] - zodo[:2]
                zf = zodo + np.linalg.solve(Om_f, np.linalg.inv(Sicp) @ dz)
                X[k] = oplus(X[k - 1], zf)
                Sz = np.linalg.inv(Om_f)
        # lan truyền hiệp phương sai nút mới (chuỗi):  Σ_k = J1 Σ_{k-1} J1ᵀ + J2 Σ_z J2ᵀ
        zr = relpose(X[k - 1], X[k]); th = X[k - 1, 2]
        J1 = np.array([[1, 0, -np.sin(th) * zr[0] - np.cos(th) * zr[1]],
                       [0, 1, np.cos(th) * zr[0] - np.sin(th) * zr[1]], [0, 0, 1]])
        J2 = np.zeros((3, 3)); J2[:2, :2] = rot(th); J2[2, 2] = 1
        Pcur = J1 @ Pnodes[k - 1] @ J1.T + J2 @ Sz @ J2.T
        Pnodes[k] = Pcur

        # --- phát hiện đóng vòng (Loop closure) trong bán kính tìm kiếm
        added = False
        if k % c['lc_every'] == 0 and k > c['lc_min_gap']:
            cand = np.arange(0, k - c['lc_min_gap'])
            d = np.linalg.norm(X[cand, :2] - X[k, :2], axis=1)
            cand = cand[d < c['lc_radius']]
            if len(cand):
                cand = cand[np.argsort(d[d < c['lc_radius']])][:2]
                for j in cand:
                    n_lc_try += 1
                    z0 = relpose(X[j], X[k])
                    best = None
                    for dth0 in (0.0, np.deg2rad(5), -np.deg2rad(5)):
                        r_ = icp_pl(pts[k], pts[j], nrm[j][0], nrm[j][1], z0 + [0, 0, dth0])
                        if r_ is not None and (best is None or r_['inlier'] > best['inlier'] + 1e-9):
                            best = r_
                        if best is not None and best['inlier'] > 0.85:
                            break
                    if best is not None and best['inlier'] > c['lc_min_inlier'] and best['rms'] < c['lc_max_rms']:
                        Slc = c['cov_inflate'] * best['cov'] + np.diag([1e-6, 1e-6, 1e-8])
                        add_edge(j, k, best['z'], np.linalg.inv(Slc), 2)
                        lc_count += 1; added = True
                        if keep_history:
                            hist['lc_events'].append((k, int(j)))
        if added:   # back-end: tối ưu toàn đồ thị khi có ràng buộc đóng vòng mới
            Ia, Ja = np.array(I), np.array(J)
            Xo, H, ch, _ = optimize_pose_graph(X[:k + 1], Ia, Ja, np.array(Z), np.array(OM), Xtrue[0], anchor_info)
            X[:k + 1] = Xo
            Pn, _ = marginal_covs(H)
            Pnodes[:k + 1] = Pn
            if keep_history:
                hist['chi2'].append((k, ch))
                hist['snapshots'][k] = Xo.copy()
        if keep_history:
            hist['Xonline'].append(X[:k + 1].copy())
            hist['Pnode'].append(Pnodes[k].copy())

    I, J, Z, OM, KIND = map(np.array, (I, J, Z, OM, KIND))

    # --- (C) Pose-graph SLAM: tối ưu cuối toàn cục (batch)
    Xslam, H, chi_hist, e = optimize_pose_graph(X, I, J, Z, OM, Xtrue[0], anchor_info, iters=30)
    Pslam, Sfull = marginal_covs(H)

    # --- (B) Không đóng vòng (No LC): chỉ cạnh tuần tự odo (+ ICP liên tiếp)
    m_nolc = KIND != 2
    Xinit_nolc = np.zeros_like(Xdr); Xinit_nolc[0] = Xtrue[0]
    for k in range(1, N):
        Xinit_nolc[k] = oplus(Xinit_nolc[k - 1], odo_to_rel(*odo[k - 1]))
    Xnolc, Hn, _, _ = optimize_pose_graph(Xinit_nolc, I[m_nolc], J[m_nolc], Z[m_nolc], OM[m_nolc],
                                          Xtrue[0], anchor_info, iters=30)
    Pnolc, _ = marginal_covs(Hn)

    # --- (A') đồ thị chỉ odometry -> phải trùng dead reckoning (kiểm chứng lý thuyết)
    m_odo = KIND == 0
    Xodo_pg, Ho, _, _ = optimize_pose_graph(Xinit_nolc, I[m_odo], J[m_odo], Z[m_odo], OM[m_odo],
                                            Xtrue[0], anchor_info, iters=30)
    Podo_pg, _ = marginal_covs(Ho)

    out = dict(cfg=c, gt=gt, Xtrue=Xtrue, odo=odo, ctrl=ctrl, scans=scans, angles=lid.angles,
               Xdr=Xdr, Pdr=Pdr, Xnolc=Xnolc, Pnolc=Pnolc, Xslam=Xslam, Pslam=Pslam,
               Xodo_pg=Xodo_pg, Podo_pg=Podo_pg, H=H, Sfull=Sfull,
               I=I, J=J, Z=Z, OM=OM, KIND=KIND, chi2_hist=chi_hist, edge_err=e,
               n_lc=int(lc_count), n_lc_try=int(n_lc_try), hist=hist)
    if verbose:
        print(f"seed={seed}: N={N}, LC={lc_count}/{n_lc_try}, ATE DR={ate_rmse(Xdr, Xtrue):.3f} "
              f"NoLC={ate_rmse(Xnolc, Xtrue):.3f} SLAM={ate_rmse(Xslam, Xtrue):.3f}")
    return out
