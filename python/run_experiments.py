# -*- coding: utf-8 -*-
"""
run_experiments.py - Chạy toàn bộ thí nghiệm, xuất HÌNH (../figures) và SỐ LIỆU (../results)

  Kịch bản 1 (so sánh 3 phương pháp tại mức nhiễu chuẩn):
      (A) Dead reckoning (odometry + EKF prediction)       - như slide Ch.4 part 2
      (B) Không đóng vòng (No LC: odometry + ghép quét LiDAR liên tiếp)
      (C) Pose-Graph SLAM (B + cạnh đóng vòng + tối ưu Gauss-Newton)
  Kịch bản 2: Monte Carlo kiểm định độ nhất quán (sai số nằm trong biên 95 %?, NEES)
  Kịch bản 3: Khảo sát độ nhạy theo nhiễu khoảng cách LiDAR σ_r = 1, 3, 5, 8 cm

Chạy:  python run_experiments.py            (đầy đủ, ~10 phút)
       python run_experiments.py --quick    (ít lần Monte Carlo, ~3 phút)
"""
import os, sys, json, time, argparse
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap
import pgslam_core as C
try:
    sys.stdout.reconfigure(encoding="utf-8"); sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

HERE = os.path.dirname(os.path.abspath(__file__))
FIG = os.path.join(HERE, '..', 'figures'); RES_DIR = os.path.join(HERE, '..', 'results')
os.makedirs(FIG, exist_ok=True); os.makedirs(RES_DIR, exist_ok=True)

plt.rcParams.update({'font.size': 10, 'axes.grid': True, 'grid.alpha': 0.3,
                     'figure.dpi': 110, 'savefig.dpi': 200, 'savefig.bbox': 'tight'})
COL = dict(gt='k', dr='#d62728', nolc='#ff7f0e', slam='#1f77b4', lc='#e377c2')
NAMES = dict(dr='Dead reckoning (odometry)', nolc='Không đóng vòng (No LC)', slam='Pose-Graph SLAM')


def save(fig, name):
    p = os.path.join(FIG, name); fig.savefig(p); plt.close(fig); print('  ->', name)


def show_gt(ax, gt, alpha=1.0):
    ax.imshow(gt, origin='lower', extent=[0, C.MAP_W, 0, C.MAP_H], cmap=ListedColormap(['white', '#444444']),
              alpha=alpha, interpolation='nearest')
    ax.set_xlim(0, C.MAP_W); ax.set_ylim(0, C.MAP_H); ax.set_aspect('equal')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')


def show_occ(ax, P, known):
    img = np.where(known, 1 - P, 0.5)
    ax.imshow(img, origin='lower', extent=[0, C.MAP_W, 0, C.MAP_H], cmap='gray', vmin=0, vmax=1,
              interpolation='nearest')
    ax.set_xlim(0, C.MAP_W); ax.set_ylim(0, C.MAP_H); ax.set_aspect('equal')
    ax.set_xlabel('X [m]'); ax.set_ylabel('Y [m]')


def chi2_ppf(p, nu):
    """Xấp xỉ Wilson-Hilferty cho phân vị χ² (không cần scipy)."""
    z = {0.025: -1.959964, 0.975: 1.959964, 0.05: -1.644854, 0.95: 1.644854}[p]
    return nu * (1 - 2 / (9 * nu) + z * np.sqrt(2 / (9 * nu))) ** 3


def methods(o):
    return [('dr', o['Xdr'], o['Pdr']), ('nolc', o['Xnolc'], o['Pnolc']), ('slam', o['Xslam'], o['Pslam'])]


# ============================================================================
def scenario1(seed=7):
    print('[Kịch bản 1] So sánh DR / No-LC / Pose-Graph SLAM, seed =', seed)
    t0 = time.time()
    o = C.run_simulation(seed=seed, keep_history=True, verbose=True)
    print('  thời gian chạy SLAM: %.1f s' % (time.time() - t0))
    gt, Xt = o['gt'], o['Xtrue']; k = np.arange(len(Xt))
    met = {}
    for key, X, P in methods(o):
        cs = C.consistency(X, P, Xt)
        e = C.pose_errors(X, Xt)
        rpe1 = C.rpe(X, Xt, 1); rpe20 = C.rpe(X, Xt, 20)
        met[key] = dict(ate=C.ate_rmse(X, Xt), ate_max=float(np.max(np.hypot(e[:, 0], e[:, 1]))),
                        final=float(np.hypot(*e[-1, :2])), head_rmse=float(np.rad2deg(np.sqrt(np.mean(e[:, 2] ** 2)))),
                        rpe1_t=rpe1[0], rpe1_r=rpe1[1], rpe20_t=rpe20[0], rpe20_r=rpe20[1],
                        sig_final=float(np.sqrt(P[-1, 0, 0] + P[-1, 1, 1])),
                        sig_max=float(np.max(np.sqrt(P[:, 0, 0] + P[:, 1, 1]))),
                        sig_th_final_deg=float(np.rad2deg(np.sqrt(P[-1, 2, 2]))),
                        sig_th_max_deg=float(np.rad2deg(np.max(np.sqrt(P[:, 2, 2])))),
                        in_x=cs['in_x'], in_y=cs['in_y'], in_t=cs['in_t'], in_ell=cs['in_ell'], anees=cs['anees'])
    # --- Hội tụ Gauss-Newton khi khởi tạo toàn đồ thị bằng dead reckoning (batch)
    _, _, ch_batch, _ = C.optimize_pose_graph(o['Xdr'].copy(), o['I'], o['J'], o['Z'], o['OM'], Xt[0],
                                              np.linalg.inv(np.diag([1e-6, 1e-6, 1e-8])), iters=30)
    o['chi2_hist'] = ch_batch
    # --- Bản đồ
    maps = {}
    for key, X in [('gtpose', Xt), ('dr', o['Xdr']), ('nolc', o['Xnolc']), ('slam', o['Xslam'])]:
        L = C.build_occupancy(X, o['scans'], o['angles'], o['cfg']['max_range'])
        maps[key] = C.map_metrics(L, gt)
    for key in ['dr', 'nolc', 'slam']:
        met[key]['acc'] = maps[key]['acc']; met[key]['iou'] = maps[key]['iou']
    met['gtpose'] = dict(acc=maps['gtpose']['acc'], iou=maps['gtpose']['iou'])
    met['info'] = dict(N=len(Xt), n_edges=int(len(o['I'])), n_odo=int((o['KIND'] == 0).sum()),
                       n_seq=int((o['KIND'] == 1).sum()), n_lc=int((o['KIND'] == 2).sum()),
                       n_lc_try=o['n_lc_try'], chi2_init=float(o['chi2_hist'][0]), chi2_final=float(o['chi2_hist'][-1]), gn_iters=int(len(o['chi2_hist']) - 1),
                       dof=int(3 * len(o['I']) - 3 * (len(Xt) - 1)),
                       dr_vs_odopg_maxdiff=float(np.abs(o['Xdr'] - o['Xodo_pg']).max()),
                       path_len=float(np.sum(np.abs(o['ctrl'][:, 0]))))

    # ---------------- FIG 1: môi trường + LiDAR
    fig, ax = plt.subplots(1, 2, figsize=(11, 5))
    show_gt(ax[0], gt); ax[0].plot(Xt[:, 0], Xt[:, 1], '-', c='b', lw=1.6, label='Quỹ đạo chuẩn $X_{true}$')
    ax[0].plot(*Xt[0, :2], 'go', ms=9, label='Xuất phát / kết thúc')
    for i, (x, y) in enumerate(C.WAYPOINTS[:-1]):
        ax[0].annotate(f'W{i}', (x, y), xytext=(4, 4), textcoords='offset points', fontsize=8, color='b')
    ax[0].set_title('a) Bản đồ chuẩn nhà xưởng $M_{true}$ (20×20 m, ô 5 cm)'); ax[0].legend(loc='upper center', fontsize=8)
    kk = 40; p = Xt[kk]; r = o['scans'][kk]; A = o['angles'] + p[2]; m = np.isfinite(r)
    show_gt(ax[1], gt, 0.6)
    for a, rr in zip(A[m][::2], r[m][::2]):
        ax[1].plot([p[0], p[0] + rr * np.cos(a)], [p[1], p[1] + rr * np.sin(a)], '-', c='r', lw=0.4, alpha=0.35)
    ax[1].plot(p[0] + r[m] * np.cos(A[m]), p[1] + r[m] * np.sin(A[m]), '.', c='r', ms=3, label='Điểm phản xạ (có nhiễu)')
    ax[1].plot(*p[:2], 'bo', ms=7, label=f'Robot tại tư thế k={kk}')
    ax[1].set_title(f'b) Mô phỏng LiDAR 2D: {len(A)} tia, $\\sigma_r$={o["cfg"]["sigma_r"]*100:.0f} cm, '
                    f'$\\sigma_\\beta$={o["cfg"]["sigma_b_deg"]}°'); ax[1].legend(loc='upper right', fontsize=8)
    save(fig, 'fig1_moi_truong_lidar.png')

    # ---------------- FIG 2: Dead reckoning như slide (ellipse 95% + biên ±2σ)
    fig, ax = plt.subplots(1, 2, figsize=(12, 5))
    show_gt(ax[0], gt, 0.25)
    ax[0].plot(Xt[:, 0], Xt[:, 1], '-k', lw=1.5, label='Quỹ đạo thực')
    ax[0].plot(o['Xdr'][:, 0], o['Xdr'][:, 1], '-', c=COL['dr'], lw=1.3, label='Ước lượng dead reckoning')
    for i in range(0, len(Xt), 12):
        ex, ey = C.ellipse_pts(o['Xdr'][i, :2], o['Pdr'][i, :2, :2])
        ax[0].plot(ex, ey, '-', c='g', lw=0.8)
    ax[0].plot([], [], '-g', label='Elip tin cậy 95 % ($\\chi^2_2$=5.991)')
    ax[0].legend(loc='upper center', fontsize=8); ax[0].set_title('a) Dead reckoning: elip bất định tăng không giới hạn')
    e = C.pose_errors(o['Xdr'], Xt); s = np.sqrt(o['Pdr'][:, 0, 0])
    ax[1].plot(k, e[:, 0], '-', c=COL['dr'], lw=1.2, label='Sai số $\\tilde x_k = \\hat x_k - x_k$')
    ax[1].plot(k, 2 * s, '--k', lw=1.2, label='Biên +2σ (95 %)'); ax[1].plot(k, -2 * s, '--k', lw=1.2, label='Biên −2σ (95 %)')
    ax[1].fill_between(k, -2 * s, 2 * s, color='gray', alpha=0.15)
    ax[1].set_xlabel('Bước k'); ax[1].set_ylabel('Sai số x [m]'); ax[1].legend(fontsize=8, loc='upper left')
    ax[1].set_title('b) Sai số tọa độ x và đường biên ±2σ (slide 4.5.2.2)')
    save(fig, 'fig2_dead_reckoning_95.png')

    # ---------------- FIG 3: Quỹ đạo 3 phương pháp + elip 95 %
    fig, ax = plt.subplots(1, 2, figsize=(12.5, 5.6))
    for a_, (key, X, P) in zip(ax, [('nolc', o['Xnolc'], o['Pnolc']), ('slam', o['Xslam'], o['Pslam'])]):
        show_gt(a_, gt, 0.25)
        a_.plot(Xt[:, 0], Xt[:, 1], '-k', lw=1.6, label='Ground truth')
        a_.plot(o['Xdr'][:, 0], o['Xdr'][:, 1], ':', c=COL['dr'], lw=1.2, label=f'DR (ATE={met["dr"]["ate"]:.3f} m)')
        a_.plot(X[:, 0], X[:, 1], '-', c=COL[key], lw=1.5, label=f'{NAMES[key]} (ATE={met[key]["ate"]:.3f} m)')
        for i in range(0, len(Xt), 10):
            ex, ey = C.ellipse_pts(X[i, :2], P[i, :2, :2])
            a_.plot(ex, ey, '-', c='g', lw=0.8)
        a_.plot([], [], '-g', label='Elip 95 %')
        if key == 'slam':
            for i_, j_ in zip(o['I'][o['KIND'] == 2], o['J'][o['KIND'] == 2]):
                a_.plot(X[[i_, j_], 0], X[[i_, j_], 1], '-', c=COL['lc'], lw=0.8, alpha=0.8)
            a_.plot([], [], '-', c=COL['lc'], label=f'Cạnh đóng vòng ({met["info"]["n_lc"]})')
        a_.legend(loc='upper center', fontsize=7.5, ncol=2)
    ax[0].set_title('a) Không đóng vòng: elip phình to dần'); ax[1].set_title('b) Pose-Graph SLAM: elip bị "ghìm" nhỏ')
    save(fig, 'fig3_quy_dao_elip95.png')

    # ---------------- FIG 4: Sai số x, y, θ với 2 đường biên ±2σ cho 3 phương pháp
    fig, ax = plt.subplots(3, 3, figsize=(14, 8.5), sharex=True)
    labs = ['x [m]', 'y [m]', 'θ [độ]']
    for c_, (key, X, P) in enumerate(methods(o)):
        e = C.pose_errors(X, Xt)
        for r_ in range(3):
            sc = np.rad2deg(1) if r_ == 2 else 1.0
            s = np.sqrt(P[:, r_, r_]) * sc
            a_ = ax[r_, c_]
            a_.fill_between(k, -2 * s, 2 * s, color=COL[key], alpha=0.12)
            a_.plot(k, 2 * s, '--', c='k', lw=1.0); a_.plot(k, -2 * s, '--', c='k', lw=1.0)
            a_.plot(k, e[:, r_] * sc, '-', c=COL[key], lw=1.2)
            inside = [met[key]['in_x'], met[key]['in_y'], met[key]['in_t']][r_]
            a_.text(0.02, 0.92, f'{inside:.0f} % mẫu trong ±2σ', transform=a_.transAxes, fontsize=8,
                    bbox=dict(fc='white', ec='none', alpha=0.7))
            if c_ == 0: a_.set_ylabel('Sai số ' + labs[r_])
            if r_ == 0: a_.set_title(NAMES[key])
            if r_ == 2: a_.set_xlabel('Bước k')
    for lk in o['hist']['lc_events'][:1]:
        pass
    ax[0, 0].plot([], [], '-', c='gray', label='sai số ước lượng'); ax[0, 0].plot([], [], '--k', label='biên ±2σ (95 %)')
    ax[0, 0].legend(fontsize=7.5, loc='lower left')
    fig.suptitle('Sai số tư thế và hai đường biên tin cậy 95 % (±2σ từ Σ = H⁻¹ hoặc P của EKF)', y=1.0)
    fig.tight_layout()
    save(fig, 'fig4_sai_so_bien95_3pp.png')

    # ---------------- FIG 5: ATE e_k theo k + độ bất định σ_pos
    fig, ax = plt.subplots(1, 2, figsize=(12, 4.4))
    for key, X, P in methods(o):
        e = C.pose_errors(X, Xt)
        ax[0].plot(k, np.hypot(e[:, 0], e[:, 1]), '-', c=COL[key], lw=1.3,
                   label=f'{NAMES[key]}: RMSE={met[key]["ate"]:.3f} m')
        ax[1].plot(k, np.sqrt(P[:, 0, 0] + P[:, 1, 1]), '-', c=COL[key], lw=1.3, label=NAMES[key])
    lc_k = sorted(set(o['J'][o['KIND'] == 2]))
    ax[1].plot(lc_k, np.zeros(len(lc_k)), '|', c=COL['lc'], ms=10, label='Bước có đóng vòng')
    ax[0].set_xlabel('Bước k'); ax[0].set_ylabel('$e_k$ [m]'); ax[0].legend(fontsize=8); ax[0].set_title('a) Sai số vị trí tuyệt đối ATE theo bước k')
    ax[1].set_xlabel('Bước k'); ax[1].set_ylabel('$\\sqrt{\\sigma_x^2+\\sigma_y^2}$ [m]'); ax[1].legend(fontsize=8)
    ax[1].set_title('b) Độ bất định vị trí (từ ma trận hiệp phương sai)')
    save(fig, 'fig5_ate_va_do_bat_dinh.png')

    # ---------------- FIG 6: Đồ thị tư thế, cấu trúc thưa H, hội tụ
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.8))
    X = o['Xslam']
    for kind, col, lab in [(0, '#7f7f7f', 'Cạnh odometry/ghép quét'), (2, COL['lc'], 'Cạnh đóng vòng')]:
        sel = o['KIND'] == kind
        for i_, j_ in zip(o['I'][sel], o['J'][sel]):
            ax[0].plot(X[[i_, j_], 0], X[[i_, j_], 1], '-', c=col, lw=0.7 if kind else 1.0)
        ax[0].plot([], [], '-', c=col, label=lab)
    ax[0].plot(X[:, 0], X[:, 1], '.', c=COL['slam'], ms=3, label=f'Nút tư thế ({len(X)})')
    ax[0].set_aspect('equal'); ax[0].legend(fontsize=8, loc='upper center'); ax[0].set_title('a) Đồ thị tư thế sau tối ưu')
    ax[0].set_xlabel('X [m]'); ax[0].set_ylabel('Y [m]')
    ax[1].spy(np.abs(o['H']) > 1e-9, markersize=0.3, color=COL['slam'])
    dens = 100 * np.mean(np.abs(o['H']) > 1e-9)
    ax[1].set_title(f'b) Cấu trúc thưa của H ({o["H"].shape[0]}×{o["H"].shape[1]}, {dens:.1f} % khác 0)')
    ch = o['chi2_hist']
    ax[2].semilogy(np.arange(len(ch)), ch, '-o', c=COL['slam'])
    ax[2].axhline(met['info']['dof'], ls='--', c='k', lw=1, label=f'bậc tự do ≈ {met["info"]["dof"]}')
    ax[2].set_xlabel('Vòng lặp Gauss-Newton'); ax[2].set_ylabel('F(x) = Σ eᵀΩe'); ax[2].legend(fontsize=8)
    ax[2].set_title('c) Hội tụ F(x) (khởi tạo bằng dead reckoning)')
    save(fig, 'fig6_do_thi_tu_the.png')

    # ---------------- FIG 7: Bản đồ lưới chiếm chỗ + overlay
    fig, ax = plt.subplots(1, 4, figsize=(17, 4.6))
    show_gt(ax[0], gt); ax[0].set_title('a) Ground truth')
    for a_, key in zip(ax[1:3], ['dr', 'slam']):
        show_occ(a_, maps[key]['P'], maps[key]['known'])
        a_.set_title(f'{"b" if key=="dr" else "c"}) {NAMES[key]}\nAcc={maps[key]["acc"]:.1f} %, IoU={maps[key]["iou"]:.1f} %')
    ms = maps['slam']; rgb = np.ones(gt.shape + (3,))
    tp = ms['occ'] & gt & ms['known']; fp = ms['occ'] & ~gt & ms['known']; fn = ~ms['occ'] & gt & ms['known']
    rgb[gt & ~ms['known']] = [0.85, 0.85, 0.85]
    rgb[tp] = [0.1, 0.7, 0.1]; rgb[fp] = [0.9, 0.1, 0.1]; rgb[fn] = [0.1, 0.3, 0.9]
    ax[3].imshow(rgb, origin='lower', extent=[0, 20, 0, 20], interpolation='nearest'); ax[3].set_aspect('equal')
    ax[3].set_title('d) Đối chiếu ô lưới SLAM\n(lục: TP, đỏ: FP, lam: FN, xám: chưa quan sát)', fontsize=9)
    save(fig, 'fig7_ban_do_luoi.png')

    # bản đồ No LC riêng (phụ lục)
    fig, ax = plt.subplots(1, 2, figsize=(10, 4.8))
    for a_, key in zip(ax, ['nolc', 'gtpose']):
        show_occ(a_, maps[key]['P'], maps[key]['known'])
        a_.set_title(f'{NAMES.get(key, "Dựng bằng tư thế thật (cận trên)")}\nAcc={maps[key]["acc"]:.1f} %, IoU={maps[key]["iou"]:.1f} %')
    save(fig, 'fig7b_ban_do_nolc_va_canTren.png')

    met['info']['H_density'] = float(dens)
    return o, met


# ============================================================================
def scenario2(M=20, base_seed=1000):
    print(f'[Kịch bản 2] Monte Carlo kiểm định nhất quán, M = {M}')
    N = None; NE = {k: [] for k in ['dr', 'nolc', 'slam']}; AT = {k: [] for k in NE}; IN = {k: [] for k in NE}
    for m in range(M):
        o = C.run_simulation(seed=base_seed + m)
        for key, X, P in methods(o):
            cs = C.consistency(X, P, o['Xtrue'])
            NE[key].append(cs['nees']); AT[key].append(C.ate_rmse(X, o['Xtrue']))
            IN[key].append([cs['in_x'], cs['in_y'], cs['in_t'], cs['in_ell']])
        print(f'  run {m+1}/{M}: LC={o["n_lc"]}, ATE dr/nolc/slam = {AT["dr"][-1]:.3f}/{AT["nolc"][-1]:.3f}/{AT["slam"][-1]:.3f}')
        sys.stdout.flush()
    lo, hi = chi2_ppf(0.025, 2 * M) / M, chi2_ppf(0.975, 2 * M) / M
    fig, ax = plt.subplots(1, 2, figsize=(12.5, 4.4))
    out = {}
    for key in NE:
        ne = np.mean(np.array(NE[key]), 0)
        ax[0].plot(np.arange(1, len(ne) + 1), ne, '-', c=COL[key], lw=1.1, label=NAMES[key])
        out[key] = dict(anees=float(ne.mean()), frac_in_band=float(100 * np.mean((ne >= lo) & (ne <= hi))),
                        ate_mean=float(np.mean(AT[key])), ate_std=float(np.std(AT[key])),
                        in_x=float(np.mean(IN[key], 0)[0]), in_y=float(np.mean(IN[key], 0)[1]),
                        in_t=float(np.mean(IN[key], 0)[2]), in_ell=float(np.mean(IN[key], 0)[3]))
    ax[0].axhline(2, c='k', lw=1, label='Giá trị kỳ vọng = 2 (bậc tự do)')
    ax[0].axhline(lo, c='k', ls='--', lw=1, label=f'Biên tin cậy 95 % [{lo:.2f}, {hi:.2f}]'); ax[0].axhline(hi, c='k', ls='--', lw=1)
    ax[0].set_xlabel('Bước k'); ax[0].set_ylabel('NEES trung bình $\\bar\\epsilon_k$'); ax[0].set_ylim(0, 6)
    ax[0].legend(fontsize=8); ax[0].set_title(f'a) NEES vị trí trung bình trên {M} lần chạy Monte Carlo')
    xs = np.arange(3); w = 0.2
    for i, (lab, idx) in enumerate([('x', 0), ('y', 1), ('θ', 2), ('elip 95 %', 3)]):
        ax[1].bar(xs + (i - 1.5) * w, [np.mean(IN[k], 0)[idx] for k in NE], w, label=lab)
    ax[1].axhline(95.4, c='k', ls='--', lw=1, label='Lý thuyết ±2σ: 95.4 %')
    ax[1].set_xticks(xs); ax[1].set_xticklabels([NAMES[k] for k in NE], fontsize=8.5)
    ax[1].set_ylim(50, 102); ax[1].set_ylabel('% mẫu sai số nằm trong biên'); ax[1].legend(fontsize=8, ncol=3, loc='lower center')
    ax[1].set_title('b) Tỷ lệ sai số nằm trong biên 95 %')
    save(fig, 'fig8_monte_carlo_nhat_quan.png')
    out['bounds'] = [lo, hi]; out['M'] = M
    # boxplot ATE
    fig, ax = plt.subplots(figsize=(6.5, 4))
    ax.boxplot([AT[k] for k in NE], tick_labels=[NAMES[k] for k in NE])
    ax.set_ylabel('ATE RMSE [m]'); ax.set_title(f'Phân bố ATE RMSE qua {M} lần Monte Carlo'); ax.set_yscale('log')
    save(fig, 'fig8b_boxplot_ate.png')
    return out


# ============================================================================
def scenario3(levels=(0.01, 0.03, 0.05, 0.08), M=6, base_seed=2000):
    print(f'[Kịch bản 3] Độ nhạy theo σ_r, M = {M} lần / mức')
    rows = []
    for sr in levels:
        R = []
        for m in range(M):
            o = C.run_simulation({'sigma_r': sr}, seed=base_seed + m)
            Ls = C.build_occupancy(o['Xslam'], o['scans'], o['angles'], o['cfg']['max_range'])
            Ln = C.build_occupancy(o['Xnolc'], o['scans'], o['angles'], o['cfg']['max_range'])
            ms, mn = C.map_metrics(Ls, o['gt']), C.map_metrics(Ln, o['gt'])
            cs = C.consistency(o['Xslam'], o['Pslam'], o['Xtrue'])
            R.append([o['n_lc'], C.ate_rmse(o['Xdr'], o['Xtrue']), C.ate_rmse(o['Xnolc'], o['Xtrue']),
                      C.ate_rmse(o['Xslam'], o['Xtrue']), mn['acc'], mn['iou'], ms['acc'], ms['iou'], cs['in_ell']])
        R = np.array(R); mu, sd = R.mean(0), R.std(0)
        rows.append(dict(sigma_r=sr, n_lc=mu[0], n_lc_sd=sd[0], ate_dr=mu[1], ate_nolc=mu[2], ate_nolc_sd=sd[2],
                         ate_slam=mu[3], ate_slam_sd=sd[3], acc_nolc=mu[4], iou_nolc=mu[5],
                         acc_slam=mu[6], iou_slam=mu[7], iou_slam_sd=sd[7], in_ell=mu[8]))
        print(f'  σ_r={sr:.2f}: LC={mu[0]:.0f}, ATE noLC={mu[2]:.3f}, SLAM={mu[3]:.3f}, Acc={mu[6]:.1f}, IoU={mu[7]:.1f}')
        sys.stdout.flush()
    s = np.array([r['sigma_r'] for r in rows]) * 100
    fig, ax = plt.subplots(1, 3, figsize=(15, 4.2))
    ax[0].errorbar(s, [r['ate_nolc'] for r in rows], [r['ate_nolc_sd'] for r in rows], fmt='-s', c=COL['nolc'], capsize=3, label=NAMES['nolc'])
    ax[0].errorbar(s, [r['ate_slam'] for r in rows], [r['ate_slam_sd'] for r in rows], fmt='-o', c=COL['slam'], capsize=3, label=NAMES['slam'])
    ax[0].axhline(0.15, c='k', ls=':', lw=1, label='Yêu cầu ATE < 0.15 m')
    ax[0].set_xlabel('$\\sigma_r$ [cm]'); ax[0].set_ylabel('ATE RMSE [m]'); ax[0].legend(fontsize=8); ax[0].set_title('a) Sai số quỹ đạo')
    ax[1].plot(s, [r['acc_nolc'] for r in rows], '-s', c=COL['nolc'], label='Acc - No LC')
    ax[1].plot(s, [r['acc_slam'] for r in rows], '-o', c=COL['slam'], label='Acc - SLAM')
    ax[1].plot(s, [r['iou_nolc'] for r in rows], '--s', c=COL['nolc'], label='IoU - No LC')
    ax[1].plot(s, [r['iou_slam'] for r in rows], '--o', c=COL['slam'], label='IoU - SLAM')
    ax[1].set_xlabel('$\\sigma_r$ [cm]'); ax[1].set_ylabel('[%]'); ax[1].legend(fontsize=8); ax[1].set_title('b) Chất lượng bản đồ')
    ax[2].bar(s, [r['n_lc'] for r in rows], width=1.2, color=COL['lc'])
    ax[2].set_xlabel('$\\sigma_r$ [cm]'); ax[2].set_ylabel('Số cạnh đóng vòng'); ax[2].set_title('c) Số đóng vòng được chấp nhận')
    save(fig, 'fig9_do_nhay_nhieu_lidar.png')
    return rows


# ============================================================================
if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('--quick', action='store_true'); a = ap.parse_args()
    T0 = time.time()
    o, met = scenario1()
    mc = scenario2(M=6 if a.quick else 20)
    sens = scenario3(M=2 if a.quick else 6)
    res = dict(scenario1=met, monte_carlo=mc, sensitivity=sens, cfg=C.DEFAULT, runtime_s=time.time() - T0)
    with open(os.path.join(RES_DIR, 'results.json'), 'w', encoding='utf-8') as f:
        json.dump(res, f, indent=2, ensure_ascii=False, default=float)
    # Bảng markdown
    L = ['# KẾT QUẢ MÔ PHỎNG (tự sinh bởi run_experiments.py)\n',
         '## Bảng 4.1 – So sánh 3 phương pháp (σ_r = 3 cm, 1 lần chạy đại diện)\n',
         '| Chỉ số | Dead reckoning | Không đóng vòng | Pose-Graph SLAM |', '|---|---|---|---|']
    rowsdef = [('ATE RMSE [m]', 'ate', '%.3f'), ('Sai số vị trí lớn nhất [m]', 'ate_max', '%.3f'),
               ('Sai số điểm cuối [m]', 'final', '%.3f'), ('RMSE hướng [độ]', 'head_rmse', '%.2f'),
               ('RPE Δ=1 tịnh tiến [m]', 'rpe1_t', '%.4f'), ('RPE Δ=20 tịnh tiến [m]', 'rpe20_t', '%.3f'),
               ('σ_pos cuối [m]', 'sig_final', '%.3f'), ('σ_pos lớn nhất [m]', 'sig_max', '%.3f'),
               ('% e_x trong ±2σ', 'in_x', '%.1f'), ('% e_y trong ±2σ', 'in_y', '%.1f'), ('% e_θ trong ±2σ', 'in_t', '%.1f'),
               ('% trong elip 95 %', 'in_ell', '%.1f'), ('ANEES (kỳ vọng 2)', 'anees', '%.2f'),
               ('Map Accuracy [%]', 'acc', '%.2f'), ('Obstacle IoU [%]', 'iou', '%.2f')]
    for lab, key, fmt in rowsdef:
        L.append(f'| {lab} | ' + ' | '.join(fmt % met[m][key] for m in ['dr', 'nolc', 'slam']) + ' |')
    L.append(f"\nBản đồ dựng bằng tư thế thật (cận trên): Acc = {met['gtpose']['acc']:.2f} %, IoU = {met['gtpose']['iou']:.2f} %")
    L.append(f"\nThông tin đồ thị: {json.dumps(met['info'], ensure_ascii=False)}\n")
    L += [f'## Bảng 4.2 – Monte Carlo (M = {mc["M"]})\n', '| Phương pháp | ATE TB ± ĐLC [m] | % x | % y | % θ | % elip | ANEES |', '|---|---|---|---|---|---|---|']
    for k in ['dr', 'nolc', 'slam']:
        r = mc[k]; L.append(f"| {NAMES[k]} | {r['ate_mean']:.3f} ± {r['ate_std']:.3f} | {r['in_x']:.1f} | {r['in_y']:.1f} | {r['in_t']:.1f} | {r['in_ell']:.1f} | {r['anees']:.2f} |")
    L.append(f"\nBiên 95 % của NEES trung bình: [{mc['bounds'][0]:.2f}, {mc['bounds'][1]:.2f}]\n")
    L += ['## Bảng 4.3 – Độ nhạy theo σ_r\n', '| σ_r [m] | Số LC | ATE DR | ATE No LC | ATE SLAM | Acc No LC | IoU No LC | Acc SLAM | IoU SLAM | % elip 95 % |', '|---|---|---|---|---|---|---|---|---|---|']
    for r in sens:
        L.append(f"| {r['sigma_r']:.2f} | {r['n_lc']:.0f} | {r['ate_dr']:.3f} | {r['ate_nolc']:.3f} ± {r['ate_nolc_sd']:.3f} | {r['ate_slam']:.3f} ± {r['ate_slam_sd']:.3f} | {r['acc_nolc']:.2f} | {r['iou_nolc']:.2f} | {r['acc_slam']:.2f} | {r['iou_slam']:.2f} | {r['in_ell']:.1f} |")
    L.append(f"\nTổng thời gian: {res['runtime_s']/60:.1f} phút")
    with open(os.path.join(RES_DIR, 'results_tables.md'), 'w', encoding='utf-8') as f:
        f.write('\n'.join(L))
    print('\n'.join(L))
