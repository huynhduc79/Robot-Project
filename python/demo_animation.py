# -*- coding: utf-8 -*-
"""
demo_animation.py - DEMO ĐỒ HỌA ĐỘNG: robot tuần tra nhà xưởng, LiDAR quét, Pose-Graph SLAM
đóng vòng và "nắn" lại quỹ đạo; elip 95 % và hai đường biên ±2σ cập nhật theo thời gian thực.

Chạy:
    python demo_animation.py              # mở cửa sổ animation (phím SPACE: tạm dừng/tiếp tục)
    python demo_animation.py --save       # lưu thêm ../figures/demo_slam.gif
    python demo_animation.py --sigma_r 0.05 --seed 3
"""
import os, sys, argparse
import numpy as np
import matplotlib
import pgslam_core as C
try:
    sys.stdout.reconfigure(encoding="utf-8"); sys.stderr.reconfigure(encoding="utf-8")
except Exception:
    pass

ap = argparse.ArgumentParser()
ap.add_argument('--save', action='store_true'); ap.add_argument('--seed', type=int, default=7)
ap.add_argument('--sigma_r', type=float, default=0.03); ap.add_argument('--every', type=int, default=1)
args = ap.parse_args()
if args.save:
    matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.colors import ListedColormap

print('Đang chạy mô phỏng SLAM (khoảng 10-20 s)...')
o = C.run_simulation({'sigma_r': args.sigma_r}, seed=args.seed, keep_history=True, verbose=True)
gt, Xt, Xdr, Pdr = o['gt'], o['Xtrue'], o['Xdr'], o['Pdr']
H = o['hist']; N = len(Xt); ang = o['angles']; RMAX = o['cfg']['max_range']
lc_by_k = {}
for k, j in H['lc_events']:
    lc_by_k.setdefault(k, []).append(j)

fig = plt.figure(figsize=(14, 7.2))
gs = fig.add_gridspec(2, 2, width_ratios=[1.15, 1], height_ratios=[1.25, 1])
axW = fig.add_subplot(gs[:, 0]); axM = fig.add_subplot(gs[0, 1]); axE = fig.add_subplot(gs[1, 1])
axW.imshow(gt, origin='lower', extent=[0, 20, 0, 20], cmap=ListedColormap(['white', '#9a9a9a']), interpolation='nearest')
axW.set_xlim(0, 20); axW.set_ylim(0, 20); axW.set_aspect('equal'); axW.set_xlabel('X [m]'); axW.set_ylabel('Y [m]')
lTrue, = axW.plot([], [], '-k', lw=1.4, label='Quỹ đạo thực')
lDR, = axW.plot([], [], ':', c='#d62728', lw=1.4, label='Dead reckoning (odometry)')
lSL, = axW.plot([], [], '-', c='#1f77b4', lw=1.8, label='Pose-Graph SLAM')
eDR, = axW.plot([], [], '-', c='#d62728', lw=1.0)
eSL, = axW.plot([], [], '-', c='g', lw=1.4, label='Elip 95 % SLAM')
pts, = axW.plot([], [], '.', c='orange', ms=2.5, label='Điểm LiDAR (theo tư thế ước lượng)')
rob, = axW.plot([], [], 'o', c='#1f77b4', ms=7)
lcl = []
axW.legend(loc='upper center', fontsize=7.5, ncol=2, framealpha=0.85)
ttl = axW.set_title('')

L = np.zeros(gt.shape)
img = axM.imshow(np.full(gt.shape, 0.5), origin='lower', extent=[0, 20, 0, 20], cmap='gray', vmin=0, vmax=1, interpolation='nearest')
axM.set_title('Bản đồ lưới chiếm chỗ đang xây dựng (log-odds)', fontsize=9); axM.set_xticks([]); axM.set_yticks([])

ks = np.arange(N)
axE.set_xlim(0, N); axE.set_xlabel('Bước k'); axE.set_ylabel('Sai số x [m]'); axE.grid(alpha=.3)
eDRline, = axE.plot([], [], ':', c='#d62728', lw=1.2, label='sai số x - DR')
bDRu, = axE.plot([], [], '--', c='#d62728', lw=0.8, alpha=.6); bDRl, = axE.plot([], [], '--', c='#d62728', lw=0.8, alpha=.6)
eSLline, = axE.plot([], [], '-', c='#1f77b4', lw=1.3, label='sai số x - SLAM (online)')
bSLu, = axE.plot([], [], '--k', lw=1.0, label='biên ±2σ SLAM (95 %)'); bSLl, = axE.plot([], [], '--k', lw=1.0)
axE.legend(fontsize=7.5, loc='upper left'); axE.set_title('Sai số và hai đường biên tin cậy 95 %', fontsize=9)
sDRx = 2 * np.sqrt(Pdr[:, 0, 0]); ymax = max(0.3, 1.1 * np.max(sDRx))
axE.set_ylim(-ymax, ymax)

err_sl = np.zeros(N); sig_sl = np.zeros(N); state = dict(pending=False, last_rebuild=0, paused=False)


def rebuild(k):
    L[:] = 0
    Xo = H['Xonline'][k - 1]
    for i in range(k + 1):
        C.insert_scan(L, Xo[i], o['scans'][i], ang, RMAX)


def update(k):
    if k == 0:
        return []
    Xo = H['Xonline'][k - 1]; Pk = H['Pnode'][k - 1]
    lTrue.set_data(Xt[:k + 1, 0], Xt[:k + 1, 1])
    lDR.set_data(Xdr[:k + 1, 0], Xdr[:k + 1, 1])
    lSL.set_data(Xo[:, 0], Xo[:, 1])
    ex, ey = C.ellipse_pts(Xdr[k, :2], Pdr[k, :2, :2]); eDR.set_data(ex, ey)
    ex, ey = C.ellipse_pts(Xo[k, :2], Pk[:2, :2]); eSL.set_data(ex, ey)
    r = o['scans'][k]; m = np.isfinite(r); a = ang[m] + Xo[k, 2]
    pts.set_data(Xo[k, 0] + r[m] * np.cos(a), Xo[k, 1] + r[m] * np.sin(a))
    rob.set_data([Xo[k, 0]], [Xo[k, 1]])
    for ln in lcl:
        ln.remove()
    lcl.clear()
    if k in lc_by_k:
        for j in lc_by_k[k]:
            lcl.append(axW.plot([Xo[j, 0], Xo[k, 0]], [Xo[j, 1], Xo[k, 1]], '-', c='m', lw=2.2)[0])
        state['pending'] = True
    # bản đồ: chèn quét mới; khi có đóng vòng thì dựng lại theo tư thế đã tối ưu
    if state['pending'] and k - state['last_rebuild'] >= 10:
        rebuild(k); state['pending'] = False; state['last_rebuild'] = k
    else:
        C.insert_scan(L, Xo[k], o['scans'][k], ang, RMAX)
    img.set_data(1 - 1 / (1 + np.exp(-L)))
    err_sl[k] = Xo[k, 0] - Xt[k, 0]; sig_sl[k] = 2 * np.sqrt(Pk[0, 0])
    eDRline.set_data(ks[:k + 1], Xdr[:k + 1, 0] - Xt[:k + 1, 0])
    bDRu.set_data(ks[:k + 1], sDRx[:k + 1]); bDRl.set_data(ks[:k + 1], -sDRx[:k + 1])
    eSLline.set_data(ks[:k + 1], err_sl[:k + 1])
    bSLu.set_data(ks[:k + 1], sig_sl[:k + 1]); bSLl.set_data(ks[:k + 1], -sig_sl[:k + 1])
    eD = np.hypot(*(Xdr[k, :2] - Xt[k, :2])); eS = np.hypot(*(Xo[k, :2] - Xt[k, :2]))
    nlc = sum(len(v) for kk, v in lc_by_k.items() if kk <= k)
    ttl.set_text(f'Bước k = {k}/{N-1}   |   sai số DR = {eD:.2f} m   |   sai số SLAM = {eS:.3f} m   |   số đóng vòng = {nlc}'
                 + ('   ◀ ĐÓNG VÒNG!' if k in lc_by_k else ''))
    ttl.set_fontsize(9)
    return []


def on_key(ev):
    if ev.key == ' ':
        state['paused'] = not state['paused']
        (anim.event_source.stop if state['paused'] else anim.event_source.start)()


frames = list(range(0, N, args.every)) + [N - 1] * 15
anim = FuncAnimation(fig, update, frames=frames, interval=60, blit=False, repeat=False)
fig.tight_layout()
if args.save:
    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'figures', 'demo_slam.gif')
    print('Đang lưu GIF ...'); anim.save(out, writer=PillowWriter(fps=12), dpi=70); print('Đã lưu', out)
else:
    fig.canvas.mpl_connect('key_press_event', on_key)
    plt.show()
