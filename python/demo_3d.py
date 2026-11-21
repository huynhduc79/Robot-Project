# -*- coding: utf-8 -*-
"""
demo_3d.py - DEMO 3D HAI KHUNG (PyVista / VTK)

  KHUNG TRÁI  - GROUND TRUTH : nhà xưởng thật (tường, kệ A/B/C, máy CNC D, trụ), robot chạy theo
                               tư thế THẬT (quỹ đạo Simulink sl_drivepoint), tia LiDAR quét.
  KHUNG PHẢI  - PHÒNG TRỐNG  : robot được thả vào một sàn trống, chỉ biết odometry + LiDAR.
                               Robot đặt theo tư thế ƯỚC LƯỢNG của Pose-Graph SLAM; bản đồ lưới chiếm chỗ
                               (log-odds hits/misses, giống báo cáo) hiện dần trên sàn: trắng = trống,
                               xám = chưa biết, ô vật cản được dựng thành khối 3D.
  Hai khung dùng chung camera (xoay một bên, bên kia xoay theo).

Dữ liệu: phát lại kết quả MATLAB (matlab/results_matlab/csv) nên mọi con số trùng báo cáo.

Chế độ:
  --mode live   (mặc định) vừa chạy vừa tạo map theo tư thế SLAM online; khi kết thúc dựng lại map
                theo quỹ đạo đã tối ưu batch và hiện Map Accuracy / IoU như báo cáo.
  --mode after  robot chạy hết vòng trước (chỉ vẽ quỹ đạo), sau đó map được tạo từng scan theo
                quỹ đạo SLAM đã tối ưu (đúng cách dựng Hình 4.6 của báo cáo).
Tùy chọn: --follow (camera bám robot), --save (xuất GIF + ảnh, không mở cửa sổ)

Chạy:   pip install pyvista imageio
        python demo_3d.py
        python demo_3d.py --mode after
"""
import os, sys, argparse
import numpy as np
try:
    sys.stdout.reconfigure(encoding='utf-8')
except Exception:
    pass
import pyvista as pv
import pgslam_core as C            # dùng chung hàm dựng bản đồ log-odds và chỉ số Acc/IoU

ap = argparse.ArgumentParser()
ap.add_argument('--mode', choices=['live', 'after'], default='live')
ap.add_argument('--save', action='store_true'); ap.add_argument('--follow', action='store_true')
ap.add_argument('--sub', type=int, default=None, help='số khung nội suy giữa 2 keyframe')
ap.add_argument('--fps', type=int, default=20)
ap.add_argument('--test', type=int, default=0, help=argparse.SUPPRESS)   # chạy thử N khung rồi tự đóng
args = ap.parse_args()
if args.sub is None:
    args.sub = 1 if args.save else 3

HERE = os.path.dirname(os.path.abspath(__file__))
CSV = os.path.join(HERE, '..', 'matlab', 'results_matlab', 'csv')
if not os.path.isdir(CSV):
    sys.exit('Chưa có dữ liệu. Hãy chạy main_pose_graph_slam.m trong MATLAB trước (tạo matlab/results_matlab/csv).')
rd = lambda f: np.loadtxt(os.path.join(CSV, f), delimiter=',', ndmin=2)
OBS = rd('obstacles.csv'); DEN = rd('traj_dense.csv'); POS = rd('poses.csv'); ONL = rd('online.csv')
SC = rd('scans.csv'); ANG = SC[0]; SCANS = SC[1:].copy(); SCANS[SCANS < 0] = np.nan
try:
    LC = rd('loop_closures.csv').astype(int)
except Exception:
    LC = np.zeros((0, 2), int)
N = len(POS)
Xt, Xdr = POS[:, 1:4], POS[:, 4:7]
Xsl, KF = POS[:, 18:21], POS[:, 25].astype(int) - 1
Xon, Pon = ONL[:, 1:4], ONL[:, 4:8]
td, Xd = DEN[:, 0], DEN[:, 1:4]
lc_at = {}
for k, j in LC:
    lc_at.setdefault(int(k), []).append(int(j))
MAXR, RES, NG, LIDAR_Z, WALL_H = 12.0, 0.05, 400, 0.42, 1.0
GT = C.make_gt_map()
FONT = next((f for f in ['C:/Windows/Fonts/arial.ttf', 'C:/Windows/Fonts/segoeui.ttf',
                         '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'] if os.path.exists(f)), None)
FK = {'font_file': FONT} if FONT else {}

pl = pv.Plotter(shape=(1, 2), off_screen=args.save, border=True,
                window_size=(1280, 560) if args.save else (1700, 800))


# ------------------------------------------------------------------ tiện ích
def polyline(P, z):
    P = np.asarray(P)[:, :2]
    if len(P) < 2:
        P = np.vstack([P, P])
    return pv.lines_from_points(np.c_[P, np.full(len(P), z)])


def ellipse(mu, pxx, pxy, pyy, z, n=48):
    w, V = np.linalg.eigh(np.array([[pxx, pxy], [pxy, pyy]])); w = np.maximum(w, 0)
    t = np.linspace(0, 2 * np.pi, n); E = V @ np.diag(np.sqrt(5.991 * w)) @ np.vstack([np.cos(t), np.sin(t)])
    return polyline(np.c_[mu[0] + E[0], mu[1] + E[1]], z)


def robot_meshes():
    return [(pv.Box(bounds=(-0.35, 0.35, -0.25, 0.25, 0.08, 0.34)), '#1f77b4'),
            (pv.Cone(center=(0.42, 0, 0.22), direction=(1, 0, 0), height=0.15, radius=0.1), '#ffcc00'),
            (pv.Cylinder(center=(0.1, 0, LIDAR_Z - 0.04), direction=(0, 0, 1), radius=0.08, height=0.1), '#222222')] + \
           [(pv.Cylinder(center=(sx, sy, 0.08), direction=(0, 1, 0), radius=0.08, height=0.06), '#111111')
            for sx in (-0.22, 0.22) for sy in (-0.27, 0.27)]


def pose_matrix(x, y, th):
    c, s = np.cos(th), np.sin(th)
    return np.array([[c, -s, 0, x], [s, c, 0, y], [0, 0, 1, 0], [0, 0, 0, 1.0]])


def scan_world(k, pose, step=1):
    r = SCANS[k]; m = np.isfinite(r); a = ANG[m] + pose[2]
    return np.c_[pose[0] + r[m] * np.cos(a), pose[1] + r[m] * np.sin(a), np.full(m.sum(), LIDAR_Z)][::step]


def rays_mesh(origin_pose, hits):
    o = np.array([origin_pose[0], origin_pose[1], LIDAR_Z]); n = len(hits)
    if n == 0:
        return pv.PolyData(np.zeros((2, 3)), lines=np.array([2, 0, 1]))
    lines = np.c_[np.full(n, 2), np.zeros(n, int), np.arange(1, n + 1)].ravel()
    return pv.PolyData(np.vstack([o, hits]), lines=lines)


# ================================================================== KHUNG TRÁI: GROUND TRUTH
pl.subplot(0, 0)
pl.set_background('#e6ecf2', top='#ffffff')
pl.add_mesh(pv.Plane(center=(10, 10, 0), i_size=20, j_size=20, i_resolution=40, j_resolution=40),
            color='#c9c9c4', show_edges=True, edge_color='#b3b3ad', line_width=0.5)
for i, (x0, x1, y0, y1) in enumerate(OBS):
    if i < 4:   h, col, op = 2.5, '#8f9aa6', 0.35
    elif i in (4, 6, 8): h, col, op = 2.2, '#e0a33a', 1.0
    elif i in (5, 7, 9): h, col, op = 1.6, '#c98a2c', 1.0
    elif i == 10: h, col, op = 1.4, '#4f7fb3', 1.0
    else: h, col, op = 3.0, '#6b6f75', 1.0
    pl.add_mesh(pv.Box(bounds=(x0, x1, y0, y1, 0, h)), color=col, opacity=op)
    if i in (4, 6, 8):
        for zz in (0.7, 1.4):
            pl.add_mesh(pv.Box(bounds=(x0 - .02, x1 + .02, y0 - .02, y1 + .02, zz, zz + 0.05)), color='#7a5a20')
for lab, (x, y) in {'Kệ A': (6.5, 6.4), 'Kệ B': (13.5, 6.5), 'Kệ C': (6.5, 13.6), 'CNC D': (13.6, 13.6)}.items():
    pl.add_point_labels([[x, y, 2.6]], [lab], font_size=13, point_size=1, shape_opacity=0.3, always_visible=True, **FK)
robotL = [pl.add_mesh(m, color=c) for m, c in robot_meshes()]
mL_path = polyline(Xt[:1], 0.03); pl.add_mesh(mL_path, color='black', line_width=3)
mL_rays = rays_mesh(Xt[0], scan_world(0, Xt[0])); pl.add_mesh(mL_rays, color='#ff3b3b', opacity=0.25, line_width=1)
mL_hits = pv.PolyData(scan_world(0, Xt[0])); pl.add_mesh(mL_hits, color='#ff2020', point_size=6, render_points_as_spheres=True)
pl.add_text('GROUND TRUTH (nhà xưởng thật)', position='upper_edge', font_size=10, color='black', **FK)
hudL = pl.add_text('', position='lower_left', font_size=10, color='black', **FK)
pl.add_text('Khung phải - Xanh: SLAM | Đỏ: dead reckoning\nLục: elip 95% | Hồng: đóng vòng | Khối: ô vật cản (P ≥ 0.65)',
            position='lower_right', font_size=8, color='#333333', **FK)

# ================================================================== KHUNG PHẢI: PHÒNG TRỐNG + TẠO MAP
pl.subplot(0, 1)
pl.set_background('#e6ecf2', top='#ffffff')
grid = pv.ImageData(dimensions=(NG + 1, NG + 1, 1), spacing=(RES, RES, 1), origin=(0, 0, 0.005))
grid.cell_data['P'] = np.full(NG * NG, 0.5)             # 0.5 = chưa biết
pl.add_mesh(grid, scalars='P', cmap='gray_r', clim=[0, 1], show_scalar_bar=False, name='floor')
pl.add_mesh(pv.Box(bounds=(0, 20, 0, 20, 0, 0.001)).extract_feature_edges(), color='#888888', line_width=1)  # chỉ khung phòng
robotR = [pl.add_mesh(m, color=c) for m, c in robot_meshes()]
mR_sl = polyline(Xon[:1], 0.04); pl.add_mesh(mR_sl, color='#1f4fd6', line_width=4)
mR_dr = polyline(Xdr[:1], 0.03); pl.add_mesh(mR_dr, color='#d62728', line_width=2, opacity=0.8)
mR_ell = ellipse(Xon[0], *Pon[0, :3], 0.06); pl.add_mesh(mR_ell, color='#19a319', line_width=4)
mR_rays = rays_mesh(Xon[0], scan_world(0, Xon[0])); pl.add_mesh(mR_rays, color='#ff8c00', opacity=0.2, line_width=1)
mR_lc = pv.PolyData(np.zeros((2, 3)), lines=np.array([2, 0, 1]))
lc_actor = pl.add_mesh(mR_lc, color='#ff00cc', line_width=7); lc_actor.visibility = False
pl.add_text('PHÒNG TRỐNG: robot tự tạo bản đồ (SLAM)', position='upper_edge', font_size=10, color='black', **FK)
hudR = pl.add_text('', position='lower_left', font_size=10, color='black', **FK)
pl.link_views()
pl.camera_position = [(10, -15, 21), (10, 9, 0), (0, 0, 1)]

L = np.zeros((NG, NG))


def show_map(final_text=None):
    P = 1.0 / (1.0 + np.exp(-L))
    grid.cell_data['P'] = P.ravel()           # chỉ số ô: ix + NG*iy (khớp L[iy, ix])
    occ = P >= 0.65
    iy, ix = np.nonzero(occ)
    pl.subplot(0, 1)
    if len(ix):
        cubes = pv.PolyData(np.c_[(ix + 0.5) * RES, (iy + 0.5) * RES, np.full(len(ix), WALL_H / 2)])
        g = cubes.glyph(geom=pv.Cube(x_length=RES, y_length=RES, z_length=WALL_H), orient=False, scale=False)
        pl.add_mesh(g, color='#3a3f47', name='walls')
    if final_text:
        m = C.map_metrics(L, GT)
        hudR.SetText(0, f'{final_text}\nMap Accuracy = {m["acc"]:.2f} %   Obstacle IoU = {m["iou"]:.1f} %')


# ------------------------------------------------------------------ dãy khung hình
frames = []
for k in range(N - 1):
    i0, i1 = KF[k], KF[k + 1]
    frames += [('run', k, int(round(i0 + (i1 - i0) * s / args.sub))) for s in range(args.sub)]
frames.append(('run', N - 1, KF[-1]))
if args.mode == 'after':
    frames += [('build', k, KF[-1]) for k in range(N)]       # dựng map từng scan sau khi chạy xong
frames += [('final', N - 1, KF[-1])] + [('hold', N - 1, KF[-1])] * 25
state = {'f': 0, 'last_k': -1, 'lc_ttl': 0}
nlc_total = sum(len(v) for v in lc_at.values())


def step(*_):
    if state['f'] >= len(frames):
        return
    kind, k, i = frames[state['f']]; state['f'] += 1
    if kind == 'run':
        x, y, th = Xd[i]
        for a in robotL:
            a.user_matrix = pose_matrix(x, y, th)
        dx = Xon[k] - Xt[k]           # khung phải: tư thế ước lượng = tư thế thật + sai số SLAM hiện tại
        for a in robotR:
            a.user_matrix = pose_matrix(x + dx[0], y + dx[1], th + dx[2])
        if k != state['last_k']:
            state['last_k'] = k
            hitsT = scan_world(k, Xt[k]); hitsE = scan_world(k, Xon[k])
            mL_path.copy_from(polyline(Xt[:k + 1], 0.03))
            mL_rays.copy_from(rays_mesh(Xt[k], hitsT)); mL_hits.copy_from(pv.PolyData(hitsT))
            mR_sl.copy_from(polyline(Xon[:k + 1], 0.04)); mR_dr.copy_from(polyline(Xdr[:k + 1], 0.03))
            mR_ell.copy_from(ellipse(Xon[k], *Pon[k, :3], 0.06)); mR_rays.copy_from(rays_mesh(Xon[k], hitsE))
            if args.mode == 'live':
                C.insert_scan(L, Xon[k], SCANS[k], ANG, MAXR); show_map()
            if k in lc_at:
                j = lc_at[k][0]
                mR_lc.copy_from(pv.lines_from_points(np.array([[Xon[k, 0], Xon[k, 1], 0.6], [Xsl[j, 0], Xsl[j, 1], 0.6]])))
                lc_actor.visibility = True; state['lc_ttl'] = 2 * args.sub
            nlc = sum(len(v) for kk, v in lc_at.items() if kk <= k)
            eD = np.hypot(*(Xdr[k, :2] - Xt[k, :2])); eS = np.hypot(*(Xon[k, :2] - Xt[k, :2]))
            hudL.SetText(0, f'Nút k = {k}/{N-1}    t = {td[i]:.1f} s    quét LiDAR 240 tia')
            hudR.SetText(0, f'Sai số SLAM = {eS:.3f} m   (dead reckoning = {eD:.2f} m)\nSố cạnh đóng vòng = {nlc}'
                            + ('    >>> ĐÓNG VÒNG! <<<' if k in lc_at else '')
                            + ('\nĐang tạo bản đồ trong khi chạy' if args.mode == 'live' else '\nĐang khảo sát (bản đồ sẽ dựng sau khi chạy xong)'))
        if state['lc_ttl'] > 0:
            state['lc_ttl'] -= 1
            lc_actor.visibility = state['lc_ttl'] > 0
        if args.follow:
            for sp in ((0, 0), (0, 1)):
                pl.subplot(*sp)
                pl.camera.position = (x - 6 * np.cos(th), y - 6 * np.sin(th), 6)
                pl.camera.focal_point = (x + 2 * np.cos(th), y + 2 * np.sin(th), 0.3); pl.camera.up = (0, 0, 1)
    elif kind == 'build':                                       # mode after: dựng map từng scan theo quỹ đạo tối ưu
        C.insert_scan(L, Xsl[k], SCANS[k], ANG, MAXR)
        mR_rays.copy_from(rays_mesh(Xsl[k], scan_world(k, Xsl[k])))
        for a in robotR:
            a.user_matrix = pose_matrix(*Xsl[k])
        if k % 3 == 0 or k == N - 1:
            show_map()
        hudR.SetText(0, f'Dựng bản đồ từ quỹ đạo SLAM đã tối ưu: scan {k + 1}/{N}')
    elif kind == 'final':
        pl.subplot(0, 1)
        pl.add_mesh(polyline(Xsl, 0.07), color='#00b7ff', line_width=5, name='final')
        lc_actor.visibility = False
        L[:] = 0                                               # bản đồ cuối = theo quỹ đạo tối ưu batch (như Hình 4.6)
        for q in range(N):
            C.insert_scan(L, Xsl[q], SCANS[q], ANG, MAXR)
        show_map(f'HOÀN THÀNH: bản đồ theo quỹ đạo SLAM tối ưu ({nlc_total} đóng vòng)')
    pl.render()


if args.save:
    out = os.path.join(HERE, '..', 'figures', f'demo_3d_{args.mode}.gif')
    pl.open_gif(out, fps=12, subrectangles=True)
    mid = int(0.45 * N * args.sub)
    for n_ in range(len(frames)):
        step(); pl.write_frame()
        if n_ == mid:
            pl.screenshot(os.path.join(HERE, '..', 'figures', f'demo_3d_{args.mode}_mid.png'))
        if frames[n_][0] == 'final':
            pl.screenshot(os.path.join(HERE, '..', 'figures', f'demo_3d_{args.mode}_final.png'))
    pl.close(); print('Đã lưu', out)
else:
    import time
    print(f'Chế độ {args.mode}: {N} keyframe, {len(frames)} khung hình. Chuột: xoay/zoom (2 khung đồng bộ), '
          f'phím SPACE: tạm dừng/tiếp tục, đóng cửa sổ để thoát.')
    paused = {'v': False}
    pl.add_key_event('space', lambda: paused.__setitem__('v', not paused['v']))
    pl.show(interactive_update=True, auto_close=False)       # mở cửa sổ, không chặn chương trình
    dt = 1.0 / args.fps

    closed = {'v': False}

    def on_exit(*_):                                          # người dùng bấm X / phím q
        closed['v'] = True
    pl.iren.add_observer('ExitEvent', on_exit)

    def alive():
        return not closed['v'] and getattr(pl, 'render_window', None) is not None

    def safe_update(ms, redraw):
        try:
            pl.update(ms, force_redraw=redraw)
        except Exception:
            closed['v'] = True

    n_done = 0
    while alive() and state['f'] < len(frames):
        t0 = time.time()
        if not paused['v']:
            step(); n_done += 1
            if args.test and n_done >= args.test:
                break
        safe_update(1, True)                                  # xử lý chuột/bàn phím + vẽ lại
        time.sleep(max(0.0, dt - (time.time() - t0)))
    print(f'Đã chạy {n_done} khung hình.' + ('' if args.test else ' Đóng cửa sổ để thoát.'))
    if args.test:
        pl.screenshot(os.path.join(os.environ.get('TEMP', HERE), 'demo_3d_test.png'))
        pl.iren.interactor.InvokeEvent('ExitEvent')           # mô phỏng thao tác bấm X để kiểm tra
    while alive():                                            # giữ cửa sổ để xem bản đồ cuối, xoay/zoom tự do
        safe_update(30, False); time.sleep(0.03)
    # thoát gọn: tắt log cảnh báo OpenGL của VTK rồi mới đóng cửa sổ
    try:
        from vtkmodules.vtkCommonCore import vtkObject, vtkLogger
        vtkObject.GlobalWarningDisplayOff(); vtkLogger.SetStderrVerbosity(vtkLogger.VERBOSITY_OFF)
    except Exception:
        pass
    try:
        pl.close()
    except Exception:
        pass
    print('Đã đóng demo.')
