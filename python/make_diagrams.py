# -*- coding: utf-8 -*-
"""Vẽ các sơ đồ minh họa cho báo cáo: lưu đồ hệ thống (Hình 3.1) và khái niệm pose-graph (Hình 2.2)."""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch, Ellipse

FIG = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'figures')
os.makedirs(FIG, exist_ok=True)


def box(ax, x, y, w, h, txt, fc):
    ax.add_patch(FancyBboxPatch((x - w / 2, y - h / 2), w, h, boxstyle='round,pad=0.02,rounding_size=0.08',
                                fc=fc, ec='#333333', lw=1.1))
    ax.text(x, y, txt, ha='center', va='center', fontsize=8.6, wrap=True)


def arrow(ax, p, q, txt=None, c='#333333'):
    ax.add_patch(FancyArrowPatch(p, q, arrowstyle='-|>', mutation_scale=12, color=c, lw=1.2))
    if txt:
        ax.text((p[0] + q[0]) / 2 + 0.05, (p[1] + q[1]) / 2 + 0.08, txt, fontsize=7.5, color=c)


# ---------------- Hình 3.1: lưu đồ hệ thống
fig, ax = plt.subplots(figsize=(11, 6.2)); ax.set_xlim(0, 11); ax.set_ylim(0, 6.4); ax.axis('off')
G, B, O, R, P = '#e8f4e8', '#e6f0fa', '#fff1e0', '#fde8e8', '#f1e8fa'
box(ax, 1.4, 5.6, 2.5, 0.8, 'B1. Bản đồ chuẩn $M_{true}$\n20×20 m, ô 5 cm\n(tường, kệ A–D, trụ)', G)
box(ax, 1.4, 4.3, 2.5, 0.8, 'B1. Quỹ đạo thật $X_{true}$: Simulink\nsl_drivepoint (Corke 2023)\n11 waypoint → keyframe', G)
box(ax, 4.4, 5.6, 2.6, 0.8, 'B2. Odometry có nhiễu\n$\\hat\\delta_k=\\delta_k+v_k$, $v\\sim N(0,V)$', B)
box(ax, 4.4, 4.3, 2.6, 0.8, 'B2. LiDAR 2D có nhiễu\n240 tia, $\\sigma_r$, $\\sigma_\\beta$ (ray casting)', B)
box(ax, 7.6, 5.6, 2.8, 0.8, '(A) Dead reckoning + EKF\n$P_{k+1}=F_xPF_x^T+F_vVF_v^T$', R)
box(ax, 7.6, 4.3, 2.8, 0.8, 'B3. Front-end\nICP điểm–đường liên tiếp\n+ phát hiện đóng vòng (r < 3 m)', O)
box(ax, 7.6, 2.9, 2.8, 0.8, 'B3. Back-end: Gauss–Newton\n$H\\Delta x=-b$, neo nút 1\n$\\Sigma = H^{-1}$', O)
box(ax, 4.4, 2.9, 2.6, 0.8, '(B) Không đóng vòng\n(chỉ cạnh tuần tự)\n(C) Pose-Graph SLAM', P)
box(ax, 1.4, 2.9, 2.5, 0.8, 'B4. Bản đồ lưới log-odds\nhits / misses từ $\\hat X$', G)
box(ax, 3.2, 1.3, 6.0, 1.05, 'B5. Đánh giá định lượng so với ground truth:\nATE, RPE, sai số $x,y,\\theta$ với biên ±2σ (95 %), elip 95 % ($\\chi^2_2=5.991$)\n'
    'NEES/ANEES (Monte Carlo), Map Accuracy, Obstacle IoU, độ nhạy theo $\\sigma_r$', '#f5f5f5')
box(ax, 8.9, 1.3, 2.6, 0.9, 'Hình, bảng (MATLAB)\n+ CSV → demo 3D\n(Python, PyVista)', '#ffffff')
arrow(ax, (1.4, 5.2), (1.4, 4.7)); arrow(ax, (2.65, 4.3), (3.1, 5.4)); arrow(ax, (2.65, 4.3), (3.1, 4.3))
arrow(ax, (5.7, 5.6), (6.2, 5.6)); arrow(ax, (5.7, 4.3), (6.2, 4.3)); arrow(ax, (5.7, 5.5), (6.2, 4.5))
arrow(ax, (7.6, 3.9), (7.6, 3.3), 'đồ thị tư thế'); arrow(ax, (6.2, 2.9), (5.7, 2.9)); arrow(ax, (3.1, 2.9), (2.65, 2.9))
arrow(ax, (1.4, 2.5), (1.6, 1.75)); arrow(ax, (4.4, 2.5), (4.0, 1.75)); arrow(ax, (9.0, 5.45), (9.9, 1.75))
arrow(ax, (6.2, 1.3), (7.6, 1.3))
ax.set_title('Lưu đồ quy trình mô phỏng và đánh giá LiDAR Pose-Graph SLAM', fontsize=11)
fig.savefig(os.path.join(FIG, 'fig_luu_do_he_thong.png'), dpi=200, bbox_inches='tight'); plt.close(fig)

# ---------------- Hình 2.2: khái niệm pose-graph (Corke Fig. 6.29 - tự vẽ lại)
fig, ax = plt.subplots(1, 2, figsize=(11, 4.4))
tr = np.array([[0, 0], [4, 0.3], [4.5, 4], [0.6, 4.6], [0.3, 0.6]])
est = tr + np.array([[0, 0], [0.1, 0.2], [0.5, 0.5], [0.9, 0.9], [1.3, 1.0]])
for a, X, t in [(ax[0], est, 'a) Trước tối ưu: sai số odometry tích lũy'), (ax[1], tr, 'b) Sau tối ưu: sai số được phân bổ trên toàn đồ thị')]:
    for i in range(len(X) - 1):
        a.annotate('', X[i + 1], X[i], arrowprops=dict(arrowstyle='-|>', color='#1f77b4', lw=1.6))
        a.text(*(X[i] + X[i + 1]) / 2 + [0.1, 0.1], f'$z_{{{i+1},{i+2}}}$', color='#1f77b4', fontsize=9)
    a.plot([X[-1, 0], X[0, 0]], [X[-1, 1], X[0, 1]], '--', c='#e377c2', lw=2)
    a.text(*(X[-1] + X[0]) / 2 + [0.15, 0], '$z_{5,1}$ (đóng vòng)', color='#c2185b', fontsize=9)
    for i, p in enumerate(X):
        s = 0.25 + 0.18 * i if a is ax[0] else 0.3
        a.add_patch(Ellipse(p, 2 * s, 1.4 * s, angle=30, fc='none', ec='g', lw=0.8, ls=':'))
        a.plot(*p, 'o', ms=13, mfc='white', mec='#1f77b4', mew=1.6); a.text(*p, f'{i+1}', ha='center', va='center', fontsize=9)
    a.set_aspect('equal'); a.set_xlim(-1.2, 6.8); a.set_ylim(-1.2, 6.5); a.axis('off'); a.set_title(t, fontsize=10)
ax[0].text(-1, -1.1, 'Nút = tư thế $x_i$; mũi tên = ràng buộc odometry/ghép quét $(z_{ij},\\Omega_{ij})$; '
           'nét đứt = ràng buộc đóng vòng; elip = độ bất định', fontsize=8)
fig.savefig(os.path.join(FIG, 'fig_khai_niem_pose_graph.png'), dpi=200, bbox_inches='tight'); plt.close(fig)
print('OK')
