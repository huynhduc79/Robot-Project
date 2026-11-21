# KẾT QUẢ MÔ PHỎNG (tự sinh bởi run_experiments.py)

## Bảng 4.1 – So sánh 3 phương pháp (σ_r = 3 cm, 1 lần chạy đại diện)

| Chỉ số | Dead reckoning | Không đóng vòng | Pose-Graph SLAM |
|---|---|---|---|
| ATE RMSE [m] | 1.183 | 0.080 | 0.017 |
| Sai số vị trí lớn nhất [m] | 2.079 | 0.144 | 0.036 |
| Sai số điểm cuối [m] | 2.011 | 0.103 | 0.001 |
| RMSE hướng [độ] | 6.89 | 0.25 | 0.15 |
| RPE Δ=1 tịnh tiến [m] | 0.0184 | 0.0054 | 0.0053 |
| RPE Δ=20 tịnh tiến [m] | 0.155 | 0.017 | 0.014 |
| σ_pos cuối [m] | 1.687 | 0.246 | 0.005 |
| σ_pos lớn nhất [m] | 1.687 | 0.246 | 0.073 |
| % e_x trong ±2σ | 93.2 | 100.0 | 99.5 |
| % e_y trong ±2σ | 100.0 | 100.0 | 100.0 |
| % e_θ trong ±2σ | 100.0 | 100.0 | 100.0 |
| % trong elip 95 % | 95.1 | 100.0 | 100.0 |
| ANEES (kỳ vọng 2) | 2.00 | 0.72 | 0.39 |
| Map Accuracy [%] | 90.69 | 98.00 | 99.05 |
| Obstacle IoU [%] | 9.00 | 55.03 | 75.01 |

Bản đồ dựng bằng tư thế thật (cận trên): Acc = 99.09 %, IoU = 75.62 %

Thông tin đồ thị: {"N": 206, "n_edges": 437, "n_odo": 205, "n_seq": 204, "n_lc": 28, "n_lc_try": 28, "chi2_init": 1927636.1780846925, "chi2_final": 648.765205959438, "gn_iters": 4, "dof": 696, "dr_vs_odopg_maxdiff": 5.417888360170764e-14, "path_len": 75.0, "H_density": 1.5825137985567812}

## Bảng 4.2 – Monte Carlo (M = 20)

| Phương pháp | ATE TB ± ĐLC [m] | % x | % y | % θ | % elip | ANEES |
|---|---|---|---|---|---|---|
| Dead reckoning (odometry) | 0.948 ± 0.515 | 94.3 | 92.0 | 93.0 | 95.6 | 2.02 |
| Không đóng vòng (No LC) | 0.084 ± 0.030 | 99.8 | 100.0 | 99.9 | 99.9 | 0.69 |
| Pose-Graph SLAM | 0.028 ± 0.013 | 99.3 | 98.2 | 98.3 | 98.6 | 1.17 |

Biên 95 % của NEES trung bình: [1.22, 2.97]

## Bảng 4.3 – Độ nhạy theo σ_r

| σ_r [m] | Số LC | ATE DR | ATE No LC | ATE SLAM | Acc No LC | IoU No LC | Acc SLAM | IoU SLAM | % elip 95 % |
|---|---|---|---|---|---|---|---|---|---|
| 0.01 | 28 | 1.021 | 0.040 ± 0.030 | 0.015 ± 0.005 | 98.71 | 61.86 | 98.97 | 67.16 | 99.8 |
| 0.03 | 28 | 1.021 | 0.066 ± 0.025 | 0.031 ± 0.011 | 98.27 | 60.62 | 98.81 | 70.23 | 98.5 |
| 0.05 | 29 | 1.021 | 0.129 ± 0.048 | 0.059 ± 0.041 | 97.07 | 49.80 | 98.22 | 65.10 | 85.7 |
| 0.08 | 22 | 1.021 | 0.503 ± 0.165 | 0.139 ± 0.031 | 92.58 | 18.04 | 95.69 | 39.00 | 89.7 |

Tổng thời gian: 5.9 phút