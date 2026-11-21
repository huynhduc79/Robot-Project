# KẾ HOẠCH & NHẬT KÝ CÔNG VIỆC – Đồ án cuối kỳ Kỹ thuật Robot

**Đề tài:** Nghiên cứu ứng dụng LiDAR Pose-Graph SLAM để định vị, xây dựng và đánh giá định lượng bản đồ lưới chiếm chỗ cho robot tự hành (AMR) trong nhà xưởng — có đánh giá sai số với **2 đường biên tin cậy 95 % (±2σ)** và **elip 95 %**.

File này ghi lại *tôi (Claude) đã làm gì, vì sao, các file nằm ở đâu*.

---

## 1. Phân vai công cụ (theo đề xuất của bạn, phiên bản 2)

| Phần | Công cụ | Lý do |
|---|---|---|
| Quỹ đạo robot | **Simulink `sl_drivepoint`** của sách Corke 2023 (RVC3) | Mô hình có sẵn trong sách: bộ điều khiển "đi tới điểm" + Bicycle Kinematic Model (Kv = 0.5, Kh = 2) |
| SLAM, biên ±2σ 95 %, elip 95 %, Monte Carlo, độ nhạy, đối chiếu `lidarSLAM` | **MATLAB** (`matlab/main_pose_graph_slam.m`) | Nguồn số liệu & hình **chính thức** của báo cáo; giống cách slide/sách trình bày |
| Demo trực quan robot trong phòng | **Python 3D – PyVista** (`python/demo_3d.py`) | Phát lại CSV do MATLAB xuất ⇒ con số trên màn hình trùng báo cáo |
| Kiểm chứng độc lập | Python 2D (`python/pgslam_core.py`, `run_experiments.py`) | Cài đặt lại cùng thuật toán, cho cùng xu hướng (ATE SLAM 0.017 m) |

Lưu ý: các model `sl_drive*` là **model điều khiển chuyển động**, không tính hiệp phương sai — biên 95 % vẫn phải tính bằng EKF (DR) và Σ = H⁻¹ (pose-graph).

## 2. Nguồn đã đọc

| Nguồn | Nội dung dùng |
|---|---|
| Hướng dẫn + Rubric của thầy | Cấu trúc 5 chương, TNR 13, giãn dòng 1.3–1.5, rubric C1–C5 |
| Slide Ch.4 part 1 & 2 | Odometry, EKF dự đoán, ±2σ = 95 % (p2 tr.19), elip sai số (tr.23), landmark làm elip co (tr.32–33), EKF-SLAM (tr.45–52), overconfident (p1 tr.96–100) |
| Corke 2023 | sl_drivepoint (Hình 4.7, tr.133–135, ct. 4.4, 4.6–4.7); dead reckoning (6.1)–(6.7), tr.219–225; pose-graph (6.31)–(6.33), tr.257–266; App. E, F.2.4, G |
| Toolbox RVC3 | `sl_drivepoint.slx`, `EKF.m` (`ploterror` vẽ biên √χ²₂(0.95)σ ≈ 2.45σ), `plotvehicle` |
| 4 bài báo (`D:\Downloads`) | Elfes 1989, Grisetti 2010 (ct. 3–22), Kümmerle 2009, Hess 2016 (`45466.pdf`) |

## 3. Sản phẩm

| Sản phẩm | Đường dẫn |
|---|---|
| **Word – bản chính thức** (ghi đè file của bạn) | `D:\KT robot\Báo cáo_ ... Robot tự hành.docx` |
| **Word – bản mở rộng** (13 khung giải thích lý thuyết + nguồn + mã) | `D:\KT robot\Báo cáo_ ... (BAN MO RONG - giai thich ly thuyet).docx` |
| Bản gốc (sao lưu) | `D:\KT robot\Báo cáo_ ... (BAN GOC - backup).docx` |
| Chương trình MATLAB chính | `matlab/main_pose_graph_slam.m` → `matlab/figures_matlab/*.png`, `matlab/results_matlab/results.json`, `.../csv/*.csv` |
| Demo 3D hai khung (ground truth ↔ phòng trống tạo map) | `python/demo_3d.py` (`--mode live/after`) → `figures/demo_3d_live.gif`, `demo_3d_after.gif` |
| Dựng 2 bản Word | `python/build_report.py` |
| Hướng dẫn lý thuyết + cách làm + bảo vệ | `HUONG_DAN.md` |

## 4. Kết quả chính (MATLAB, σ_r = 3 cm)

| | Dead reckoning | Không đóng vòng | Pose-Graph SLAM |
|---|---|---|---|
| ATE Monte Carlo 20 lần | 0.873 m | 0.097 m | **0.029 m** |
| % sai số trong ±2σ (x/y/θ, MC) | 95.1 / 95.3 / 97.3 | 99.0 / 98.1 / 95.3 | 99.3 / 98.5 / 98.3 |
| ANEES (kỳ vọng 2, biên [1.22, 2.97]) | 1.94 ✔ | 0.92 (thận trọng) | 1.17 (hơi thận trọng) |
| ATE lần đại diện (seed 7) | 1.912 m | 0.097 m | **0.019 m** |
| Map Accuracy / IoU (lần đại diện) | 88.2 % / 5.1 % (tường kép) | 98.0 % / 56.0 % | **99.1 % / 76.1 %** |

Lần đại diện: chỉ 29 % sai số x của DR nằm trong ±2σ (sai số DR tương quan mạnh theo thời gian) → báo cáo giải thích và dùng Monte Carlo để kết luận tính nhất quán.

`lidarSLAM` (Navigation Toolbox): tắt LC 0.245 m, bật LC 0.122 m — không có hiệp phương sai.
Độ nhạy: σ_r = 8 cm → ATE SLAM 0.170 m (**vượt** yêu cầu 0.15 m) và chỉ 84.5 % trong elip 95 % (quá tự tin) — đã ghi trung thực là hạn chế.

## 5. Việc BẠN cần làm trước khi nộp

- [ ] Mở **cả 2** file Word → chọn **Yes** khi Word hỏi cập nhật trường (tạo Mục lục).
- [ ] Điền họ tên, MSSV, lớp, GVHD ở bìa. Nộp **bản chính thức**; bản mở rộng để học/ôn bảo vệ.
- [ ] Kiểm tra bố cục hình/bảng trong Word (tôi không có Word để render thử).
- [ ] Chạy thử `main_pose_graph_slam` trong MATLAB (bật `P.doAnimation`) và `python demo_3d.py` để quen thao tác.
- [ ] Có thể rút gọn Phụ lục B (mã nguồn) nếu vượt 25 trang.
