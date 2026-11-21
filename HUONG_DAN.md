# HƯỚNG DẪN: LÝ THUYẾT POSE-GRAPH SLAM, BIÊN TIN CẬY 95 % VÀ CÁCH LÀM ĐỒ ÁN

Tài liệu này giúp bạn **hiểu** để tự trình bày và trả lời khi bảo vệ. Muốn xem suy dẫn đầy đủ + nguồn từng trang, mở **bản Word mở rộng** (13 khung “GIẢI THÍCH MỞ RỘNG” + Phụ lục C). Đọc theo thứ tự: Phần 1 (lý thuyết, bám slide) → Phần 2 (cách đồ án được xây dựng) → Phần 3 (chạy chương trình) → Phần 4 (đọc kết quả) → Phần 5 (câu hỏi bảo vệ).

---

## PHẦN 1 – LÝ THUYẾT (theo đúng mạch slide Chương 4 và sách Corke 2023)

### 1.1. Dead reckoning: vì sao robot "bị lạc" (slide Ch.4 part 2, mục 4.5.2)

Robot đo odometry mỗi bước: δ_k = (δd, δθ) – đi được bao xa, quay bao nhiêu. Mô hình (slide 4.5.2.1):

```
x(k+1) = x(k) + δd·cos θ(k)
y(k+1) = y(k) + δd·sin θ(k)
θ(k+1) = θ(k) + δθ
```

Odometry có nhiễu: δ̂ = δ + v, v ~ N(0, V), V = diag(σ_d², σ_θ²). Đồ án dùng đúng số trong sách: σ_d = 0.02 m, σ_θ = 0.5°.

**Lan truyền bất định (bước dự đoán EKF, slide 4.5.2.2):**

```
P(k+1) = Fx·P(k)·Fxᵀ + Fv·V·Fvᵀ
Fx = [1 0 −δd·sinθ; 0 1 δd·cosθ; 0 0 1]      Fv = [cosθ 0; sinθ 0; 0 1]
```

Số hạng `Fv V Fvᵀ` luôn dương ⇒ **P chỉ tăng, không bao giờ giảm** ⇒ bất định tăng không giới hạn. Đây là lý do phải có thêm thông tin từ ngoài (landmark, LiDAR, đóng vòng).

### 1.2. "Đường 95 % confidence với 2 đường bound" là gì?

- Với phân bố chuẩn 1 chiều: P(|sai số| ≤ 2σ) = 95.4 %. Slide viết: *"There is a 95 % chance that the robot's x-coordinate is within the ±2σ bound"*.
- Cách vẽ: trục hoành là bước k; vẽ **sai số** x̃_k = x̂_k − x_k (cần ground truth), và **2 đường biên** +2σ_k, −2σ_k với σ_k = √P_k(1,1). Làm tương tự cho y và θ.
- Đọc hình:
  - ~95 % số điểm sai số nằm giữa 2 đường biên → bộ ước lượng **nhất quán** (tự đánh giá đúng độ tin cậy).
  - Sai số hay vượt biên → **quá tự tin (overconfident)** – nguy hiểm (slide part 1, ví dụ đo chiều cao tòa nhà).
  - Sai số luôn rất nhỏ so với biên → **quá thận trọng** – an toàn nhưng lãng phí.
- Trong 2D: **elip tin cậy 95 %** là tập điểm p thỏa (p−μ)ᵀ P⁻¹ (p−μ) ≤ 5.991 (5.991 = χ²₂ ở mức 95 %). Bán trục = √(5.991·λ) theo vector riêng của khối 2×2 vị trí của P.

### 1.3. Từ EKF sang đồ thị tư thế (Corke 2023, mục 6.4)

Slide 4.5.3: khi robot nhìn lại landmark đã biết, elip co lại. Pose-graph SLAM tổng quát hóa ý này:

- **Nút** = tư thế robot x_i = (x, y, θ) tại các thời điểm.
- **Cạnh** = ràng buộc tương đối z_ij (từ odometry, từ ghép quét LiDAR, hoặc từ **đóng vòng** khi quay lại nơi cũ), kèm **ma trận thông tin** Ω_ij = Σ_ij⁻¹ (độ tin cậy của phép đo).
- **Sai số cạnh** (Grisetti 2010; Corke ct. 6.33): "điều robot đo được" so với "điều đồ thị đang suy ra":

```
e_ij = [ R_zᵀ ( R_iᵀ (t_j − t_i) − t_z ) ;  θ_j − θ_i − θ_z ]
```

- **Bài toán tối ưu:** x* = argmin Σ e_ijᵀ Ω_ij e_ij. Corke ví như *kết cấu lò xo*: mỗi cạnh là lò xo có độ cứng Ω, tối ưu = thả cho hệ về năng lượng nhỏ nhất ⇒ sai số tích lũy được **chia đều** cho toàn quỹ đạo.

### 1.4. Giải bằng Gauss–Newton (back-end)

1. Tuyến tính hóa: e_ij(x+Δx) ≈ e_ij + A_ij Δx_i + B_ij Δx_j.
   A_ij = [−R_zᵀR_iᵀ , R_zᵀ (∂R_iᵀ/∂θ_i)(t_j − t_i) ; 0 0 −1],  B_ij = [R_zᵀR_iᵀ , 0 ; 0 0 1].
2. Mỗi cạnh cộng vào 4 khối của H và 2 khối của b: H_ii += AᵀΩA, H_ij += AᵀΩB, H_jj += BᵀΩB, b_i += AᵀΩe, b_j += BᵀΩe.
3. **Neo nút đầu** (H_11 += I lớn) vì cả đồ thị có thể tịnh tiến/xoay tự do (gauge freedom).
4. Giải H·Δx = −b, cập nhật x ← x + Δx, lặp đến khi Δx ≈ 0. H **thưa** (chỉ ~1.6 % phần tử ≠ 0) nên giải rất nhanh.

### 1.5. ⭐ Điểm mấu chốt của đồ án: lấy biên 95 % cho SLAM

Tại nghiệm, F(x) ≈ hằng số + Δxᵀ H Δx ⇒ hậu nghiệm ≈ Gauss với **ma trận thông tin H** ⇒

```
Σ = H⁻¹,   σ_x,k = √Σ_kk(1,1)
```

Thay P của EKF bằng khối Σ_kk ⇒ vẽ được 2 đường biên ±2σ và elip 95 % cho pose-graph SLAM **giống hệt cách slide làm với EKF**.

**Kiểm chứng hay:** nếu đồ thị chỉ có cạnh odometry nối tiếp (không đóng vòng), nghiệm và Σ **trùng đúng** dead reckoning + P của EKF. Đồ án đã kiểm chứng bằng số: sai khác 5·10⁻¹⁴ m. Đây là câu trả lời mạnh nếu thầy hỏi "pose-graph liên quan gì đến EKF trong slide?".

### 1.6. Front-end LiDAR

- **Ghép quét liên tiếp (ICP điểm–đường):** tìm z sao cho điểm quét mới, sau khi biến đổi, nằm sát các "đường" (bề mặt) của quét cũ: min Σ ρ(nᵀ(R p + t − q)). Hiệp phương sai Σ_z ≈ s²(JᵀWJ)⁻¹ (xấp xỉ Laplace, Censi 2007). Hành lang dài ⇒ Σ_z lớn theo phương dọc hành lang (ràng buộc yếu) – đúng hiện tượng Kümmerle 2009 nói.
- **Đóng vòng:** khi ước lượng hiện tại gần một nút cũ (< 3 m, cách ≥ 20 nút) → chạy ICP; chấp nhận nếu > 60 % điểm khớp và RMS < 6 cm. MATLAB `lidarSLAM` (Hess 2016) dùng submap + Branch-and-Bound – trình bày lý thuyết ở mục 2.4 báo cáo.

### 1.7. Bản đồ và thước đo

- Occupancy grid (Elfes 1989) dạng log-odds: ô "trúng" cộng log(0.7/0.3), ô tia đi qua cộng log(0.4/0.6) (hits/misses của Hess 2016).
- **ATE RMSE** = √(mean(e_k²)); **RPE** (Kümmerle 2009) đo sai số tương đối giữa 2 tư thế cách Δ bước.
- **Map Accuracy** = (TP+TN)/(tổng), **Obstacle IoU** = TP/(TP+FP+FN) trên các ô đã quan sát.
- **NEES** ε_k = eᵀP⁻¹e ~ χ²₂ (kỳ vọng 2). **ANEES** Monte Carlo M lần: nhất quán nếu nằm trong [χ²_{2M}(0.025)/M, χ²_{2M}(0.975)/M] = [1.22, 2.97] với M = 20.

---

## PHẦN 2 – ĐỒ ÁN ĐƯỢC XÂY DỰNG NHƯ THẾ NÀO

**Phân vai:** MATLAB/Simulink tạo **toàn bộ số liệu và hình của báo cáo**; Python chỉ làm **demo 3D** (phát lại CSV của MATLAB).

1. **Ground truth:** lưới 400×400 (ô 5 cm), tường + 4 kệ + 4 trụ (thiết kế của bạn).
2. **Quỹ đạo thật – Simulink `sl_drivepoint` (Corke 2023, Hình 4.7, tr.135):** chạy lần lượt 10 đoạn, mỗi đoạn `x0` = trạng thái cuối đoạn trước, `xg` = waypoint kế tiếp. Luật điều khiển: v* = Kv·khoảng cách (Kv = 0.5), γ = Kh·(θ* ⊖ θ) (Kh = 2); xe đạp L = 1 m, v ≤ 1 m/s, γ ≤ 1 rad ⇒ robot **cua tròn**, không quay tại chỗ. Kết quả 5 972 mẫu, 119 s, 78 m, 0 va chạm.
3. **Keyframe:** tạo nút mới khi đi ≥ 0.45 m hoặc quay ≥ 15° (Corke 6.4.2) ⇒ 190 nút.
4. **Cảm biến:** odometry = tư thế tương đối thật giữa 2 keyframe + nhiễu V = diag(0.02², 0.005², 0.5°²); LiDAR dò tia 240 tia, 12 m, σ_r = 3 cm, σ_β = 0.2°.
5. **(A) DR + EKF** → **(B) không đóng vòng** (odometry + ICP liên tiếp) → **(C) pose-graph SLAM** (+ đóng vòng, Gauss–Newton, Σ = H⁻¹).
6. **Đánh giá:** 3 cột sai số x, y, θ + 2 đường biên ±2σ; elip 95 %; ATE/RPE; bản đồ; Monte Carlo 20 lần; độ nhạy σ_r; đối chiếu `lidarSLAM`.
7. **Xuất:** `figures_matlab/*.png`, `results_matlab/results.json`, `results_matlab/csv/*.csv` → `demo_3d.py` và `build_report.py`.

---

## PHẦN 3 – CHẠY CHƯƠNG TRÌNH

### Bước 1 – MATLAB (bắt buộc, ~10 phút; cần Simulink, RVC3 Toolbox trong `D:\KT robot\RVC3-MATLAB-main`)

```matlab
cd 'D:\KT robot\PoseGraphSLAM_Project\matlab'
main_pose_graph_slam
```

- Lần đầu chạy Simulink (~30 s), quỹ đạo được lưu vào `results_matlab/traj_simulink.mat` (lần sau dùng lại; xoá file này nếu đổi waypoint).
- Chạy trong giao diện MATLAB thì cuối chương trình có **animation 2D** (robot, tia LiDAR, elip 95 %, đồ thị sai số x với 2 đường biên).
- Chỉnh ở mục 0: `P.sigma_r`, `P.sigma_th`, `P.M_mc` (số lần Monte Carlo), `P.doMC/doSens/doToolbox` (tắt bớt cho nhanh), `P.covInflate`.

### Bước 2 – Demo 3D Python (2 khung)

```bash
pip install pyvista imageio
```

```bash
cd "D:\KT robot\PoseGraphSLAM_Project\python" && python demo_3d.py
```

- **Khung trái – Ground truth:** nhà xưởng thật, robot chạy theo tư thế thật, tia LiDAR đỏ.
- **Khung phải – Phòng trống:** robot chỉ có odometry + LiDAR, đặt theo tư thế SLAM ước lượng; bản đồ hiện dần trên sàn (trắng = trống, xám = chưa biết, khối đen = vật cản). Cuối cùng hiện Map Accuracy 99.09 % / IoU 76.1 % – trùng báo cáo.
- `--mode live` (mặc định): vừa chạy vừa tạo map. `--mode after`: chạy hết vòng rồi mới dựng map từng scan.
- `--follow`: camera bám robot. `--save`: xuất `figures/demo_3d_live.gif` / `demo_3d_after.gif`.
- Chuột trái xoay (2 khung xoay cùng nhau), cuộn để zoom, `q` để thoát.

### Bước 3 – Dựng lại 2 bản Word (sau khi chạy lại MATLAB)

```bash
cd "D:\KT robot\PoseGraphSLAM_Project\python" && python build_report.py
```

---

## PHẦN 4 – ĐỌC KẾT QUẢ (số liệu MATLAB trong báo cáo)

| Hình | Ý chính cần nói |
|---|---|
| 1.1 | Quỹ đạo Simulink cua tròn ở góc, 190 keyframe, LiDAR 240 tia |
| 3.2 | Sơ đồ khối `sl_drivepoint` (xuất trực tiếp từ model của sách) |
| 4.1 | Tái hiện slide: elip DR phình to. Lần chạy này chỉ 29 % sai số x trong ±2σ – **vì sai số DR tương quan theo thời gian**; Monte Carlo cho 95.1 % ⇒ EKF vẫn nhất quán (ANEES 1.94 ≈ 2) |
| **4.3** | **Hình trọng tâm:** sai số x, y, θ + 2 đường biên của 3 phương pháp; biên SLAM hẹp (±0.1 m) và "thắt" tại đóng vòng |
| 4.4 | σ_pos (thang log) của SLAM sụt mỗi lần đóng vòng |
| 4.5 | 190 nút, 404 cạnh, 26 đóng vòng; H thưa 1.7 %; F(x) 1.9·10⁶ → 676 trong 4 vòng, ≈ bậc tự do 645 |
| 4.6 | DR: tường kép, IoU 5 %; SLAM: IoU 76 % (cận trên 75.5 %) |
| 4.7–4.8 | Monte Carlo: SLAM 0.029 m; ANEES SLAM 1.17 (hơi thận trọng, dưới biên 1.22) |
| 4.9 | σ_r = 8 cm: SLAM 0.170 m > 0.15 m và 84.5 % trong elip ⇒ **quá tự tin ở nhiễu lớn** – hạn chế + hướng khắc phục (`P.covInflate`) |
| 4.10 | `lidarSLAM` 0.122 m (bật LC) vs tự cài đặt 0.019 m; lidarSLAM không có Σ ⇒ không vẽ được biên 95 % |
| 4.11–4.12 | Demo 3D hai khung: ground truth ↔ phòng trống tự tạo map; bản đồ cuối trùng số liệu báo cáo |

---

## PHẦN 5 – CÂU HỎI BẢO VỆ THƯỜNG GẶP

1. **Vì sao ±2σ là 95 %?** Phân bố chuẩn: P(|X−μ| ≤ 2σ) = 95.4 %; chính xác 95 % là 1.96σ. Slide dùng ±2σ.
2. **σ của pose-graph lấy ở đâu?** Từ Σ = H⁻¹ – nghịch đảo ma trận thông tin tại nghiệm (xấp xỉ Laplace). Không phải từ EKF.
3. **Vì sao phải neo nút đầu?** Sai số cạnh chỉ phụ thuộc tư thế tương đối ⇒ H suy biến (3 bậc tự do tịnh tiến/xoay toàn cục). Neo nút 1 làm H khả nghịch.
4. **Không đóng vòng khác dead reckoning thế nào?** (B) có thêm cạnh ghép quét LiDAR liên tiếp nên trôi ít hơn ~9 lần (0.097 so với 0.873 m), nhưng vẫn trôi và biên vẫn tăng.
11. **Simulink dùng để làm gì?** `sl_drivepoint` chỉ sinh **quỹ đạo thật** tuân theo động học xe (không tính bất định). Biên 95 % đến từ EKF (DR) và Σ = H⁻¹ (pose-graph).
12. **Vì sao lần chạy đại diện DR chỉ 29 % trong biên?** Sai số DR tương quan theo thời gian; một lần chạy là một mẫu của cả quỹ đạo. Phải dùng Monte Carlo (95.1 %) để kết luận.
13. **Sao không dùng `EKF.ploterror` của sách?** Hàm đó vẽ ±√5.991σ ≈ ±2.45σ cho từng trục; slide dùng ±2σ. Báo cáo theo slide và ghi rõ quy ước.
5. **Đóng vòng sai thì sao?** Kéo méo cả đồ thị, F(x) tăng, edge residual lớn (Corke Hình 6.38). Đồ án chống bằng ngưỡng khớp > 60 %, RMS < 6 cm, bán kính 3 m; hướng phát triển: `trimLoopClosures` (GNC).
6. **ANEES < 2 nghĩa là gì?** Hệ thống thận trọng: biên rộng hơn sai số thật – an toàn. > biên trên mới nguy hiểm (quá tự tin).
7. **Vì sao Map Accuracy của DR vẫn 90 %?** Đa số ô là ô trống, được nhận đúng dù tường lệch. Vì vậy cần thêm **Obstacle IoU** (DR chỉ 9 %).
8. **Tại sao tự cài đặt mà không dùng lidarSLAM?** lidarSLAM không trả về hiệp phương sai → không vẽ được biên 95 %. Đồ án vẫn chạy lidarSLAM để đối chiếu (ATE 0.122 m khi bật LC, 0.245 m khi tắt).
9. **Độ phức tạp?** H thưa → giải O(N) đến O(N^1.5); nhưng H⁻¹ dạng đặc O(N³) – với bản đồ lớn cần covariance recovery từ phân rã Cholesky thưa.
10. **Phần nào nhóm tự làm?** Xem Bảng 3.1: môi trường, mô phỏng LiDAR, ICP, Gauss–Newton, Σ = H⁻¹, toàn bộ khung đánh giá 95 %/NEES/Monte Carlo, demo.

---

## PHẦN 6 – CHECKLIST THEO RUBRIC

- **C1 (10 %)** – Chủ đề gắn ứng dụng cụ thể (AMR nhà xưởng), 5 mục tiêu kiểm chứng được + tiêu chí số (mục 1.2). ✔
- **C2 (20 %)** – Lý thuyết bám slide (2.2) + sách + 4 bài báo, có bảng so sánh phương pháp (Bảng 2.1). ✔
- **C3 (30 %)** – Bài toán, đầu vào/ra, giả thiết, thuật toán từng bước, thông số (Bảng 3.2), lưu đồ (Hình 3.1). ✔
- **C4 (25 %)** – 4 kịch bản, 11 hình kết quả, 5 bảng, đối chiếu yêu cầu (Bảng 4.5), hạn chế trung thực (σ_r = 8 cm chưa đạt). ✔
- **C5 (15 %)** – TNR 13, giãn dòng 1.4, hình/bảng/công thức đánh số, IEEE, phân định đóng góp (Bảng 3.1). ✔ *(bạn cần cập nhật Mục lục trong Word và điền thông tin bìa)*
