%% =====================================================================================
%  LIDAR POSE-GRAPH SLAM CHO ROBOT AMR TRONG NHÀ XƯỞNG - MÔ PHỎNG 2D & ĐÁNH GIÁ BIÊN 95 %
%  -------------------------------------------------------------------------------------
%  Quỹ đạo thật : mô hình Simulink sl_drivepoint của sách Corke et al. (2023) (RVC3 Toolbox)
%                 - bộ điều khiển "đi tới điểm" + mô hình động học xe đạp (Bicycle Kinematic Model)
%                 - chạy lần lượt qua 11 waypoint của nhà xưởng
%  (A) Dead reckoning + EKF dự đoán  P = Fx P Fx' + Fv V Fv'          (slide Ch.4, mục 4.5.2)
%  (B) Không đóng vòng: cạnh odometry + ghép quét LiDAR liên tiếp (ICP điểm-đường)
%  (C) Pose-Graph SLAM: (B) + cạnh đóng vòng, tối ưu Gauss-Newton trên H thưa (Grisetti 2010)
%  Biên tin cậy: ±2σ (95 %) từ P (EKF) hoặc Σ = H^-1 (pose graph); elip 95 % (χ²₂ = 5.991)
%  Kịch bản: KB1 so sánh 3 phương pháp | KB2 Monte Carlo nhất quán | KB3 độ nhạy σ_r
%            KB4 đối chiếu lidarSLAM (Navigation Toolbox)
%  Đầu ra  : figures_matlab/*.png, results_matlab/results.json, results_matlab/csv (cho demo 3D)
%
%  Yêu cầu: MATLAB R2020b+, Simulink, RVC3 Toolbox (thư mục RVC3-MATLAB-main), Navigation
%           Toolbox (chỉ cho KB4).          Chạy:  >> main_pose_graph_slam
% ======================================================================================
clear; clc; close all;
here = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(here, '..', '..', 'RVC3-MATLAB-main', 'toolbox')));   % RVC3 toolbox của sách
set(groot, 'defaultAxesToolbarVisible', 'off');
figDir = fullfile(here, 'figures_matlab'); resDir = fullfile(here, 'results_matlab');
if ~exist(figDir,'dir'), mkdir(figDir); end
if ~exist(fullfile(resDir,'csv'),'dir'), mkdir(fullfile(resDir,'csv')); end

%% 0. THAM SỐ
P.seed        = 7;               % seed kịch bản đại diện
P.sigma_d     = 0.02;            % [m]   ĐLC nhiễu tịnh tiến odometry mỗi keyframe (Corke: 0.02 m)
P.sigma_lat   = 0.005;           % [m]   ĐLC nhiễu ngang (trượt bánh)
P.sigma_th    = deg2rad(0.5);    % [rad] ĐLC nhiễu góc odometry mỗi keyframe (Corke: 0.5°)
P.sigma_r     = 0.03;            % [m]   ĐLC nhiễu khoảng cách LiDAR
P.sigma_b     = deg2rad(0.2);    % [rad] ĐLC nhiễu góc tia LiDAR
P.maxRange    = 12;  P.angRes = deg2rad(1.5);          % 240 tia / 360°
P.kfDist      = 0.45; P.kfAngle = deg2rad(15);        % ngưỡng tạo keyframe (nút đồ thị)
P.lcRadius    = 3.0; P.lcMinGap = 20; P.lcEvery = 2;   % đóng vòng
P.lcMinInlier = 0.6; P.lcMaxRms = 0.06;
P.covInflate  = 1.0;             % hệ số thổi phồng hiệp phương sai ICP
P.M_mc        = 20;              % số lần Monte Carlo (KB2)
P.sens        = [0.01 0.03 0.05 0.08]; P.M_sens = 6;  % KB3
P.doMC        = true; P.doSens = true; P.doToolbox = true;
P.doAnimation = true;            % animation 2D sau khi tính xong (tắt khi chạy batch)
if ~usejava('desktop'), P.doAnimation = false; end
RES = 0.05; NG = 400; k95 = 2;   % biên ±2σ theo slide

%% 1. BẢN ĐỒ CHUẨN NHÀ XƯỞNG (Ground truth)
OBS = [0 20 0 .25; 0 20 19.75 20; 0 .25 0 20; 19.75 20 0 20;
       4.8 8.2 4.8 8.0; 4.8 6.2 4.0 4.8; 11.8 15.2 4.8 8.2; 13.8 15.2 8.2 8.8;
       4.8 8.2 12 15.2; 7.0 8.2 15.2 16; 12 15.2 12 15.2;
       9.4 10.6 .25 1.15; 9.4 10.6 18.85 19.75; .25 1.15 9.4 10.6; 18.85 19.75 9.4 10.6];
OBSname = {'Tường Nam','Tường Bắc','Tường Tây','Tường Đông','Kệ A','Kệ A phụ','Kệ B','Kệ B phụ', ...
           'Kệ C','Kệ C phụ','Máy CNC D','Trụ Nam','Trụ Bắc','Trụ Tây','Trụ Đông'};
GT = false(NG, NG);
for q = 1:size(OBS,1)
    o = OBS(q,:);
    GT(floor(o(3)/RES+1e-9)+1 : ceil(o(4)/RES-1e-9), floor(o(1)/RES+1e-9)+1 : ceil(o(2)/RES-1e-9)) = true;
end
WP = [2.5 2.5; 10 2.5; 17.5 2.5; 17.5 10; 17.5 17.5; 10 17.5; 10 10; 2.5 10; 2.5 17.5; 2.5 10; 2.5 2.5];

%% 2. QUỸ ĐẠO THẬT TỪ SIMULINK sl_drivepoint (Corke 2023, mục 4.1.1.1 "Moving to a Point")
trajFile = fullfile(resDir, 'traj_simulink.mat');
if exist(trajFile, 'file')
    S = load(trajFile); Xd = S.Xd; td = S.td; tSim = S.tSim;
else
    fprintf('Chạy Simulink sl_drivepoint qua %d waypoint...\n', size(WP,1)-1);
    load_system('sl_drivepoint'); tic;
    Xd = [WP(1,:) 0]; td = 0;
    for i = 2:size(WP,1)
        x0 = Xd(end,:); xg = WP(i,:);                       %#ok<NASGU> biến được model đọc
        out = sim('sl_drivepoint', 'StopTime', '80', 'MaxStep', '0.02', 'SrcWorkspace', 'current');
        y = out.y; t = out.t;
        Xd = [Xd; y(2:end,:)]; td = [td; td(end) + t(2:end)]; %#ok<AGROW>
    end
    Xd(:,3) = wrapPi(Xd(:,3)); tSim = toc;
    save(trajFile, 'Xd', 'td', 'tSim');
    try   % lưu ảnh sơ đồ khối Simulink cho báo cáo
        open_system('sl_drivepoint'); print('-ssl_drivepoint', '-dpng', '-r200', fullfile(figDir, 'fig0_simulink_sl_drivepoint.png'));
    catch, end
end
nColl = sum(GT(sub2ind([NG NG], min(max(floor(Xd(:,2)/RES)+1,1),NG), min(max(floor(Xd(:,1)/RES)+1,1),NG))));
% Trích keyframe (nút đồ thị): đi được ≥ kfDist hoặc quay ≥ kfAngle (Corke 6.4.2)
kf = 1; acc_d = 0;
for i = 2:size(Xd,1)
    acc_d = acc_d + norm(Xd(i,1:2) - Xd(i-1,1:2));
    if acc_d >= P.kfDist || abs(wrapPi(Xd(i,3) - Xd(kf(end),3))) >= P.kfAngle
        kf(end+1) = i; acc_d = 0; %#ok<SAGROW>
    end
end
if kf(end) ~= size(Xd,1), kf(end+1) = size(Xd,1); end
Xt = Xd(kf,:); N = size(Xt,1);
pathLen = sum(vecnorm(diff(Xd(:,1:2)), 2, 2));
fprintf('Simulink: %d mẫu, %.1f s mô phỏng, %.1f m, va chạm = %d | %d keyframe\n', size(Xd,1), td(end), pathLen, nColl, N);

%% 3. KỊCH BẢN 1 - CHẠY ĐẦY ĐỦ (seed đại diện)
R1 = runSLAM(P, P.seed, Xt, GT, RES, NG, true);
names = {'Dead reckoning', 'Không đóng vòng', 'Pose-Graph SLAM'}; keys = {'dr','nolc','slam'};
Xs = {R1.Xdr, R1.Xnolc, R1.Xslam}; Ps = {R1.Pdr, R1.Pnolc, R1.Pslam};
cols = {[.84 .15 .16], [1 .5 .05], [.12 .47 .71]};
for i = 1:3
    M1.(keys{i}) = metrics(Xs{i}, Ps{i}, Xt, k95);
    Lm{i} = buildMap(Xs{i}, R1.scans, R1.ang, P.maxRange, RES, NG); MM{i} = mapMetrics(Lm{i}, GT); %#ok<SAGROW>
    M1.(keys{i}).acc = MM{i}.acc; M1.(keys{i}).iou = MM{i}.iou;
end
Lgt = buildMap(Xt, R1.scans, R1.ang, P.maxRange, RES, NG); MMgt = mapMetrics(Lgt, GT);
M1.gtpose = struct('acc', MMgt.acc, 'iou', MMgt.iou);
[~, ~, chiB] = optimizePoseGraph(R1.Xdr, R1.E, Xt(1,:), R1.Ainfo, 30);    % hội tụ từ khởi tạo DR
M1.info = struct('N', N, 'n_edges', numel(R1.E.I), 'n_odo', sum(R1.E.K==0), 'n_seq', sum(R1.E.K==1), ...
    'n_lc', sum(R1.E.K==2), 'n_lc_try', R1.nTry, 'chi2_init', chiB(1), 'chi2_final', chiB(end), ...
    'gn_iters', numel(chiB)-1, 'dof', 3*numel(R1.E.I) - 3*(N-1), 'dr_vs_odopg_maxdiff', R1.drDiff, ...
    'path_len', pathLen, 'H_density', 100*nnz(R1.H)/numel(R1.H), 'n_dense', size(Xd,1), ...
    't_sim', td(end), 'collisions', nColl, 'cpu_frontend', R1.cpu);
fprintf('\n%-18s | ATE[m] | Cuối[m] | %%x   %%y   %%θ  | ANEES | Acc%% | IoU%%\n', 'Phương pháp');
for i = 1:3
    m = M1.(keys{i});
    fprintf('%-18s | %6.3f | %6.3f | %4.0f %4.0f %4.0f | %5.2f | %5.2f | %5.1f\n', names{i}, m.ate, m.final, m.in_x, m.in_y, m.in_t, m.anees, m.acc, m.iou);
end

%% 4. HÌNH VẼ KỊCH BẢN 1
k = (0:N-1)';
% --- Hình 1: môi trường, quỹ đạo Simulink, LiDAR
f = figure('Color','w','Position',[40 60 1150 480]);
subplot(1,2,1); showGT(GT, 1); hold on;
plot(Xd(:,1), Xd(:,2), 'b-', 'LineWidth', 1.4); plot(WP(:,1), WP(:,2), 'ms', 'MarkerFaceColor', 'm', 'MarkerSize', 5);
plot(Xt(:,1), Xt(:,2), 'k.', 'MarkerSize', 6); plot(Xt(1,1), Xt(1,2), 'go', 'MarkerFaceColor', 'g', 'MarkerSize', 8);
legend('Quỹ đạo Simulink sl\_drivepoint', 'Waypoint', sprintf('Keyframe (%d nút)', N), 'Xuất phát', 'Location', 'north', 'FontSize', 8);
title('a) Bản đồ chuẩn M_{true} và quỹ đạo thật');
subplot(1,2,2); showGT(GT, 0.6); hold on; kk = 41;
r = R1.scans{kk}; m = isfinite(r); A = R1.ang(m) + Xt(kk,3);
px = Xt(kk,1) + r(m).*cos(A); py = Xt(kk,2) + r(m).*sin(A);
plot([repmat(Xt(kk,1),1,numel(px)); px'], [repmat(Xt(kk,2),1,numel(py)); py'], '-', 'Color', [1 0 0 .2]);
plot(px, py, 'r.', 'MarkerSize', 7); plotvehicle(Xt(kk,:), 'size', 0.8, 'fillcolor', 'b');
title(sprintf('b) LiDAR 2D tại nút k = %d (%d tia, \\sigma_r = %.0f cm)', kk-1, numel(R1.ang), 100*P.sigma_r));
saveFig(f, figDir, 'fig1_moi_truong_lidar.png');

% --- Hình 2: Dead reckoning - elip 95 % và sai số x với 2 đường biên (như slide)
f = figure('Color','w','Position',[40 60 1150 470]);
subplot(1,2,1); showGT(GT, .25); hold on;
h1 = plot(Xt(:,1), Xt(:,2), 'k-', 'LineWidth', 1.5); h2 = plot(R1.Xdr(:,1), R1.Xdr(:,2), '-', 'Color', cols{1}, 'LineWidth', 1.3);
for q = 1:12:N, [ex, ey] = ellipse95(R1.Xdr(q,1:2), R1.Pdr(1:2,1:2,q)); h3 = plot(ex, ey, 'g-'); end
legend([h1 h2 h3], 'Quỹ đạo thật', 'Dead reckoning', 'Elip tin cậy 95 % (\chi^2_2 = 5.991)', 'Location', 'north', 'FontSize', 8);
title('a) Dead reckoning: elip bất định tăng không giới hạn');
subplot(1,2,2); e = errs(R1.Xdr, Xt); s = k95*sqrt(squeeze(R1.Pdr(1,1,:)));
boundPlot(k, e(:,1), s, cols{1}); xlabel('Bước k'); ylabel('Sai số x [m]');
legend('vùng \pm2\sigma', 'biên +2\sigma (95 %)', 'biên -2\sigma (95 %)', 'sai số x_k', 'Location', 'northwest', 'FontSize', 8);
title('b) Sai số x và hai đường biên \pm2\sigma (slide 4.5.2.2)');
saveFig(f, figDir, 'fig2_dead_reckoning_95.png');

% --- Hình 3: quỹ đạo + elip 95 % (No LC, SLAM)
f = figure('Color','w','Position',[40 60 1200 540]);
for p_ = 1:2
    i = p_ + 1; subplot(1,2,p_); showGT(GT, .25); hold on;
    h1 = plot(Xt(:,1), Xt(:,2), 'k-', 'LineWidth', 1.6); h2 = plot(R1.Xdr(:,1), R1.Xdr(:,2), ':', 'Color', cols{1}, 'LineWidth', 1.2);
    h3 = plot(Xs{i}(:,1), Xs{i}(:,2), '-', 'Color', cols{i}, 'LineWidth', 1.5);
    for q = 1:10:N, [ex, ey] = ellipse95(Xs{i}(q,1:2), Ps{i}(1:2,1:2,q)); h4 = plot(ex, ey, 'g-'); end
    hh = [h1 h2 h3 h4]; ll = {'Ground truth', sprintf('DR (ATE = %.3f m)', M1.dr.ate), sprintf('%s (ATE = %.3f m)', names{i}, M1.(keys{i}).ate), 'Elip 95 %'};
    if i == 3
        for q = find(R1.E.K == 2)', h5 = plot(R1.Xslam([R1.E.I(q) R1.E.J(q)],1), R1.Xslam([R1.E.I(q) R1.E.J(q)],2), 'm-', 'LineWidth', 1.2); end
        hh(end+1) = h5; ll{end+1} = sprintf('Cạnh đóng vòng (%d)', sum(R1.E.K==2)); %#ok<SAGROW>
    end
    legend(hh, ll, 'Location', 'north', 'FontSize', 8);
    if i == 2, title('a) Không đóng vòng: elip phình to dần'); else, title('b) Pose-Graph SLAM: elip bị "ghìm" nhỏ'); end
end
saveFig(f, figDir, 'fig3_quy_dao_elip95.png');

% --- Hình 4: sai số x, y, θ + 2 đường biên ±2σ cho 3 phương pháp (HÌNH TRỌNG TÂM)
f = figure('Color','w','Position',[20 20 1400 820]);
lab = {'x [m]', 'y [m]', '\theta [độ]'}; inn = {'in_x','in_y','in_t'};
for i = 1:3
    e = errs(Xs{i}, Xt);
    for r_ = 1:3
        sc = 1; if r_ == 3, sc = 180/pi; end
        s = k95*sqrt(squeeze(Ps{i}(r_,r_,:)))*sc;
        subplot(3,3,(r_-1)*3+i); boundPlot(k, e(:,r_)*sc, s, cols{i});
        text(0.02, 0.9, sprintf('%.0f %% mẫu trong \\pm2\\sigma', M1.(keys{i}).(inn{r_})), 'Units', 'normalized', 'FontSize', 8, 'BackgroundColor', 'w');
        if r_ == 1, title(names{i}); end
        if i == 1, ylabel(['Sai số ' lab{r_}]); end
        if r_ == 3, xlabel('Bước k'); end
    end
end
sgtitle('Sai số tư thế và hai đường biên tin cậy 95 % (\pm2\sigma từ P của EKF hoặc \Sigma = H^{-1})');
saveFig(f, figDir, 'fig4_sai_so_bien95_3pp.png');

% --- Hình 5: ATE e_k và độ bất định
f = figure('Color','w','Position',[40 60 1200 430]);
subplot(1,2,1); hold on; grid on;
for i = 1:3, e = errs(Xs{i}, Xt); plot(k, hypot(e(:,1), e(:,2)), '-', 'Color', cols{i}, 'LineWidth', 1.3); end
legend(arrayfun(@(i) sprintf('%s: RMSE = %.3f m', names{i}, M1.(keys{i}).ate), 1:3, 'UniformOutput', false), 'Location', 'northwest', 'FontSize', 8);
xlabel('Bước k'); ylabel('e_k [m]'); title('a) Sai số vị trí tuyệt đối theo bước k');
subplot(1,2,2); hold on; grid on;
for i = 1:3, plot(k, sqrt(squeeze(Ps{i}(1,1,:) + Ps{i}(2,2,:))), '-', 'Color', cols{i}, 'LineWidth', 1.3); end
lcK = unique(R1.E.J(R1.E.K==2)) - 1; plot(lcK, zeros(size(lcK)), 'm|', 'MarkerSize', 10);
set(gca, 'YScale', 'log'); legend([names, {'Bước có đóng vòng'}], 'Location', 'northwest', 'FontSize', 8);
xlabel('Bước k'); ylabel('\surd(\sigma_x^2+\sigma_y^2) [m]'); title('b) Độ bất định vị trí (thang log)');
saveFig(f, figDir, 'fig5_ate_va_do_bat_dinh.png');

% --- Hình 6: đồ thị tư thế, H thưa, hội tụ
f = figure('Color','w','Position',[20 60 1400 450]);
subplot(1,3,1); hold on; axis equal; grid on;
for q = 1:numel(R1.E.I)
    c = [.6 .6 .6]; lw = 0.8; if R1.E.K(q) == 2, c = [.9 .2 .7]; lw = 1.2; end
    plot(R1.Xslam([R1.E.I(q) R1.E.J(q)],1), R1.Xslam([R1.E.I(q) R1.E.J(q)],2), '-', 'Color', c, 'LineWidth', lw);
end
plot(R1.Xslam(:,1), R1.Xslam(:,2), '.', 'Color', cols{3}); xlabel('X [m]'); ylabel('Y [m]');
title(sprintf('a) %d nút, %d cạnh (%d đóng vòng)', N, numel(R1.E.I), sum(R1.E.K==2)));
subplot(1,3,2); spy(R1.H, 2); title(sprintf('b) H %dx%d thưa (%.1f %% khác 0)', size(R1.H,1), size(R1.H,2), M1.info.H_density));
subplot(1,3,3); semilogy(0:numel(chiB)-1, chiB, '-o', 'LineWidth', 1.3); grid on; hold on;
yline(M1.info.dof, 'k--', sprintf('bậc tự do \\approx %d', M1.info.dof));
xlabel('Vòng lặp Gauss-Newton'); ylabel('F(x) = \Sigma e^T\Omega e'); title('c) Hội tụ F(x) (khởi tạo bằng DR)');
saveFig(f, figDir, 'fig6_do_thi_tu_the.png');

% --- Hình 7: bản đồ lưới
f = figure('Color','w','Position',[20 60 1500 400]);
subplot(1,4,1); showGT(GT, 1); title('a) Ground truth');
for p_ = [1 3]
    subplot(1,4,1 + (p_+1)/2); Pm = 1 ./ (1 + exp(-Lm{p_})); img = 1 - Pm; img(~MM{p_}.known) = .5;
    imagesc([0 20], [0 20], img); set(gca, 'YDir', 'normal'); axis equal tight; colormap(gca, gray); clim([0 1]);
    title(sprintf('%s) %s\nAcc = %.1f %%, IoU = %.1f %%', char('a' + (p_+1)/2), names{p_}, MM{p_}.acc, MM{p_}.iou));
end
subplot(1,4,4); ms = MM{3}; rgb = ones(NG, NG, 3); R_ = rgb(:,:,1); G_ = R_; B_ = R_;
grey = GT & ~ms.known; tp = ms.occ & GT & ms.known; fp = ms.occ & ~GT & ms.known; fn = ~ms.occ & GT & ms.known;
R_(grey) = .85; G_(grey) = .85; B_(grey) = .85; R_(tp) = .1; G_(tp) = .7; B_(tp) = .1;
R_(fp) = .9; G_(fp) = .1; B_(fp) = .1; R_(fn) = .1; G_(fn) = .3; B_(fn) = .9;
image([0 20], [0 20], cat(3, R_, G_, B_)); set(gca, 'YDir', 'normal'); axis equal tight;
title({'d) Đối chiếu ô lưới SLAM', '(lục TP, đỏ FP, lam FN, xám chưa quan sát)'}, 'FontSize', 9);
saveFig(f, figDir, 'fig7_ban_do_luoi.png');

%% 5. KỊCH BẢN 2 - MONTE CARLO KIỂM ĐỊNH NHẤT QUÁN
MC = struct();
if P.doMC
    fprintf('\nMonte Carlo M = %d ...\n', P.M_mc);
    NE = zeros(P.M_mc, N-1, 3); AT = zeros(P.M_mc, 3); IN = zeros(P.M_mc, 4, 3);
    for mc = 1:P.M_mc
        Rm = runSLAM(P, 1000 + mc, Xt, GT, RES, NG, false);
        XX = {Rm.Xdr, Rm.Xnolc, Rm.Xslam}; PP = {Rm.Pdr, Rm.Pnolc, Rm.Pslam};
        for i = 1:3
            mm = metrics(XX{i}, PP{i}, Xt, k95);
            NE(mc,:,i) = mm.nees; AT(mc,i) = mm.ate; IN(mc,:,i) = [mm.in_x mm.in_y mm.in_t mm.in_ell];
        end
        fprintf('  lần %2d: LC = %d, ATE = %.3f / %.3f / %.3f\n', mc, sum(Rm.E.K==2), AT(mc,:));
    end
    lo = chi2q(0.025, 2*P.M_mc)/P.M_mc; hi = chi2q(0.975, 2*P.M_mc)/P.M_mc;
    for i = 1:3
        ne = mean(NE(:,:,i), 1);
        MC.(keys{i}) = struct('anees', mean(ne), 'frac_in_band', 100*mean(ne >= lo & ne <= hi), 'ate_mean', mean(AT(:,i)), ...
            'ate_std', std(AT(:,i)), 'in_x', mean(IN(:,1,i)), 'in_y', mean(IN(:,2,i)), 'in_t', mean(IN(:,3,i)), 'in_ell', mean(IN(:,4,i)));
    end
    MC.bounds = [lo hi]; MC.M = P.M_mc;
    f = figure('Color','w','Position',[40 60 1250 440]);
    subplot(1,2,1); hold on; grid on;
    for i = 1:3, plot(1:N-1, mean(NE(:,:,i),1), '-', 'Color', cols{i}, 'LineWidth', 1.1); end
    yline(2, 'k-', 'kỳ vọng = 2'); yline(lo, 'k--'); yline(hi, 'k--', sprintf('biên 95 %% [%.2f, %.2f]', lo, hi));
    ylim([0 6]); xlabel('Bước k'); ylabel('NEES trung bình'); legend(names, 'Location', 'northwest', 'FontSize', 8);
    title(sprintf('a) NEES vị trí trung bình (%d lần Monte Carlo)', P.M_mc));
    subplot(1,2,2); B = squeeze(mean(IN, 1))'; bar(B); hold on; yline(95.4, 'k--', 'lý thuyết \pm2\sigma: 95.4 %');
    set(gca, 'XTickLabel', names); ylim([50 102]); ylabel('% mẫu nằm trong biên'); grid on;
    legend('x', 'y', '\theta', 'elip 95 %', 'Location', 'southoutside', 'Orientation', 'horizontal'); title('b) Tỷ lệ sai số nằm trong biên 95 %');
    saveFig(f, figDir, 'fig8_monte_carlo_nhat_quan.png');
    f = figure('Color','w','Position',[40 60 650 420]); boxplot(AT, 'Labels', names); set(gca, 'YScale', 'log'); grid on;
    ylabel('ATE RMSE [m]'); title(sprintf('Phân bố ATE RMSE qua %d lần Monte Carlo', P.M_mc));
    saveFig(f, figDir, 'fig8b_boxplot_ate.png');
end

%% 6. KỊCH BẢN 3 - ĐỘ NHẠY THEO σ_r
SE = struct([]);
if P.doSens
    fprintf('\nĐộ nhạy theo sigma_r ...\n');
    for s_ = 1:numel(P.sens)
        Ps_ = P; Ps_.sigma_r = P.sens(s_); A_ = zeros(P.M_sens, 9);
        for mc = 1:P.M_sens
            Rm = runSLAM(Ps_, 2000 + mc, Xt, GT, RES, NG, false);
            ms = mapMetrics(buildMap(Rm.Xslam, Rm.scans, Rm.ang, P.maxRange, RES, NG), GT);
            mn = mapMetrics(buildMap(Rm.Xnolc, Rm.scans, Rm.ang, P.maxRange, RES, NG), GT);
            mm = metrics(Rm.Xslam, Rm.Pslam, Xt, k95);
            A_(mc,:) = [sum(Rm.E.K==2), ateOf(Rm.Xdr, Xt), ateOf(Rm.Xnolc, Xt), mm.ate, mn.acc, mn.iou, ms.acc, ms.iou, mm.in_ell];
        end
        mu = mean(A_, 1); sd = std(A_, 0, 1);
        SE(s_).sigma_r = P.sens(s_); SE(s_).n_lc = mu(1); SE(s_).ate_dr = mu(2); SE(s_).ate_nolc = mu(3); SE(s_).ate_nolc_sd = sd(3);
        SE(s_).ate_slam = mu(4); SE(s_).ate_slam_sd = sd(4); SE(s_).acc_nolc = mu(5); SE(s_).iou_nolc = mu(6);
        SE(s_).acc_slam = mu(7); SE(s_).iou_slam = mu(8); SE(s_).iou_slam_sd = sd(8); SE(s_).in_ell = mu(9);
        fprintf('  sigma_r = %.2f: LC = %.0f, ATE noLC = %.3f, SLAM = %.3f, IoU = %.1f, elip = %.1f\n', P.sens(s_), mu(1), mu(3), mu(4), mu(8), mu(9));
    end
    s = 100*[SE.sigma_r];
    f = figure('Color','w','Position',[20 60 1450 400]);
    subplot(1,3,1); errorbar(s, [SE.ate_nolc], [SE.ate_nolc_sd], '-s', 'Color', cols{2}, 'LineWidth', 1.3); hold on;
    errorbar(s, [SE.ate_slam], [SE.ate_slam_sd], '-o', 'Color', cols{3}, 'LineWidth', 1.3); yline(0.15, 'k:', 'yêu cầu 0.15 m');
    grid on; xlabel('\sigma_r [cm]'); ylabel('ATE RMSE [m]'); legend(names(2:3), 'Location', 'northwest'); title('a) Sai số quỹ đạo');
    subplot(1,3,2); plot(s, [SE.acc_nolc], '-s', s, [SE.acc_slam], '-o', s, [SE.iou_nolc], '--s', s, [SE.iou_slam], '--o', 'LineWidth', 1.3);
    grid on; xlabel('\sigma_r [cm]'); ylabel('[%]'); legend('Acc - No LC', 'Acc - SLAM', 'IoU - No LC', 'IoU - SLAM', 'Location', 'southwest'); title('b) Chất lượng bản đồ');
    subplot(1,3,3); yyaxis left; bar(s, [SE.n_lc], 0.5, 'FaceColor', [.9 .5 .8]); ylabel('Số cạnh đóng vòng');
    yyaxis right; plot(s, [SE.in_ell], '-ko', 'LineWidth', 1.3); ylabel('% trong elip 95 %'); yline(95, 'k:');
    grid on; xlabel('\sigma_r [cm]'); title('c) Đóng vòng và độ tin cậy');
    saveFig(f, figDir, 'fig9_do_nhay_nhieu_lidar.png');
end

%% 7. KỊCH BẢN 4 - ĐỐI CHIẾU lidarSLAM (Navigation Toolbox, kiến trúc Hess et al. 2016)
TB = struct();
if P.doToolbox && exist('lidarSLAM', 'class') == 8
    thr = [1e6, 150]; tags = {'off', 'on'}; Xtb = cell(1,2);
    for t_ = 1:2
        slam = lidarSLAM(1/RES, P.maxRange); slam.LoopClosureThreshold = thr(t_); slam.LoopClosureSearchRadius = 6;
        accd = false(N,1);
        for q = 1:N, accd(q) = addScan(slam, removeInvalidData(lidarScan(R1.scans{q}, R1.ang), 'RangeLimits', [0.05 P.maxRange])); end
        [~, pr] = scansAndPoses(slam); c0 = cos(Xt(1,3)); s0 = sin(Xt(1,3));
        pw = [Xt(1,1) + c0*pr(:,1) - s0*pr(:,2), Xt(1,2) + s0*pr(:,1) + c0*pr(:,2), wrapPi(pr(:,3) + Xt(1,3))];
        Xa = Xt(accd,:); Xtb{t_} = pw;
        TB.(tags{t_}) = struct('n_acc', sum(accd), 'n_lc', slam.PoseGraph.NumLoopClosureEdges, 'ate', sqrt(mean(sum((pw(:,1:2) - Xa(:,1:2)).^2, 2))));
        fprintf('lidarSLAM %-3s: %d scan, %d đóng vòng, ATE = %.3f m\n', tags{t_}, sum(accd), TB.(tags{t_}).n_lc, TB.(tags{t_}).ate);
    end
    f = figure('Color','w','Position',[40 60 650 560]); showGT(GT, .25); hold on;
    plot(Xt(:,1), Xt(:,2), 'k-', 'LineWidth', 1.6); plot(Xtb{1}(:,1), Xtb{1}(:,2), '--', 'Color', cols{2}, 'LineWidth', 1.3);
    plot(Xtb{2}(:,1), Xtb{2}(:,2), '-', 'Color', [.5 .2 .7], 'LineWidth', 1.3); plot(R1.Xslam(:,1), R1.Xslam(:,2), '-', 'Color', cols{3}, 'LineWidth', 1.3);
    legend('Ground truth', sprintf('lidarSLAM tắt LC (ATE %.3f m)', TB.off.ate), sprintf('lidarSLAM bật LC (ATE %.3f m)', TB.on.ate), ...
        sprintf('Pose-graph tự cài đặt (ATE %.3f m)', M1.slam.ate), 'Location', 'north', 'FontSize', 8);
    title('Đối chiếu với lidarSLAM (Navigation Toolbox)');
    saveFig(f, figDir, 'fig10_doi_chieu_lidarSLAM.png');
end

%% 8. XUẤT KẾT QUẢ (JSON cho báo cáo, CSV cho demo 3D Python)
cfg = struct('sigma_d', P.sigma_d, 'sigma_lat', P.sigma_lat, 'sigma_th_deg', rad2deg(P.sigma_th), 'sigma_r', P.sigma_r, ...
    'sigma_b_deg', rad2deg(P.sigma_b), 'max_range', P.maxRange, 'ang_res_deg', rad2deg(P.angRes), 'kf_dist', P.kfDist, ...
    'kf_angle_deg', rad2deg(P.kfAngle), 'lc_radius', P.lcRadius, 'lc_min_gap', P.lcMinGap, 'lc_min_inlier', P.lcMinInlier, ...
    'lc_max_rms', P.lcMaxRms, 'cov_inflate', P.covInflate, 'seed', P.seed, 'M_sens', P.M_sens);
res = struct('scenario1', M1, 'monte_carlo', MC, 'sensitivity', SE, 'lidarslam', TB, 'cfg', cfg, 'matlab', version);
for i = 1:3, res.scenario1.(keys{i}) = rmfield(res.scenario1.(keys{i}), 'nees'); end
fid = fopen(fullfile(resDir, 'results.json'), 'w', 'n', 'UTF-8'); fprintf(fid, '%s', jsonencode(res, 'PrettyPrint', true)); fclose(fid);
C = fullfile(resDir, 'csv');
writematrix(OBS, fullfile(C, 'obstacles.csv'));
writematrix([td Xd], fullfile(C, 'traj_dense.csv'));
pk = @(Pc) [squeeze(Pc(1,1,:)) squeeze(Pc(1,2,:)) squeeze(Pc(2,2,:)) squeeze(Pc(3,3,:))];
writematrix([k Xt R1.Xdr pk(R1.Pdr) R1.Xnolc pk(R1.Pnolc) R1.Xslam pk(R1.Pslam) kf(:)], fullfile(C, 'poses.csv'));
Xon = zeros(N,3); Pon = zeros(N,4);
for q = 1:N, Xon(q,:) = R1.hist.X{q}(end,:); Pq = R1.hist.P{q}; Pon(q,:) = [Pq(1,1) Pq(1,2) Pq(2,2) Pq(3,3)]; end
writematrix([k Xon Pon], fullfile(C, 'online.csv'));
writematrix(R1.hist.lc - 1, fullfile(C, 'loop_closures.csv'));          % [k j] chỉ số từ 0
sc = cell2mat(cellfun(@(r) r(:)', R1.scans, 'UniformOutput', false)); sc(~isfinite(sc)) = -1;
writematrix([R1.ang(:)'; sc], fullfile(C, 'scans.csv'));                % dòng 1: góc tia; -1 = không phản xạ
fprintf('\nĐã lưu hình vào %s, kết quả vào %s\n', figDir, resDir);

%% 9. ANIMATION 2D (mô phỏng đồ họa trong MATLAB)
if P.doAnimation
    fa = figure('Name', 'Animation - Pose-Graph SLAM', 'Color', 'w', 'Position', [60 40 1250 600]);
    lcK = R1.hist.lc;
    for q = 2:N
        Xo = R1.hist.X{q}; Pk = R1.hist.P{q};
        subplot(1,2,1); cla; showGT(GT, .35); hold on;
        plot(Xt(1:q,1), Xt(1:q,2), 'k-', 'LineWidth', 1.3); plot(R1.Xdr(1:q,1), R1.Xdr(1:q,2), ':', 'Color', cols{1}, 'LineWidth', 1.3);
        plot(Xo(:,1), Xo(:,2), '-', 'Color', cols{3}, 'LineWidth', 1.7);
        [ex, ey] = ellipse95(R1.Xdr(q,1:2), R1.Pdr(1:2,1:2,q)); plot(ex, ey, '-', 'Color', cols{1});
        [ex, ey] = ellipse95(Xo(q,1:2), Pk(1:2,1:2)); plot(ex, ey, 'g-', 'LineWidth', 1.4);
        r = R1.scans{q}; m = isfinite(r); A = R1.ang(m) + Xo(q,3);
        plot(Xo(q,1) + r(m).*cos(A), Xo(q,2) + r(m).*sin(A), '.', 'Color', [1 .55 0], 'MarkerSize', 4);
        plotvehicle(Xo(q,:), 'size', 0.8, 'fillcolor', 'b');
        for z = find(lcK(:,1) == q)', plot(Xo([lcK(z,2) q],1), Xo([lcK(z,2) q],2), 'm-', 'LineWidth', 2.5); end
        title(sprintf('k = %d | sai số DR = %.2f m | SLAM = %.3f m', q-1, norm(R1.Xdr(q,1:2)-Xt(q,1:2)), norm(Xo(q,1:2)-Xt(q,1:2))));
        subplot(1,2,2); cla; hold on; grid on;
        ex_ = cellfun(@(x) x(end,1), R1.hist.X(1:q)) - Xt(1:q,1); sx_ = k95*sqrt(cellfun(@(p) p(1,1), R1.hist.P(1:q)));
        plot(0:q-1, R1.Xdr(1:q,1)-Xt(1:q,1), ':', 'Color', cols{1}, 'LineWidth', 1.2); plot(0:q-1, ex_, '-', 'Color', cols{3}, 'LineWidth', 1.3);
        plot(0:q-1, sx_, 'k--', 0:q-1, -sx_, 'k--'); xlim([0 N]); ylim([-1 1]*max(0.3, 2.2*sqrt(max(R1.Pdr(1,1,:)))));
        xlabel('Bước k'); ylabel('Sai số x [m]'); legend('DR', 'SLAM (online)', 'biên \pm2\sigma (95 %)', 'Location', 'northwest');
        title('Sai số x và hai đường biên 95 %'); drawnow limitrate;
    end
end

%% ======================= HÀM CỤC BỘ =======================
function R = runSLAM(P, seed, Xt, GT, RES, NG, keepHist) %#ok<INUSD>
    % Một lần mô phỏng đầy đủ: odometry nhiễu, LiDAR, DR/EKF, front-end, back-end.
    rng(seed); N = size(Xt,1);
    V = diag([P.sigma_d^2, P.sigma_lat^2, P.sigma_th^2]);          % nhiễu odometry (khung thân xe)
    zt = zeros(N-1,3); for k = 2:N, zt(k-1,:) = relpose(Xt(k-1,:), Xt(k,:)); end
    odo = zt + randn(N-1,3) * sqrt(V);                             % odometry đo được
    P0 = diag([1e-6 1e-6 1e-8]); Ainfo = inv(P0);
    [Xdr, Pdr] = deadReckoning(Xt(1,:), odo, V, P0);               % (A)
    ang = (-pi : P.angRes : pi - 1e-9)';
    scans = cell(N,1); pts = cell(N,1); nrm = cell(N,1); nok = cell(N,1);
    for k = 1:N
        A = ang + P.sigma_b*randn(size(ang)) + Xt(k,3);
        r = rayCast(GT, Xt(k,1:2), A, P.maxRange, RES);
        scans{k} = r + P.sigma_r*randn(size(r)); m = isfinite(scans{k});
        pts{k} = [scans{k}(m).*cos(ang(m)), scans{k}(m).*sin(ang(m))];
        [nrm{k}, nok{k}] = scanNormals(pts{k});
    end
    Oodo = inv(V);
    E.I = zeros(0,1); E.J = zeros(0,1); E.Z = zeros(0,3); E.OM = zeros(3,3,0); E.K = zeros(0,1);
    X = zeros(N,3); X(1,:) = Xt(1,:); Pn = zeros(3,3,N); Pn(:,:,1) = P0;
    hist.X = cell(N,1); hist.P = cell(N,1); hist.lc = zeros(0,2); hist.X{1} = X(1,:); hist.P{1} = P0;
    nTry = 0; tic;
    for k = 2:N
        z = odo(k-1,:); X(k,:) = oplus(X(k-1,:), z); E = addEdge(E, k-1, k, z, Oodo, 0); Sz = V;
        res = icpPL(pts{k}, pts{k-1}, nrm{k-1}, nok{k-1}, z);
        if ~isempty(res) && res.inlier > 0.7 && res.rms < P.lcMaxRms
            Oicp = inv(P.covInflate*res.cov + diag([1e-6 1e-6 1e-8])); E = addEdge(E, k-1, k, res.z, Oicp, 1);
            dz = res.z - z; dz(3) = wrapPi(dz(3)); zf = z + ((Oodo + Oicp) \ (Oicp*dz'))';
            X(k,:) = oplus(X(k-1,:), zf); Sz = inv(Oodo + Oicp);
        end
        zr = relpose(X(k-1,:), X(k,:)); th = X(k-1,3);
        J1 = [1 0 -sin(th)*zr(1)-cos(th)*zr(2); 0 1 cos(th)*zr(1)-sin(th)*zr(2); 0 0 1];
        J2 = blkdiag([cos(th) -sin(th); sin(th) cos(th)], 1);
        Pn(:,:,k) = J1*Pn(:,:,k-1)*J1' + J2*Sz*J2';
        added = false;
        if mod(k-1, P.lcEvery) == 0 && k > P.lcMinGap + 1
            cand = 1:(k - P.lcMinGap - 1); d = vecnorm(X(cand,1:2) - X(k,1:2), 2, 2); sel = find(d < P.lcRadius);
            if ~isempty(sel)
                [~, o_] = sort(d(sel)); cand = cand(sel(o_(1:min(2,numel(o_)))));
                for j = cand
                    nTry = nTry + 1; z0 = relpose(X(j,:), X(k,:)); best = [];
                    for dth0 = [0, deg2rad(5), -deg2rad(5)]
                        r_ = icpPL(pts{k}, pts{j}, nrm{j}, nok{j}, z0 + [0 0 dth0]);
                        if ~isempty(r_) && (isempty(best) || r_.inlier > best.inlier), best = r_; end
                        if ~isempty(best) && best.inlier > 0.85, break; end
                    end
                    if ~isempty(best) && best.inlier > P.lcMinInlier && best.rms < P.lcMaxRms
                        E = addEdge(E, j, k, best.z, inv(P.covInflate*best.cov + diag([1e-6 1e-6 1e-8])), 2);
                        hist.lc(end+1,:) = [k j]; added = true;
                    end
                end
            end
        end
        if added
            [Xo, H] = optimizePoseGraph(X(1:k,:), E, Xt(1,:), Ainfo, 10); X(1:k,:) = Xo; S = inv(full(H));
            for q = 1:k, Pn(:,:,q) = S(3*q-2:3*q, 3*q-2:3*q); end
        end
        hist.X{k} = X(1:k,:); hist.P{k} = Pn(:,:,k);
    end
    R.cpu = toc;
    [R.Xslam, R.H] = optimizePoseGraph(X, E, Xt(1,:), Ainfo, 30); R.Pslam = margCov(R.H);              % (C)
    Xinit = Xt(1,:); for k = 2:N, Xinit(k,:) = oplus(Xinit(k-1,:), odo(k-1,:)); end
    [R.Xnolc, Hn] = optimizePoseGraph(Xinit, subEdges(E, E.K ~= 2), Xt(1,:), Ainfo, 30); R.Pnolc = margCov(Hn); % (B)
    Xo_ = optimizePoseGraph(Xinit, subEdges(E, E.K == 0), Xt(1,:), Ainfo, 30);                      % (A') kiểm chứng
    R.drDiff = max(abs(Xo_(:) - Xdr(:)));
    R.Xdr = Xdr; R.Pdr = Pdr; R.E = E; R.scans = scans; R.ang = ang; R.hist = hist; R.nTry = nTry; R.Ainfo = Ainfo;
end
function m = metrics(X, Pc, Xt, k95)
    e = errs(X, Xt); N = size(X,1);
    s = sqrt([squeeze(Pc(1,1,:)), squeeze(Pc(2,2,:)), squeeze(Pc(3,3,:))]);
    inb = 100*mean(abs(e(2:end,:)) <= k95*s(2:end,:), 1);
    nees = zeros(1, N-1); for k = 2:N, nees(k-1) = e(k,1:2) / Pc(1:2,1:2,k) * e(k,1:2)'; end
    ep = hypot(e(:,1), e(:,2));
    [r1t, r1r] = rpe(X, Xt, 1); [r20t, r20r] = rpe(X, Xt, 20);
    m = struct('ate', sqrt(mean(ep.^2)), 'ate_max', max(ep), 'final', ep(end), 'head_rmse', rad2deg(sqrt(mean(e(:,3).^2))), ...
        'rpe1_t', r1t, 'rpe1_r', r1r, 'rpe20_t', r20t, 'rpe20_r', r20r, 'sig_final', sqrt(Pc(1,1,end) + Pc(2,2,end)), ...
        'sig_max', max(sqrt(squeeze(Pc(1,1,:) + Pc(2,2,:)))), 'sig_th_final_deg', rad2deg(s(end,3)), 'sig_th_max_deg', rad2deg(max(s(:,3))), ...
        'in_x', inb(1), 'in_y', inb(2), 'in_t', inb(3), 'in_ell', 100*mean(nees <= 5.991), 'anees', mean(nees), 'nees', nees);
end
function a = ateOf(X, Xt), e = errs(X, Xt); a = sqrt(mean(e(:,1).^2 + e(:,2).^2)); end
function e = errs(X, Xt), e = X - Xt; e(:,3) = wrapPi(e(:,3)); end
function [t, r] = rpe(X, Xt, dl)
    n = size(X,1) - dl; et = zeros(n,1); er = zeros(n,1);
    for i = 1:n
        err = relpose(relpose(Xt(i,:), Xt(i+dl,:)), relpose(X(i,:), X(i+dl,:)));
        et(i) = hypot(err(1), err(2)); er(i) = abs(err(3));
    end
    t = sqrt(mean(et.^2)); r = rad2deg(sqrt(mean(er.^2)));
end
function q = chi2q(p, nu)
    if exist('chi2inv', 'file'), q = chi2inv(p, nu); return; end
    z = sqrt(2)*erfinv(2*p - 1); q = nu*(1 - 2/(9*nu) + z*sqrt(2/(9*nu)))^3;    % Wilson-Hilferty
end
function boundPlot(k, e, s, c)
    hold on; grid on;
    fill([k; flipud(k)], [s; -flipud(s)], c, 'FaceAlpha', .12, 'EdgeColor', 'none');
    plot(k, s, 'k--', 'LineWidth', 1); plot(k, -s, 'k--', 'LineWidth', 1);
    plot(k, e, '-', 'Color', c, 'LineWidth', 1.2); xlim([k(1) k(end)]);
end
function saveFig(f, d, name), exportgraphics(f, fullfile(d, name), 'Resolution', 200); end
function a = wrapPi(a), a = mod(a + pi, 2*pi) - pi; end
function c = oplus(a, b)
    c = [a(1) + cos(a(3))*b(1) - sin(a(3))*b(2), a(2) + sin(a(3))*b(1) + cos(a(3))*b(2), wrapPi(a(3) + b(3))];
end
function z = relpose(a, b)
    d = b(1:2) - a(1:2); c = cos(a(3)); s = sin(a(3));
    z = [c*d(1) + s*d(2), -s*d(1) + c*d(2), wrapPi(b(3) - a(3))];
end
function [X, Pk] = deadReckoning(x0, odo, V, P0)
    % Odometry dạng tư thế tương đối z = (dx, dy, dθ) trong khung thân xe (tổng quát hóa
    % mô hình (δd, δθ) của slide; slide là trường hợp dy = 0). EKF dự đoán:
    %   x(k+1) = x(k) ⊕ z,   P(k+1) = Fx P Fx' + Fv V Fv'
    n = size(odo,1) + 1; X = zeros(n,3); Pk = zeros(3,3,n); X(1,:) = x0; Pk(:,:,1) = P0;
    for k = 1:n-1
        z = odo(k,:); th = X(k,3); c = cos(th); s = sin(th);
        X(k+1,:) = oplus(X(k,:), z);
        Fx = [1 0 -s*z(1) - c*z(2); 0 1 c*z(1) - s*z(2); 0 0 1];
        Fv = [c -s 0; s c 0; 0 0 1];
        Pk(:,:,k+1) = Fx*Pk(:,:,k)*Fx' + Fv*V*Fv';
    end
end
function r = rayCast(G, p, A, rmax, RES)
    step = RES/2; rr = step:step:rmax+step; ca = cos(A); sa = sin(A);
    hit = occ(G, p(1) + ca*rr, p(2) + sa*rr, RES); has = any(hit, 2); [~, first] = max(hit, [], 2);
    hi = rr(first)'; lo = max(hi - step, 0);
    for it = 1:6, mid = (lo + hi)/2; h = occ(G, p(1) + ca.*mid, p(2) + sa.*mid, RES); hi(h) = mid(h); lo(~h) = mid(~h); end
    r = hi; r(~has | r > rmax) = NaN;
end
function h = occ(G, x, y, RES)
    n = size(G,1); ix = floor(x/RES) + 1; iy = floor(y/RES) + 1; out = ix < 1 | iy < 1 | ix > n | iy > n;
    h = G(sub2ind(size(G), min(max(iy,1),n), min(max(ix,1),n))) | out;
end
function [Nm, ok] = scanNormals(p)
    n = size(p,1); Nm = zeros(n,2); ok = false(n,1); if n < 3, return; end
    prv = circshift(p, 1); nxt = circshift(p, -1); t = nxt - prv; L = vecnorm(t, 2, 2);
    ok = vecnorm(p - prv, 2, 2) < .35 & vecnorm(nxt - p, 2, 2) < .35 & L > 1e-6;
    Nm(ok,:) = [-t(ok,2), t(ok,1)] ./ L(ok);
end
function res = icpPL(src, dst, dN, dOK, x0)
    % ICP điểm-đường (Huber) - hiệp phương sai Σ ≈ s²(JᵀWJ)⁻¹ (xấp xỉ Laplace, Censi 2007)
    res = []; x = x0(:)'; Hh = eye(3); huber = 0.05;
    for it = 0:29
        R = [cos(x(3)) -sin(x(3)); sin(x(3)) cos(x(3))]; Pp = src*R' + x(1:2);
        D2 = sum(Pp.^2,2) + sum(dst.^2,2)' - 2*Pp*dst'; [d2, nn] = min(D2, [], 2); d = sqrt(max(d2,0));
        thr = max(.25, 1.2*0.8^it); m = d < thr & dOK(nn); if sum(m) < 30, return; end
        q = dst(nn(m),:); nq = dN(nn(m),:); ps = src(m,:); r = sum((Pp(m,:) - q).*nq, 2);
        c = cos(x(3)); s = sin(x(3)); dP = [-s*ps(:,1) - c*ps(:,2), c*ps(:,1) - s*ps(:,2)];
        Jc = [nq, sum(nq.*dP, 2)]; w = ones(size(r)); big = abs(r) > huber; w(big) = huber ./ abs(r(big));
        Hh = Jc' * (Jc .* w); g = Jc' * (w .* r); dx = -(Hh + 1e-9*eye(3)) \ g;
        x = x + dx'; x(3) = wrapPi(x(3));
        if norm(dx(1:2)) < 1e-5 && abs(dx(3)) < 1e-6 && thr <= .25, break; end
    end
    R = [cos(x(3)) -sin(x(3)); sin(x(3)) cos(x(3))]; Pp = src*R' + x(1:2);
    D2 = sum(Pp.^2,2) + sum(dst.^2,2)' - 2*Pp*dst'; [d2, nn] = min(D2, [], 2); d = sqrt(max(d2,0));
    m = d < .15 & dOK(nn); if sum(m) < 30, return; end
    r = sum((Pp(m,:) - dst(nn(m),:)).*dN(nn(m),:), 2); s2 = max(sum(r.^2)/max(sum(m)-3,1), 1e-6);
    res.z = x; res.cov = s2 * inv(Hh + 1e-9*eye(3)); res.inlier = mean(m); res.rms = sqrt(mean(r.^2));
end
function E = addEdge(E, i, j, z, Om, kind)
    E.I(end+1,1) = i; E.J(end+1,1) = j; E.Z(end+1,:) = z; E.OM(:,:,end+1) = Om; E.K(end+1,1) = kind;
end
function F = subEdges(E, m), F.I = E.I(m); F.J = E.J(m); F.Z = E.Z(m,:); F.OM = E.OM(:,:,m); F.K = E.K(m); end
function [e, A, B] = edgeJac(X, I, J, Z)
    % Grisetti (2010):  e_ij = [R_z'(R_i'(t_j - t_i) - t_z); θ_j - θ_i - θ_z],  Jacobian A_ij, B_ij
    m = numel(I); xi = X(I,:); xj = X(J,:);
    ci = cos(xi(:,3)); si = sin(xi(:,3)); cz = cos(Z(:,3)); sz = sin(Z(:,3)); dt = xj(:,1:2) - xi(:,1:2);
    u = [ci.*dt(:,1) + si.*dt(:,2), -si.*dt(:,1) + ci.*dt(:,2)]; v = u - Z(:,1:2);
    e = [cz.*v(:,1) + sz.*v(:,2), -sz.*v(:,1) + cz.*v(:,2), wrapPi(xj(:,3) - xi(:,3) - Z(:,3))];
    cc = cos(Z(:,3) + xi(:,3)); ss = sin(Z(:,3) + xi(:,3));
    du = [-si.*dt(:,1) + ci.*dt(:,2), -ci.*dt(:,1) - si.*dt(:,2)];
    dth = [cz.*du(:,1) + sz.*du(:,2), -sz.*du(:,1) + cz.*du(:,2)];
    A = zeros(3,3,m); B = zeros(3,3,m);
    A(1,1,:) = -cc; A(1,2,:) = -ss; A(2,1,:) = ss; A(2,2,:) = -cc; A(1,3,:) = dth(:,1); A(2,3,:) = dth(:,2); A(3,3,:) = -1;
    B(1,1,:) = cc;  B(1,2,:) = ss;  B(2,1,:) = -ss; B(2,2,:) = cc; B(3,3,:) = 1;
end
function [X, H, chi2] = optimizePoseGraph(X, E, xa, Ainfo, iters)
    % Gauss-Newton:  H Δx = -b,  H = Σ Jᵀ Ω J (ma trận thưa),  neo nút 1 bằng Ainfo
    n = size(X,1); chi2 = []; done = false;
    for it = 1:iters+1
        [e, A, B] = edgeJac(X, E.I, E.J, E.Z);
        OA = pagemtimes(E.OM, A); OB = pagemtimes(E.OM, B);
        Hii = pagemtimes(A, 'transpose', OA, 'none'); Hij = pagemtimes(A, 'transpose', OB, 'none'); Hjj = pagemtimes(B, 'transpose', OB, 'none');
        ee = reshape(e', 3, 1, []);
        bi = reshape(pagemtimes(OA, 'transpose', ee, 'none'), 3, []); bj = reshape(pagemtimes(OB, 'transpose', ee, 'none'), 3, []);
        [R1, C1, V1] = blocks(E.I, E.I, Hii); [R2, C2, V2] = blocks(E.I, E.J, Hij);
        [R3, C3, V3] = blocks(E.J, E.I, pagetranspose(Hij)); [R4, C4, V4] = blocks(E.J, E.J, Hjj);
        H = sparse([R1;R2;R3;R4], [C1;C2;C3;C4], [V1;V2;V3;V4], 3*n, 3*n);
        b = accumarray([reshape(3*(E.I'-1)+(1:3)',[],1); reshape(3*(E.J'-1)+(1:3)',[],1)], [bi(:); bj(:)], [3*n 1]);
        H(1:3,1:3) = H(1:3,1:3) + Ainfo; r0 = X(1,:) - xa; r0(3) = wrapPi(r0(3)); b(1:3) = b(1:3) + Ainfo*r0';
        chi2(end+1) = sum(reshape(pagemtimes(pagemtimes(ee, 'transpose', E.OM, 'none'), ee), [], 1)); %#ok<AGROW>
        if it == iters+1 || done, break; end
        dx = -(H \ b); X = X + reshape(dx, 3, [])'; X(:,3) = wrapPi(X(:,3));
        if max(abs(dx)) < 1e-6, done = true; end          % tính lại H tại nghiệm rồi dừng
    end
end
function [R, Cc, Vv] = blocks(I, J, Blk)
    m = numel(I); [a, b] = ndgrid(1:3, 1:3);
    R = reshape(3*(reshape(I,1,1,m)-1) + a, [], 1); Cc = reshape(3*(reshape(J,1,1,m)-1) + b, [], 1); Vv = Blk(:);
end
function Pc = margCov(H)
    S = inv(full(H)); n = size(H,1)/3; Pc = zeros(3,3,n);
    for q = 1:n, Pc(:,:,q) = S(3*q-2:3*q, 3*q-2:3*q); end        % Σ = H⁻¹ (khối biên từng nút)
end
function L = buildMap(X, scans, ang, rmax, RES, n)
    % Lưới log-odds, cập nhật hits / misses (Elfes 1989, Hess 2016)
    lh = log(.7/.3); lm = log(.4/.6); L = zeros(n,n); step = RES/2; s = (0:ceil(rmax/step)-1)*step;
    for k = 1:size(X,1)
        rg = scans{k}; A = ang + X(k,3); val = isfinite(rg); rr = rg; rr(~val) = rmax;
        msk = s < rr - RES/2; xs = X(k,1) + cos(A)*s; ys = X(k,2) + sin(A)*s;
        ix = floor(xs/RES) + 1; iy = floor(ys/RES) + 1; in = msk & ix >= 1 & iy >= 1 & ix <= n & iy <= n;
        miss = unique(sub2ind([n n], iy(in), ix(in)));
        hx = floor((X(k,1) + cos(A(val)).*rg(val))/RES) + 1; hy = floor((X(k,2) + sin(A(val)).*rg(val))/RES) + 1;
        hin = hx >= 1 & hy >= 1 & hx <= n & hy <= n; hit = unique(sub2ind([n n], hy(hin), hx(hin)));
        miss = setdiff(miss, hit);
        L(miss) = min(max(L(miss) + lm, -4), 4); L(hit) = min(max(L(hit) + lh, -4), 4);
    end
end
function M = mapMetrics(L, GT)
    Pm = 1 ./ (1 + exp(-L)); known = abs(Pm - .5) > .05; occ = Pm >= .65;
    TP = sum(occ(:) & GT(:) & known(:)); TN = sum(~occ(:) & ~GT(:) & known(:));
    FP = sum(occ(:) & ~GT(:) & known(:)); FN = sum(~occ(:) & GT(:) & known(:));
    M.acc = 100*(TP + TN)/max(TP + TN + FP + FN, 1); M.iou = 100*TP/max(TP + FP + FN, 1); M.known = known; M.occ = occ;
end
function [ex, ey] = ellipse95(mu, Cm)
    [Vv, D] = eig((Cm + Cm')/2); t = linspace(0, 2*pi, 60);
    Ept = Vv * sqrt(5.991*max(D, 0)) * [cos(t); sin(t)]; ex = mu(1) + Ept(1,:); ey = mu(2) + Ept(2,:);
end
function showGT(G, alpha)
    imagesc([0 20], [0 20], 1 - alpha*double(G)); set(gca, 'YDir', 'normal'); colormap(gca, gray); clim([0 1]);
    axis equal; xlim([0 20]); ylim([0 20]); xlabel('X [m]'); ylabel('Y [m]'); hold on;
end
