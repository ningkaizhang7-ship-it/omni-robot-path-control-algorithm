function r = run_demo(T, dt)
%RUN_DEMO  第一版控制律（不含 RBF 网络）的完整复现：曲线 + 指标 + 动画。
%
%   run_demo()        % 80 s, 1 ms
%   run_demo(20, 1e-3)
%
%   误差统计分两段：前 T/2 是"寻找轨迹"（从 q0 追到参考线上），后 T/2 是
%   "纯循迹"（已在轨迹上）。表中 max|e1| 等峰值只在纯循迹段统计，全程值另列。
%
%   输出（omr/out/）：
%     fig_v1_tracking.png   轨迹、位置误差、速度误差、绝对偏差
%     fig_v1_control.png    力矩、速度、PNC、航向
%     metrics_v1.csv        指标一行
%     run_v1.mat            仿真日志（1/10 抽样）
%     omr_animation.gif     三轮车动画（20 fps）
%
%   第二版（RBFNN 观测器）与两版叠加动画见 run_compare_observer.m。

here = fileparts(mfilename('fullpath'));      % omr/app
omrRoot = fileparts(here);                    % omr
addpath(omrRoot);
omr_setup();
if nargin < 1 || isempty(T),  T = 80;    end
if nargin < 2 || isempty(dt), dt = 1e-3; end
tSeek = T/2;                                   % 寻找/循迹分界
outDir = fullfile(omrRoot, 'out');
if ~exist(outDir, 'dir'), mkdir(outDir); end

% ---- simulate -----------------------------------------------------------
p = omr_params("v1");
p.solver = "ode4";
r = omr_sim(p, T, dt);
m = metrics(r, tSeek);
fprintf('%-6s %9s %10s %9s %9s %9s %8s\n', 'run', 'rms_all', 'rms_track', ...
    'max_seek', 'max_track', 'end|e1|', 'max|tau|');
fprintf('%-6s %9.4f %10.4f %9.4f %9.4f %9.4f %8.1f\n', 'v1', ...
    m.rmsE1, m.rmsE1t, m.maxE1s, m.maxE1t, m.endE1, m.maxTaut);
fprintf('（max_seek = 0–%.0f s；其余为 %.0f–%.0f s 纯循迹段）\n', tSeek, tSeek, T);

% ---- persist ------------------------------------------------------------
save(fullfile(outDir, 'run_v1.mat'), 'r', '-v7');
fid = fopen(fullfile(outDir, 'metrics_v1.csv'), 'w');
fprintf(fid, ['preset,rms_e1_all,rms_e1_track,max_e1_seek,max_e1_track,' ...
              'end_e1,max_e2_track,max_tau_track,cpu_s\n']);
fprintf(fid, 'v1,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.2f\n', ...
    m.rmsE1, m.rmsE1t, m.maxE1s, m.maxE1t, m.endE1, m.maxE2t, m.maxTaut, r.cpu);
fclose(fid);

% ---- figures ------------------------------------------------------------
make_fig_tracking(r, outDir, tSeek);
make_fig_control(r, outDir, tSeek);

% ---- animation ----------------------------------------------------------
% 第 3 个参数是 GIF 的播放帧率（不是仿真步长）；80 会让 161 帧在 2 s 内放完
omr_animate(r, fullfile(outDir, 'omr_animation.gif'), 20);
fprintf('done: %s\n', outDir);
end

% -------------------------------------------------------------------------
function m = metrics(r, tSeek)
% 全程统计（含 0.4 m 的起始瞬态），只作参考
m.rmsE1 = sqrt(mean(r.e1(:).^2));
m.endE1 = norm(r.e1(:, end));
% 分段：seek = 起始追线；track = 纯循迹（峰值指标只统计这一段）
ws = r.t <  tSeek;
wt = r.t >= tSeek;
m.rmsE1t = sqrt(mean(reshape(r.e1(:, wt), [], 1).^2));
m.maxE1s = max(abs(reshape(r.e1(:, ws), [], 1)));
m.maxE1t = max(abs(reshape(r.e1(:, wt), [], 1)));
m.maxE2t = max(abs(reshape(r.e2(:, wt), [], 1)));
m.maxTaut = max(abs(reshape(r.tau(:, wt), [], 1)));
end

% -------------------------------------------------------------------------
function make_fig_tracking(r, outDir, tSeek)
[qr, ~] = omr_reference(r.t);
c = lines(3);
f = figure('Color', 'w', 'Position', [80 80 1180 780]);

subplot(2,2,1);
plot(qr(1,:), qr(2,:), 'k--', 'LineWidth', 1.1); hold on;
plot(r.q(1,:), r.q(2,:), 'Color', c(1,:), 'LineWidth', 1.8);
plot(r.q(1,1), r.q(2,1), 'o', 'MarkerSize', 8, 'MarkerFaceColor', c(3,:), 'MarkerEdgeColor', 'k');
plot(r.q(1,end), r.q(2,end), 's', 'MarkerSize', 8, 'MarkerFaceColor', c(2,:), 'MarkerEdgeColor', 'k');
grid on; axis equal; xlabel('x [m]'); ylabel('y [m]');
title('(a) Plane trajectory'); legend({'reference','robot','start','end'}, 'Location', 'southeast');

subplot(2,2,2);
plot(r.t, r.e1, 'LineWidth', 1.3); hold on;
patch([tSeek r.t(end) r.t(end) tSeek], [-0.6 -0.6 0.6 0.6], ...
      [0.2 0.6 0.2], 'FaceAlpha', 0.07, 'EdgeColor', 'none');
yline(0.5, 'r--'); yline(-0.5, 'r--');
xline(tSeek, 'k--', 'LineWidth', 1.0);
grid on; xlabel('t [s]'); ylabel('e_1'); ylim([-0.6 0.6]);
title('(b) Tracking error e_1 (shaded: pure tracking)');
legend({'x','y','\theta','bound'}, 'Location', 'northeast');

subplot(2,2,3);
plot(r.t, r.e2, 'LineWidth', 1.3); hold on;
patch([tSeek r.t(end) r.t(end) tSeek], [-1 -1 1 1], ...
      [0.2 0.6 0.2], 'FaceAlpha', 0.07, 'EdgeColor', 'none');
yline(0.5, 'r--'); yline(-0.5, 'r--');
xline(tSeek, 'k--', 'LineWidth', 1.0);
grid on; xlabel('t [s]'); ylabel('e_2'); ylim([-1 1]);
title('(c) Velocity error e_2'); legend({'x','y','\theta','bound'}, 'Location', 'northeast');

subplot(2,2,4);
plot(r.t, r.q(1,:) - qr(1,:), r.t, r.q(2,:) - qr(2,:), r.t, r.q(3,:) - qr(3,:), 'LineWidth', 1.2);
hold on; xline(tSeek, 'k--', 'LineWidth', 1.0);
grid on; xlabel('t [s]'); ylabel('q - q_r');
title('(d) Absolute tracking deviation'); legend({'x','y','\theta'}, 'Location', 'northeast');

print(f, fullfile(outDir, 'fig_v1_tracking.png'), '-dpng', '-r140');
close(f);
end

% -------------------------------------------------------------------------
function make_fig_control(r, outDir, tSeek)
f = figure('Color', 'w', 'Position', [80 80 1180 780]);
subplot(2,2,1);
plot(r.t, r.tau, 'LineWidth', 1.1); hold on;
plot(r.t, r.sat, '--', 'LineWidth', 0.9);
grid on; xlabel('t [s]'); ylabel('\tau [N]');
title('(a) Commanded and saturated torque');
legend({'\tau_x','\tau_y','\tau_\theta'}, 'Location', 'best');

subplot(2,2,2);
plot(r.t, r.V, 'LineWidth', 1.2); grid on;
xlabel('t [s]'); ylabel('V'); title('(b) Body velocities');
legend({'v_x','v_y','\omega'}, 'Location', 'northeast');

subplot(2,2,3);
plot(r.t, r.F, 'LineWidth', 1.2); hold on; yline(1, 'k:'); yline(-1, 'k:');
grid on; xlabel('t [s]'); ylabel('F'); title('(c) PNC activation function');

subplot(2,2,4);
plot(r.t, r.q(3,:), 'LineWidth', 1.2); hold on;
plot(r.t, r.qr(3,:), 'k--', 'LineWidth', 1.0);
grid on; xlabel('t [s]'); ylabel('\theta [rad]');
title('(d) Heading tracking'); legend({'actual','reference'}, 'Location', 'best');

print(f, fullfile(outDir, 'fig_v1_control.png'), '-dpng', '-r140');
close(f);
end
