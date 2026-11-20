function C = run_compare_observer(T, dt, makeGif)
%RUN_COMPARE_OBSERVER  第一版 vs 第二版（RBFNN 观测器）的控制效果对照。
%
%   C = run_compare_observer()             % 80 s、1 ms、含叠加动画
%   C = run_compare_observer(20, 1e-3)     % 短时试跑
%   C = run_compare_observer(80, 1e-3, false)   % 跳过动画
%
%   两版用**完全相同的**参数（omr_params：k1=0.5、k2=0.1、障碍边界 0.5、
%   λ2 下限 1e-3、比值上限 50、被控对象吃 sat(tau)），唯一的变量是控制律里
%   未知项 G = A1*V + F(V) 怎么来：
%
%     v1   omr_control      Ghat = 0，不补偿（第一版，修正后）
%     v2   omr_control_v2   Ghat = phi'*What，权重按式 (35) 自适应（第二版，RBFNN）
%
%   **误差统计分两段**：前 T/2 是"寻找轨迹"（从 q0=[0.4;-0.4;0.3] 追到参考线上，
%   误差 0.4 m 量级），后 T/2 是"纯循迹"（已在轨迹上，误差 cm 量级）。
%   表中 max|e1| 等峰值指标只在纯循迹段统计，全程值另列作参考。
%
%   输出（都在 omr/out/）：
%     compare_v1_v2.csv     两版的指标表（含分段统计）
%     metrics_v2.csv        第二版的指标（与 run_demo 的 metrics_v1.csv 对称）
%     run_v2.mat            两版的仿真日志（1/10 抽样）
%     fig_v1_vs_v2.png      四子图对照
%     omr_compare.gif       两版叠加动画（同一时刻的机器人 + 误差游标）

if nargin < 3 || isempty(makeGif), makeGif = true; end

here = fileparts(mfilename('fullpath'));      % omr/app
omrRoot = fileparts(here);                    % omr
addpath(omrRoot);
omr_setup();
outDir = fullfile(omrRoot, 'out');
if ~exist(outDir, 'dir'), mkdir(outDir); end
if nargin < 1 || isempty(T),  T = 80;    end
if nargin < 2 || isempty(dt), dt = 1e-3; end
tSeek = T/2;                                  % 寻找/循迹分界

% ---- 两版配置：只有 p.controller 不同 -----------------------------------
names = ["v1", "v2"];
C = struct();
C.tSeek = tSeek;
rows = cell(0, 10);
fprintf('%-4s %9s %10s %9s %9s %9s %9s %9s %7s %8s\n', 'run', 'rms_all', ...
    'rms_track', 'max_seek', 'max_track', 'end|e1|', 'max|e2|t', 'max|tau|t', 'sat[%]', 'rms|dG|');
for k = 1:numel(names)
    p = omr_params(names(k));
    p.solver = "ode4";
    r = omr_sim(p, T, dt);
    C.(names(k)) = r;
    m = metrics(r, tSeek);
    row = {char(names(k)), m.rmsE1, m.rmsE1t, m.maxE1s, m.maxE1t, m.endE1, ...
           m.maxE2t, m.maxTaut, m.satPct, m.rmsDG};
    rows(end+1, :) = row; %#ok<AGROW>
    fprintf('%-4s %9.4f %10.4f %9.4f %9.4f %9.4f %9.3f %9.1f %7.2f %8.4f\n', row{:});
end
fprintf('（max_seek = 0–%.0f s 寻找轨迹段；其余峰值/稳态指标均为 %.0f–%.0f s 纯循迹段）\n', ...
    tSeek, tSeek, T);

% ---- 观测器补回了多少 ---------------------------------------------------
m1 = metrics(C.v1, tSeek); m2 = metrics(C.v2, tSeek);
gainTrack = 100*(m1.rmsE1t - m2.rmsE1t)/m1.rmsE1t;
gainMax   = 100*(m1.maxE1t - m2.maxE1t)/m1.maxE1t;
gainEnd   = 100*(m1.endE1  - m2.endE1 )/max(m1.endE1, eps);
gainTau   = 100*(m1.maxTaut - m2.maxTaut)/m1.maxTaut;
fprintf('\n纯循迹段（t >= %.0f s）：\n', tSeek);
fprintf('  rms|e1|    v1 %.4f → v2 %.4f  (%.1f%%)\n', m1.rmsE1t, m2.rmsE1t, gainTrack);
fprintf('  max|e1|    v1 %.4f → v2 %.4f  (%.1f%%)\n', m1.maxE1t, m2.maxE1t, gainMax);
fprintf('  末端|e1|   v1 %.4f → v2 %.4f  (%.1f%%)\n', m1.endE1,  m2.endE1,  gainEnd);
fprintf('  max|tau|   v1 %.1f   → v2 %.1f    (%.1f%%)\n', m1.maxTaut, m2.maxTaut, gainTau);
C.gains = [gainTrack gainMax gainEnd gainTau];

% ---- 落盘 ---------------------------------------------------------------
save(fullfile(outDir, 'run_v2.mat'), 'C', '-v7');
fid = fopen(fullfile(outDir, 'compare_v1_v2.csv'), 'w');
fprintf(fid, ['run,rms_e1_all,rms_e1_track,max_e1_seek,max_e1_track,end_e1,' ...
              'max_e2_track,max_tau_track,sat_pct,rms_dG\n']);
for k = 1:size(rows, 1)
    fprintf(fid, '%s,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.3f,%.6f\n', rows{k, :});
end
fclose(fid);
fid = fopen(fullfile(outDir, 'metrics_v2.csv'), 'w');
fprintf(fid, ['preset,rms_e1_all,rms_e1_track,max_e1_seek,max_e1_track,end_e1,' ...
              'max_e2_track,max_tau_track,cpu_s\n']);
fprintf(fid, 'v2,%.6f,%.6f,%.6f,%.6f,%.6f,%.3f,%.3f,%.2f\n', ...
    m2.rmsE1, m2.rmsE1t, m2.maxE1s, m2.maxE1t, m2.endE1, m2.maxE2t, m2.maxTaut, C.v2.cpu);
fclose(fid);

make_fig(C, outDir);

if makeGif
    omr_animate_compare(C.v1, C.v2, fullfile(outDir, 'omr_compare.gif'), 20, [], tSeek);
end
fprintf('done: %s\n', outDir);
end

% -------------------------------------------------------------------------
function m = metrics(r, tSeek)
% 全程统计（含 0.4 m 的大瞬态），只作参考
m.rmsE1  = sqrt(mean(r.e1(:).^2));
m.maxE1  = max(abs(r.e1(:)));
m.endE1  = norm(r.e1(:, end));
% 分段：seek = 起始追线阶段；track = 纯循迹阶段（峰值指标只统计这一段）
ws = r.t <  tSeek;      % 寻找轨迹
wt = r.t >= tSeek;      % 纯循迹
m.rmsE1t = sqrt(mean(reshape(r.e1(:, wt), [], 1).^2));
m.maxE1s = max(abs(reshape(r.e1(:, ws), [], 1)));
m.maxE1t = max(abs(reshape(r.e1(:, wt), [], 1)));
m.maxE2t = max(abs(reshape(r.e2(:, wt), [], 1)));
m.maxTaut = max(abs(reshape(r.tau(:, wt), [], 1)));
% 饱和占比：|tau - sat| > 1e-9 的采样点比例（%）
m.satPct = 100*mean(abs(r.tau(:) - r.sat(:)) > 1e-9);
dG = r.G - r.Ghat;
m.rmsDG = sqrt(mean(dG(:).^2));
end

% -------------------------------------------------------------------------
function make_fig(C, outDir)
names = ["v1", "v2"];
c = [0.85 0.33 0.10;    % v1 橙
     0.00 0.45 0.74];   % v2 蓝
tSeek = C.tSeek;
f = figure('Color', 'w', 'Position', [60 60 1240 820]);

% (a) 跟踪误差模长（阴影 = 纯循迹段，max 只在这一段统计）
subplot(2,2,1); hold on; grid on;
yl = [1e-4 1];
for k = 1:numel(names)
    r = C.(names(k));
    plot(r.t, sqrt(sum(r.e1.^2, 1)), 'Color', c(k,:), 'LineWidth', 1.4);
end
patch([tSeek C.v1.t(end) C.v1.t(end) tSeek], [yl(1) yl(1) yl(2) yl(2)], ...
      [0.2 0.6 0.2], 'FaceAlpha', 0.07, 'EdgeColor', 'none');
xline(tSeek, 'k--', 'LineWidth', 1.0);
set(gca, 'YScale', 'log'); ylim(yl); xlim([0 C.v1.t(end)]);
xlabel('t [s]'); ylabel('||e_1|| [m]');
title('(a) Tracking error (shaded = pure tracking phase)');
legend({'v1', 'v2'}, 'Location', 'southwest');

% (b) 力矩幅值
subplot(2,2,2); hold on; grid on;
for k = 1:numel(names)
    r = C.(names(k));
    plot(r.t, max(abs(r.tau), [], 1), 'Color', c(k,:), 'LineWidth', 1.1);
end
set(gca, 'YScale', 'log');
xlabel('t [s]'); ylabel('max |\tau| [N\cdotm]');
title('(b) Control torque');
legend({'v1', 'v2'}, 'Location', 'best');

% (c) 未知项真值 vs 辨识值
subplot(2,2,3); hold on; grid on;
w = C.v2.t <= 20;
plot(C.v2.t(w), C.v2.G(1,w), 'k-', 'LineWidth', 1.6);
plot(C.v2.t(w), C.v2.Ghat(1,w), 'Color', c(2,:), 'LineWidth', 1.4);
xlabel('t [s]'); ylabel('G_x [m/s^2]');
title('(c) Unknown term G_x and its estimate');
legend({'G_x (true)', 'Ghat_x (RBFNN)'}, 'Location', 'best');

% (d) 指标柱状对比（只统计纯循迹段）
subplot(2,2,4); hold on; grid on;
mAt = zeros(3, numel(names));
for k = 1:numel(names)
    m = metrics(C.(names(k)), tSeek);
    mAt(:, k) = [m.rmsE1t; m.endE1; m.maxE1t];
end
b = bar(mAt, 'grouped');
for k = 1:numel(b)
    b(k).FaceColor = c(min(k, size(c,1)), :);
end
set(gca, 'YScale', 'log');
set(gca, 'XTick', 1:3, 'XTickLabel', {'rms |e_1|', 'end |e_1|', 'max |e_1|'});
ylabel('error [m] (t \geq T/2)');
legend(cellstr(names), 'Location', 'northwest');
title('(d) Error metrics, pure tracking phase');

print(f, fullfile(outDir, 'fig_v1_vs_v2.png'), '-dpng', '-r140');
end
