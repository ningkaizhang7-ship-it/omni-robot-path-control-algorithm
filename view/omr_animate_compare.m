function omr_animate_compare(r1, r2, gifFile, fps, stride, tSwitch)
%OMR_ANIMATE_COMPARE  Overlay animation of two closed-loop runs (v1 vs v2).
%
%   omr_animate_compare(C.v1, C.v2, 'out/omr_compare.gif', 20)
%
%   Three panels:
%     left   full plane trajectory — both trails drawn on top of the reference
%     right  zoom on the reference point (±0.35 m) — this is where the few-cm
%            difference between the two runs becomes visible at all
%     top    ||e1||(t) for both runs, with the seeking/tracking phase split
%            marked by the shaded region and the moving cursor
%
%   r1, r2 : result structs returned by omr_sim, on the same time grid

if nargin < 4 || isempty(fps),      fps = 20;    end
if nargin < 5 || isempty(stride),   stride = max(1, round(numel(r1.t)/160)); end
if nargin < 6 || isempty(tSwitch),  tSwitch = r1.t(end)/2; end

t = r1.t(:).';
if numel(r2.t) ~= numel(t) || max(abs(r2.t(:).' - t)) > 1e-9
    % 两条轨迹必须同一时间栅格，否则先互插值到 r1 的栅格上
    r2.q  = interp1(r2.t(:), r2.q.',  t).';
    r2.qr = interp1(r2.t(:), r2.qr.', t).';
    r2.e1 = interp1(r2.t(:), r2.e1.', t).';
    r2.t  = t;
end

c1 = [0.85 0.33 0.10];      % v1 orange
c2 = [0.00 0.45 0.74];      % v2 blue
Rb = 0.13;                  % body radius in the zoom panel [m]

% ---- layout -------------------------------------------------------------
fig = figure('Color', 'w', 'Position', [80 80 1240 760], 'Visible', 'off');
axM = axes('Parent', fig, 'Position', [0.04 0.07 0.46 0.86]);
axZ = axes('Parent', fig, 'Position', [0.545 0.07 0.42 0.37]);
axE = axes('Parent', fig, 'Position', [0.545 0.545 0.42 0.35]);

% ---- main panel ---------------------------------------------------------
hold(axM, 'on'); grid(axM, 'on'); axis(axM, 'equal');
xall = [r1.q(1,:) r2.q(1,:) r1.qr(1,:)];
yall = [r1.q(2,:) r2.q(2,:) r1.qr(2,:)];
pad = 0.25;
xlim(axM, [min(xall)-pad, max(xall)+pad]);
ylim(axM, [min(yall)-pad, max(yall)+pad]);
xlabel(axM, 'x [m]'); ylabel(axM, 'y [m]');
title(axM, 'Trajectory: v1 (orange) vs v2 RBF (blue)');
plot(axM, r1.qr(1,:), r1.qr(2,:), '--', 'Color', [0.55 0.55 0.55], 'LineWidth', 1.1);
tr1 = plot(axM, r1.q(1,1), r1.q(2,1), '-', 'Color', c1, 'LineWidth', 1.8);
tr2 = plot(axM, r2.q(1,1), r2.q(2,1), '-', 'Color', c2, 'LineWidth', 1.8);
p1  = plot(axM, r1.q(1,1), r1.q(2,1), 'o', 'MarkerSize', 8, ...
           'MarkerFaceColor', c1, 'MarkerEdgeColor', 'k');
p2  = plot(axM, r2.q(1,1), r2.q(2,1), 'o', 'MarkerSize', 8, ...
           'MarkerFaceColor', c2, 'MarkerEdgeColor', 'k');
hl = legend(axM, [tr1, tr2], {'v1 (no observer)', 'v2 (RBFNN)'}, 'Location', 'southwest');
set(hl, 'Box', 'off');

% ---- zoom panel ---------------------------------------------------------
hold(axZ, 'on'); grid(axZ, 'on'); axis(axZ, 'equal');
title(axZ, 'Zoom on the reference point (same instant)');
xlabel(axZ, 'x [m]'); ylabel(axZ, 'y [m]');

th = linspace(0, 2*pi, 60);
refPt = plot(axZ, 0, 0, 'p', 'MarkerSize', 12, 'MarkerFaceColor', [0.3 0.3 0.3], ...
             'MarkerEdgeColor', 'k');
body1 = patch(axZ, 'XData', [], 'YData', [], 'FaceColor', 'none', ...
              'EdgeColor', c1, 'LineWidth', 2.0);
body2 = patch(axZ, 'XData', [], 'YData', [], 'FaceColor', c2, 'FaceAlpha', 0.15, ...
              'EdgeColor', c2, 'LineWidth', 1.8);
head1 = plot(axZ, [0 0], [0 0], '-', 'Color', c1, 'LineWidth', 2.0);
head2 = plot(axZ, [0 0], [0 0], '-', 'Color', c2, 'LineWidth', 2.0);
seg   = plot(axZ, [0 0], [0 0], '-', 'Color', [0.45 0.45 0.45], 'LineWidth', 1.2);

% ---- error panel --------------------------------------------------------
semilogy(axE, t, sqrt(sum(r1.e1.^2, 1)), 'Color', c1, 'LineWidth', 1.4); hold(axE, 'on');
semilogy(axE, t, sqrt(sum(r2.e1.^2, 1)), 'Color', c2, 'LineWidth', 1.4);
grid(axE, 'on');
set(axE, 'YScale', 'log');
yl = [1e-4, 1];
ylim(axE, yl);
xlim(axE, [0 t(end)]);
xlabel(axE, 't [s]'); ylabel(axE, '||e_1|| [m]');
title(axE, 'Tracking error norm (shaded = pure tracking phase)');
% 阴影标出"纯循迹"阶段（后一半），max 误差只在这一段统计
patch(axE, [tSwitch t(end) t(end) tSwitch], [yl(1) yl(1) yl(2) yl(2)], ...
      [0.2 0.6 0.2], 'FaceAlpha', 0.08, 'EdgeColor', 'none');
xline(axE, tSwitch, 'k--', 'LineWidth', 1.0);
ylim(axE, yl);
cur = plot(axE, [0 0], yl, 'k-', 'LineWidth', 0.8);
m1  = plot(axE, 0, 0, 'o', 'MarkerFaceColor', c1, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
m2  = plot(axE, 0, 0, 'o', 'MarkerFaceColor', c2, 'MarkerEdgeColor', 'k', 'MarkerSize', 5);
lg = legend(axE, {'v1', 'v2'}, 'Location', 'southwest'); set(lg, 'Box', 'off');

txt = annotation(fig, 'textbox', [0.04 0.925 0.46 0.06], 'String', '', ...
                 'FontName', 'Consolas', 'FontSize', 10, 'EdgeColor', 'none');

% ---- frames -------------------------------------------------------------
first = true; nf = 0;
for k = 1:stride:numel(t)
    x1 = r1.q(1,k); y1 = r1.q(2,k); a1 = r1.q(3,k);
    x2 = r2.q(1,k); y2 = r2.q(2,k); a2 = r2.q(3,k);

    % main panel: trails + current dots
    set(tr1, 'XData', r1.q(1,1:k), 'YData', r1.q(2,1:k));
    set(tr2, 'XData', r2.q(1,1:k), 'YData', r2.q(2,1:k));
    set(p1, 'XData', x1, 'YData', y1);
    set(p2, 'XData', x2, 'YData', y2);

    % 误差范数（缩放窗口与误差游标都要用，先算）
    e1 = norm(r1.e1(:,k)); e2 = norm(r2.e1(:,k));

    % zoom panel: 以参考点为中心，窗口随当前误差自适应（追赶段自动拉远）
    xr = r1.qr(1,k); yr = r1.qr(2,k);
    zr = max(0.32, 1.8*max([e1 e2]));      % 半宽：至少 32 cm（容得下两个车身），误差大时放宽
    xlim(axZ, [xr-zr, xr+zr]); ylim(axZ, [yr-zr, yr+zr]);
    set(refPt, 'XData', xr, 'YData', yr);
    set(seg, 'XData', [xr x1], 'YData', [yr y1]);
    set(body1, 'XData', x1 + Rb*cos(th), 'YData', y1 + Rb*sin(th));
    set(body2, 'XData', x2 + Rb*cos(th), 'YData', y2 + Rb*sin(th));
    set(head1, 'XData', [x1 x1 + 1.6*Rb*cos(a1)], 'YData', [y1 y1 + 1.6*Rb*sin(a1)]);
    set(head2, 'XData', [x2 x2 + 1.6*Rb*cos(a2)], 'YData', [y2 y2 + 1.6*Rb*sin(a2)]);

    % error cursor
    set(cur, 'XData', [t(k) t(k)]);
    set(m1, 'XData', t(k), 'YData', max(e1, yl(1)));
    set(m2, 'XData', t(k), 'YData', max(e2, yl(1)));

    if t(k) < tSwitch, phase = 'seeking'; else, phase = 'tracking'; end

    txt.String = sprintf(['t = %5.2f s   [%s]      ||e1||: v1 = %6.1f mm, v2 = %6.1f mm' ...
                          '      |tau|max: v1 = %5.1f, v2 = %5.1f N.m'], ...
        t(k), phase, 1e3*e1, 1e3*e2, ...
        max(abs(r1.tau(:,k))), max(abs(r2.tau(:,k))));

    drawnow limitrate;
    fr = getframe(fig);
    [A, map] = rgb2ind(fr.cdata, 256, 'nodither');
    if first
        imwrite(A, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', 1/fps);
        first = false;
    else
        imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 1/fps);
    end
    nf = nf + 1;
end
close(fig);
fprintf('compare animation: %s (%d frames, %.1f s)\n', gifFile, nf, nf/fps);
end
