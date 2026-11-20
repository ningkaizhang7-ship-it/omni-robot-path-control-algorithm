function omr_animate(r, gifFile, fps, stride)
%OMR_ANIMATE  Animate the three-wheeled omnidirectional robot tracking the path.
%
%   omr_animate(r, 'out/anim.gif', 20, 40)
%   r      : result struct returned by omr_sim
%   fps    : frames per second of the resulting GIF (default 20)
%   stride : sample stride between frames (default: ~160 frames total)

if nargin < 3 || isempty(fps),    fps = 20;   end
if nargin < 4 || isempty(stride), stride = max(1, round(numel(r.t)/160)); end

Rb = 0.14;              % body radius [m]
Rw = 0.16;              % wheel ring radius [m]
zero = [cos(0) sin(0); cos(2*pi/3) sin(2*pi/3); cos(4*pi/3) sin(4*pi/3)];

lim = [min([r.q(1,:) r.qr(1,:)]) max([r.q(1,:) r.qr(1,:)]) ...
       min([r.q(2,:) r.qr(2,:)]) max([r.q(2,:) r.qr(2,:)])];
pad = 0.3;
xl = [lim(1)-pad, lim(2)+pad];
yl = [lim(3)-pad, lim(4)+pad];

fig = figure('Color', 'w', 'Position', [100 100 760 700], 'Visible', 'off');
ax = axes('Parent', fig); hold(ax, 'on'); grid(ax, 'on'); axis(ax, 'equal');
xlim(ax, xl); ylim(ax, yl);
xlabel(ax, 'x [m]'); ylabel(ax, 'y [m]');
title(ax, 'Three-wheeled omnidirectional robot — trajectory tracking');

% static reference
plot(ax, r.qr(1,:), r.qr(2,:), '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.1);

% growing trail
trail = plot(ax, r.q(1,1), r.q(2,1), '-', 'Color', [0.10 0.45 0.80], 'LineWidth', 1.6);

% robot graphics
hBody = patch(ax, 'XData', [], 'YData', [], 'FaceColor', [0.92 0.92 0.95], ...
              'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 1.2);
% gobjects 预分配图形对象句柄数组（句柄不是数值，不能用 zeros 预分配）
hWheel = gobjects(3,1);
for k = 1:3
    hWheel(k) = plot(ax, [0 0], [0 0], '-', 'Color', [0.15 0.15 0.15], 'LineWidth', 5);
end
hHead = plot(ax, [0 0], [0 0], '-', 'Color', [0.85 0.25 0.15], 'LineWidth', 2.2);
hDot  = plot(ax, 0, 0, 'o', 'MarkerSize', 7, 'MarkerFaceColor', [0.85 0.25 0.15], ...
             'MarkerEdgeColor', 'k');
hTxt = text(ax, xl(1)+0.05, yl(2)-0.12, '', 'FontName', 'Consolas', 'FontSize', 9, ...
            'VerticalAlignment', 'top', 'BackgroundColor', [1 1 1 0.85]);

th = linspace(0, 2*pi, 60);
first = true;
nf = 0;
for k = 1:stride:numel(r.t)
    x = r.q(1,k); y = r.q(2,k);
    if size(r.q, 1) >= 3
        a = r.q(3,k);
    else                       % heading not logged: use the direction of travel
        k2 = min(k + 5, numel(r.t));
        a = atan2(r.q(2,k2) - y, r.q(1,k2) - x);
        if ~isfinite(a), a = 0; end
    end
    set(hBody, 'XData', x + Rb*cos(th), 'YData', y + Rb*sin(th));
    for w = 1:3
        c  = [cos(a) -sin(a); sin(a) cos(a)]*zero(w,:)';   % wheel direction
        t  = [-c(2); c(1)];                                % tangent
        p0 = [x y] + 0.72*Rw*c';
        p1 = [x y] + 1.04*Rw*c';
        set(hWheel(w), 'XData', [p0(1) p1(1)] + 0.055*t(1), ...
                       'YData', [p0(2) p1(2)] + 0.055*t(2));
    end
    set(hHead, 'XData', [x x + 0.22*cos(a)], 'YData', [y y + 0.22*sin(a)]);
    set(hDot, 'XData', x, 'YData', y);
    set(trail, 'XData', r.q(1,1:k), 'YData', r.q(2,1:k));
    set(hTxt, 'String', sprintf('t = %5.2f s\n||e_1|| = %.4f\n|\\tau|_{max} = %6.1f N', ...
        r.t(k), norm(r.e1(:,k)), max(abs(r.tau(:,k)))));
    % drawnow limitrate：限制刷新频率，避免逐帧重绘拖慢渲染
    drawnow limitrate;
    fr = getframe(fig);
    % GIF 只支持索引色：先把抓到的 RGB 帧量化到 256 色调色板；
    % 'nodither' 表示不做抖动，颜色略糙但文件更小。
    [A, map] = rgb2ind(fr.cdata, 256, 'nodither');
    if first
        imwrite(A, map, gifFile, 'gif', 'LoopCount', Inf, 'DelayTime', 1/fps);
        first = false;
    else
        % WriteMode='append' 把这一帧追加到同一个 GIF；DelayTime 单位为秒。
        imwrite(A, map, gifFile, 'gif', 'WriteMode', 'append', 'DelayTime', 1/fps);
    end
    nf = nf + 1;
end
close(fig);
fprintf('animation: %s (%d frames, %.1f s)\n', gifFile, nf, nf/fps);
end
