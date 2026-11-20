function r = omr_sim(p, T, dt, verbose)
%OMR_SIM  Fixed-step RK4 simulation of the 22482 closed loop.
%
%   r = omr_sim(p, 80, 1e-3)  integrates 0..T with step dt (default 80 s, 1 ms)
%                             and returns the logged signals.

if nargin < 2 || isempty(T),  T = 80;    end
if nargin < 3 || isempty(dt), dt = 1e-3; end
if nargin < 4, verbose = false; end

nW = 0;
if p.use_nn, nW = 3*p.nn_nodes; end
x = [p.q0; p.V0; p.beta10; p.beta20; p.xi10; p.xi20; zeros(nW,1)];

% 控制器版本：p.controller = "v1"（第一版，不含 RBF 网络，Ghat = 0）
% / "v2"（第二版，独立 RBFNN 观测器）。两者只有 Ghat 的来源不同。
ctrl = "v1";
if isfield(p, 'controller'), ctrl = string(p.controller); end

N = round(T/dt) + 1;
t = (0:N-1)'*dt;

r.t  = t;
% 预分配日志数组（N = 采样点数），避免循环内反复扩容
r.q  = zeros(3,N);
r.V  = zeros(3,N);
r.e1 = zeros(3,N);
r.e2 = zeros(3,N);
r.tau = zeros(3,N);
r.sat = zeros(3,N);
r.F   = zeros(3,N);
r.alpha = zeros(3,N);
r.xi1 = zeros(3,N);
r.beta1 = zeros(3,N);
r.beta2 = zeros(3,N);
r.W   = zeros(nW, N);
% 两版都记录辨识值与真值：第一版的 Ghat 恒为 0，正好把"不补偿"画出来
r.Ghat = zeros(3, N);
r.G    = zeros(3, N);

h = dt;
tic;
i = p.idx;   % 状态切片索引结构体：p.idx.q = 1:3、p.idx.V = 4:6 ……（见 omr_params）
% 控制律在 RK 各中间级怎么取值 —— 这是与 Simulink 对齐的关键：
%   "stage"（默认）MATLAB Function 块是直通块，Simulink 在每个 RK 中间级都会
%                 用它当时的中间状态重算一次控制律，故这里也每级重算；
%   "hold"        只在本步起点算一次、整步冻结。相当于零阶保持，会把 4 阶精度
%                 降成 1 阶，0–5 s 与 Simulink 的偏差大 30 倍，仅用于对照。
if ~isfield(p, 'ctrl_mode'), p.ctrl_mode = "stage"; end
for k = 1:N
    tk = t(k);
    % 步首算一次控制律：只用于日志（记录的是本步起点的真实控制量）
    c = omr_ctrl_eval(tk, x, p);
    r.q(:,k)     = x(i.q);
    r.V(:,k)     = x(i.V);
    r.e1(:,k)    = c.e1;
    r.e2(:,k)    = c.e2;
    r.tau(:,k)   = c.tau;
    r.sat(:,k)   = c.sat_tau;
    r.F(:,k)     = c.F;
    r.alpha(:,k) = c.alpha;
    r.xi1(:,k)   = x(i.xi1);
    r.beta1(:,k) = x(i.b1);
    r.beta2(:,k) = x(i.b2);
    % RBF 权重紧跟在 18 个基础状态之后；use_nn 关闭时 nW = 0，此段跳过
    if nW > 0, r.W(:,k) = x(i.W); end
    r.Ghat(:,k) = c.Ghat;
    r.G(:,k)    = c.G;

    if k == N, break; end

    if p.ctrl_mode == "hold", cstep = c; else, cstep = []; end
    switch p.solver
        case "ode1"
            x = x + h*omr_rhs(tk, x, p, cstep);
        case "ode3"
            f1 = omr_rhs(tk,       x,          p, cstep);
            f2 = omr_rhs(tk + h/2, x + h/2*f1, p, cstep);
            f3 = omr_rhs(tk + h,   x - h*f1 + 2*h*f2, p, cstep);
            x = x + h/6*(f1 + 4*f2 + f3);
        otherwise   % ode4
            f1 = omr_rhs(tk,       x,          p, cstep);
            f2 = omr_rhs(tk + h/2, x + h/2*f1, p, cstep);
            f3 = omr_rhs(tk + h/2, x + h/2*f2, p, cstep);
            f4 = omr_rhs(tk + h,   x + h*f3,   p, cstep);
            x = x + h/6*(f1 + 2*f2 + 2*f3 + f4);
    end

    if any(~isfinite(x))
        warning('omr_sim:diverged', 'Non-finite state at t = %.4f s', tk);
        N = k;
        r = truncate(r, N);
        break
    end
    if verbose && mod(k, 10000) == 0
        fprintf('  t = %6.2f s  (%.1f s elapsed)\n', tk, toc);
    end
end
r.preset = p.preset;
r.dt = dt;
r.cpu = toc;
[qr, dqr] = omr_reference(r.t, p.dist_amp);
r.qr = qr;
r.dqr = dqr;
end

% -------------------------------------------------------------------------
function r = truncate(r, N)
% 发散时把已记录的日志截断到第 N 列。
% fieldnames 返回字段名元胞；r.(fn{i}) 是动态字段名语法，用来遍历结构体。
fn = fieldnames(r);
for i = 1:numel(fn)
    v = r.(fn{i});
    if isnumeric(v) && size(v, 2) == numel(r.t)
        r.(fn{i}) = v(:, 1:N);
    end
end
if isfield(r, 't'), r.t = r.t(1:N); end
if isfield(r, 'qr'), r.qr = r.qr(:, 1:N); end
if isfield(r, 'dqr'), r.dqr = r.dqr(:, 1:N); end
end
