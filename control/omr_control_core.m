function c = omr_control_core(t, x, p, variant)
%OMR_CONTROL_CORE  One evaluation of the 22482 control law (shared core).
%
%   c = omr_control_core(t, x, p, variant)
%
%   variant 决定"未知项 G = A1*V + F(V)"怎么来（这是两个版本唯一的差别）：
%     "v1"    第一版：不做 RBF 补偿，Ghat = 0（未知项完全交给控制律的鲁棒项吸收）。
%     "v2"    第二版：Ghat 由独立的 RBFNN 辨识器给出（式 9 + 35 + 投影）。
%
%   两个版本共用下面的全部方程（式 6、8、10、11、12、16、18、22、24、25、34），
%   只是 Ghat 的来源不同，所以控制效果的差异可以干净地归因到观测器本身。
%
%   控制器状态在 x 中的位置由 p.idx 给出（见 omr_params）：
%     beta1、beta2（饱和补偿）、xi1、xi2（二阶滑模微分器）、W（RBF 权重，仅 v2）。

i = p.idx;
q   = x(i.q);
V   = x(i.V);
b1v = x(i.b1);      % beta1（饱和补偿状态）
b2v = x(i.b2);      % beta2
xi1 = x(i.xi1);
xi2 = x(i.xi2);

% 参考轨迹与它的一阶导数；扰动 d 只进被控对象，控制器不需要（也不再传 dist_amp）
[qr, dqr] = omr_reference(t, 0);
th = q(3);
S = [cos(th), -sin(th), 0; sin(th), cos(th), 0; 0, 0, 1];

% ---- step 1: kinematic error and virtual control (eq. 11, 18) -----------
e1 = q - qr - b1v;
alpha = omr_alpha(S, e1, dqr, b1v, p);

% ---- step 2: dynamic error (eq. 12) ------------------------------------
e2 = V - alpha - b2v;

% ---- barrier functions and their gradients (eq. 16) --------------------
% 与 .slx 的 MATLAB Function4 一致：cos 项先加 1e-8 再平方，
% 这样分母的零点从 cos=0 挪到 cos=-1e-8，极端情况下与大模型逐个样本对齐。
lam1 = e1./((cos(pi*e1.^2./(2*p.bd1.^2)) + 1e-8).^2);
lam2 = e2./((cos(pi*e2.^2./(2*p.bd2.^2)) + 1e-8).^2);
% 可选上限：|e2| -> b2 时 lam2 无界，控制律里的 -1.5*lam2 会给出上千 N·m。
% 物理上执行器只有 +-tau_max，把 lam2 钳住等价于承认"障碍在饱和下不可达"。
if isfield(p, 'lam2_max') && isfinite(p.lam2_max)
    lam2 = min(max(lam2, -p.lam2_max), p.lam2_max);
end

% ---- tan-type BLF feedback on e2 ---------------------------------------
% 式 (25) 的第一项：-k2*b2^2*sin(pi*e2^2/b2^2)/(2*pi*e2)。
% 分子在 e2 = 0 处是 0 的二阶小量，只要分母不为 0 就无奇点；
% 这里只把 |e2| < 1e-12 的分母抬到 1e-12（此时分子也是 0），不再用分段近似式。
e2g = e2 + (abs(e2) < 1e-12)*1e-12;
g = p.k2.*p.bd2.^2.*sin(pi*e2.^2./p.bd2.^2)./(2*pi*e2g);

% ---- unknown dynamics: Ghat from the RBFNN observer, or zero -------------
FV = [p.a2*V(3)*V(2); -p.a2*V(3)*V(1); 0];
G  = p.A1*V + FV;                      % 真值，仅供诊断（不进控制律）
phi_all = zeros(p.nn_nodes, 3);
obsinfo = struct('Wnorm', 0, 'proj', false, 'phi_max', 0);

switch variant
    case "v1"
        % 第一版：没有观测器，未知项不补偿
        Ghat = zeros(3,1);
        dW   = zeros(0,1);
    case "v2"
        [Ghat, phi_all, dW, obsinfo] = omr_rbfnn_observer_v2(V, x(i.W), lam2, p);
    otherwise
        error('omr_control_core:badVariant', 'Unknown controller variant "%s".', variant);
end

% ---- filter derivative (eq. 24) ----------------------------------------
arg_f = xi1 - alpha;
dxi1 = -p.r1*sqrt(abs(arg_f)).*sign(arg_f) + xi2;
dxi2 = -p.r2*sign(arg_f);

% ---- control torque (eq. 25 + 34) --------------------------------------
% 式 (25) 的 λ1/λ2 项：论文写的是逐元素相除。两个 3×1 向量之间的 "/" 在 MATLAB
% 里是矩阵右除（返回 3×3，等价于只取 argmax|λ2| 那一个通道），是原 .slx 的写法
% 缺陷，这里不再保留。λ2 -> 0 时该项是真奇点，因此给分母一个下限、给比值一个上限
% （p.lam2_floor / p.ratio_max），否则 80 s 内会给出千牛·米级冲激。
lam2s = sign(lam2 + (lam2 == 0)).*max(abs(lam2), p.lam2_floor);
ratio = lam1./lam2s;
ratio = min(max(ratio, -p.ratio_max), p.ratio_max);
ratio_term = ratio.*(S*e2);
tau = p.Binv*(-g - ratio_term - p.L2*b2v - Ghat + dxi1 - 1.5*lam2);

% ---- PNC safety layer (thesis, section 3.5) ----------------------------
if p.pnc_on
    Fv = pnc_shape(S*V, p);
    dVr = S\dqr;                        % 复用上面已算过的 dqr，不再重复调 omr_reference
    z = dVr - p.k2.*e2 - p.k1.*e1;
    Aa = diag(p.pnc_lamb + abs(z));
    % 注意：PNC 用的是 Binv_mat（归一化轮矩阵 [-1 -1 2; ...] 的逆），
    % 与主控制律里的 Binv（由物理参数导出的 B 的逆）不是同一个矩阵。
    tau = tau + p.Binv_mat*p.M*Aa*Fv;
else
    Fv = zeros(3,1);
end

% ---- saturation and the auxiliary compensation system (eq. 6, 10) ------
sat_tau = min(max(tau, p.tau_min), p.tau_max);
dtau = tau - sat_tau;

db1 = -p.L1*b1v + S*b2v;
db2 = -p.L2*b2v + p.B*dtau;

c.tau = tau;
c.sat_tau = sat_tau;
c.alpha = alpha;
c.e1 = e1;
c.e2 = e2;
c.lam1 = lam1;
c.lam2 = lam2;
c.F = Fv;
c.Ghat = Ghat;
c.G = G;
c.phi = phi_all;
c.Wnorm = obsinfo.Wnorm;
c.proj = obsinfo.proj;
c.dx = [db1; db2; dxi1; dxi2; dW];
c.dxi1 = dxi1;
c.dxi2 = dxi2;
c.S = S;
end

% -------------------------------------------------------------------------
function Fv = pnc_shape(dq, p)
% PNC 的分段速度边界函数：速度进入 ±0.6 附近的过渡带时输出 0→±1 的平滑量，越界后饱和到 ±1。
% 用 cos 过渡而不是硬切换，是为了避免力矩突跳。
% Piecewise velocity-boundary function of the PNC layer (thesis eq. 3.21).
Fv = zeros(3,1);
for k = 1:3
    lo = p.pnc_dqL(k); up = p.pnc_dqU(k); de = p.pnc_Del(k);
    if dq(k) < lo
        Fv(k) = 1;
    elseif dq(k) < lo + de
        Fv(k) = (1 - cos(pi/de*(dq(k) + lo - de)))/2;
    elseif dq(k) <= up - de
        Fv(k) = 0;
    elseif dq(k) <= up
        Fv(k) = (-1 + cos(pi/de*(dq(k) - up + de)))/2;
    else
        Fv(k) = -1;
    end
end
end
