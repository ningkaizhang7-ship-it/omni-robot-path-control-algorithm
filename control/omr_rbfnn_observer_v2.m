function [Ghat, phi, dW, info] = omr_rbfnn_observer_v2(V, W, lam2, p)
%OMR_RBFNN_OBSERVER_V2  RBF 神经网络模型辨识器（论文 22482 假设 3、4，式 (9)、(35)）
%
%   [Ghat, phi, dW, info] = omr_rbfnn_observer_v2(V, W, lam2, p)
%
%   论文里并没有一个独立的"状态观测器"：假设 3 说明被控对象参数未知，
%   未建模项 G_i = [A1*V + F(V)]_i 是未知连续函数；假设 4 说明用 RBF 网络
%   逼近它时误差 δ_i 有界。于是"观测器"实际是一个直接自适应辨识器：
%
%       整定模型：  G_i   = (W_i.'  * phi_i) + delta_i        (9)
%       估计模型：  Ghat_i = (What_i.' * phi_i)
%       权重更新：  dWhat_i = rho*(-rho_w*What_i + lam2_i*phi_i)   (35)
%
%   估计值 Ghat 直接进入控制律 (25)（取代未知的 G），权重 What 是闭环状态
%   的一部分（x(19:18+3n)），由积分器推进，因此本函数只返回导数 dW。
%
%   输入：
%     V     3x1  机体速度
%     W     3n x1 当前权重（按轴分块：第 k 块是 What_k）
%     lam2  3x1  障碍函数 λ2，为式 (35) 提供驱动信号
%     p     参数结构体（nn_nodes / nn_c / nn_sigma / rho / rho_w / nn_Wmax）
%
%   输出：
%     Ghat  3x1  未知项估计 Ghat = What.'*phi
%     phi   n x3 各轴的基函数取值（调试用）
%     dW    3n x1 权重导数（式 35）
%     info  诊断：各轴权重范数、投影触发次数、基函数峰值
%
%   注意：基函数的输入取该轴的速度 V(k)（单输入、n 个高斯节点），这与随论文
%   交付的 Simulink 模型完全一致；若要把三轴耦合 [vx vy w] 纳入基函数，
%   只需把下面两处 phi 的算法改成 exp(-||V - c_i||^2/(2*sigma^2))。

n  = p.nn_nodes;
Ghat = zeros(3,1);
phi  = zeros(n,3);
dW   = zeros(3*n,1);
info = struct('Wnorm', zeros(3,1), 'proj', 0, 'phi_max', 0);

% 权重范数上限：假设 4 只保证逼近误差 δ 有界，前提是权重本身不发散。
% 式 (35) 在 |e2| 大（λ2 很大）时会让 What 无界增长，故加一个投影。
if isfield(p, 'nn_Wmax') && ~isempty(p.nn_Wmax)
    Wmax = p.nn_Wmax;
else
    Wmax = Inf;
end

for k = 1:3
    Wk = W((k-1)*n + (1:n));

    % 高斯基函数：x = V(k)，中心 nn_c，宽度 nn_sigma
    ph = exp(-((V(k) - p.nn_c).^2)/(2*p.nn_sigma^2));
    phi(:,k) = ph;

    % 式 (9) 的估计部分
    Ghat(k) = Wk.'*ph;

    % 式 (35) 的权重更新律
    dWk = p.rho*(-p.rho_w*Wk + lam2(k)*ph);

    % 假设 4 的投影保护：若 ||What|| 已到上限且还要继续增大，就去掉沿径向的分量
    Wn = norm(Wk);
    if isfinite(Wmax) && Wn >= Wmax
        WtW = Wk.'*Wk;
        radial = Wk.'*dWk;
        if WtW > 0 && radial > 0
            dWk = dWk - (radial/WtW)*Wk;
            info.proj = info.proj + 1;
        end
    end

    dW((k-1)*n + (1:n)) = dWk;
    info.Wnorm(k) = Wn;
end
info.phi_max = max(phi(:));
end
