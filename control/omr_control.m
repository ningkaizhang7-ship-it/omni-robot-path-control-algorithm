function c = omr_control(t, x, p)
%OMR_CONTROL  第一版控制律：不含 RBF 网络。
%
%   c = omr_control(t, x, p)
%
%   与 omr_control_core(t, x, p, "v1") 完全等价：未知项 G = A1*V + F(V) 不补偿
%   （Ghat = 0），饱和补偿、DSC 与障碍反馈照常工作。所有方程都在
%   omr_control_core.m 里，这里只是一层"选版本"的包装。
%
%   配套参数由 omr_params("v1") 给出（p.nx = 18，状态里没有 RBF 权重段）。

c = omr_control_core(t, x, p, "v1");
end
