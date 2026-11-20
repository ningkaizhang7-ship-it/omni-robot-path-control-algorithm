function c = omr_control_v2(t, x, p)
%OMR_CONTROL_V2  第二版控制律：22482 + 独立的 RBFNN 辨识器（论文假设 3、4）。
%
%   c = omr_control_v2(t, x, p)
%
%   与第一版（omr_control）的唯一区别：未知项 G = A1*V + F(V) 一律取辨识值
%   Ghat = What'*phi，权重由式 (35) 在 omr_rbfnn_observer_v2 里推进（带
%   ||What|| <= nn_Wmax 的投影）；真值 G 只出现在 c.G 里作诊断，从不进入式 (25)。
%
%   配套参数由 omr_params("v2") 给出（p.nx = 18 + 3*11，状态里有 RBF 权重段）。

if p.nx_ctrl == 0
    error('omr_control_v2:noWeights', ...
        ['v2 需要 RBF 权重状态（p.nx_ctrl > 0）。', ...
         '请用 omr_params("v2") 取参数。']);
end

c = omr_control_core(t, x, p, "v2");
end
