function c = omr_ctrl_eval(t, x, p)
%OMR_CTRL_EVAL  按 p.controller 选择控制律版本，返回一次控制律求值。
%
%   "v1"（默认） omr_control       第一版：不含 RBF 网络，Ghat = 0
%   "v2"         omr_control_v2    第二版：独立 RBFNN 辨识器（式 9 + 35）
%
%   omr_sim 与 omr_rhs 都调这一个函数，保证"日志里记的控制量"和"积分器里真正用的
%   控制量"来自同一份实现；否则两个地方各写一次 switch，很容易漂移。

ctrl = "v1";
if isfield(p, 'controller'), ctrl = string(p.controller); end
switch ctrl
    case "v1"
        c = omr_control(t, x, p);
    case "v2"
        c = omr_control_v2(t, x, p);
    otherwise
        error('omr_ctrl_eval:badController', ...
            'Unknown p.controller "%s" (use "v1" or "v2").', ctrl);
end
end
