function dx = omr_rhs(t, x, p, c)
%OMR_RHS  Closed-loop right-hand side: kinematics + dynamics + controller.
%
%   dx = omr_rhs(t, x, p)      controller evaluated at (t,x)  <- default
%   dx = omr_rhs(t, x, p, c)   controller output frozen at a previous
%                              evaluation (only used by p.ctrl_mode = "hold")
%
%   注意：Simulink 里的 MATLAB Function 是直通块，求解器的每个 RK 中间级都会
%   用该级的中间状态重新求一次控制律，所以默认行为（不传 c）才是与 .slx 一致的。
% p.idx 是 omr_params 里构造的索引结构体：字段 q/V/b1/b2/xi1/xi2 各存一段下标区间，
% 整个闭环状态被打包成一个列向量 x，用 x(i.q) 这类写法切片。
% 好处是改状态顺序时只需改 omr_params 一处，不必在代码里到处改 1:3、4:6 这些魔法数字。
i = p.idx;
q = x(i.q);
V = x(i.V);
% p 是控制器以为的参数，p.plant 是被控对象的真实参数；mismatch 预设下两者故意不同。
pp = p.plant;

% 第 4 个入参 c 是可选的控制器输出快照；nargin 是实际传入的参数个数，
% 不传（或不给值）时在这里现场重算一次。具体用哪个版本由 p.controller 决定，
% 统一走 omr_ctrl_eval，避免 omr_sim 与 omr_rhs 各写一份 switch 而慢慢漂移。
if nargin < 4 || isempty(c)
    c = omr_ctrl_eval(t, x, p);
end

th = q(3);
S = [cos(th), -sin(th), 0; sin(th), cos(th), 0; 0, 0, 1];
dq = S*V;

FV = [pp.a2*V(3)*V(2); -pp.a2*V(3)*V(1); 0];
% 只需要第 4 个返回值 d；~ 是占位符，表示丢弃对应位置的输出。
[~, ~, ~, d] = omr_reference(t, p.dist_amp);

% 被控对象吃的是饱和后的力矩（论文式 (8)）：执行器只有 +-tau_max，
% 限幅必须作用在机器人本体上，而不只是进补偿器。
u = c.sat_tau;

dV = pp.A1*V + FV + pp.B*u + d;
dx = [dq; dV; c.dx];
end
