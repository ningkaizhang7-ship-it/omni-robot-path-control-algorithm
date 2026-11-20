function alpha = omr_alpha(S, e1, dqr, b1v, p)
%OMR_ALPHA  Virtual control law, eq. (18) of 22482.
%
%   alpha = omr_alpha(S, e1, dqr, b1v, p)
%
%   alpha = S^-1 * [ -k1*b1^2*sin(pi*e1^2/b1^2)/(2*pi*e1) + dqr - L1*beta1 ]
%
%   单独抽成函数有两个用处：
%     1) 控制律（omr_control_core）与初值构造（omr_params 里的 xi1(0)）用同一份公式，
%        不会出现"两份实现慢慢漂移"的问题；
%     2) 论文式 (18) 的分子分母都依赖 e1，集中在一处便于检查除零与正则项。
%
%   注意：交付的 Simulink 模型在分母上多加了 +0.01（2*pi*e1 + 0.01），论文没有。
%   这个正则项把 e1 = 0 的可去奇点平移成 e1 = -0.01/(2*pi) 处的真极点，
%   是 .slx 长时曲线出现 e2 尖峰的直接原因之一（见 docs）。这里保留该写法以对齐模型。

fb = zeros(3,1);
for k = 1:3
    arg = pi*e1(k)^2/(p.bd1(k)^2);
    fb(k) = -p.k1(k)*p.bd1(k)^2*sin(arg)/(2*pi*e1(k) + 0.01);
end
% '\' 是左除，即解线性方程组 S*alpha = (...)，比 inv(S)*(...) 更快也更稳；
% S 是正交矩阵，所以结果与 S'*(...) 数值等价。
alpha = S\(fb + dqr - p.L1*b1v);
end
