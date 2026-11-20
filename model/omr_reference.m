function [qr, dqr, ddqr, d] = omr_reference(t, amp)
%OMR_REFERENCE  Reference trajectory and disturbance of the 22482 case study.
%   qr   = [1.5 sin(0.1t); sin(0.2t); sin(0.1t)]
%   d    = [0.01 sin(t); 0.01 sin(2t); 0.01 sin(3t)]
%
%   t may be a scalar or a vector; the outputs are 3-by-numel(t) so that the
%   whole reference can be evaluated in one call.
if nargin < 2, amp = 0.01; end
% t(:) 先把标量、行向量或列向量统一成列向量；.' 是非共轭转置，再转成行向量。
% 这样下面的 [sin(...); sin(...); sin(...)] 才能稳定得到 3×n 的输出（n = numel(t)）。
t = t(:).';
qr   = [1.5*sin(0.1*t); sin(0.2*t); sin(0.1*t)];
dqr  = [0.15*cos(0.1*t); 0.2*cos(0.2*t); 0.1*cos(0.1*t)];
ddqr = [-0.015*sin(0.1*t); -0.04*sin(0.2*t); -0.01*sin(0.1*t)];
d    = amp*[sin(t); sin(2*t); sin(3*t)];
end
