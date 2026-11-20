function p = omr_params(preset)
%OMR_PARAMS  Parameter sets for the three-wheeled omnidirectional robot (22482).
%
%   p = omr_params()        % "v1": 第一版（不含 RBF 网络，Ghat = 0）
%   p = omr_params("v1")
%   p = omr_params("v2")    % 第二版：加 RBFNN 观测器（论文假设 3、4）
%
%   两版共用同一套物理参数、增益与正则化设置，唯一差别是 p.controller：
%     "v1" -> omr_control     未知项 G = A1*V + F(V) 不补偿
%     "v2" -> omr_control_v2  未知项由 RBF 网络辨识，Ghat 进式 (25)
%
% 主要字段：
%   p.m,L,r,IV,Iw,c,n,k     机器人物理参数（质量、力臂、轮半径、转动惯量、摩擦、减速比、力矩增益）
%   p.A1,p.B,p.Binv,p.M     由物理参数导出的矩阵（线性耗散、输入矩阵、其逆、惯性矩阵）
%   p.Bmat,p.Binv_mat       PNC 安全层用的归一化轮矩阵及其逆
%   p.k1,p.k2,r1,r2,L1,L2   控制器增益（障碍反馈、动态面、补偿系统）
%   p.bd1,p.bd2             误差障碍边界 b1、b2
%   p.tau_max,p.tau_min     执行器力矩限幅
%   p.lam2_max,lam2_floor   障碍项 λ2 的钳位与 λ1/λ2 的分母下限
%   p.ratio_max             λ1/λ2 的比值上限
%   p.nn_*                  RBF 网络节点数、中心、宽度、自适应增益（仅 v2）
%   p.plant                 被控对象真实参数（默认与控制器一致）
%   p.idx                   状态切片索引；p.nx 为闭环状态总维数
%   p.controller            控制律版本 "v1" / "v2"

if nargin < 1, preset = "v1"; end
p = base_params();
% 用 string（双引号）而不是 char（单引号），便于后面 switch 直接比较
p.preset = string(preset);

% R2017a 起 switch 的 case 可以直接写 string 字面量，无需 strcmp
switch preset
    case "v1"
        % 第一版：没有观测器。未知项 G 不做任何补偿（Ghat = 0），
        % 由 -1.5*lam2 与 -lam1/lam2*S*e2 这两项把它压住。
        p.controller = "v1";
        p.use_nn     = false;

    case "v2"
        % 第二版：论文假设 3、4 的 RBFNN 观测器，Ĝ 进式 (25)，Ŵ 按式 (35) 推进。
        p.controller = "v2";
        p.use_nn     = true;

    otherwise
        error('omr_params:unknownPreset', ...
            'Unknown preset "%s" (use "v1" or "v2").', preset);
end

if ~isfield(p, 'plant')
    p.plant = p;                     % 真实对象 == 名义模型
end

% p.idx：把各段状态在 x 中的下标打包成一个结构体——
%   p.idx.q   = 1:3    位姿 [x y theta]
%   p.idx.V   = 4:6    机体系速度 [vx vy w]
%   p.idx.b1  = 7:9    饱和补偿状态 beta1
%   p.idx.b2  = 10:12  饱和补偿状态 beta2
%   p.idx.xi1 = 13:15  动态面滤波器状态 xi1
%   p.idx.xi2 = 16:18  动态面滤波器状态 xi2
% 这样 x(i.q) 就是位姿段，改状态排列顺序时只需改这一处。
p.idx = struct('q', 1:3, 'V', 4:6, 'b1', 7:9, 'b2', 10:12, ...
               'xi1', 13:15, 'xi2', 16:18);
if p.use_nn
    p.nx_ctrl = 3*p.nn_nodes;
else
    p.nx_ctrl = 0;
end
% 闭环状态维数 = 18 个基础状态 + 3×节点数 的 RBF 权重（v1 时就是 18）
p.nx = 18 + p.nx_ctrl;
% W 段下标：19 : 18+3n；v1 时是空区间（19:18 = []），x(p.idx.W) 取到 0×1
p.idx.W = 19:(18 + p.nx_ctrl);

% xi1(0)：真实动态面的初值应当落在 alpha(0) 上。按式 (18) 现算，
% 这样改任何参数（q0、k1、q_r 的幅值……）都不会像硬编码那样失配。
[qr0, dqr0] = omr_reference(0, p.dist_amp);
th0 = p.q0(3);
S0  = [cos(th0), -sin(th0), 0; sin(th0), cos(th0), 0; 0, 0, 1];
p.xi10 = omr_alpha(S0, p.q0 - qr0 - p.beta10, dqr0, p.beta10, p);
end

% -------------------------------------------------------------------------
function p = base_params()
p.m  = 9;        % kg
p.L  = 0.4;      % m, centre -> wheel
p.r  = 0.05;     % m, wheel radius
p.IV = 11;       % kg*m^2, body inertia
p.Iw = 0.05;     % kg*m^2, wheel inertia
p.c  = 0.02;     % viscous friction
p.n  = 1;        % gear ratio
p.k  = 1;        % driven torque gain

p.k1 = [0.5; 0.5; 0.5];
p.k2 = [0.1; 0.1; 0.1];
p.r1 = 1.1;
p.r2 = 1.1;
p.L1 = 20*eye(3);
p.L2 = 20*eye(3);
p.bd1 = [0.5; 0.5; 0.5];
p.bd2 = [0.5; 0.5; 0.5];
p.tau_max = 300;
p.tau_min = -300;

% lam2 = e2/cos^2(pi*e2^2/(2*b2^2)) 在 |e2| -> b2 时无界，控制律里的 -1.5*lam2
% 会给出上千 N·m。执行器只有 +-300 N·m，故按 tau_max 反推上限（纯数值保险），
% 具体值在 rebuild 之后算（需要 Binv）。
p.lam2_max   = Inf;
% λ1/λ2 的奇点处理：分母下限 + 比值上限。论文原式没有这两项，
% 但 |λ2| 可以小到 1e-8，逐元素相除会给出 1e4 量级力矩（见 README 的实测）。
p.lam2_floor = 1e-3;
p.ratio_max  = 50;

p.pnc_on   = false;    % PNC 是学位论文的扩展，22482 原文没有
p.pnc_dqL  = [-0.6; -0.6; -0.6];
p.pnc_dqU  = -p.pnc_dqL;
p.pnc_Del  = [0.05; 0.05; 0.05];
p.pnc_lamb = [1; 1; 1];

p.q0     = [0.4; -0.4; 0.3];
p.V0     = [0.1; 0.1; 0.1];
p.beta10 = zeros(3,1);
p.beta20 = zeros(3,1);
p.xi10   = zeros(3,1);   % 由 alpha(0) 覆盖，见文件末尾
p.xi20   = zeros(3,1);

p.nn_nodes = 11;
p.nn_c     = linspace(-1, 1, p.nn_nodes)';
p.nn_sigma = 2;
p.rho      = 2;
p.rho_w    = 2;

p.dist_amp = 0.01;

p.solver = "ode4";     % "ode1" | "ode3" | "ode4"
% 控制律在 RK 中间级的取值方式："stage" 每级重算（与 Simulink 直通块一致，默认）
%                                "hold"  本步冻结（降为 1 阶，仅用于对照）
p.ctrl_mode = "stage";

p = rebuild(p);
% lam2 的上限必须等 B 算出来才能定：|tau| = |Binv|*1.5*|lam2| <= tau_max
p.lam2_max = p.tau_max/(1.5*norm(p.Binv, inf));
end

% -------------------------------------------------------------------------
function p = rebuild(p)
% 由 m、L、r、I_V、I_w、c、n、k 重新推导 A1、B、Binv、M 等派生矩阵。
p.a1 = 3*p.c/(3*p.Iw + 2*p.m*p.r^2);
p.a2 = 2*p.m*p.r^2/(3*p.Iw + 2*p.m*p.r^2);
p.a3 = 3*p.c*p.L^2/(3*p.Iw*p.L^2 + p.IV*p.r^2);
p.b1 = p.n*p.r/(3*p.Iw + 2*p.m*p.r^2);
p.b2 = p.k*p.r*p.L/(3*p.Iw*p.L^2 + p.IV*p.r^2);

p.A1 = diag([p.a1, p.a1, p.a3]);
% NOTE: eq. (5) of the journal prints +sqrt(3)*b1 twice, which makes B singular.
% 式 (3) 的 f1 - f2 自带负号，物理上第二行必须一正一负。
p.B = [-p.b1, -p.b1,  2*p.b1;
        sqrt(3)*p.b1, -sqrt(3)*p.b1, 0;
        p.b2,  p.b2,  p.b2];
p.Binv = inv(p.B);
% 归一化轮矩阵（PNC 安全层用）
p.Bmat = [-1, -1, 2; sqrt(3), -sqrt(3), 0; 1/p.L, 1/p.L, 1/p.L];
p.Binv_mat = inv(p.Bmat);
p.M = diag([p.m + 3*p.Iw/(2*p.r^2), p.m + 3*p.Iw/(2*p.r^2), ...
            p.IV + 3*p.Iw*p.L^2/p.r^2]);
p.detB = det(p.B);
% 论文式 (5) 把第二行印成 +sqrt(3)*b1 两次，那样 det(B) = 0；这里用的是式 (3)
% 推出的正确符号。加一条断言，防止有人照论文改回来之后仿真仍"能跑"却不自知。
assert(abs(p.detB) > 1e-9, 'omr_params:singularB', ...
    'B 奇异（det = %g）：检查 b1 的符号——论文式 (5) 第二行印错，应以式 (3) 为准。', ...
    p.detB);
end
