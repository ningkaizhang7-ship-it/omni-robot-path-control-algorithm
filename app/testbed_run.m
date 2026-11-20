function R = testbed_run(animateFirst)
%TESTBED_RUN  Scenario test bench driving the shipped Simulink model.
%
%   Swaps the reference-trajectory code of the "MATLAB Function" block in
%   model_OMR_extend_R2022b.slx, runs the closed loop, checks the constraints
%   and (optionally) renders the animation of the first scenario.
%
%   R = testbed_run()      % all scenarios, animation of scenario 1
%   R = testbed_run(false) % no animation

if nargin < 1, animateFirst = true; end

here = fileparts(mfilename('fullpath'));      % omr/app
omrRoot = fileparts(here);                    % omr
addpath(omrRoot);
omr_setup();
projRoot = fileparts(omrRoot);                % project root
outDir = fullfile(omrRoot, 'out');
if ~exist(outDir, 'dir'), mkdir(outDir); end

mdl = 'model_OMR_extend_R2022b';
mdlFile = fullfile(projRoot, [mdl '.slx']);
S = testbed_scenarios();
R = struct([]);

fprintf('%-12s %6s %10s %10s %10s %10s %10s\n', 'scenario', 'T', 'max|e1|', ...
    'max|e2|', 'max|tau|', 'endPos', 'sat%');

for k = 1:numel(S)
    % 场景之间先关掉模型（close_system 第 2 个参数 0 = 丢弃未保存改动），
    % 保证每个场景都从干净的模型状态开始。
    bd = bdIsLoaded(mdl);
    if bd, close_system(mdl, 0); end
    load_system(mdlFile);

    % sfroot 取 Stateflow 根对象，再按类型 + 路径定位参考轨迹那个图表；
    % 直接改写它的 Script 字段，就等于在运行时换掉参考轨迹代码（不落盘、不改原文件）。
    sf = sfroot;
    ch = sf.find('-isa', 'Stateflow.EMChart', '-and', 'Path', [mdl '/MATLAB Function']);
    if isempty(ch)
        error('testbed_run:chartNotFound', 'reference chart not found');
    end
    ch(1).Script = S(k).code;

    set_param(mdl, 'StopTime', num2str(S(k).T));
    ok = true;
    try
        sout = sim(mdl);
    catch e
        ok = false;
        fprintf('%-12s  SIM_FAIL: %s\n', S(k).name, e.message);
    end

    if ok
        t = sout.get('x_log').Time(:);
        q = [squeeze(sout.get('x_log').Data)'; squeeze(sout.get('y_log').Data)'];
        e1 = squeeze(sout.get('e1_log').Data);
        e2 = squeeze(sout.get('e2_log').Data);
        ta = squeeze(sout.get('tau_log').Data);
        xr = squeeze(sout.get('xr_log').Data); xr = xr(:);
        yr = squeeze(sout.get('yr_log').Data); yr = yr(:);
        if size(e1,2) ~= numel(t), e1 = e1.'; end
        if size(e2,2) ~= numel(t), e2 = e2.'; end
        if size(ta,2) ~= numel(t), ta = ta.'; end

        poserr = hypot(q(1,:)' - xr, q(2,:)' - yr);
        r.name = S(k).name;
        r.t = t; r.q = q; r.e1 = e1; r.e2 = e2; r.tau = ta;
        r.xr = xr; r.yr = yr;
        r.maxE1 = max(abs(e1(:)))/0.5;
        r.maxE2 = max(abs(e2(:)))/0.5;
        r.maxTau = max(abs(ta(:)));
        r.endPos = poserr(end);
        r.maxPos = max(poserr);
        r.satFrac = 100*mean(abs(ta(:)) >= 299.9);
        R = [R; r]; %#ok<AGROW>
        fprintf('%-12s %6.0f %10.3f %10.2f %10.0f %10.4f %10.2f\n', S(k).name, S(k).T, ...
            r.maxE1, r.maxE2, r.maxTau, r.endPos, r.satFrac);
    end
    close_system(mdl, 0);
end

save(fullfile(outDir, 'testbed.mat'), 'R', '-v7');
report_csv(R, fullfile(outDir, 'testbed_metrics.csv'));
fprintf('wrote %s\n', fullfile(outDir, 'testbed_metrics.csv'));

if animateFirst && ~isempty(R)
    % animation needs the reference evaluated on the logged grid
    r1 = R(1);
    qr = [r1.xr'; r1.yr'; zeros(1, numel(r1.t))];
    r1.qr = qr;
    omr_animate(r1, fullfile(outDir, 'testbed_fig8.gif'), 20);
end
end

% -------------------------------------------------------------------------
function report_csv(R, file)
fid = fopen(file, 'w');
fprintf(fid, 'scenario,max_e1_over_b1,max_e2_over_b2,max_tau,end_pos_err,max_pos_err,sat_percent\n');
for k = 1:numel(R)
    fprintf(fid, '%s,%.4f,%.4f,%.2f,%.5f,%.5f,%.3f\n', R(k).name, R(k).maxE1, ...
        R(k).maxE2, R(k).maxTau, R(k).endPos, R(k).maxPos, R(k).satFrac);
end
fclose(fid);
end
