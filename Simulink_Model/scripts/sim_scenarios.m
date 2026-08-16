function sim_scenarios()
%% 4 工况仿真：电流环 / 电流环+速度环 × 空载 / 负载
% 依赖：CyberDog_Motor_FOC.slx + CyberDog_Motor_FOC_Test.slx（MCB PMSM + Average-Value Inverter）
% 用法：sim_scenarios
% 输出：图1（电流环场景）、图2（电流+速度环场景），并打印关键指标
%
% 仿真/生成模块化说明：
%   算法模型 CyberDog_Motor_FOC.slx 保持不变（代码生成源）；
%   本脚本 + 测试台为纯仿真 harness（plant/负载/观测）。

scriptDir = fileparts(mfilename('fullpath'));
mdlDir = fullfile(scriptDir, '..');
addpath(mdlDir);
addpath(scriptDir);   % scripts 自身入路径，确保 motor_params 可调用

mdl = 'CyberDog_Motor_FOC_Test';
if isempty(which(mdl))
    error('请先打开 Simulink_Model/CyberDog_Motor_FOC_Test.slx');
end

% ---- 参数定义：统一从 motor_params.m（幂等，不覆盖已调值）----
motor_params;

% ---- 场景参数 ----
IQREF   = 2.0;    % 场景A：电流环目标 (A)
REFSPD  = 5.0;    % 场景B：速度目标 (RPS)
TLLOAD  = 0.03;   % 负载转矩 (N·m)
T_A = 0.5;        % 场景A 仿真时长（电流环；空载下转速会持续上升，取短些看跟踪）
T_B = 1.0;        % 场景B 仿真时长（速度环，看收敛）

% ---- 输出索引映射（根 outport 端口顺序）----
ops = find_system(mdl,'SearchDepth',1,'BlockType','Outport');
idx = struct();
for i = 1:numel(ops)
    nm = get_param(ops{i},'Name');
    idx.(nm) = str2double(get_param(ops{i},'Port'));
end
iIQ = idx.iq; iSP = idx.speed_fbk; iIA = idx.ia;

% ---- 切换到外部输入模式（脚本用 setExternalInput 驱动根 inport）----
setSwitches(mdl, '1');

% ---- 运行 4 工况 ----
runCase = @(iqr,rs,cm,tl,Ts) sim(Simulink.SimulationInput(mdl)...
    .setExternalInput(makeDS(iqr,rs,cm,tl,Ts)).setModelParameter('StopTime',num2str(Ts)));

r = struct();
r.A1 = runCase(IQREF, 0, 0, 0,      T_A);   % 电流环 空载
r.A2 = runCase(IQREF, 0, 0, TLLOAD, T_A);   % 电流环 负载
r.B1 = runCase(0, REFSPD, 1, 0,      T_B);  % 电流+速度 空载
r.B2 = runCase(0, REFSPD, 1, TLLOAD, T_B);  % 电流+速度 负载

ext = @(o) struct('iq', o.yout.get(iIQ).Values.Data, ...
                  'sp', o.yout.get(iSP).Values.Data, ...
                  'ia', o.yout.get(iIA).Values.Data, ...
                  't',  o.tout);
A1 = ext(r.A1); A2 = ext(r.A2); B1 = ext(r.B1); B2 = ext(r.B2);

% ---- 打印关键指标 ----
nA = numel(A1.t); nB = numel(B1.t);
fprintf('===== 场景A 电流环 (iq_ref=%.1fA) =====\n', IQREF);
fprintf('  空载: iq末段均值=%.3fA, 0.5s转速=%.1fRPS (持续上升)\n', mean(A1.iq(end-500:end)), A1.sp(end));
fprintf('  负载: iq末段均值=%.3fA, 0.5s转速=%.1fRPS\n', mean(A2.iq(end-500:end)), A2.sp(end));
fprintf('===== 场景B 电流+速度环 (ref=%.0fRPS) =====\n', REFSPD);
fprintf('  空载: 稳态转速=%.3fRPS, 稳态iq=%.3fA\n', mean(B1.sp(end-500:end)), mean(B1.iq(end-500:end)));
fprintf('  负载: 稳态转速=%.3fRPS, 稳态iq=%.3fA\n', mean(B2.sp(end-500:end)), mean(B2.iq(end-500:end)));

% ---- 绘图 ----
fs = 11;
% 图1：电流环场景
f1 = figure('Name','场景A 电流环','Color','w');
subplot(2,1,1);
plot(A1.t, A1.iq, 'b'); hold on; plot(A2.t, A2.iq, 'r');
yline(IQREF,'k--','iq_ref');
xlabel('时间 (s)'); ylabel('iq 电流 (A)'); grid on; legend('空载','负载','iq_ref','Location','best');
title(sprintf('场景A 电流环：iq 跟踪 (目标 %.1fA)', IQREF));
subplot(2,1,2);
plot(A1.t, A1.sp, 'b'); hold on; plot(A2.t, A2.sp, 'r');
xlabel('时间 (s)'); ylabel('转速 (RPS)'); grid on; legend('空载','负载','Location','best');
title('场景A 电流环：转速行为（电流×转矩驱动的自由转速）');

% 图2：电流+速度环场景
f2 = figure('Name','场景B 电流+速度环','Color','w');
subplot(2,1,1);
plot(B1.t, B1.sp, 'b'); hold on; plot(B2.t, B2.sp, 'r');
yline(REFSPD,'k--','ref_speed');
xlabel('时间 (s)'); ylabel('转速 (RPS)'); grid on; legend('空载','负载','ref','Location','best');
title(sprintf('场景B 电流+速度环：转速跟踪 (目标 %.0fRPS)', REFSPD));
subplot(2,1,2);
plot(B1.t, B1.iq, 'b'); hold on; plot(B2.t, B2.iq, 'r');
xlabel('时间 (s)'); ylabel('iq 电流 (A)'); grid on; legend('空载','负载','Location','best');
title('场景B 电流+速度环：iq 电流（负载下需要更大电流）');

% ---- 恢复为手动输入模式（便于 GUI 直接点 Run 交互）----
setSwitches(mdl, '0');
end

function ds = makeDS(iqr, rs, cm, tl, Ts)
dt = 1e-4; t = (0:dt:Ts)';
ds = Simulink.SimulationData.Dataset()...
    .addElement(timeseries(repmat(iqr,size(t)),t),'iq_ref')...
    .addElement(timeseries(repmat(rs,size(t)),t),'ref_speed')...
    .addElement(timeseries(repmat(cm,size(t)),t),'ctrl_mode')...
    .addElement(timeseries(repmat(tl,size(t)),t),'TL');
end

function setSwitches(mdl, sw)
% 切换 4 个手动开关：'0'=手动常量（交互），'1'=外部输入（脚本）
for nm = {'Sw_iq_ref','Sw_ref_speed','Sw_ctrl_mode','Sw_TL'}
    try
        set_param([mdl '/' nm{1}], 'sw', sw);
    catch
    end
end
end

