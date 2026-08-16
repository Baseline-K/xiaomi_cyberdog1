function m2_simtest()
%% M2 闭环仿真验证：速度环闭环 + 死区/齿槽前馈通断
% 依赖：Simulink_Model/CyberDog_Motor_FOC.slx + CyberDog_Motor_FOC_Test.slx
% 用法：m2_simtest
% 验证：1) 速度环收敛到 ref_speed；2) 死区补偿/齿槽前馈通断均有可观测效果

scriptDir = fileparts(mfilename('fullpath'));
mdlDir = fullfile(scriptDir, '..');
addpath(mdlDir);
addpath(scriptDir);   % scripts 自身入路径，确保 motor_params 可调用

mdl = 'CyberDog_Motor_FOC_Test';
if isempty(which(mdl)), disp('请先在 MATLAB 中打开 CyberDog_Motor_FOC_Test.slx'); return; end

% ---- 参数定义：统一从 motor_params.m（幂等，不覆盖已调值）----
motor_params;

% ---- 输出索引映射（根 outport 端口顺序）----
ops = find_system(mdl,'SearchDepth',1,'BlockType','Outport');
idx = struct();
for i = 1:numel(ops)
    nm = get_param(ops{i},'Name');
    idx.(nm) = str2double(get_param(ops{i},'Port'));
end
iSP = idx.speed_fbk; iIA = idx.ia; iPLL = idx.speed_meas_rps;

% ---- 切换到外部输入模式（脚本用 setExternalInput）----
setSwitches(mdl, '1');

% ---- 闭环仿真 ----
dt = 1e-4; t = (0:dt:1)';
mk = @(v) timeseries(v, t);
ds = Simulink.SimulationData.Dataset()...
  .addElement(timeseries(0*ones(size(t)),t),'iq_ref')...
  .addElement(timeseries(5*ones(size(t)),t),'ref_speed')...
  .addElement(timeseries(1*ones(size(t)),t),'ctrl_mode')...
  .addElement(timeseries(0*ones(size(t)),t),'TL');
run1 = @() sim(Simulink.SimulationInput(mdl).setExternalInput(ds).setModelParameter('StopTime','1'));
n = numel(t); r = (n-999):n;

% 基准（死区开，齿槽关）
evalin('base','DeadComp_En.Value = 1; CoggingFF_En.Value = 0;');
oB = run1();
spP = oB.yout.get(iSP).Values.Data;   % plant 真值速度
spL = oB.yout.get(iPLL).Values.Data;  % PLL 估计速度（模型内锁相环提取）
iaB = oB.yout.get(iIA).Values.Data;
fprintf('速度环(PLL估计): 稳态=%.3f RPS (目标5), std=%.3f, 峰值=%.3f, 超调=%.1f%%\n', ...
    mean(spL(r)), std(spL(r)), max(spL), (max(spL)-5)/5*100);
fprintf('PLL vs plant: PLL=%.3f  plant=%.3f RPS (|diff|=%.4f)\n', ...
    mean(spL(r)), mean(spP(r)), abs(mean(spL(r))-mean(spP(r))));
assert(abs(mean(spL(r))-5) < 0.5, '速度环未收敛到目标');
assert(abs(mean(spL(r))-mean(spP(r))) < 0.3, 'PLL速度估计未跟踪plant');

% 死区通断（高电流工况：死区补偿在低电流下效果微弱）
dsDT = Simulink.SimulationData.Dataset()...
  .addElement(timeseries(2*ones(size(t)),t),'iq_ref')...
  .addElement(timeseries(0*ones(size(t)),t),'ref_speed')...
  .addElement(timeseries(0*ones(size(t)),t),'ctrl_mode')...
  .addElement(timeseries(0.05*ones(size(t)),t),'TL');
runDT = @() sim(Simulink.SimulationInput(mdl).setExternalInput(dsDT).setModelParameter('StopTime','0.5'));
evalin('base','DeadComp_En.Value = 1;');
oDT1 = runDT(); iaDT1 = oDT1.yout.get(iIA).Values.Data;
evalin('base','DeadComp_En.Value = 0;');
oDT0 = runDT(); iaDT0 = oDT0.yout.get(iIA).Values.Data;
evalin('base','DeadComp_En.Value = 1;');
nDT = numel(iaDT1); rDT = (nDT-499):nDT;
dDC = max(abs(iaDT1(rDT)-iaDT0(rDT)));
fprintf('死区补偿 通/断: 稳态电流差异=%.4f A %s\n', dDC, tern(dDC>0.01,'✓生效','(差异过小)'));
assert(dDC > 0.001, '死区补偿未生效');

% 齿槽通断
evalin('base','CoggingFF_En.Value = 1;');
oC = run1(); iaC = oC.yout.get(iIA).Values.Data;
evalin('base','CoggingFF_En.Value = 0;');
dCG = max(abs(iaB(r)-iaC(r)));
fprintf('齿槽前馈 通/断: 稳态电流差异=%.4f A %s\n', dCG, tern(dCG>0.01,'✓生效','(差异过小)'));
assert(dCG > 0.001, '齿槽前馈未生效');

fprintf('M2 闭环仿真验证通过。\n');

% ---- 恢复为手动输入模式（便于 GUI 直接点 Run 交互）----
setSwitches(mdl, '0');
end

function s = tern(c, a, b), if c, s=a; else, s=b; end, end

function setSwitches(mdl, sw)
% 切换 4 个手动开关：'0'=手动常量（交互），'1'=外部输入（脚本）
for nm = {'Sw_iq_ref','Sw_ref_speed','Sw_ctrl_mode','Sw_TL'}
    try
        set_param([mdl '/' nm{1}], 'sw', sw);
    catch
    end
end
end

