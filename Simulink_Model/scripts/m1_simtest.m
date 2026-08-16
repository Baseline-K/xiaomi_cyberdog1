function m1_simtest()
%% M1 骨架仿真验证：零输入应输出 duty=0.5；iq_ref=2A 应输出正弦调制且在[0,1]
% 依赖：Simulink_Model/CyberDog_Motor_FOC.slx（可调参数缺省时自动创建）
%
% 用法：在 MATLAB 命令行运行  m1_simtest
% 返回：无（结果打印到命令窗口）

mdl = 'CyberDog_Motor_FOC';
scriptDir = fileparts(mfilename('fullpath'));
mdlDir = fullfile(scriptDir, '..');
if isempty(which(mdl))
    addpath(mdlDir);
end

% ---- 参数缺省创建（与 build 脚本保持一致）----
ensureParam('CurrQ_Kp', 0.02, 'q轴电流环比例增益');
ensureParam('CurrQ_Ki', 2.0,  'q轴电流环积分增益');
ensureParam('CurrD_Kp', 0.02, 'd轴电流环比例增益');
ensureParam('CurrD_Ki', 2.0,  'd轴电流环积分增益');
ensureParam('Curr_MaxOut', 12.0,  '电流环输出上限(V)');
ensureParam('Curr_MinOut', -12.0, '电流环输出下限(V)');
ensureParam('Vbus_nom', 24.0, '母线电压(V)');
ensureParam('MaxMod', 0.95, '最大调制度');

% ---- 测试输入 ----
dt = 1e-4; t = (0:dt:0.1)';
th = mod(2*pi*50*t, 2*pi);          % 50Hz 电角度
mk = @(v) timeseries(single(v), t);   % 模型 inport 为 single

inp0 = Simulink.SimulationData.Dataset;
inp0 = inp0.addElement(mk(zeros(size(t))), 'ia');
inp0 = inp0.addElement(mk(zeros(size(t))), 'ib');
inp0 = inp0.addElement(mk(th), 'eleangle');      % 原始电角度（PLL 在模型内部）
inp0 = inp0.addElement(mk(zeros(size(t))), 'iq_ref');
inp0 = inp0.addElement(mk(zeros(size(t))), 'id_ref');
inp0 = inp0.addElement(mk(zeros(size(t))), 'pll_reset');  % PLL 不复位
inp0 = inp0.addElement(mk(zeros(size(t))), 'ref_speed');
inp0 = inp0.addElement(mk(zeros(size(t))), 'ctrl_mode');

% ---- 测试1：零输入 -> duty 应为 0.5 ----
runSim = @(dsx) sim(Simulink.SimulationInput(mdl).setExternalInput(dsx)...
    .setModelParameter('StopTime','0.1','SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset'));
out0 = runSim(inp0);
du = out0.yout.get(1).Values.Data;
fprintf('测试1(零输入): duty_u 均值=%.4f 范围=[%.4f, %.4f]\n', mean(du), min(du), max(du));
assert(abs(mean(du)-0.5) < 1e-6 && abs(min(du)-0.5) < 1e-6, '测试1失败：duty 应为 0.5');

% ---- 测试2：iq_ref=2A -> 正弦调制，落在[0,1]，无NaN ----
inp1 = inp0; inp1 = inp1.setElement(4, mk(2*ones(size(t))));
out1 = runSim(inp1);
y1 = out1.yout;
du1 = y1.get(1).Values.Data; dv1 = y1.get(2).Values.Data; dw1 = y1.get(3).Values.Data;
fprintf('测试2(iq_ref=2A): duty_u=[%.4f, %.4f] duty_v=[%.4f, %.4f] duty_w=[%.4f, %.4f]\n', ...
    min(du1),max(du1), min(dv1),max(dv1), min(dw1),max(dw1));
assert(all(du1>=0 & du1<=1 & dv1>=0 & dv1<=1 & dw1>=0 & dw1<=1), '测试2失败：duty 超出[0,1]');
assert(all(~isnan(du1) & ~isnan(dv1) & ~isnan(dw1)), '测试2失败：存在 NaN');

fprintf('M1 骨架仿真验证通过。\n');
end

function ensureParam(name, val, desc)
if ~evalin('base', ['exist(''' name ''',''var'')']) || ~isa(evalin('base', name), 'Simulink.Parameter')
    p = Simulink.Parameter(val);
    p.StorageClass = 'ExportedGlobal';
    p.Description = desc;
    assignin('base', name, p);
end
end
