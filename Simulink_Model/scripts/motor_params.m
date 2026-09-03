function motor_params()
%% 定义 CyberDog_Motor_FOC 及测试台（plant）的全部可调参数（若不存在）
% 幂等：已存在的参数不覆盖，便于在 base workspace 运行时调整。
% 已设为模型的 InitFcn 回调，点 Run 时自动执行。
%
% 说明：参数以 Simulink.Parameter + ExportedGlobal 定义，后续代码生成时
%       固件可写入这些全局变量（与计划 §4 一致）。

% ---- FOC 电流环 ----
ensureParam('CurrQ_Kp',    0.6,    'q轴电流环比例增益');
ensureParam('CurrQ_Ki',    600,    'q轴电流环积分增益');
ensureParam('CurrD_Kp',    0.6,    'd轴电流环比例增益');
ensureParam('CurrD_Ki',    600,    'd轴电流环积分增益');
ensureParam('Curr_MaxOut', 12.0,   '电流环输出上限(V)');
ensureParam('Curr_MinOut', -12.0,  '电流环输出下限(V)');

% ---- FOC 逆变器/调制 ----
ensureParam('Vbus_nom', 24.0, '母线电压(V)');
ensureParam('MaxMod',   0.95, '最大调制度');

% ---- 速度环 ----
ensureParam('Speed_Kp',      0.3,   '速度环比例增益');
ensureParam('Speed_Ki',      0.0006, '速度环积分增益(1kHz+UseI*Ts 每周期增量)');
ensureParam('Speed_MaxOut',  5.0,  '速度环输出上限(A, iq_ref)');
ensureParam('Speed_MinOut', -5.0,  '速度环输出下限(A, iq_ref)');
ensureParam('Speed_Loop_N',  10,   '速度环分频(每N个电流环周期更新一次)');

% ---- 位置环（M5：位置→速度级联，独立第三模式；PD 结构先只给 P）----
ensureParam('Pos_Kp',      10.0, '位置环比例增益(RPS/rad)');
ensureParam('Pos_Kd',      0.0,  '位置环微分增益(预留, 先0)');
ensureParam('Pos_Ki',      0.0,  '位置环积分增益(预留, 未接线)');
ensureParam('Pos_MaxOut',  5.0,  '位置环输出上限(RPS, 作速度环参考)');
ensureParam('Pos_MinOut', -5.0,  '位置环输出下限(RPS)');

% ---- 电机参数 ----
ensureParam('Pole_Pairs', 7, '极对数');

% ---- PLL 角度/速度滤波 (MCB PLL with Feed Forward, 带宽 1000Hz) ----
% 掩码增益约定: Kp = 2*zeta*wn (wn=2*pi*1000, zeta=1);  Ki = wn^2 (连续)
% 块内部逐样本 I = Ki*BlkSampleTime。
% 注意: MCB 块掩码参数标记为"不可调"→ Kp/Ki 是构建期常量(普通 double, 内联进生成代码)，
%       改带宽需改此处并重新生成代码；InvTwoPiPolePairs 是 Gain 块参数(可调) → 导出全局由固件写。
assignin('base','PLL_Kp', 2*2*pi*1000);       % = 12566.4 (构建期常量)
assignin('base','PLL_Ki', (2*pi*1000)^2);     % = 3.9478e7 (构建期常量)
ensureParam('InvTwoPiPolePairs', 1/(2*pi*7), '电角速度->机械RPS 1/(2*pi*PolePairs)');
% 速度一阶IIR截止(Hz): 挡10k->1k降采样混叠、不拖慢50Hz速度环
% 开环实测(量化编码器输入,1000Hz环路): fc=500->std0.41 / 200->0.165 / 100->0.082 / 50->0.041 RPS
% 取200(速度环50Hz处滞后~14°); 台架可按噪声再调100-500
% 构建期常量(掩码编译期算IIR系数), 不做运行时全局
assignin('base','PLL_SpeedCutoffFreq', 200);
evalin('base','InvTwoPiPolePairs.Value = 1/(2*pi*Pole_Pairs.Value);');

% ---- 死区补偿（矢量对齐 LUT，20 点最大 1.5A：0.05~0.5A 密、0.5~1.5A 疏）----
ensureParam('DeadComp_En',    1, '死区补偿使能(0/1)');
ensureParam('DeadComp_Lut_I', [0.05 0.10 0.15 0.20 0.25 0.30 0.35 0.40 0.45 0.50 0.60 0.70 0.80 0.90 1.00 1.10 1.20 1.30 1.40 1.50], '死区补偿LUT 电流断点(A)');
ensureParam('DeadComp_Lut_V', zeros(20,1), '死区补偿LUT 补偿电压(V)');

% ---- 齿槽转矩前馈（角度→iq LUT，360 点电角度每度）----
ensureParam('CoggingFF_En',      0, '齿槽转矩前馈使能(0/1)');
ensureParam('Cogging_Lut_Angle', linspace(0,2*pi,360), '齿槽前馈LUT 电角度断点(rad)');
ensureParam('Cogging_Lut_V',     zeros(360,1), '齿槽前馈LUT iq补偿(A)');

% ---- 电机参数（按固件 Motor_Params_t 关键项，用于电流环增益推导）----
ensureParam('Motor_Phase_L', 0.0002, '相电感(H)');
ensureParam('Motor_Phase_R', 0.2,    '相电阻(Ω)');
ensureParam('Motor_VBUS',    24.0,   '母线电压(V)');

% ---- 电流环增益：按固件 current_q_pid_Init 公式推导 ----
%   固件: kp = L·BW·6.18;  ki = R·BW·6.18·timFactor
%   模型 PID 块离散积分自带 ×Ts(=timFactor)，故模型 I 增益 = R·BW·6.18
%   改调参：改 Motor_Phase_L/R 或 CUR_BW 即可，增益自动跟随
evalin('base','CurrQ_Kp.Value = Motor_Phase_L.Value * 400.0 * 6.18;');
% PID 块已勾选 UseI*Ts → I 参数直接用固件 ki = Ki*Ts（无需再除 Ts）
evalin('base','CurrQ_Ki.Value = Motor_Phase_R.Value * 400.0 * 6.18 * 1e-4;');
evalin('base','CurrD_Kp.Value = CurrQ_Kp.Value;');
evalin('base','CurrD_Ki.Value = CurrQ_Ki.Value;');
evalin('base','Curr_MaxOut.Value = Motor_VBUS.Value * 0.5773502691896257;');  % q 轴 ±VBUS/√3
evalin('base','Curr_MinOut.Value = -Curr_MaxOut.Value;');
ensureParam('CurrD_MaxOut', 3.0,  'd轴电流环输出上限(V)');   % d 轴按固件 ±3V
ensureParam('CurrD_MinOut', -3.0, 'd轴电流环输出下限(V)');

% ---- 派生系数：避免多参数表达式内联（代码生成需单一可调参数）----
ensureParam('VmaxCoeff', 24.0*0.5773502691896257*0.95, '电压限幅系数 Vbus/√3·MaxMod');
ensureParam('InvVbus',   1/24.0, '母线电压倒数 1/Vbus');
evalin('base','VmaxCoeff.Value = Vbus_nom.Value * 0.5773502691896257 * MaxMod.Value;');
evalin('base','InvVbus.Value = 1.0 / Vbus_nom.Value;');

% ---- 速度环增益：手动整定（占位 plant，固件公式待真实电机参数代入）----
% 固件公式: Kp = BW·6.28·J/(1.5·P·Flux)·0.8;  Ki = BW·6.28·Kp·timFactor·update_counter·0.5
% 说明: 固件公式对占位 plant 参数超调过大，先用手动整定值，后续换真实电机参数再启用公式
% evalin('base','Speed_Kp.Value = 50.0 * 6.28 * J_plant.Value / (1.5 * PolePairs_plant.Value * Flux_plant.Value) * 0.8;');
% evalin('base','Speed_Ki.Value = 50.0 * 6.28 * Speed_Kp.Value * 1e-3 * 0.5;');

% ---- 测试台手动输入（交互测试：点 Run 直接用的输入）----
ensureParam('T_iq_ref',    0.0, '手动 iq_ref 目标(A)');
ensureParam('T_ref_speed', 5.0, '手动 ref_speed 目标(RPS)');
ensureParam('T_ctrl_mode', 1,   '手动 ctrl_mode (0=转矩,1=速度)');
ensureParam('T_TL',        0.0, '手动负载转矩(N·m)');

% ---- 测试台 plant（MCB PMSM + 逆变器）----
ensureParam('R_plant',        0.2,    'plant相电阻(Ω)');
ensureParam('L_plant',        0.0002, 'plant相电感(H)');
ensureParam('Flux_plant',     0.01,   'plant磁链(Wb)');
ensureParam('PolePairs_plant',7,      'plant极对数');
ensureParam('J_plant',        0.00015,'plant转动惯量(kg·m²)');
ensureParam('B_plant',        0.0002, 'plant粘滞摩擦(N·m·s/rad)');
ensureParam('Vbus_plant',     24.0,   'plant母线电压(V)');

fprintf('[motor_params] 参数已就绪\n');
end

function ensureParam(name, val, desc)
if ~evalin('base', ['exist(''' name ''',''var'')']) || ~isa(evalin('base', name),'Simulink.Parameter')
    p = Simulink.Parameter(val);
    p.StorageClass = 'ExportedGlobal';
    p.Description = desc;
    assignin('base', name, p);
end
end
