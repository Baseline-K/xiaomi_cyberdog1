function m3_pll_test()
%% M3 开环 PLL 测试：官方 MCB PLL 块 vs 手写 Angle_PLL_filter（含 bug）
%  向 CyberDog_Motor_FOC 模型喂合成编码器电角度（含 12bit 量化），
%  验证 PLL 角度跟踪 / 速度提取 / 静止 / 速度阶跃，并与 filter.c 手写 PLL
%  （含死区 bug、失效限幅 bug）在相同数据上对比。
%
%  运行：在 MATLAB 中 cd 到 Simulink_Model/scripts 后直接运行 m3_pll_test
%  通过标准：angle err RMS<0.01rad, speed mean≈0.714±0.02 RPS, 速度阶跃跟踪,
%            静止时手写 PLL 强制 0（bug）而 MCB 版自然收敛 0。
%  失败时抛出 error（可作为 CI/测试）。

scriptdir = fileparts(mfilename('fullpath'));
addpath(fullfile(scriptdir,'..'));          % Simulink_Model/（含 .slx）
addpath(scriptdir);                         % scripts/
motor_params();                             % 定义/刷新全部参数（幂等，SpeedCutoffFreq 强制 200）

model = 'CyberDog_Motor_FOC';
load_system(model);
DeadComp_En.Value = 0;                      % 开环零电流，旁路死区补偿避免 0/0

% ---------------- 测试输入 ----------------
T = 0.3; dt = 1e-4; t = (0:dt:T)';
quant = 2*pi/4096*7;                        % AS5600 12bit 机械角 × 7 极对 → 电角度量化步进
th_in = mod(2*pi*5*t, 2*pi);                % 5Hz 电角度斜坡
th_q  = mod(round(th_in/quant)*quant, 2*pi);

z = timeseries(single(zeros(size(t))), t);
ds = Simulink.SimulationData.Dataset;
ds = addElement(ds, z, 'ia');       ds = addElement(ds, z, 'ib');
ds = addElement(ds, timeseries(single(th_q),t), 'eleangle');
ds = addElement(ds, z, 'iq_ref');   ds = addElement(ds, z, 'id_ref');
ds = addElement(ds, z, 'pll_reset'); ds = addElement(ds, z, 'ref_speed');
ds = addElement(ds, z, 'ctrl_mode');
runSim = @(dsx) sim(Simulink.SimulationInput(model).setExternalInput(dsx)...
    .setModelParameter('StopTime',num2str(T),'SaveOutput','on','OutputSaveName','yout','SaveFormat','Dataset'));

% ---------------- 1. 5Hz 斜坡：角度/速度跟踪 ----------------
out = runSim(ds); y = out.yout;
th_filt = double(y{4}.Values.Data); sp = double(y{5}.Values.Data); tt = y{4}.Values.Time;
th_ref = interp1(t, th_q, tt);
err = mod(th_filt - th_ref + pi, 2*pi) - pi;
si = tt > T-0.1;                             % 稳态窗口
ang_err_rms = sqrt(mean(err(si).^2));
sp_mean = mean(sp(si)); sp_std = std(sp(si));
exp_sp = 5/7;                                % 5Hz 电 → 0.7143 机械 RPS

% 手写 PLL（MATLAB 移植，含 deadband + 失效限幅 bug）
[~, ~, hand_om_filt] = hand_pll(th_q, 1e4, 2*pi*1000, 1.07e-3, 1e6);
hand_sp = hand_om_filt/(2*pi*7);

fprintf('--- 1. 5Hz 电角度斜坡（量化输入）---\n');
fprintf('  角度误差 RMS(稳态)  = %.5f rad          (要求 <0.01)\n', ang_err_rms);
fprintf('  速度均值            = %.5f RPS           (期望 0.7143)\n', sp_mean);
fprintf('  速度纹波 std        = %.5f RPS           (fc=200Hz)\n', sp_std);
fprintf('  手写PLL速度均值     = %.5f RPS           (对比)\n', mean(hand_sp(si)));
assert(ang_err_rms < 0.01, 'PLL 角度跟踪误差超限');
assert(abs(sp_mean - exp_sp) < 0.02, 'PLL 速度均值偏差超限');

% ---------------- 2. 静止：死区 bug 对比 ----------------
ds2 = ds; ds2 = setElement(ds2, 3, timeseries(single(0.5*ones(size(t))), t));
out2 = runSim(ds2);
sp2 = double(out2.yout{5}.Values.Data); tt2 = out2.yout{5}.Values.Time;
s2 = tt2 > 0.01;                              % 跳过启动锁定瞬态
[~, ~, hand_om2] = hand_pll(0.5*ones(size(t)), 1e4, 2*pi*1000, 1.07e-3, 1e6);
fprintf('--- 2. 静止（常值角度）---\n');
fprintf('  手写PLL: 死区把速度强制 0    max|speed|=%.6f RPS (bug)\n', max(abs(hand_om2/(2*pi*7))));
fprintf('  MCB PLL: 自然收敛 ~0         max|speed|=%.6f RPS\n', max(abs(sp2(s2))));
fprintf('  MCB PLL 静止均值 = %.6f RPS\n', mean(sp2(s2)));
assert(max(abs(sp2(s2))) < 0.01, 'MCB PLL 静止仍有大速度输出');

% ---------------- 3. 速度阶跃 5->15Hz 电 ----------------
th3 = mod(2*pi*5*t + 2*pi*10*max(0,t-0.15), 2*pi);
th3 = mod(round(th3/quant)*quant, 2*pi);
ds3 = ds; ds3 = setElement(ds3, 3, timeseries(single(th3), t));
out3 = runSim(ds3);
sp3 = double(out3.yout{5}.Values.Data); tt3 = out3.yout{5}.Values.Time;
s_b = tt3>0.12 & tt3<0.14; s_a = tt3>0.25;
fprintf('--- 3. 速度阶跃 5->15Hz 电（期望 0.714 -> 2.143 RPS）---\n');
fprintf('  阶跃前均值=%.4f RPS  阶跃后均值=%.4f RPS  超调=%.4f RPS\n', ...
    mean(sp3(s_b)), mean(sp3(s_a)), max(sp3(s_a))-15/7);
assert(abs(mean(sp3(s_b))-5/7) < 0.1 && abs(mean(sp3(s_a))-15/7) < 0.1, '速度阶跃跟踪失败');

fprintf('\n[M3 PLL 开环测试] 全部通过\n');
end

% ================= 手写 PLL 的 MATLAB 移植（对应 filter.c Angle_PLL_filter，含 bug） =================
function [theta_out, omega, omega_filt] = hand_pll(theta_in, FOC_FREQ, wn, deadband, maxspeed)
% 复刻 filter.c: Kp=2*wn, Ki=wn^2/FOC_FREQ(离散); 死区; Limit_Sat 返回值被丢弃(不生效)
Ts = 1/FOC_FREQ; n = numel(theta_in);
Kp = 2*wn; Ki = wn*wn/FOC_FREQ;
theta_out = zeros(n,1); omega = zeros(n,1); omega_filt = zeros(n,1);
th = 0; sum_err = 0; om = 0; omf = 0; init = false;
for k = 1:n
    if ~init
        th = theta_in(k); init = true; theta_out(k) = th; continue;
    end
    se = sin(theta_in(k))*cos(th) - cos(theta_in(k))*sin(th);
    ce = cos(theta_in(k))*cos(th) + sin(theta_in(k))*sin(th);
    err = atan2(se, ce);
    if abs(err) < deadband, err = 0; end          % 死区（低速速度归零 bug）
    sum_err = sum_err + err;                      % （原 Limit_Sat 丢弃 → 无积分限幅）
    om = Kp*err + Ki*sum_err;                     % （原 Limit_Sat 丢弃 → 无速度限幅）
    th = th + om*Ts;
    th = mod(th, 2*pi); if th<0, th = th+2*pi; end
    omf = 0.7*om + 0.3*omf;                       % 固件 Lpf(...,0.7)
    theta_out(k)=th; omega(k)=om; omega_filt(k)=omf;
end
end
