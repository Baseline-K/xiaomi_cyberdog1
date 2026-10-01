function add_accel_est(fcHz)
%ADD_ACCEL_EST  Add one mechanical acceleration estimate to CyberDog_Motor_FOC.
%
%   Filtered derivative of the PLL mechanical speed (the model's EncoderPLL
%   output speed_meas_rps), built from primitive blocks so it stays single
%   precision like the rest of the model:
%
%       a[k] = (1-K)*a[k-1] + (K/Ts)*( v[k] - v[k-1] )      [RPS/s]
%       K    = 1 - exp(-2*pi*fc*Ts),   Ts = 1e-4 s (10 kHz FOC)
%
%   Adds the root outport 'accel_meas_rps2' right next to 'speed_meas_rps'.
%   Units are RPS/s on purpose: that is what U.ref_accel and
%   Y.speed_meas_rps already use, so no 2*pi conversion is needed anywhere.
%
%   Usage:
%       add_accel_est          % recommended default: fc = 100 Hz
%       add_accel_est(50)      % smoother, more lag
%       add_accel_est(200)     % faster, noisier
%
%   Idempotent: running it again only refreshes the two gain coefficients.

if nargin < 1 || isempty(fcHz)
    fcHz = 100;
end

here     = fileparts(mfilename('fullpath'));
modelDir = fileparts(here);
addpath(here);
cd(modelDir);

modelName = 'CyberDog_Motor_FOC';
Ts   = 1e-4;
K    = 1 - exp(-2*pi*fcHz*Ts);
gIn  = K / Ts;          % input gain  K/Ts
gFb  = 1 - K;           % feedback gain

fprintf('[add_accel_est] fc=%.1f Hz  Ts=%g s  K=%.6f  K/Ts=%.4f  1-K=%.6f\n', ...
        fcHz, Ts, K, gIn, gFb);

load_system(modelName);

sysPath = [modelName '/AccelEst'];
outPath = [modelName '/accel_meas_rps2'];

% ---------------------------------------------------------------- idempotent
if getSimulinkBlockHandle(sysPath) > 0
    set_param([sysPath '/Gain_K_Ts'], 'Gain', num2str(gIn, '%.10g'));
    set_param([sysPath '/Gain_Fdbk'], 'Gain', num2str(gFb, '%.10g'));
    set_param(sysPath, 'Description', sprintf('accel estimate, fc=%.1f Hz', fcHz));
    save_system(modelName);
    fprintf('[add_accel_est] AccelEst already present - coefficients refreshed.\n');
    return;
end

% ------------------------------------------------------------ 1) subsystem
add_block('built-in/Subsystem', sysPath, 'MakeNameUnique', 'off');
set_param(sysPath, 'Description', sprintf('accel estimate, fc=%.1f Hz', fcHz));

% defensive: drop anything a default Subsystem may bring along
try
    delete_line(sysPath, 'In1/1', 'Out1/1');
catch
end
for blk = {'In1', 'Out1'}
    if getSimulinkBlockHandle([sysPath '/' blk{1}]) > 0
        delete_block([sysPath '/' blk{1}]);
    end
end

add_block('simulink/Sources/In1',  [sysPath '/v'],         'Port', '1', ...
    'OutDataTypeStr', 'single', 'Position', [ 20 100  50 114]);
add_block('simulink/Sinks/Out1',   [sysPath '/accel'],     'Port', '1', ...
    'OutDataTypeStr', 'single', 'Position', [620 138 650 152]);

% NOTE: Unit Delay has no OutDataTypeStr in R2025b - it inherits the data type
% from its input, which is pinned to single by the subsystem Inport above.
add_block('simulink/Discrete/Unit Delay', [sysPath '/v_prev'], ...
    'InitialCondition', '0', 'SampleTime', '-1', ...
    'Position', [110 92 170 134]);

add_block('simulink/Math Operations/Sum', [sysPath '/dv'], ...
    'Inputs', '+-', 'OutDataTypeStr', 'single', 'Position', [220 96 250 136]);

add_block('simulink/Math Operations/Gain', [sysPath '/Gain_K_Ts'], ...
    'Gain', num2str(gIn, '%.10g'), 'OutDataTypeStr', 'single', ...
    'Position', [300 96 360 136]);

add_block('simulink/Math Operations/Sum', [sysPath '/a_sum'], ...
    'Inputs', '++', 'OutDataTypeStr', 'single', 'Position', [410 118 440 158]);

add_block('simulink/Discrete/Unit Delay', [sysPath '/a_prev'], ...
    'InitialCondition', '0', 'SampleTime', '-1', ...
    'Position', [480 230 540 272]);

add_block('simulink/Math Operations/Gain', [sysPath '/Gain_Fdbk'], ...
    'Gain', num2str(gFb, '%.10g'), 'OutDataTypeStr', 'single', ...
    'Position', [300 236 360 276]);

% ------------------------------------------------------------ 2) inside wires
add_line(sysPath, 'v/1',        'v_prev/1',      'autorouting', 'on');
add_line(sysPath, 'v/1',        'dv/1',          'autorouting', 'on');
add_line(sysPath, 'v_prev/1',   'dv/2',          'autorouting', 'on');
add_line(sysPath, 'dv/1',       'Gain_K_Ts/1',   'autorouting', 'on');
add_line(sysPath, 'Gain_K_Ts/1','a_sum/1',       'autorouting', 'on');
add_line(sysPath, 'a_sum/1',    'accel/1',       'autorouting', 'on');
add_line(sysPath, 'a_sum/1',    'a_prev/1',      'autorouting', 'on');
add_line(sysPath, 'a_prev/1',   'Gain_Fdbk/1',   'autorouting', 'on');
add_line(sysPath, 'Gain_Fdbk/1','a_sum/2',       'autorouting', 'on');

% ------------------------------------------------------------ 3) root level
srcOut    = [modelName '/speed_meas_rps'];
phOut     = get_param(srcOut, 'PortHandles');
lineH     = get_param(phOut.Inport(1), 'Line');
srcPortH  = get_param(lineH, 'SrcPortHandle');
srcBlk    = get_param(srcPortH, 'Parent');
srcPos    = get_param(srcBlk, 'Position');
srcName   = get_param(srcBlk, 'Name');

outs    = find_system(modelName, 'SearchDepth', 1, 'BlockType', 'Outport');
nums    = zeros(1, numel(outs));
for k = 1:numel(outs)
    nums(k) = str2double(get_param(outs{k}, 'Port'));
end
nextPort = max(nums) + 1;

outPos = get_param(srcOut, 'Position');
outY   = outPos(2) + 60;

set_param(sysPath, 'Position', [srcPos(1), srcPos(2) + 60, srcPos(1) + 260, srcPos(2) + 190]);
add_block('simulink/Sinks/Out1', outPath, ...
    'Port', num2str(nextPort), 'OutDataTypeStr', 'single', ...
    'Position', [srcPos(1) + 320, outY, srcPos(1) + 350, outY + 14]);

phSys = get_param(sysPath, 'PortHandles');
add_line(modelName, srcPortH, phSys.Inport(1), 'autorouting', 'on');
add_line(modelName, phSys.Outport(1), get_param(outPath, 'PortHandles').Inport(1), 'autorouting', 'on');

save_system(modelName);

fprintf('[add_accel_est] source of speed_meas_rps = %s\n', srcName);
fprintf('[add_accel_est] added root outport %s (port %d)\n', outPath, nextPort);
fprintf('[add_accel_est] saved %s\n', modelName);
end
