%% Fit a practical dead-time compensation LUT
% Primary dataset (blue): 85%; secondary dataset (orange): 15%.
% A short quadratic Savitzky-Golay smoother removes point-to-point noise
% while retaining the measured low-current plateau and high-current rise.

dead_lut_I_A = [ ...
    0.05 0.10 0.15 0.20 0.25 0.30 0.35 0.40 0.45 0.50 ...
    0.60 0.70 0.80 0.90 1.00 1.10 1.20 1.30 1.40 1.50];

blue_voltage_V = [ ...
    0.0783979446 0.0928440690 0.1062120500 0.1144288780 0.1180228590 ...
    0.1210918430 0.1232030390 0.1213740110 0.1160281900 0.1171035770 ...
    0.1073126790 0.1064603330 0.1032989030 0.1165223120 0.1309335230 ...
    0.1534061430 0.1849372390 0.2204387190 0.2649173740 0.3047542570];

secondary_voltage_V = [ ...
    0.08999722278 0.1146392520 0.1335574390 0.1469503640 0.1519996520 ...
    0.14768677900 0.1424975400 0.1328917740 0.1279740330 0.1245332960 ...
    0.11370384700 0.1085379120 0.1065578460 0.1089334490 0.1237363820 ...
    0.14858698800 0.1696553230 0.2043409350 0.2379789350 0.3184723850];

blue_weight = 0.85;
weighted_voltage_V = blue_weight * blue_voltage_V + ...
    (1 - blue_weight) * secondary_voltage_V;
fitted_voltage_V = smoothdata(weighted_voltage_V, 'sgolay', 5, Degree=2);
fitted_voltage_V = max(fitted_voltage_V, 0);

% Four decimal places are sufficient for a float LUT and make calibration
% values easy to audit in the debugger.
dead_lut_V_V = round(fitted_voltage_V, 4);

% Deployment table: add an explicit zero-current/zero-voltage point so the
% lookup's linear extrapolation cannot inject a finite compensation at zero.
% Drop the redundant 0.45 A point to retain the generated model's 20 points.
kept_indices = [1:8 10:20];
deploy_lut_I_A = [0 dead_lut_I_A(kept_indices)];
deploy_lut_V_V = [0 dead_lut_V_V(kept_indices)];

fit_rmse_to_blue_V = sqrt(mean((dead_lut_V_V - blue_voltage_V).^2));
fit_max_error_to_blue_V = max(abs(dead_lut_V_V - blue_voltage_V));

fig = figure('Color', 'w', 'Position', [100 100 1200 720]);
ax = axes(fig);
hold(ax, 'on');
plot(ax, dead_lut_I_A, blue_voltage_V, 'o-', ...
    'Color', [0.12 0.42 0.75], 'LineWidth', 1.3, ...
    'MarkerSize', 5.5, 'DisplayName', 'Primary identification (blue)');
plot(ax, dead_lut_I_A, secondary_voltage_V, 's--', ...
    'Color', [0.90 0.38 0.15], 'LineWidth', 1.1, ...
    'MarkerSize', 5.0, 'DisplayName', 'Secondary identification');
plot(ax, deploy_lut_I_A, deploy_lut_V_V, 'd-', ...
    'Color', [0.10 0.62 0.30], 'LineWidth', 2.8, ...
    'MarkerSize', 7.0, 'MarkerFaceColor', [0.20 0.75 0.38], ...
    'DisplayName', 'Recommended fitted LUT');
hold(ax, 'off');

grid(ax, 'on');
ax.XMinorGrid = 'on';
ax.YMinorGrid = 'on';
ax.GridAlpha = 0.22;
ax.Box = 'on';
ax.FontSize = 12;
ax.FontName = 'Microsoft YaHei';
ax.TickDir = 'out';
title(ax, 'Recommended dead-time compensation LUT');
subtitle(ax, '85% primary data + 15% secondary data, 5-point quadratic smoothing');
xlabel(ax, 'Current magnitude |I| (A)');
ylabel(ax, 'Dead-time compensation voltage V_{dead} (V)');
legend(ax, 'Location', 'northwest', 'Box', 'off');
xlim(ax, [0 1.55]);
y_axis_max = ceil(max([blue_voltage_V secondary_voltage_V dead_lut_V_V]) / 0.05) * 0.05;
ylim(ax, [0 y_axis_max]);

summary_text = sprintf(['RMSE to blue = %.4f V\n' ...
    'Max error to blue = %.4f V'], ...
    fit_rmse_to_blue_V, fit_max_error_to_blue_V);
text(ax, 0.98, 0.05, summary_text, 'Units', 'normalized', ...
    'HorizontalAlignment', 'right', 'VerticalAlignment', 'bottom', ...
    'FontName', 'Consolas', 'FontSize', 11, ...
    'BackgroundColor', 'w', 'EdgeColor', [0.75 0.75 0.75], ...
    'Margin', 7);

script_dir = fileparts(mfilename('fullpath'));
output_path = fullfile(script_dir, 'deadtime_recommended_fitted_lut.png');
exportgraphics(ax, output_path, 'Resolution', 220);

fprintf('Output: %s\n', output_path);
fprintf('static const float DEAD_LUT_I[20] = {\n    ');
fprintf('%.2ff, ', deploy_lut_I_A(1:end-1));
fprintf('%.2ff\n};\n', deploy_lut_I_A(end));
fprintf('static const float DEAD_LUT_V[20] = {\n    ');
fprintf('%.4ff, ', deploy_lut_V_V(1:end-1));
fprintf('%.4ff\n};\n', deploy_lut_V_V(end));
fprintf('RMSE to blue: %.7f V; max error to blue: %.7f V\n', ...
    fit_rmse_to_blue_V, fit_max_error_to_blue_V);
