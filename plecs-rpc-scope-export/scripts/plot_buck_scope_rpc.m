clear; clc; close all;

csvPath = 'C:\\Users\\liaom\\.openclaw\\workspace\\matlab\\task_2026-03-19_plecs_rpc_pipeline\\buck_scope_data_rpc.csv';
outPng  = 'C:\\Users\\liaom\\.openclaw\\workspace\\matlab\\task_2026-03-19_plecs_rpc_pipeline\\buck_scope_plot_matlab.png';

T = readtable(csvPath);

t = T{:,1} * 1e3;
iL = T{:,2};
vout = T{:,3};

idx = max(1, round(0.8*height(T))):height(T);
fprintf('Average IL (last 20%%): %.6f A\n', mean(iL(idx)));
fprintf('Average Vout (last 20%%): %.6f V\n', mean(vout(idx)));
fprintf('Vout ripple (last 20%%): %.6f Vpp\n', max(vout(idx)) - min(vout(idx)));

fig = figure('Color','w','Position',[100 100 1100 700]);
subplot(2,1,1);
plot(t, iL, 'LineWidth', 1.0); grid on;
ylabel('I_L (A)'); title('Inductor Current');

subplot(2,1,2);
plot(t, vout, 'LineWidth', 1.0); grid on;
ylabel('V_{out} (V)'); xlabel('Time (ms)'); title('Output Voltage');

exportgraphics(fig, outPng, 'Resolution', 200);
fprintf('Saved plot: %s\n', outPng);
