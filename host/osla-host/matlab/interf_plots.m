%% plots
clear all; close all;
fontsize = 17;

% Define folder and base name
folder = 'plots';
% Ensure the folder exists
if ~exist(folder, 'dir')
    mkdir(folder);
end
%% awgn ber
% plots experimental ber measurements
fig = figure(1); % EsN0
ax(1) = axes(fig, 'Color', 'w');
hold(ax(1), 'on'); % Important: allow multiple lines to be plotted

EsN0_fwd_db = [0  1  2  3  4  5];
semilogy(ax(1),EsN0_fwd_db,qfunc(sqrt(2*10.^(EsN0_fwd_db/10))),'r--x',...
    'LineWidth', 1.5, 'DisplayName','Simulated BPSK');

EsN0_dbs = [0, 1, 2, 3, 4, 5, 6];
ber = [0.0752704, 0.054579, 0.0330078, 0.0213995, 0.00997383, 0.00555506, 0.00196687];
num_bits = [6656, 9216, 15360, 23552, 50432, 90368, 254720];
num_errs = [501, 503, 507, 504, 503, 502, 501];
rss_dbms = [-111.794, -110.803, -109.928, -108.915, -107.811, -106.86, -105.806] - 10*log10(50)-1.31;
semilogy(ax(1),EsN0_dbs,ber,'r-o','LineWidth', 1.5,'MarkerSize', 6.5, 'DisplayName',"Wireless BPSK");

EsN0_fwd_db = [0  1  2  3  4  5];
semilogy(ax(1),EsN0_fwd_db, exp(-4*10.^(EsN0_fwd_db/10)),'g--x',...
'LineWidth', 1.5,'MarkerSize', 6.5, 'DisplayName','Theoretical OSLA (Zero Latency)');


EsN0_dbs1 = [0, 1, 2, 3, 4, 5, 6];
ber = [0.0186435, 0.00935683, 0.0025956, 0.000620332, 8.67292e-05, 1.11905e-05, 6.99991e-07];
num_bits = [5632, 11008, 38912, 162816, 1164544, 9025536, 10000128];
num_errs = [105, 103, 101, 101, 101, 101, 7];
rss_dbms1 = [-130.107, -129.14, -128.191, -127.171, -126.126, -125.116, -124.081];
semilogy(ax(1), EsN0_dbs1, ber, 'g-o','LineWidth', 1.5,'MarkerSize', 6.5, 'DisplayName', 'Wireless OSLA');


% EsN0_dbs = [0, 1, 2, 3, 4, 5];
% ber = [0.0159375, 0.0054796, 0.00128115, 0.000189678, 2.69948e-05, 9.99808e-07];
% num_bits = [6400, 18432, 79616, 532480, 1000192, 1000192];
% num_errs = [102, 101, 102, 101, 27, 1];
% rss_dbms = [-131.078, -130.084, -129.068, -128.022, -127.09, -126.031];
% avg_sym_len = [31.4327, 32.1825, 31.7825, 31.6721, 31.4577, 31.4323];
% semilogy(ax(1), EsN0_dbs, ber, '-o', 'DisplayName', 'Wireless OSLA2');
% 
% EsN0_dbs = [0, 1, 2, 3, 4, 5];
% ber = [0.0186435, 0.0109592, 0.00226742, 0.000736066, 1.289e-4, 8.99827e-06];
% num_bits = [5632, 9216, 44544, 137216,415232, 1000192];
% num_errs = [105, 101, 101, 101,101, 9];
% rss_dbms = [ -130.728, -129.785, -128.782, -127.769,-126.776,-125.812];
% avg_sym_len = [33.9023, 35.9974, 34.0333, 34.8385,37.8351, 34.6698];
% semilogy(ax(1), EsN0_dbs, ber, '-o', 'DisplayName', 'Wireless OSLA3');


% EsN0_dbs = [0, 1, 2, 3, 4, 5, 6];
% ber = [0.0226924, 0.0188397, 0.00874013, 0.00734992, 0.0062624, 0.00445348, 0.0041489];
% num_bits = [44288, 53504, 117504, 136192, 161280, 224768, 513280];
% num_errs = [1005, 1008, 1027, 1001, 1010, 1001, 2083];
% rss_dbms = [-128.827, -127.606, -126.5, -125.296, -123.586, -122.51, -121.161];
% avg_sym_len = [32.2523, 32.6581, 32.5779, 32.5246, 32.4662, 32.4632, 32.1992];
% semilogy(ax(1),EsN0_dbs,ber,'c-o','LineWidth', 1.5,'MarkerSize', 6.5, 'DisplayName',"Wireless OSLA, ISM Band");

%Forward/Backward model: intf_mitigation_fi / noiseless_feedback
SIR = [Inf];
ber = [0.022352   0.0083112   0.0030281   0.0006075  9.5982e-05  7.9985e-06];
EsN0_dbs = [0  1  2  3  4  5];
Tavg = [31.7648       31.949      31.9086      32.0325      31.7808      31.8442];
% sim details: n_bit_err = [103  100  100  100   96    8];
% max_iters: max_n_iters = 3906.25;
semilogy(ax(1),EsN0_dbs,ber,'b--x','LineWidth', 1.5,'MarkerSize', 6.5, 'DisplayName',"Simulated OSLA-FIM (Algorithm 1)");

EsN0_dbs = [0, 1, 2, 3, 4, 5, 6];
ber = [0.0266113, 0.00739976, 0.00268086, 0.000640809, 0.000110811, 1.074975e-5, 0];
num_bits = [4096, 67840, 589056, 781824, 4521216, 6707200, 10000128];
num_errs = [109, 502, 501, 501, 501, 71, 0];
rss_dbms = [-130.741, -129.738, -128.773, -127.769, -126.695, -125.751, -124.692];
avg_sym_len = [32.2134, 31.5472, 28.5514, 32.7042, 32.0732, 31.6162, 27.946];
semilogy(ax(1),EsN0_dbs,ber,'b-o','LineWidth', 1.5,'MarkerSize', 6.5, 'DisplayName',"Wireless OSLA-FIM (Algorithm 1)");

% EsN0_dbs = [5, 4, 3, 2, 1, 0];
% ber = [0.000225957, 0.000283945, 0.00150506, 0.00509489, 0.0139569, 0.0317899];
% num_bits = [1000192, 1000192, 665088, 196864, 71936, 31488];
% num_errs = [226, 284, 1001, 1003, 1004, 1001];
% rss_dbms = [-125.748, -126.748, -127.748, -128.748, -129.748, -130.748];
% avg_sym_len = [138.046, 140.233, 141.335, 142.266, 141.51, 140.325];
% EsN0_dbs = [5, 4, 3, 2, 1, 0];
% ber = [5.49894e-05, 0.000283945, 0.00162073, 0.00610009, 0.0163411, 0.0350516];
% num_bits = [1000192, 1000192, 618240, 164096, 61440, 28672];
% num_errs = [55, 284, 1002, 1001, 1004, 1005];
% rss_dbms = [-125.748, -126.748, -127.748, -128.748, -129.748, -130.748];
% avg_sym_len = [136.085, 138.863, 139.7, 141.052, 142.868, 140.767];
% h = semilogy(ax(1), EsN0_dbs, ber, 'M-o', 'LineWidth', 1.5, 'MarkerSize', 6.5);
% h.DisplayName = ['Wireless OSLA-FIM (Algorithm 1)', newline, 'ISM Band'];
xlabel(ax(1), 'Es/N0 (dB)');
ylabel(ax(1), 'BER');
grid(ax(1), 'on');
legend(ax(1), 'Location', 'southwest','Fontsize', fontsize);

% Setup top axis
pos = ax(1).Position;
ax(2) = axes(fig, ...
    'Position', [pos(1), pos(2) + 0.0, pos(3), pos(4)], ...
    'Color', 'none', ...
    'XAxisLocation', 'top', ...
    'YAxisLocation', 'right', ...
    'XColor', 'k', ...
    'YColor', 'none', ...
    'Box', 'off');

% Sync limits and scales
ax(2).XLim = ax(1).XLim;
ax(2).YLim = ax(1).YLim;
ax(2).XScale = 'linear';
ax(2).YScale = 'log';
linkaxes(ax, 'xy');
ax(2).YTick = [];

% Set top labels (for first dataset only)
% ax(2).XTick = EsN0_dbs1;
% ax(2).XTickLabel = arrayfun(@(x) sprintf('%.1f', x), rss_dbms1, 'UniformOutput', false);
% Subsample every Nth tick
N = 2;
tick_idx = 1:N:length(EsN0_dbs1);

ax(2).XTick = EsN0_dbs1(tick_idx);
ax(2).XTickLabel = arrayfun(@(x) sprintf('%.1f', x), rss_dbms1(tick_idx), 'UniformOutput', false);

xlabel(ax(2), 'Received Power (dBm)');

set(ax(1), 'YScale', 'log');  % Reapply semilog to main axis

% Set font size for both axes
set(ax(1), 'FontSize', fontsize);  % or any size you prefer
set(ax(2), 'FontSize', fontsize);

% Optionally increase font size for xlabel, ylabel, title
xlabel(ax(1), 'EsN0 (dB)', 'FontSize', fontsize);  % adjust as needed
ylabel(ax(1), 'Bit Error Rate', 'FontSize', fontsize);
xlabel(ax(2), 'Average Received Power (dBm)', 'FontSize', fontsize);

set(gcf, 'Units', 'normalized');
set(gcf, 'Position', [0.1, 0.1, 0.53, 0.73]);  % Make the figure window larger

set(gcf, 'PaperPositionMode', 'auto');  % Ensure full content fits

filename = 'awgn_ber';

% Build full paths
png_path = fullfile(folder, [filename, '.png']);
svg_path = fullfile(folder, [filename, '.svg']);

% Export the figure
print(gcf, png_path, '-dpng', '-r600');         % High-res PNG
% exportgraphics(gcf, svg_path, 'ContentType', 'vector');  % Vector SVG% Build full paths
print(gcf, svg_path, '-dsvg');  % Export to SVG


% 
% %% rss
% figure();
% 
% semilogy(rss_dbms,ber,'-o','DisplayName',"BPSK");
% hold on
% grid on
% legend('Location', 'southwest')
% 
% 
% semilogy(rss_dbms,ber,'-o','DisplayName',"OSLA");
% 
% 
% semilogy(rss_dbms,ber,'-o','DisplayName',"OSLA");
% 
% 
% % title("OSLA-BPSK")

%% ISM
fig = figure(); % EsN0
ax(1) = axes(fig, 'Color', 'w');
hold(ax(1), 'on'); % Important: allow multiple lines to be plotted

EsN0_dbs = [5, 4, 3, 2, 1, 0];
ber = [0.0279757, 0.0449668, 0.0581578, 0.0831616, 0.111842, 0.135715];
num_bits = [357632, 222720, 172032, 120320, 89600, 73728];
num_errs = [10005, 10015, 10005, 10006, 10021, 10006];
rss_dbms = [-125.748, -126.748, -127.748, -128.748, -129.748, -130.748];
avg_sym_len = [32, 32, 32, 32, 32, 32];
semilogy(ax(1),EsN0_dbs,ber,'r-o','LineWidth', 1.5,'MarkerSize', 6.5, 'DisplayName',"Wireless BPSK");

EsN0_dbs = [0, 1, 2, 3, 4, 5, 6];
ber = [0.0226924, 0.0188397, 0.00874013, 0.00734992, 0.0062624, 0.00445348, 0.0041489];
num_bits = [44288, 53504, 117504, 136192, 161280, 224768, 513280];
num_errs = [1005, 1008, 1027, 1001, 1010, 1001, 2083];
rss_dbms = [-128.827, -127.606, -126.5, -125.296, -123.586, -122.51, -121.161];
avg_sym_len = [32.2523, 32.6581, 32.5779, 32.5246, 32.4662, 32.4632, 32.1992];
EsN0_dbs = [5, 4, 3, 2, 1, 0];
ber = [0.0113809, 0.0162412, 0.0208181, 0.0304366, 0.0547641, 0.0899605];
num_bits = [882176, 616704, 481024, 329472, 182784, 111360];
num_errs = [10040, 10016, 10014, 10028, 10010, 10018];
rss_dbms = [-125.748, -126.748, -127.748, -128.748, -129.748, -130.748];
avg_sym_len = [37.1879, 37.3028, 37.4439, 36.4367, 35.3436, 33.2773];
semilogy(ax(1),EsN0_dbs,ber,'g-o','LineWidth', 1.5,'MarkerSize', 6.5, 'DisplayName',"Wireless OSLA");

EsN0_dbs = [5, 4, 3, 2, 1, 0];
ber = [0.000225957, 0.000283945, 0.00150506, 0.00509489, 0.0139569, 0.0317899];
num_bits = [1000192, 1000192, 665088, 196864, 71936, 31488];
num_errs = [226, 284, 1001, 1003, 1004, 1001];
rss_dbms = [-125.748, -126.748, -127.748, -128.748, -129.748, -130.748];
avg_sym_len = [138.046, 140.233, 141.335, 142.266, 141.51, 140.325];
EsN0_dbs = [5, 4, 3, 2, 1, 0];
ber = [5.49894e-05, 0.000283945, 0.00162073, 0.00610009, 0.0163411, 0.0350516];
num_bits = [1000192, 1000192, 618240, 164096, 61440, 28672];
num_errs = [55, 284, 1002, 1001, 1004, 1005];
rss_dbms = [-125.748, -126.748, -127.748, -128.748, -129.748, -130.748];
avg_sym_len = [136.085, 138.863, 139.7, 141.052, 142.868, 140.767];
h = semilogy(ax(1), EsN0_dbs, ber, 'b-o', 'LineWidth', 1.5, 'MarkerSize', 6.5);
h.DisplayName = ['Wireless OSLA-FIM (Algorithm 1)'];

xlabel(ax(1), 'Es/Ni (dB)', 'FontSize', fontsize);
ylabel(ax(1), 'BER', 'FontSize', fontsize);
grid(ax(1), 'on');
legend(ax(1), 'Location', 'southwest','Fontsize', fontsize);

% Setup top axis
pos = ax(1).Position;
ax(2) = axes(fig, ...
    'Position', [pos(1), pos(2) + 0.0, pos(3), pos(4)], ...
    'Color', 'none', ...
    'XAxisLocation', 'top', ...
    'YAxisLocation', 'right', ...
    'XColor', 'k', ...
    'YColor', 'none', ...
    'Box', 'off');

% Sync limits and scales
ax(2).XLim = ax(1).XLim;
ax(2).YLim = ax(1).YLim;
ax(2).XScale = 'linear';
ax(2).YScale = 'log';
linkaxes(ax, 'xy');
ax(2).YTick = [];

% Set top labels (for first dataset only)
ax(2).XTick = EsN0_dbs1;
ax(2).XTickLabel = arrayfun(@(x) sprintf('%.1f', x), rss_dbms1, 'UniformOutput', false);
xlabel(ax(2), 'Received Power (dBm)', 'FontSize', fontsize);

set(ax(1), 'YScale', 'log');  % Reapply semilog to main axis
title("ISM Band")

% Set font size for both axes
set(ax(1), 'FontSize', fontsize);  % or any size you prefer
set(ax(2), 'FontSize', fontsize);

%% AWGN Interference
% plots experimental ber measurements
fig = figure(); % EsN0
ax(1) = axes(fig, 'Color', 'w');
hold(ax(1), 'on'); % Important: allow multiple lines to be plotted

%Top axis setup
N0_dbm = -173.414;
noise_rss_dbm = -148.677+30;
EsN0_db = 4;
EsNidb_axis = fliplr([35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15]);
interf_rss_dbm = EsN0_db - EsNidb_axis + noise_rss_dbm;

%Forward/Backward model: bpsk_fwd_fp_interf / noiseless_feedback
EsN0_fwd_db = 4;
ber = [0.14375     0.13516      0.1204    0.088778    0.060961     0.03156    0.019301    0.013686    0.012784     0.01388     0.01244];
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
Tavg = [32  32  32  32  32  32  32  32  32  32  32];
% sim details: n_bit_err = [552  519  524  500  515  509  504  501  504  501  500];
% max_iters: max_n_iters = 3906.25;
semilogy(ax(1),EsNidb,ber,'r--x','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Simulated BPSK");

EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [0.00892223, 0.0112786, 0.0102476, 0.0118354, 0.0186942, 0.0356658, 0.0738932, 0.090625, 0.13151, 0.136068, 0.148242];
num_bits = [22528, 18176, 19712, 17152, 10752, 5888, 3072, 2560, 1536, 1536, 1024];
num_errs = [201, 205, 202, 203, 201, 210, 227, 232, 202, 209, 203];
rss_dbms = [-127.02, -127.024, -127.048, -127.02, -127.014, -127.053, -127.035, -127.074, -126.976, -127.004, -127.032];
avg_sym_len = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32];
semilogy(ax(1),EsNidb,ber,'r-o','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless BPSK"); 


 EsN0_fwd_db = 4;
ber = [0.40469     0.29743     0.27539     0.15625     0.04633   0.0082925  0.00059674  0.00012798  6.9987e-05  5.4989e-05  5.7989e-05];
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
Tavg = [6.80391      13.2215      16.2075      24.1776      29.7984      31.7715      32.0722      32.0235      32.0653      31.9408      32.0935];
% sim details: n_bit_err = [518  533  564  520  510  501  500  128   70   55   58];
% max_iters: max_n_iters = 3906.25;
semilogy(ax(1),EsNidb,ber,'g--x','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Simulated OSLA");

EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [3.69929e-05, 3.49933e-05, 5.199e-05, 0.000164372, 0.00171003, 0.0179332, 0.0921875, 0.166667, 0.314453, 0.392578, 0.367578];
num_bits = [1000192, 1000192, 1000192, 620544, 59648, 5632, 1280, 768, 512, 512, 512];
num_errs = [37, 35, 52, 102, 102, 101, 118, 128, 161, 201, 137];
rss_dbms = [-127.031, -126.979, -126.986, -127.007, -126.984, -126.96, -127.001, -127.045, -126.977, -126.983, -126.957];
avg_sym_len = [31.3574, 31.268, 31.5832, 31.1807, 31.3279, 30.726, 26.2727, 22.6224, 13.8398, 7.87695, 14.6055];
semilogy(ax(1),EsNidb,ber,'g-o','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless OSLA");

% % EsN0db = [4];
% % EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% % ber = [0.0333778, 0.00302555, 0.00249642, 0.000905826, 0.000408921, 0.000128975, 0.00010598, 8.89829e-05, 4.6991e-05, 3.59931e-05, 0.000129975];
% % num_bits = [31488, 333824, 402176, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% % num_errs = [1051, 1010, 1004, 906, 409, 129, 106, 89, 47, 36, 130];
% % rss_dbms = [-127.069, -127.07, -127.067, -127.081, -127.075, -127.066, -127.108, -127.158, -127.11, -127.104, -127.105];
% % avg_sym_len = [31.5542, 35.5254, 33.7428, 36.0861, 30.1526, 33.5515, 34.905, 34.1725, 32.8548, 31.456, 35.8726];
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.216139, 0.162239, 0.0917052, 0.0476439, 0.0196687, 0.00637778, 0.00172667, 0.000374928, 8.69833e-05, 3.29937e-05, 4.19919e-05];
% num_bits = [46336, 61952, 109056, 210688, 509440, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [10015, 10051, 10001, 10038, 10020, 6379, 1727, 375, 87, 33, 42];
% rss_dbms = [-126.791, -126.816, -126.814, -126.804, -126.802, -126.787, -126.817, -126.796, -126.813, -126.787, -126.803];
% avg_sym_len = [19.0801, 23.2269, 27.011, 29.1951, 30.8847, 31.5658, 32.4834, 32.1012, 32.1827, 32.5566, 32.7182];
% semilogy(ax(1),EsNidb,ber,'c-+','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless OSLA2");
% 
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.0174512, 0.00594073, 0.00167268, 0.00026095, 0.000163969, 1.69967e-05, 2.0996e-05, 2.19958e-05, 1.29975e-05, 8.99827e-06, 1.89964e-05];
% num_bits = [287488, 841984, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [5017, 5002, 1673, 261, 164, 17, 21, 22, 13, 9, 19];
% rss_dbms = [-127.017, -126.993, -127.009, -126.964, -126.958, -126.984, -126.95, -126.968, -126.95, -126.962, -126.96];
% avg_sym_len = [29.2255, 29.717, 29.9448, 29.3687, 29.8391, 30.0309, 29.6903, 29.7135, 29.6958, 30.0383, 29.8626];
% semilogy(ax(1),EsNidb,ber,'m-+','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless OSLA2");
% 
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.000453913, 1.49971e-05, 1.19977e-05, 2.49952e-05, 2.0996e-05, 1.49971e-05, 1.49971e-05, 2.49952e-05, 2.0996e-05, 1.59969e-05, 1.59969e-05];
% num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [454, 15, 12, 25, 21, 15, 15, 25, 21, 16, 16];
% rss_dbms = [-126.951, -126.964, -126.996, -126.951, -126.955, -127.021, -126.994, -126.976, -126.992, -126.956, -126.971];
% avg_sym_len = [90.9068, 29.9445, 29.5812, 30.0782, 30.0325, 29.9078, 29.9886, 29.998, 29.853, 29.725, 29.9272];
% semilogy(ax(1),EsNidb,ber,'r-+','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless OSLA2");


%Forward/Backward model: clairvoyant_fwd / noiseless_feedback
EsN0_fwd_db = 4;
ber = [5.7989e-05  7.2986e-05  6.4988e-05  6.5987e-05  7.2986e-05  7.5985e-05  6.7987e-05  5.6989e-05  5.9988e-05  7.1986e-05  6.0988e-05];
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
Tavg = [48.7973      48.4394      47.8798       46.167      43.2562      38.5557      35.1981      33.5293      32.3169      32.1794      32.0787];
% sim details: n_bit_err = [58  73  65  66  73  76  68  57  60  72  61];
% max_iters: max_n_iters = 3906.25;
semilogy(EsNidb,ber,'m--x','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Simulated OSLA-FIM (perfect CSI)"); 

%Forward/Backward model: fwd_fp_chip_hyp_test_comp_ignore_interf_nb / noiseless_feedback
EsN0_fwd_db = 4;
ber = [9.8981e-05  0.00011598  0.00012798  0.00013897  0.00013897  0.00013197  0.00011998  9.5982e-05  0.00010298  9.6981e-05  0.00011798];
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
Tavg = [50.4385      50.4602      49.8357       50.637      50.1137      47.4612      40.4767       34.434      32.6834        32.16      32.1475];
% sim details: n_bit_err = [99  116  128  139  139  132  120   96  103   97  118];
% max_iters: max_n_iters = 3906.25;
% semilogy(EsNidb,ber,'-o','DisplayName',"Hyp Test M = 96, Estim M = 96, comp, no mean, find_L, Pf = 0.001"); 
semilogy(ax(1),EsNidb,ber,'b--x','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Simulated OSLA-FIM (Algorithm 1)"); 

EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [0.000110044, 0.000107972, 0.000115834, 9.28442e-05, 0.000112884, 0.000114393, 0.00014305,  0.000159607, 0.000151134, 0.000127928, 0.000130949];
num_bits = [4552704, 935424, 871936, 1766400, 894720, 4379648, 706048, 1572608, 3314944, 789504, 3825920];
num_errs = [501, 101, 101, 164, 101, 501, 101, 251, 501, 101, 501];
rss_dbms = [-127.07, -126.962, -126.913, -126.946, -126.896, -126.91, -126.947, -126.884, -126.872, -126.897, -126.935];
avg_sym_len = [33.4885, 32.6134, 32.9967, 31.6972, 33.3889, 35.0988, 37.6808, 41.8073, 43.7581, 45.5521, 48.2961];
semilogy(ax(1),EsNidb,ber,'b-o','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless OSLA-FIM (Algorithm 1)");
% semilogy(EsN0_dbs,ber,'-o','DisplayName',"OSLA Wireless 2.1GHz, 2m,30db fb");


xlabel("Es/Ni (dB)", 'FontSize', fontsize);
ylabel("BER", 'FontSize', fontsize)
ylim([1e-7 1]);
grid(ax(1), 'on');
legend(ax(1), 'Location', 'southwest','Fontsize', fontsize);

% Setup top axis
pos = ax(1).Position;
ax(2) = axes(fig, ...
    'Position', [pos(1), pos(2) + 0.0, pos(3), pos(4)], ...
    'Color', 'none', ...
    'XAxisLocation', 'top', ...
    'YAxisLocation', 'right', ...
    'XColor', 'k', ...
    'YColor', 'none', ...
    'Box', 'off');

% Sync limits and scales
ax(2).XLim = ax(1).XLim;
ax(2).YLim = ax(1).YLim;
ax(2).XScale = 'linear';
ax(2).YScale = 'log';
linkaxes(ax, 'xy');
ax(2).YTick = [];

% % Set top labels (for first dataset only)
% ax(2).XTick = EsNidb_axis;
% ax(2).XTickLabel = arrayfun(@(x) sprintf('%.1f', x), interf_rss_dbm, 'UniformOutput', false);

% Choose every other tick index
tick_idx = 1:2:length(EsNidb_axis);
% Set ticks and labels at those positions
ax(2).XTick = EsNidb_axis(tick_idx);
ax(2).XTickLabel = arrayfun(@(x) sprintf('%.1f', x), interf_rss_dbm(tick_idx), 'UniformOutput', false);

xlabel(ax(2), 'Average Received Interference Power (dBm)', 'FontSize', fontsize);

set(ax(1), 'YScale', 'log');  % Reapply semilog to main axis
% title("OSLA with AWGN Interference, EsN0 = 4dB, rate unconstrained, mu = 5ms")

% Set font size for both axes
set(ax(1), 'FontSize', fontsize);  % or any size you prefer
set(ax(2), 'FontSize', fontsize);


% create file------------------
filename = 'awgn_intf_ber';
set(gcf, 'Units', 'normalized');
set(gcf, 'Position', [0.1, 0.1, 0.53, 0.73]);  % Make the figure window larger
set(gcf, 'PaperPositionMode', 'auto');  % Ensure full content fits
% Build full paths
png_path = fullfile(folder, [filename, '.png']);
svg_path = fullfile(folder, [filename, '.svg']);
% Export the figure
print(gcf, png_path, '-dpng', '-r600');         % High-res PNG
% exportgraphics(gcf, svg_path, 'ContentType', 'vector');  % Vector SVG% Build full paths
print(gcf, svg_path, '-dsvg');  % Export to SVG


%% Rate-------------------------------------------------------------

N = 32; %number of chips per symbol

%4dB-5ms mu----------------------------------------
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
TavgClairvoyant = [48.7973      48.4394      47.8798       46.167      43.2562      38.5557      35.1981      33.5293      32.3169      32.1794      32.0787];
figure;
plot(EsNidb, N./TavgClairvoyant, 'm--x', 'LineWidth', 1.5, 'MarkerSize', 6.5, 'DisplayName', 'Simulated OSLA-FIM (Perfect CSI) (AWGN Interference)'); %(AWGN Interference)
hold on;

EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
TavgInterMitigation = [50.7537      50.2384      50.1158      50.1054      49.7091      46.4529      39.3266      34.1723      32.5293      31.9311      32.1081];
plot(EsNidb, N./TavgInterMitigation, 'b--x', 'LineWidth', 1.5, 'MarkerSize', 6.5, 'DisplayName', 'Simulated OSLA-FIM (Algorithm 1) (AWGN Interference)'); %(AWGN Interference)

% 
% % EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15, -20];
% % TavgInterMitigation = [33.4885, 32.6134, 32.9967, 31.6972, 33.3889, 35.0988, 37.6808, 41.8073, 43.7581, 45.5521, 48.2961, 51.0136];
% EsNidb = [-20,-15 -10  -5   0   5  10  15  20  25  30  35];
% TavgInterMitigation = [50.7948, 48.3029, 46.775, 43.4337, 40.0814, 36.2358, 34.577, 32.6754, 31.8422, 31.6806, 31.7013, 31.3149];
% plot(EsNidb, N./TavgInterMitigation, '-s', 'LineWidth', 2, 'MarkerSize', 6.5, 'Color', 'g', 'DisplayName', 'Measured Interference Mitigation');
% 
% EsNidb = [-25, -20, -10, -5, 0, 5, 10, 15];
% TavgInterMitigation = [50.7377, 50.0309, 47.1275, 44.2168, 41.0346, 37.9434, 35.2603, 33.015];
% plot(EsNidb, N./TavgInterMitigation, '-s', 'LineWidth', 2, 'MarkerSize', 6.5, 'Color', 'm', 'DisplayName', 'Measured Interference Mitigation fixed python');
% 
% 
% EsNidb = ([-30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35]);
% ber = [6.99329e-05, 3.99616e-05, 4.9952e-05, 5.99425e-05, 0.486328, 9.99041e-05, 0.000159847, 9.99041e-05, 0.000139866, 8.99137e-05, 0.000159847, 0.000109895, 0.478516, 5.99425e-05, 2.99712e-05, 0.515625, 7.99233e-05, 5.99425e-05, 0.000159847, 6.99329e-05, 0.000139866, 0.000229779, 9.99041e-05, 3.99616e-05, 9.99041e-05, 9.99041e-05, 3.99616e-05, 9.99041e-05, 7.99233e-05, 3.99616e-05, 9.99041e-05, 0.000169837, 4.9952e-05, 0.000109895, 0.000109895, 7.99233e-05, 9.99041e-05, 4.9952e-05];
% num_bits = [100096, 100096, 100096, 100096, 512, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 512, 100096, 100096, 512, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096];
% num_errs = [7, 4, 5, 6, 249, 10, 16, 10, 14, 9, 16, 11, 245, 6, 3, 264, 8, 6, 16, 7, 14, 23, 10, 4, 10, 10, 4, 10, 8, 4, 10, 17, 5, 11, 11, 8, 10, 5];
% rss_dbms = [-126.955, -126.956, -127.038, -126.989, -127.019, -127.025, -127.018, -127.008, -127.029, -126.994, -127.022, -127.052, -126.991, -127.001, -127.005, -126.987, -127.049, -127.004, -126.974, -127.003, -127.024, -126.967, -127.004, -127.001, -127.043, -127.025, -127.005, -127.019, -126.992, -127.038, -126.986, -127.037, -127.046, -127.02, -126.996, -127.007, -126.978, -127.006];
% TavgInterMitigation = [49.4201, 49.417, 48.8405, 49.103, 45795.7, 49.4147, 48.1065, 45.6392, 42.4345, 38.7129, 37.6657, 34.4879, 43309.5, 31.4577, 48.7429, 46284.1, 48.9244, 48.041, 47.2057, 46.095, 44.2309, 40.1043, 36.5756, 34.5518, 32.9774, 32.0939, 48.4161, 50.4034, 48.1972, 48.9937, 47.8507, 46.614, 43.6707, 40.405, 37.3253, 34.0564, 32.3016, 31.8764];
% 
% mask = TavgInterMitigation < 80;
% EsNidb = EsNidb(mask);
% TavgInterMitigation = TavgInterMitigation(mask);
% plot(EsNidb, N./TavgInterMitigation, '-s', 'LineWidth', 2, 'MarkerSize', 6.5, 'Color', 'c', 'DisplayName', 'Measured Interference Mitigation AGN');
% 
% EsNidb = ([-30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35])/2+7;
% ber = [6.99329e-05, 3.99616e-05, 4.9952e-05, 5.99425e-05, 0.486328, 9.99041e-05, 0.000159847, 9.99041e-05, 0.000139866, 8.99137e-05, 0.000159847, 0.000109895, 0.478516, 5.99425e-05, 2.99712e-05, 0.515625, 7.99233e-05, 5.99425e-05, 0.000159847, 6.99329e-05, 0.000139866, 0.000229779, 9.99041e-05, 3.99616e-05, 9.99041e-05, 9.99041e-05, 3.99616e-05, 9.99041e-05, 7.99233e-05, 3.99616e-05, 9.99041e-05, 0.000169837, 4.9952e-05, 0.000109895, 0.000109895, 7.99233e-05, 9.99041e-05, 4.9952e-05];
% num_bits = [100096, 100096, 100096, 100096, 512, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 512, 100096, 100096, 512, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096];
% num_errs = [7, 4, 5, 6, 249, 10, 16, 10, 14, 9, 16, 11, 245, 6, 3, 264, 8, 6, 16, 7, 14, 23, 10, 4, 10, 10, 4, 10, 8, 4, 10, 17, 5, 11, 11, 8, 10, 5];
% rss_dbms = [-126.955, -126.956, -127.038, -126.989, -127.019, -127.025, -127.018, -127.008, -127.029, -126.994, -127.022, -127.052, -126.991, -127.001, -127.005, -126.987, -127.049, -127.004, -126.974, -127.003, -127.024, -126.967, -127.004, -127.001, -127.043, -127.025, -127.005, -127.019, -126.992, -127.038, -126.986, -127.037, -127.046, -127.02, -126.996, -127.007, -126.978, -127.006];
% TavgInterMitigation = [49.4201, 49.417, 48.8405, 49.103, 45795.7, 49.4147, 48.1065, 45.6392, 42.4345, 38.7129, 37.6657, 34.4879, 43309.5, 31.4577, 48.7429, 46284.1, 48.9244, 48.041, 47.2057, 46.095, 44.2309, 40.1043, 36.5756, 34.5518, 32.9774, 32.0939, 48.4161, 50.4034, 48.1972, 48.9937, 47.8507, 46.614, 43.6707, 40.405, 37.3253, 34.0564, 32.3016, 31.8764];
% mask = TavgInterMitigation < 80;
% EsNidb = EsNidb(mask);
% TavgInterMitigation = TavgInterMitigation(mask);
% plot(EsNidb, N./TavgInterMitigation, '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation AGN Corrected');

% EsNidb = [-30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.000195313, 9.76563e-05, 9.76563e-05, 0, 9.76563e-05, 0, ...
%         9.76563e-05, 0, 0.000195313, 0, 0, 9.76563e-05, 9.76563e-05, 0];
% num_bits = [10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, ...
%             10240, 10240, 10240, 10240, 10240, 10240];
% num_errs = [2, 1, 1, 0, 1, 0, 1, 0, 2, 0, 0, 1, 1, 0];
% rss_dbms = [-127.056, -127.004, -127.049, -127.006, -127.02, -126.98, ...
%             -127.009, -127.025, -127.018, -126.993, -127.004, -127.006, ...
%             -127.026, -127.03];
% TavgInterMitigation = [48.3081, 46.8247, 49.4343, 49.3807, 49.004, 48.0602, ...
%                48.4757, 48.2891, 45.7791, 40.0967, 33.1613, 32.0664, ...
%                31.3124, 31.5799];
% plot(EsNidb, N./TavgInterMitigation, '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation AGN Corrected Var');

% EsN0_dbs = [4];
% EsNidb = [-30, -25, -20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0, 0, 9.76563e-05, 0, 9.76563e-05, 9.76563e-05, 9.76563e-05, 0.000195313, 0.000292969, 0, 9.76563e-05, 0.000195313, 9.76563e-05, 0];
% num_bits = [10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240];
% num_errs = [0, 0, 1, 0, 1, 1, 1, 2, 3, 0, 1, 2, 1, 0];
% rss_dbms = [-127.037, -127.003, -126.984, -127.021, -127.068, -126.985, -127.021, -127.066, -127.046, -127.048, -127.061, -127.008, -127.021, -127.017];
% avg_sym_len = [48.2759, 51.1862, 50.6043, 53.2382, 50.6755, 48.2866, 50.9444, 50.7182, 47.3215, 41.6478, 35.1905, 31.9856, 31.9425, 31.765];
EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [3.89925e-05, 5.39896e-05, 4.39916e-05, 5.09902e-05, 8.29841e-05, 0.000110358, 0.000117002, 0.000101893, 5.89887e-05, 4.79908e-05, 4.29917e-05];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 915200, 863232, 991232, 1000192, 1000192, 1000192];
num_errs = [39, 54, 44, 51, 83, 101, 101, 101, 59, 48, 43];
rss_dbms = [-127.069, -127.017, -127.065, -127.059, -127.076, -126.996, -127.073, -127.036, -127.033, -127.032, -127.055];
avg_sym_len = [32.2227, 32.0849, 32.707, 35.1516, 41.9964, 47.2533, 50.7363, 50.541, 50.7622, 50.8952, 51.2615];
EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [5.99425e-05, 9.99041e-05, 2.99712e-05, 8.99137e-05, 6.99329e-05, 7.99233e-05, 0.000119885, 0.000109895, 5.99425e-05, 7.99233e-05, 6.99329e-05];
num_bits = [100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096];
num_errs = [6, 10, 3, 9, 7, 8, 12, 11, 6, 8, 7];
rss_dbms = [-127.083, -127.071, -127.078, -127.044, -127.059, -127.095, -127.047, -127.037, -127.066, -127.017, -127.029];
avg_sym_len = [31.9402, 32.2526, 32.7099, 34.5253, 41.3275, 48.1273, 49.8606, 51.2654, 51.5605, 51.2229, 50.6548];
plot(EsNidb, N./avg_sym_len, 'b-o', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Wireless OSLA-FIM (Algorithm 1) (AWGN Interference)'); % (AWGN Interference)

% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% avg_sym_len = [32.9239, 33.4961, 33.9707, 34.4836, 36.7421, 39.2078, 39.2078, 46.5727, 47.2145, 50.5197, 50.7014] ...
% + [32.1755, 32.1749, 31.633, 34.2183, 31.6868, 33.1932, 33.3237, 35.5743, 40.5968, 44.7532, 48.9692]...
% +[31.4567, 32.9992, 31.9643, 30.5337, 31.9971, 33.8497, 34.3949, 35.5664, 38.9333, 43.4503, 45.3055]...
% +[32.1735, 32.6371, 32.27, 31.8829, 34.4325, 38.2922, 40.4925, 42.6898, 47.1519, 46.5604, 44.3451]...
% +[32.5182, 32.0497, 32.2377, 32.5719, 32.5707, 34.058, 35.4143, 36.0479, 40.5104, 41.5834, 46.3158]...
% +[31.9665, 31.6287, 32.8807, 32.3097, 32.6049, 34.107, 34.8068, 36.6653, 41.0007, 43.3739, 45.2297]...
% +[31.5604, 32.4564, 32.594, 31.3241, 32.4813, 32.8265, 34.6967, 38.1684, 39.839, 44.1388, 43.7602]...
% +[32.4045, 30.7603, 30.381, 31.5358, 33.6574, 32.4845, 34.3103, 35.7418, 39.1497, 41.1502, 44.4526];
% plot(EsNidb, 8*N./avg_sym_len, 'c-o', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Wireless OSLA-FIM (Algorithm 1) (BLE Interference)');


% Tavg = [1504.1969       1190.328      860.34286      485.53239      253.92744      145.69132       96.00819       77.98925      71.699002      70.142821      69.479396];
% plot(EsNidb, N./Tavg, '-o', 'LineWidth', 2, 'MarkerSize', 6.5, 'Color', 'g', 'DisplayName', 'ARQ, CRC-9, HD = 3');

%Forward/Backward model: intf_mitigation_approx / noiseless_feedback
EsN0_fwd_db = 4;
ber = [0.00011199  0.00014597  0.00020236  0.00020774  0.00020534  0.00015813  0.00012558  0.00010334  9.5998e-05  0.00010892  0.00010191];
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
Tavg = [50.6263      50.6013      50.4741      50.1844      47.9401       40.835      34.4401      32.6751      32.1277      32.1455      31.9695];
% sim details: n_bit_err = [300  300  300  300  300  300  300  300  288  300  300];
% max_iters: max_n_iters = 11718.75;
%Forward/Backward model: intf_mitigation_approx / noiseless_feedback
EsN0_fwd_db = 4;
ber = [9.3982e-05  0.00011098  0.00016897  0.00021596  0.00017797  0.00018097  0.00010398  0.00010498  0.00011698  9.0983e-05  0.00010998];
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
Tavg = [50.7142      50.3629      50.5443      50.5558      49.5285      42.2005       34.562      32.6283      31.9128       32.081      32.1384];
% sim details: n_bit_err = [94  111  169  216  178  181  104  105  117   91  110];
% max_iters: max_n_iters = 3906.25;
plot(EsNidb, N./Tavg, 'c--x', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Simulated OSLA-FIM (Algorithm 1) (BLE Interference)');

%somewhat fits sim curve?
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.000126976, 0.000179965, 0.000192963, 0.000168968, 0.000201961, 7.7985e-05, 0.000172967, 0.00025795, 0.000158969, 0.000106979, 5.49894e-05];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [127, 180, 193, 169, 202, 78, 173, 258, 159, 107, 55];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [52.8766, 53.3837, 52.3456, 50.8669, 48.3615, 40.3485, 37.3758, 36.6296, 34.0873, 33.2087, 31.3879];

% is_fixed_length = 0, is_intf_mode = 1
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [8.39839e-05, 0.00010398, 0.000122976, 0.000165968, 0.000213959, 0.000272948, 0.00015597, 4.29917e-05, 0.000136974, 9.59816e-05, 7.59854e-05];
% num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [84, 104, 123, 166, 214, 273, 156, 43, 137, 96, 76];
% rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
% avg_sym_len = [51.2068, 51.2204, 51.386, 52.6805, 52.3864, 50.7467, 42.6662, 34.7962, 34.6814, 33.2521, 32.0762];

% is_fixed_length = 0, is_intf_mode = 1
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.000118977, 0.000182965, 6.39877e-05, 8.19843e-05, 0.000202961, 0.000203961, 7.19862e-05, 7.49856e-05, 6.69871e-05, 5.49894e-05, 9.99808e-05];
% num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [119, 183, 64, 82, 203, 204, 72, 75, 67, 55, 100];
% rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
% avg_sym_len = [53.0635, 54.6091, 49.9756, 49.6393, 51.1159, 48.358, 40.6825, 35.8468, 33.1946, 32.1454, 33.2428];

%composite
% avg_sym_len = [51.2068, 51.2204, 51.386, 52.6805, 51.1159, 48.358, 40.6825, 34.7962, 33.1946, 32.1454, 32.0762];



% is_fixed_length = 0, is_intf_mode = 1
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [5.49894e-05, 8.19843e-05, 7.99846e-05, 7.59854e-05, 0.000178966, 5.09902e-05, 4.89906e-05, 3.49933e-05, 0.00707901, 6.69871e-05, 9.99808e-05];
% num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 58624, 1000192, 1000192];
% num_errs = [55, 82, 80, 76, 179, 51, 49, 35, 415, 67, 100];
% rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
% avg_sym_len = [50.4531, 52.0488, 50.8287, 48.9612, 47.5844, 39.3317, 35.2002, 32.4213, 692.108, 33.3792, 34.0403];


% is_fixed_length = 0, is_intf_mode = 1
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.000141973, 0.000100981, 0.000119977, 0.000179965, 0.000214959, 0.000130975, 0.00010398, 6.89868e-05, 7.7985e-05, 0.000176966, 0.00015597];
% num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [142, 101, 120, 180, 215, 131, 104, 69, 78, 177, 156];
% rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
% avg_sym_len = [54.2075, 52.1414, 51.121, 52.1539, 49.5886, 41.6864, 37.2907, 34.1671, 33.6155, 35.5201, 34.6812];

%composite
avg_sym_len = [50.4531, 52.1414, 51.121, 52.1539, 49.5886, 41.6864, 35.2002, 32.4213, 32.7583, 31.8741, 32.4886];


plot(EsNidb, N./avg_sym_len, 'c-o', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Wireless OSLA-FIM (Algorithm 1) (BLE Interference)');

% Add labels and title
ylim([0 1.1]);
xlabel("EsNi (dB)", 'FontSize', fontsize);
ylabel('Normalized Data Rate', 'FontSize', fontsize);
% title('AWGN Interference EsN0 = 4dB, mu = 5ms')
legend('Location', 'best','Fontsize', fontsize);
grid on;

% Set axis font sizes
ax = gca;
set(ax, 'FontSize', fontsize);  % or any value you prefer

% create file------------------
filename = 'awgn_intf_rate';
set(gcf, 'Units', 'normalized');
set(gcf, 'Position', [0.1, 0.1, 0.53, 0.73]);  % Make the figure window larger
set(gcf, 'PaperPositionMode', 'auto');  % Ensure full content fits
% Build full paths
png_path = fullfile(folder, [filename, '.png']);
svg_path = fullfile(folder, [filename, '.svg']);
% Export the figure
print(gcf, png_path, '-dpng', '-r600');         % High-res PNG
% exportgraphics(gcf, svg_path, 'ContentType', 'vector');  % Vector SVG% Build full paths
print(gcf, svg_path, '-dsvg');  % Export to SVG



% % Goodput----------------------------
% figure;
% % EsN0_dbs = [4];
% % EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% % ber = [5.99425e-05, 9.99041e-05, 2.99712e-05, 8.99137e-05, 6.99329e-05, 7.99233e-05, 0.000119885, 0.000109895, 5.99425e-05, 7.99233e-05, 6.99329e-05];
% % num_bits = [100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096, 100096];
% % num_errs = [6, 10, 3, 9, 7, 8, 12, 11, 6, 8, 7];
% % rss_dbms = [-127.083, -127.071, -127.078, -127.044, -127.059, -127.095, -127.047, -127.037, -127.066, -127.017, -127.029];
% % avg_sym_len = [31.9402, 32.2526, 32.7099, 34.5253, 41.3275, 48.1273, 49.8606, 51.2654, 51.5605, 51.2229, 50.6548];
% % plot(EsNidb, N./avg_sym_len.*bsc_capacity(ber), '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Wireless OSLA-FIM');
% 
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% %Forward/Backward model: fwd_fp_chip_hyp_test_comp_ignore_interf_nb / noiseless_feedback
% EsN0_fwd_db = 4;
% ber = [9.8981e-05  0.00011598  0.00012798  0.00013897  0.00013897  0.00013197  0.00011998  9.5982e-05  0.00010298  9.6981e-05  0.00011798];
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% Tavg = [50.4385      50.4602      49.8357       50.637      50.1137      47.4612      40.4767       34.434      32.6834        32.16      32.1475];
% % sim details: n_bit_err = [99  116  128  139  139  132  120   96  103   97  118];
% % max_iters: max_n_iters = 3906.25;
% plot(EsNidb, N./Tavg.*bsc_capacity(ber), '-s', 'LineWidth', 2, 'MarkerSize', 6.5, 'Color', 'r', 'DisplayName', 'Simulated OSLA-FIM');
% 
% hold on;
% 
% 
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% EsN0_fwd_db = 4;
% ber = [5.7989e-05  7.2986e-05  6.4988e-05  6.5987e-05  7.2986e-05  7.5985e-05  6.7987e-05  5.6989e-05  5.9988e-05  7.1986e-05  6.0988e-05];
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% Tavg = [48.7973      48.4394      47.8798       46.167      43.2562      38.5557      35.1981      33.5293      32.3169      32.1794      32.0787];
% % sim details: n_bit_err = [58  73  65  66  73  76  68  57  60  72  61];
% max_n_iters = 3906.25;
% plot(EsNidb, N./Tavg.*bsc_capacity(ber), '-o', 'LineWidth', 2, 'MarkerSize', 6.5, 'Color', 'b', 'DisplayName', 'Simulated OSLA-FIM (Perfect CSI)');
% 
% % EsN0_dbs = [4];
% % EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% % ber = [3.69929e-05, 3.49933e-05, 5.199e-05, 0.000164372, 0.00171003, 0.0179332, 0.0921875, 0.166667, 0.314453, 0.392578, 0.367578];
% % num_bits = [1000192, 1000192, 1000192, 620544, 59648, 5632, 1280, 768, 512, 512, 512];
% % num_errs = [37, 35, 52, 102, 102, 101, 118, 128, 161, 201, 137];
% % rss_dbms = [-127.031, -126.979, -126.986, -127.007, -126.984, -126.96, -127.001, -127.045, -126.977, -126.983, -126.957];
% % avg_sym_len = [31.3574, 31.268, 31.5832, 31.1807, 31.3279, 30.726, 26.2727, 22.6224, 13.8398, 7.87695, 14.6055];
% % plot(EsNidb, N./avg_sym_len.*bsc_capacity(ber), '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Wireless OSLA');
% 
% EsN0_fwd_db = 4;
% ber = [0.40469     0.29743     0.27539     0.15625     0.04633   0.0082925  0.00059674  0.00012798  6.9987e-05  5.4989e-05  5.7989e-05];
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% avg_sym_len = [6.80391      13.2215      16.2075      24.1776      29.7984      31.7715      32.0722      32.0235      32.0653      31.9408      32.0935];
% % sim details: n_bit_err = [518  533  564  520  510  501  500  128   70   55   58];
% % max_iters: max_n_iters = 3906.25;
% plot(EsNidb, N./avg_sym_len.*bsc_capacity(ber), '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Simulated OSLA');
% 
% % EsN0_dbs = [4];
% % EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% % ber = [0.00892223, 0.0112786, 0.0102476, 0.0118354, 0.0186942, 0.0356658, 0.0738932, 0.100625, 0.13151, 0.136068, 0.148242];
% % num_bits = [22528, 18176, 19712, 17152, 10752, 5888, 3072, 2560, 1536, 1536, 1024];
% % num_errs = [201, 205, 202, 203, 201, 210, 227, 232, 202, 209, 203];
% % rss_dbms = [-127.02, -127.024, -127.048, -127.02, -127.014, -127.053, -127.035, -127.074, -126.976, -127.004, -127.032];
% % avg_sym_len = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32];
% % plot(EsNidb, N./avg_sym_len.*bsc_capacity(ber), '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Wireless BPSK');
% 
% 
% %Forward/Backward model: bpsk_fwd_fp_interf / noiseless_feedback
% EsN0_fwd_db = 4;
% ber = [0.14375     0.13516      0.1204    0.088778    0.060961     0.03156    0.019301    0.013686    0.012784     0.01388     0.01244];
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% avg_sym_len = [32  32  32  32  32  32  32  32  32  32  32];
% % sim details: n_bit_err = [552  519  524  500  515  509  504  501  504  501  500];
% % max_iters: max_n_iters = 3906.25;
% plot(EsNidb, N./avg_sym_len.*bsc_capacity(ber), '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Simulated BPSK');
% 
% 
% % % Tavg = [1504.1969       1190.328      860.34286      485.53239      253.92744      145.69132       96.00819       77.98925      71.699002      70.142821      69.479396];
% % % plot(EsNidb, N./Tavg, '-o', 'LineWidth', 2, 'MarkerSize', 6.5, 'Color', 'g', 'DisplayName', 'ARQ, CRC-9, HD = 3');
% 
% % Add labels and title
% xlabel("EsNi (dB)");
% ylabel('Normalized Binary Capacity');
% title('AGN,EsN0 = 4dB, mu = 5ms')
% legend('Location', 'best');
% grid on;



%% BLE Interference
% plots experimental ber measurements

fig = figure(); % EsN0
ax(1) = axes(fig, 'Color', 'w');
hold(ax(1), 'on'); % Important: allow multiple lines to be plotted

%Top axis setup
N0_dbm = -173.414;
noise_rss_dbm = -148.677+30;
EsN0_db = 4;
EsNidb_axis = fliplr([35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15]);
interf_rss_dbm = EsN0_db - EsNidb_axis +noise_rss_dbm;

%Forward/Backward model: bpsk_fwd_fp_interf / noiseless_feedback
EsN0_fwd_db = 4;
ber = [0.17306     0.14641     0.10567    0.066667    0.042003    0.022602    0.015688     0.01347    0.012766    0.012791    0.012413];
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
Tavg = [32  32  32  32  32  32  32  32  32  32  32];
% sim details: n_bit_err = [1019  1012  1028  1024  1000  1001  1000  1000  1000  1002  1001];
% max_iters: max_n_iters = 3906.25;
semilogy(ax(1),EsNidb,ber,'r--x','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Simulated BPSK"); 


EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [0.0174735, 0.0163086, 0.0176309, 0.016871, 0.021875, 0.0279576, 0.0350167, 0.0513672, 0.0734954, 0.0976562, 0.119256];
num_bits = [28672, 30720, 28416, 29696, 23040, 17920, 14336, 10240, 6912, 5376, 4352];
num_errs = [501, 501, 501, 501, 504, 501, 502, 526, 508, 525, 519];
rss_dbms = [-126.83, -126.829, -126.827, -126.802, -126.784, -126.853, -126.802, -126.784, -126.817, -126.819, -126.87];
avg_sym_len = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32];
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.142358, 0.126096, 0.0904405, 0.052151, 0.027542, 0.0184436, 0.00930721, 0.0130221, 0.0088743, 0.0072816, 0.0093602];
num_bits = [70400, 79360, 110592, 192000, 363264, 542464, 1000192, 768000, 1000192, 1000192, 1000192];
num_errs = [10022, 10007, 10002, 10013, 10005, 10005, 9309, 10001, 8876, 7283, 9362];
rss_dbms = [-126.687, -126.728, -126.702, -126.757, -126.647, -126.676, -126.701, -126.701, -126.744, -126.691, -126.645];
avg_sym_len = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32];
% compensate for bad noise measurement
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.137833, 0.115697, 0.0920221, 0.0566689, 0.0342422, 0.0155442, 0.00920723, 0.012402, 0.00952117, 0.0107218, 0.00814044];
num_bits = [72704, 86528, 108800, 176640, 292096, 643584, 1000192, 806400, 1000192, 932864, 1000192];
num_errs = [10021, 10011, 10012, 10010, 10002, 10004, 9209, 10001, 9523, 10002, 8142];
rss_dbms = [-126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827];
avg_sym_len = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32];
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.138643, 0.122243, 0.0922248, 0.0613349, 0.036712, 0.0178876, 0.0115228, 0.0100951, 0.0115218, 0.0136964, 0.0120127];
num_bits = [360704, 409088, 542208, 815360, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [50009, 50008, 50005, 50010, 36719, 17891, 11525, 10097, 11524, 13699, 12015];
rss_dbms = [-126.943, -126.943, -126.943, -126.943, -126.943, -126.943, -126.943, -126.943, -126.943, -126.943, -126.943];
avg_sym_len = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32];
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.145202, 0.116195, 0.0956974, 0.062384, 0.0348513, 0.020584, 0.0143762, 0.0110179, 0.0117487, 0.0172927, 0.0132035];
num_bits = [344576, 430336, 522752, 801536, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [50033, 50003, 50026, 50003, 34858, 20588, 14379, 11020, 11751, 17296, 13206];
rss_dbms = [-127.002, -127.002, -127.002, -127.002, -127.002, -127.002, -127.002, -127.002, -127.002, -127.002, -127.002];
avg_sym_len = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32];
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.14254, 0.120641, 0.0992235, 0.0644151, 0.0431967, 0.0248922, 0.0124006, 0.0153621, 0.010567, 0.00771652, 0.0137974];
num_bits = [350976, 414976, 504064, 776448, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [50028, 50063, 50015, 50015, 43205, 24897, 12403, 15365, 10569, 7718, 13800];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32];
semilogy(ax(1),EsNidb,ber,'r-o','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless BPSK"); 

%OSLA
EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [3.29937e-05, 5.199e-05, 0.00014305, 0.0014129, 0.00612981, 0.0061849, 0.022017, 0.0595703, 0.259115, 0.157031, 0.539062];
num_bits = [1000192, 1000192, 706048, 72192, 16640, 18432, 5632, 2048, 768, 1280, 256];
num_errs = [33, 52, 101, 102, 102, 114, 124, 122, 199, 201, 138];
rss_dbms = [-126.802, -126.736, -126.678, -126.723, -126.755, -126.708, -126.706, -126.734, -126.695, -126.713, -126.706];
avg_sym_len = [30.7259, 31.3853, 30.3708, 31.1163, 31.0374, 29.7004, 30.9324, 27.4385, 17.0378, 23.1055, 4.29688];

EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [0.000112691, 6.99866e-05, 0.000291846, 0.00241071, 0.00198013, 0.0490451, 0.0911458, 0.110026, 0.269531, 0.472656, 0.220703];
num_bits = [896256, 1000192, 356352, 44800, 55552, 2304, 1536, 1536, 512, 256, 512];
num_errs = [101, 70, 104, 108, 110, 113, 140, 169, 138, 121, 113];
rss_dbms = [-126.741, -126.65, -126.745, -126.764, -126.726, -126.738, -126.699, -126.724, -126.771, -126.745, -126.761];
avg_sym_len = [35.9471, 33.5012, 35.2519, 35.8813, 34.2517, 32.7713, 30.849, 28.9453, 17.0273, 5.82812, 20.4355];
EsN0_dbs = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.303571, 0.189918, 0.145744, 0.104768, 0.035831, 0.0101139, 0.00429216, 0.00083084, 0.000273947, 7.7985e-05, 5.59893e-05];
num_bits = [3584, 5376, 7424, 9984, 28160, 99072, 233216, 1000192, 1000192, 1000192, 1000192];
num_errs = [1088, 1021, 1082, 1046, 1009, 1002, 1001, 831, 274, 78, 56];
rss_dbms = [-126.777, -126.742, -126.743, -126.773, -126.749, -126.775, -126.74, -126.73, -126.772, -126.727, -126.736];
avg_sym_len = [12.9515, 19.9048, 25.6294, 26.1013, 31.9205, 32.3101, 31.5851, 32.9931, 34.0688, 32.2594, 33.439];
% semilogy(EsNidb,ber,'-o','DisplayName',"Wireless OSLA");

%Forward/Backward model: bpsk_fwd_fp_interf / noiseless_feedback
EsN0_fwd_db = 4;
ber = [0.33919     0.26146     0.17862    0.087381    0.022584   0.0024484  0.00026695  7.9985e-05  6.1988e-05  7.8985e-05  6.4988e-05];
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
Tavg = [11.4202      16.8896      21.3833      27.5163      31.1314       32.091      32.0842      31.9697      32.0879      31.7945      32.1011];
% sim details: n_bit_err = [1042  1004  1006  1029  1006  1001   267    80    62    79    65];
% max_iters: max_n_iters = 3906.25;
semilogy(ax(1),EsNidb,ber,'g--x','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Simulated OSLA");

% EsN0_dbs = [4];
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% ber = [8.39839e-05, 0.000117977, 0.000336935, 0.000971813, 0.00327095, 0.0126482, 0.0384615, 0.0894965, 0.151142, 0.177649, 0.216283];
% num_bits = [1000192, 1000192, 1000192, 1000192, 306944, 79616, 26624, 11520, 6656, 5888, 4864];
% num_errs = [84, 118, 337, 972, 1004, 1007, 1024, 1031, 1006, 1046, 1052];
% rss_dbms = [-126.664, -126.764, -126.761, -126.766, -126.766, -126.688, -126.737, -126.723, -126.718, -126.693, -126.762];
% avg_sym_len = [34.9071, 33.9911, 35.4504, 35.2014, 34.9018, 33.0141, 31.8547, 30.5724, 23.7787, 23.3595, 21.3287];
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.326074, 0.259597, 0.187201, 0.077946, 0.0220092, 0.00384026, 0.000604884, 6.19881e-05, 8.79831e-05, 3.69929e-05, 4.59912e-05];
num_bits = [30720, 38656, 53504, 128512, 454400, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [10017, 10035, 10016, 10017, 10001, 3841, 605, 62, 88, 37, 46];
rss_dbms = [-126.955, -126.778, -126.757, -126.658, -126.703, -126.72, -126.737, -126.767, -126.723, -126.707, -126.755];
avg_sym_len = [12.4926, 17.0079, 23.1594, 28.47, 31.7338, 33.0258, 35.2716, 31.9198, 33.4928, 33.1669, 32.9437];
% the tests below are later trials where the calibration goes out of sync i
% think.
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.186963, 0.121349, 0.0353399, 0.00846238, 0.00120077, 0.000127975, 5.99885e-05, 3.99923e-05, 6.09883e-05, 4.79908e-05, 5.89887e-05];
% num_bits = [54016, 82432, 283136, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [10099, 10003, 10006, 8464, 1201, 128, 60, 40, 61, 48, 59];
% rss_dbms = [-126.759, -126.758, -126.69, -126.725, -126.718, -126.693, -126.694, -126.733, -126.653, -126.7, -126.71];
% avg_sym_len = [20.8676, 26.0124, 30.0273, 32.9944, 33.0775, 32.4504, 32.7827, 32.7775, 34.0692, 32.6106, 34.0404];
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.213606, 0.118273, 0.0370446, 0.0072716, 0.00117877, 0.000173967, 7.19862e-05, 3.79927e-05, 6.49875e-05, 4.79908e-05, 1.89964e-05];
% num_bits = [46848, 84736, 270080, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [10007, 10022, 10005, 7273, 1179, 174, 72, 38, 65, 48, 19];
% rss_dbms = [-126.713, -126.716, -126.725, -126.737, -126.727, -126.738, -126.737, -126.702, -126.728, -126.717, -126.709];
% avg_sym_len = [20.317, 25.6296, 29.9206, 31.6189, 33.0634, 33.4092, 33.2424, 31.326, 34.0213, 33.5854, 30.9846];
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.224991, 0.124515, 0.0412817, 0.00952817, 0.00165568, 0.000220958, 0.0175259, 5.09902e-05, 7.09864e-05, 3.79927e-05, 4.99904e-05];
% num_bits = [44544, 80384, 242432, 1000192, 1000192, 1000192, 573952, 1000192, 1000192, 1000192, 1000192];
% num_errs = [10022, 10009, 10008, 9530, 1656, 221, 10059, 51, 71, 38, 50];
% rss_dbms = [-126.703, -126.698, -126.702, -126.764, -126.718, -126.77, -126.664, -126.758, -126.765, -126.731, -126.755];
% avg_sym_len = [20.5374, 26.0655, 30.194, 32.6483, 34.0309, 33.5548, 31.8765, 32.9391, 33.3574, 32.4951, 32.2083];
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.21016, 0.124492, 0.0469736, 0.0088353, 0.00148571, 0.000167968, 9.19823e-05, 6.09883e-05, 2.69948e-05, 5.49894e-05, 3.79927e-05];
% num_bits = [47616, 80640, 212992, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [10007, 10039, 10005, 8837, 1486, 168, 92, 61, 27, 55, 38];
% rss_dbms = [-126.691, -126.739, -126.683, -126.788, -126.749, -126.76, -126.741, -126.735, -126.774, -126.769, -126.713];
% avg_sym_len = [19.8608, 25.5644, 31.9802, 31.6791, 34.0886, 33.2078, 34.0442, 33.9438, 31.6876, 32.9072, 32.2457];
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.21799, 0.119141, 0.0473473, 0.0119323, 0.00140873, 0.000242953, 4.49914e-05, 7.89848e-05, 2.39954e-05, 8.59835e-05, 0.00405322];
% num_bits = [46080, 83968, 211712, 838144, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [10045, 10004, 10024, 10001, 1409, 243, 45, 79, 24, 86, 4054];
% rss_dbms = [-126.704, -126.723, -126.683, -126.71, -126.734, -126.765, -126.746, -126.769, -126.716, -126.712, -126.699];
% avg_sym_len = [20.0731, 25.0073, 31.5607, 34.0157, 32.8621, 33.4294, 31.4419, 33.5614, 31.6941, 34.9564, 383.673];
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.180246, 0.110084, 0.0584662, 0.0122405, 0.00174367, 0.000170967, 5.39896e-05, 3.0994e-05, 1.99962e-05, 2.0996e-05, 1.59969e-05];
% num_bits = [55552, 91648, 172544, 818432, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [10013, 10089, 10088, 10018, 1744, 171, 54, 31, 20, 21, 16];
% rss_dbms = [-126.726, -126.678, -126.683, -126.691, -126.709, -126.699, -126.693, -126.747, -126.741, -126.709, -126.715];
% avg_sym_len = [20.977, 139.407, 30.1952, 30.4068, 31.8283, 30.899, 32.5593, 30.4376, 29.2212, 31.2049, 31.069];
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.238234, 0.14577, 0.0547313, 0.0115701, 0.00179666, 0.000180965, 1.59969e-05, 2.0996e-05, 4.29917e-05, 3.69929e-05, 3.29937e-05];
% num_bits = [42240, 68608, 182784, 867584, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [10063, 10001, 10004, 10038, 1797, 181, 16, 21, 43, 37, 33];
% rss_dbms = [-126.759, -126.697, -126.723, -126.691, -126.675, -126.675, -126.711, -126.667, -126.651, -126.71, -126.71];
% avg_sym_len = [18.6678, 23.5827, 29.2278, 30.4411, 32.3739, 31.0191, 30.4535, 30.3066, 32.5052, 31.4581, 32.0504];
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.211421, 0.139662, 0.0595822, 0.0276391, 0.00112878, 0.000145972, 8.19843e-05, 2.89944e-05, 2.99942e-05, 2.79946e-05, 3.0994e-05];
% num_bits = [47616, 71680, 167936, 362240, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [10067, 10011, 10006, 10012, 1129, 146, 82, 29, 30, 28, 31];
% rss_dbms = [-126.733, -126.722, -126.725, -126.721, -126.696, -126.697, -126.696, -126.684, -126.665, -126.66, -126.662];
% avg_sym_len = [19.4927, 22.9465, 28.4609, 28.1695, 28.7201, 30.4381, 33.0037, 31.1512, 31.0187, 30.4439, 30.6417];
% EsN0db = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.225171, 0.137613, 0.0491696, 0.0128379, 0.00202661, 0.000140973, 3.99923e-05, 2.29956e-05, 1.59969e-05, 8.99827e-06, 3.29937e-05];
% num_bits = [44544, 72704, 203520, 780032, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [10030, 10005, 10007, 10014, 2027, 141, 40, 23, 16, 9, 33];
% rss_dbms = [-126.683, -126.736, -126.699, -126.678, -126.648, -126.692, -126.763, -126.699, -126.664, -126.729, -126.706];
% avg_sym_len = [17.4822, 23.7913, 28.1072, 30.7911, 33.1298, 30.4378, 31.2401, 30.3168, 30.2435, 28.7829, 31.0713];

EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.308477, 0.266707, 0.155622, 0.0812097, 0.020575, 0.00619181, 0.000445914, 0.000106979, 5.89887e-05, 3.99923e-05, 3.49933e-05];
num_bits = [162304, 187648, 321536, 615936, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [50067, 50047, 50038, 50020, 20579, 6193, 446, 107, 59, 40, 35];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [13.3066, 16.5581, 22.1097, 28.3591, 31.0302, 329.349, 31.9742, 31.4596, 30.1312, 29.8875, 29.8009];
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.311648, 0.251229, 0.173551, 0.0712933, 0.0201261, 0.00332936, 0.000383926, 8.49837e-05, 7.19862e-05, 2.5995e-05, 5.59893e-05];
num_bits = [160768, 199424, 288256, 701440, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [50103, 50101, 50027, 50008, 20130, 3330, 384, 85, 72, 26, 56];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [13.2638, 16.5083, 22.3343, 26.8008, 30.058, 30.6465, 31.1822, 30.4671, 32.3535, 29.2406, 30.2176];
%composite
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.311648, 0.251229, 0.173551, 0.0712933, 0.0201261, 0.00332936, 0.000383926, 8.49837e-05, 7.19862e-05, 4.49914e-05, 5.59893e-05];
num_bits = [160768, 199424, 288256, 701440, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [50103, 50101, 50027, 50008, 20130, 3330, 384, 85, 72, 26, 56];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [13.2638, 16.5083, 22.3343, 26.8008, 30.058, 30.6465, 31.1822, 30.4671, 32.3535, 29.2406, 30.2176];
semilogy(ax(1),EsNidb,ber,'g-o','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless OSLA");

% EsN0db = [4];
% EsNidb = [10, 15, 20, 25, 30, 35];
% ber = [0.00234355, 0.000302942, 3.89925e-05, 2.29956e-05, 1.69967e-05, 3.29937e-05];
% num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
% num_errs = [2344, 303, 39, 23, 17, 33];
% rss_dbms = [-126.711, -126.726, -126.727, -126.706, -126.739, -126.701];
% avg_sym_len = [28.7396, 31.0443, 30.8702, 30.9558, 29.8519, 32.0503];
% semilogy(ax(1),EsNidb,ber,'g-o','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless OSLA2");
% 
% EsN0db = [4];
% EsNidb = [20, 25, 30, 35];
% ber = [0.000145972, 0.00235855, 4.49914e-05, 5.99885e-05];
% num_bits = [1000192, 1000192, 1000192, 1000192];
% num_errs = [146, 2359, 45, 60];
% rss_dbms = [-127.062, -127.062, -127.062, -127.062];
% avg_sym_len = [32.7087, 250.961, 30.6687, 31.6222];
% semilogy(ax(1),EsNidb,ber,'g-o','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless OSLA3");

%Forward/Backward model: intf_mitigation_approx / noiseless_feedback
EsN0_fwd_db = 4;
ber = [0.00011199  0.00014597  0.00020236  0.00020774  0.00020534  0.00015813  0.00012558  0.00010334  9.5998e-05  0.00010892  0.00010191];
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
Tavg = [50.6263      50.6013      50.4741      50.1844      47.9401       40.835      34.4401      32.6751      32.1277      32.1455      31.9695];
% sim details: n_bit_err = [300  300  300  300  300  300  300  300  288  300  300];
% max_iters: max_n_iters = 11718.75;
semilogy(ax(1),EsNidb,ber,'b--x','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Simulated OSLA-FIM (Algorithm 1)");

% composite
EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [0.000113978, 0.000127841, 9.8981e-05, 0.000145972, 0.000108979, 0.000109562, 0.000109107, 0.000114623, 9.49818e-05, 0.000136974, 9.49818e-05];
num_bits = [1000192, 793600, 1000192, 1000192, 1000192, 921856, 925696, 881152, 1000192, 1000192, 1000192];
num_errs = [114, 100, 99, 146, 109, 101, 101, 101, 94, 137, 95];
rss_dbms = [-126.735, -126.676, -126.734, -126.775, -126.699, -126.709, -126.753, -126.726, -126.792, -126.747, -126.719];
avg_sym_len = [33.225, 33.4077, 32.5496, 34.4836, 34.3185, 36.6381, 38.9209, 41.5495, 43.6966, 47.8659, 48.5467];
EsN0db = [4];
EsNidb = [10, 15, 20, 25, 30, 35];
ber = [0.000100726, 0.000111893, 4.9011e-05, 3.71986e-05, 2.45991e-05, 7.37859e-05];
num_bits = [1995520, 1796352, 4101120, 5000192, 5000192, 2724096];
num_errs = [201, 201, 201, 186, 123, 201];
rss_dbms = [-126.714, -126.754, -126.703, -126.749, -126.68, -126.686];
avg_sym_len = [37.5061, 34.6075, 30.8675, 30.5444, 29.5171, 31.5895];

EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [8.69833e-05, 7.99846e-05, 0.000126976, 0.00010198, 8.49837e-05, 8.99827e-05, 4.59912e-05, 4.39916e-05, 5.39896e-05, 5.69891e-05, 3.69929e-05];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [87, 80, 127, 102, 85, 90, 46, 44, 54, 57, 37];
rss_dbms = [-126.74, -126.674, -126.714, -126.682, -126.688, -126.7, -126.694, -126.683, -126.684, -126.667, -126.654];
avg_sym_len = [50.7491, 47.1957, 48.1458, 46.0392, 42.8082, 37.8305, 32.5966, 31.0021, 30.4871, 30.9802, 29.8224];
%noise measure = 7k
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [8.09845e-05, 8.59835e-05, 8.09845e-05, 0.000110979, 6.89868e-05, 8.89829e-05, 3.0994e-05, 3.99923e-05, 5.29898e-05, 8.79831e-05, 8.79831e-05];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [81, 86, 81, 111, 69, 89, 31, 40, 53, 88, 88];
rss_dbms = [-126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827];
avg_sym_len = [49.8008, 49.3363, 47.8221, 47.1594, 42.8138, 39.0896, 32.8656, 31.7336, 31.2104, 33.2063, 32.7635];
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [5.89887e-05, 9.29821e-05, 0.00010198, 0.000106979, 6.79869e-05, 8.29841e-05, 3.69929e-05, 5.49894e-05, 4.99904e-05, 8.39839e-05, 9.49818e-05];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [59, 93, 102, 107, 68, 83, 37, 55, 50, 84, 95];
rss_dbms = [-126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827, -126.827];
avg_sym_len = [48.8789, 48.7814, 48.198, 47.5848, 42.6663, 38.1414, 33.291, 32.2915, 30.7634, 32.2777, 32.5024];

EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [9.99808e-05, 0.00010298, 7.89848e-05, 5.09902e-05, 5.09902e-05, 9.49818e-05, 7.09864e-05, 7.2986e-05, 8.19843e-05, 9.79812e-05, 2.29956e-05];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [100, 103, 79, 51, 51, 95, 71, 73, 82, 98, 23];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [53.8131, 53.0678, 50.1133, 44.4372, 38.7204, 36.7292, 34.6554, 34.0068, 33.5557, 35.4936, 32.6473];
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [0.00010598, 0.00010198, 0.00010498, 0.000126976, 6.49875e-05, 4.99904e-05, 6.89868e-05, 7.2986e-05, 4.6991e-05, 4.19919e-05, 0.00802302];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 42752];
num_errs = [106, 102, 105, 127, 65, 50, 69, 73, 47, 42, 343];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [54.0508, 52.4949, 51.0044, 46.7556, 39.086, 35.2015, 34.5514, 33.9471, 32.7583, 32.9012, 925.984];
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [6.79869e-05, 0.000235955, 0.000106979, 0.000131975, 0.00424736, 7.49856e-05, 5.29898e-05, 8.79831e-05, 6.59873e-05, 0.0121985, 5.69891e-05];
num_bits = [1000192, 1000192, 1000192, 1000192, 90880, 1000192, 1000192, 1000192, 1000192, 29184, 1000192];
num_errs = [68, 236, 107, 132, 386, 75, 53, 88, 66, 356, 57];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [53.0389, 57.1192, 51.6539, 52.8774, 386.769, 41.3682, 36.0677, 44.1654, 34.0374, 988.196, 33.0634];

EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [8.19843e-05, 8.09845e-05, 7.59854e-05, 9.69814e-05, 0.000151971, 5.39896e-05, 5.59893e-05, 5.199e-05, 0.00164052, 8.69833e-05, 0.000132974];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 187136, 1000192, 1000192];
num_errs = [82, 81, 76, 97, 152, 54, 56, 52, 307, 87, 133];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [51.442, 50.6732, 48.3776, 44.1615, 39.9088, 34.4763, 33.4209, 32.735, 185.564, 34.2057, 35.1165];
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [8.49837e-05, 0.000166968, 7.39858e-05, 0.000211959, 8.49837e-05, 0.000106979, 3.79927e-05, 3.59931e-05, 6.49875e-05, 6.99866e-05, 9.79812e-05];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [85, 167, 74, 212, 85, 107, 38, 36, 65, 70, 98];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [51.575, 53.8057, 48.0945, 54.4233, 38.9749, 35.6289, 32.6292, 32.859, 33.4167, 33.1253, 34.2473];

%composite
EsN0db = [4];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [8.49837e-05, 0.000166968, 0.000149971, 0.000211959, 0.000151971, 0.000106979, 9.8981e-05, 7.49856e-05, 6.49875e-05, 6.99866e-05, 9.79812e-05];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [85, 167, 74, 212, 85, 107, 38, 36, 65, 70, 98];
rss_dbms = [-127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062, -127.062];
avg_sym_len = [51.575, 53.8057, 48.0945, 54.4233, 38.9749, 35.6289, 32.6292, 32.859, 33.4167, 33.1253, 34.2473];
semilogy(ax(1),EsNidb,ber,'b-o','LineWidth', 1.5,'MarkerSize', 6.5,'DisplayName',"Wireless OSLA-FIM (Algorithm 1)");


% OSLA FIM
EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [0.000185924, 0.000149898, 0.000160902, 0.000193208, 0.000203157, 0.000191986, 0.000236105, 0.000257864, 0.000315373, 0.00023072, 0.000281205];
num_bits = [543232, 673792, 627712, 522752, 497152, 526080, 427776, 391680, 320256, 437760, 359168];
num_errs = [101, 101, 101, 101, 101, 101, 101, 101, 101, 101, 101];
rss_dbms = [-126.8, -126.845, -126.808, -126.819, -126.882, -126.837, -126.784, -126.833, -126.817, -126.867, -126.837];
avg_sym_len = [33.7894, 33.5612, 33.9707, 34.6526, 36.7421, 38.3375, 40.69, 45.3516, 48.5361, 50.5197, 53.213];
% semilogy(EsNidb,ber,'-o','DisplayName',"Wireless OSLA-FIM1");

EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [0.000123976, 0.00025795, 0.000188964, 0.000184964, 0.000232955, 0.000238954, 0.000332936, 0.000237954, 0.000705864, 0.000219958, 0.00015397];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 1000192];
num_errs = [124, 258, 189, 185, 233, 239, 333, 238, 706, 220, 154];
rss_dbms = [-126.732, -126.719, -126.732, -126.711, -126.775, -126.786, -126.733, -126.776, -126.757, -126.709, -126.751];
avg_sym_len = [32.9239, 35.0033, 34.5831, 34.3463, 36.9072, 39.2078, 42.4472, 44.9576, 48.8902, 49.7927, 50.4871];
% semilogy(EsNidb,ber,'-o','DisplayName',"Wireless OSLA-FIM2");

EsN0_dbs = [4];
EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
ber = [0.000113978, 0.000162969, 0.00020596, 0.000145972, 0.000108979, 9.29821e-05, 0.00181474, 0.000380927, 0.000192963, 0.000136974, 0.000192963];
num_bits = [1000192, 1000192, 1000192, 1000192, 1000192, 1000192, 317952, 1000192, 1000192, 1000192, 1000192];
num_errs = [114, 163, 206, 146, 109, 93, 577, 381, 193, 137, 193];
rss_dbms = [-126.735, -126.676, -126.734, -126.775, -126.699, -126.709, -126.753, -126.726, -126.792, -126.747, -126.719];
avg_sym_len = [33.225, 33.4961, 34.7846, 34.4836, 34.3185, 35.5193, 39.4974, 46.5727, 47.3997, 47.8659, 50.7014];
% semilogy(EsNidb,ber,'-o','DisplayName',"Wireless OSLA-FIM3");


EsN0_dbs = [4];
EsNidb = [25, 25, 25, 10, 10, 10, 5, 5, 5, 0, 0, 0, -5, -5, -5, -20, -20, -20];
ber = [0.000127639, 9.8981e-05, 7.39858e-05, 0.00013226, 0.000109562, 0.000143258, 0.000149898, 0.000109107, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
num_bits = [791296, 1000192, 1000192, 763648, 921856, 705024, 673792, 925696, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
num_errs = [101, 99, 74, 101, 101, 101, 101, 101, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
rss_dbms = [-126.788, -126.717, -126.759, -126.721, -126.753, -126.767, -126.741, -126.696, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
avg_sym_len = [33.4926, 32.5496, 32.446, 36.7581, 36.6381, 37.0266, 40.191, 38.9209, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

EsN0_dbs = [4];
EsNidb = [20, 20, 20, 0, 0, 0, -5, -5, -5, -15, -15, -15];
ber = [8.19843e-05, 5.39896e-05, 7.69852e-05, 0.000114623, 0.000282616, 9.19823e-05, 0.000171834, 0.000219917, 9.3982e-05, 9.49818e-05, 0.000210192, 0.000209634];
num_bits = [1000192, 1000192, 1000192, 881152, 357376, 1000192, 587776, 459264, 1000192, 1000192, 480512, 481792];
num_errs = [82, 54, 77, 101, 101, 92, 101, 101, 94, 95, 101, 101];
rss_dbms = [-126.671, -126.691, -126.785, -126.708, -126.696, -126.709, -126.753, -126.722, -126.764, -126.731, -126.724, -126.771];
avg_sym_len = [32.5317, 31.4039, 32.9612, 41.5495, 45.419, 40.9123, 46.958, 47.2145, 43.6966, 48.5467, 51.4873, 53.2725];
% semilogy(EsNidb,ber,'-o','DisplayName',"Wireless OSLA-FIM1");


% semilogy(EsN0_dbs,ber,'-o','DisplayName',"OSLA Wireless 2.1GHz, 2m,30db fb");

% %Forward/Backward model: bpsk_fwd_fp_interf / noiseless_feedback
% EsN0_fwd_db = 4;
% ber = [0.14375     0.13516      0.1204    0.088778    0.060961     0.03156    0.019301    0.013686    0.012784     0.01388     0.01244];
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% Tavg = [32  32  32  32  32  32  32  32  32  32  32];
% % sim details: n_bit_err = [552  519  524  500  515  509  504  501  504  501  500];
% % max_iters: max_n_iters = 3906.25;
% semilogy(EsNidb,ber,'-o','DisplayName',"BPSK");


% %Forward/Backward model: clairvoyant_fwd / noiseless_feedback
% EsN0_fwd_db = 4;
% ber = [5.7989e-05  7.2986e-05  6.4988e-05  6.5987e-05  7.2986e-05  7.5985e-05  6.7987e-05  5.6989e-05  5.9988e-05  7.1986e-05  6.0988e-05];
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% Tavg = [48.7973      48.4394      47.8798       46.167      43.2562      38.5557      35.1981      33.5293      32.3169      32.1794      32.0787];
% % sim details: n_bit_err = [58  73  65  66  73  76  68  57  60  72  61];
% % max_iters: max_n_iters = 3906.25;
% semilogy(EsNidb,ber,'-o','DisplayName',"OSLA-FIM (perfect CSI)"); 
% 
% %Forward/Backward model: fwd_fp_chip_hyp_test_comp_ignore_interf_nb / noiseless_feedback
% EsN0_fwd_db = 4;
% ber = [9.8981e-05  0.00011598  0.00012798  0.00013897  0.00013897  0.00013197  0.00011998  9.5982e-05  0.00010298  9.6981e-05  0.00011798];
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% Tavg = [50.4385      50.4602      49.8357       50.637      50.1137      47.4612      40.4767       34.434      32.6834        32.16      32.1475];
% % sim details: n_bit_err = [99  116  128  139  139  132  120   96  103   97  118];
% % max_iters: max_n_iters = 3906.25;
% % semilogy(EsNidb,ber,'-o','DisplayName',"Hyp Test M = 96, Estim M = 96, comp, no mean, find_L, Pf = 0.001"); 
% semilogy(EsNidb,ber,'-o','DisplayName',"OSLA-FIM (estimated CSI)"); 

xlabel("Es/Ni (dB)", 'FontSize', fontsize);
ylabel("BER", 'FontSize', fontsize)
ylim([1e-6 1]);
grid(ax(1), 'on');
legend(ax(1), 'Location', 'southwest','Fontsize', fontsize);

% Setup top axis
pos = ax(1).Position;
ax(2) = axes(fig, ...
    'Position', [pos(1), pos(2) + 0.0, pos(3), pos(4)], ...
    'Color', 'none', ...
    'XAxisLocation', 'top', ...
    'YAxisLocation', 'right', ...
    'XColor', 'k', ...
    'YColor', 'none', ...
    'Box', 'off');

% Sync limits and scales
ax(2).XLim = ax(1).XLim;
ax(2).YLim = ax(1).YLim;
ax(2).XScale = 'linear';
ax(2).YScale = 'log';
linkaxes(ax, 'xy');
ax(2).YTick = [];

% Set top labels (for first dataset only)
% ax(2).XTick = EsNidb_axis;
% ax(2).XTickLabel = arrayfun(@(x) sprintf('%.1f', x), interf_rss_dbm, 'UniformOutput', false);

% Choose every other tick index
tick_idx = 1:2:length(EsNidb_axis);
% Set ticks and labels at those positions
ax(2).XTick = EsNidb_axis(tick_idx);
ax(2).XTickLabel = arrayfun(@(x) sprintf('%.1f', x), interf_rss_dbm(tick_idx), 'UniformOutput', false);
xlabel(ax(2), 'Average Received Interference Power (dBm)', 'FontSize', fontsize);

set(ax(1), 'YScale', 'log');  % Reapply semilog to main axis
% title("OSLA with BLE Interference, EsN0 = 4dB")
% Set font size for both axes
set(ax(1), 'FontSize', fontsize);  % or any size you prefer
set(ax(2), 'FontSize', fontsize);


% create file------------------
filename = 'ble_intf_ber';
set(gcf, 'Units', 'normalized');
set(gcf, 'Position', [0.1, 0.1, 0.53, 0.73]);  % Make the figure window larger
set(gcf, 'PaperPositionMode', 'auto');  % Ensure full content fits
% Build full paths
png_path = fullfile(folder, [filename, '.png']);
svg_path = fullfile(folder, [filename, '.svg']);
% Export the figure
print(gcf, png_path, '-dpng', '-r600');         % High-res PNG
% exportgraphics(gcf, svg_path, 'ContentType', 'vector');  % Vector SVG% Build full paths
print(gcf, svg_path, '-dsvg');  % Export to SVG



% Rate-------------------------------------------------------------
figure;
N = 32; %number of chips per symbol

%4dB-5ms mu----------------------------------------
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% TavgClairvoyant = [48.7973      48.4394      47.8798       46.167      43.2562      38.5557      35.1981      33.5293      32.3169      32.1794      32.0787];
% figure;
% plot(EsNidb, N./TavgClairvoyant, 'm--x', 'LineWidth', 1.5, 'MarkerSize', 6.5, 'DisplayName', 'Simulated OSLA-FIM (Perfect CSI)');
% hold on;
% 
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% TavgInterMitigation = [50.7537      50.2384      50.1158      50.1054      49.7091      46.4529      39.3266      34.1723      32.5293      31.9311      32.1081];
% plot(EsNidb, N./TavgInterMitigation, 'b--x', 'LineWidth', 1.5, 'MarkerSize', 6.5, 'DisplayName', 'Simulated OSLA-FIM (Algorithm 1)');
% 
% 

% EsNidb = fliplr([-15 -10  -5   0   5  10  15  20  25  30  35]);
% avg_sym_len = [32.9239, 33.4961, 33.9707, 34.4836, 36.7421, 39.2078, 39.2078, 46.5727, 47.2145, 50.5197, 50.7014];
% plot(EsNidb, N./avg_sym_len, '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation BLE0');
% 
% EsN0_dbs = [5];
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% ber = [0, 0, 0, 0, 0, 0, 0, 9.76563e-05, 0.000292969, 0, 0];
% num_bits = [10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240];
% num_errs = [0, 0, 0, 0, 0, 0, 0, 1, 3, 0, 0];
% rss_dbms = [-125.74, -125.737, -125.718, -125.718, -125.741, -125.787, -125.725, -125.718, -125.691, -125.751, -125.742];
% avg_sym_len = [32.1755, 32.1749, 31.633, 34.2183, 31.6868, 33.1932, 33.3237, 35.5743, 40.5968, 44.7532, 48.9692];
% plot(EsNidb, N./avg_sym_len, '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation BLE1');
% 
% EsN0_dbs = [5];
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% ber = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
% num_bits = [10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240];
% num_errs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
% rss_dbms = [-125.68, -125.754, -125.75, -125.711, -125.702, -125.734, -125.683, -125.705, -125.713, -125.719, -125.646];
% avg_sym_len = [31.4567, 32.9992, 31.9643, 30.5337, 31.9971, 33.8497, 34.3949, 35.5664, 38.9333, 43.4503, 45.3055];
% plot(EsNidb, N./avg_sym_len, '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation BLE2');
% 
% EsN0_dbs = [5];
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% ber = [0, 0, 0, 0, 0, 9.76563e-05, 0, 0, 9.76563e-05, 0, 0];
% num_bits = [10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240];
% num_errs = [0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0];
% rss_dbms = [-125.756, -125.685, -125.702, -125.74, -125.665, -125.72, -125.689, -125.74, -125.712, -125.677, -125.745];
% avg_sym_len = [32.1677, 31.4329, 31.7622, 32.1262, 30.9653, 33.4854, 35.4975, 36.16, 38.5549, 42.5076, 46.1125];
% plot(EsNidb, N./avg_sym_len, '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation BLE3');
% 
% EsN0_dbs = [5];
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% ber = [0, 0, 0, 0, 0, 0, 0, 0, 0, 9.76563e-05, 9.76563e-05];
% num_bits = [10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240];
% num_errs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1];
% rss_dbms = [-125.721, -125.699, -125.724, -125.724, -125.696, -125.709, -125.742, -125.697, -125.703, -125.712, -125.733];
% avg_sym_len = [32.5182, 32.0497, 32.2377, 32.5719, 32.5707, 34.058, 35.4143, 36.0479, 40.5104, 41.5834, 46.3158];
% plot(EsNidb, N./avg_sym_len, '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation BLE4');
% 
% 
% 
% EsN0_dbs = [5];
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% ber = [0, 0, 0, 0, 0, 0, 9.76563e-05, 0, 9.76563e-05, 0, 0];
% num_bits = [10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240];
% num_errs = [0, 0, 0, 0, 0, 0, 1, 0, 1, 0, 0];
% rss_dbms = [-125.727, -125.731, -125.74, -125.678, -125.771, -125.721, -125.674, -125.699, -125.734, -125.701, -125.734];
% avg_sym_len = [31.9665, 31.6287, 32.8807, 32.3097, 32.6049, 34.107, 34.8068, 36.6653, 41.0007, 43.3739, 45.2297];
% plot(EsNidb, N./avg_sym_len, '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation BLE5');
% 
% EsN0_dbs = [5];
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% ber = [0, 0, 0, 0, 0, 0, 0, 0, 0, 9.76563e-05, 0];
% num_bits = [10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240];
% num_errs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0];
% rss_dbms = [-125.723, -125.728, -125.741, -125.705, -125.703, -125.709, -125.754, -125.754, -125.763, -125.748, -125.675];
% avg_sym_len = [31.5604, 32.4564, 32.594, 31.3241, 32.4813, 32.8265, 34.6967, 38.1684, 39.839, 44.1388, 43.7602];
% plot(EsNidb, N./avg_sym_len, '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation BLE6');
% 
% 
% EsN0_dbs = [5];
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% ber = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
% num_bits = [10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240, 10240];
% num_errs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
% rss_dbms = [-125.702, -125.709, -125.749, -125.729, -125.758, -125.702, -125.696, -125.68, -125.72, -125.726, -125.675];
% avg_sym_len = [32.4045, 30.7603, 30.381, 31.5358, 33.6574, 32.4845, 34.3103, 35.7418, 39.1497, 41.1502, 44.4526];
% plot(EsNidb, N./avg_sym_len, '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation BLE');
% 
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% avg_sym_len = [32.9239, 33.4961, 33.9707, 34.4836, 36.7421, 39.2078, 39.2078, 46.5727, 47.2145, 50.5197, 50.7014] ...
% + [32.1755, 32.1749, 31.633, 34.2183, 31.6868, 33.1932, 33.3237, 35.5743, 40.5968, 44.7532, 48.9692]...
% +[31.4567, 32.9992, 31.9643, 30.5337, 31.9971, 33.8497, 34.3949, 35.5664, 38.9333, 43.4503, 45.3055]...
% +[32.1735, 32.6371, 32.27, 31.8829, 34.4325, 38.2922, 40.4925, 42.6898, 47.1519, 46.5604, 44.3451]...
% +[32.5182, 32.0497, 32.2377, 32.5719, 32.5707, 34.058, 35.4143, 36.0479, 40.5104, 41.5834, 46.3158]...
% +[31.9665, 31.6287, 32.8807, 32.3097, 32.6049, 34.107, 34.8068, 36.6653, 41.0007, 43.3739, 45.2297]...
% +[31.5604, 32.4564, 32.594, 31.3241, 32.4813, 32.8265, 34.6967, 38.1684, 39.839, 44.1388, 43.7602]...
% +[32.4045, 30.7603, 30.381, 31.5358, 33.6574, 32.4845, 34.3103, 35.7418, 39.1497, 41.1502, 44.4526];
% plot(EsNidb, 8*N./avg_sym_len, 'b-o', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Wireless OSLA-FIM (Algorithm 1)');



% Tavg = [1504.1969       1190.328      860.34286      485.53239      253.92744      145.69132       96.00819       77.98925      71.699002      70.142821      69.479396];
% plot(EsNidb, N./Tavg, '-o', 'LineWidth', 2, 'MarkerSize', 6.5, 'Color', 'g', 'DisplayName', 'ARQ, CRC-9, HD = 3');

% %Forward/Backward model: bpsk_fwd_fp_interf / noiseless_feedback
% EsN0_fwd_db = 4;
% ber = [0.33919     0.26146     0.17862    0.087381    0.022584   0.0024484  0.00026695  7.9985e-05  6.1988e-05  7.8985e-05  6.4988e-05];
% EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
% Tavg = [11.4202      16.8896      21.3833      27.5163      31.1314       32.091      32.0842      31.9697      32.0879      31.7945      32.1011];
% % sim details: n_bit_err = [1042  1004  1006  1029  1006  1001   267    80    62    79    65];
% % max_iters: max_n_iters = 3906.25;
% plot(EsNidb, N./Tavg, 'g--x', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Simulated OSLA');
% hold on;

%Forward/Backward model: intf_mitigation_approx / noiseless_feedback
EsN0_fwd_db = 4;
ber = [0.00011199  0.00014597  0.00020236  0.00020774  0.00020534  0.00015813  0.00012558  0.00010334  9.5998e-05  0.00010892  0.00010191];
EsNidb = [-15 -10  -5   0   5  10  15  20  25  30  35];
Tavg = [50.6263      50.6013      50.4741      50.1844      47.9401       40.835      34.4401      32.6751      32.1277      32.1455      31.9695];
% sim details: n_bit_err = [300  300  300  300  300  300  300  300  288  300  300];
% max_iters: max_n_iters = 11718.75;
plot(EsNidb, N./Tavg, 'b--x', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Simulated OSLA-FIM (Algorithm 1) (BLE Interference)');
hold on

% Add labels and title
ylim([0 1.1]);
xlabel("EsNi (dB)", 'FontSize', fontsize);
ylabel('Normalized Data Rate', 'FontSize', fontsize);
% title('BLE, EsN0 = 4dB, mu = 5ms')
legend('Location', 'best','Fontsize', fontsize);
grid on;
% Set axis font sizes
ax = gca;
set(ax, 'FontSize', fontsize);  % or any value you prefer

% create file------------------
filename = 'ble_intf_rate';
set(gcf, 'Units', 'normalized');
set(gcf, 'Position', [0.1, 0.1, 0.53, 0.73]);  % Make the figure window larger
set(gcf, 'PaperPositionMode', 'auto');  % Ensure full content fits
% Build full paths
png_path = fullfile(folder, [filename, '.png']);
svg_path = fullfile(folder, [filename, '.svg']);
% Export the figure
print(gcf, png_path, '-dpng', '-r600');         % High-res PNG
% exportgraphics(gcf, svg_path, 'ContentType', 'vector');  % Vector SVG% Build full paths
print(gcf, svg_path, '-dsvg');  % Export to SVG



% % Goodput----------------------------------------------------------------------------------------------------
% EsN0_dbs = [4];
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% ber = [0.000113978, 0.000127841, 9.8981e-05, 0.000145972, 0.000108979, 0.000109562, 0.000109107, 0.000114623, 9.49818e-05, 0.000136974, 9.49818e-05];
% num_bits = [1000192, 793600, 1000192, 1000192, 1000192, 921856, 925696, 881152, 1000192, 1000192, 1000192];
% num_errs = [114, 100, 99, 146, 109, 101, 101, 101, 94, 137, 95];
% rss_dbms = [-126.735, -126.676, -126.734, -126.775, -126.699, -126.709, -126.753, -126.726, -126.792, -126.747, -126.719];
% avg_sym_len = [32.9239, 33.4961, 33.9707, 34.4836, 36.7421, 39.2078, 39.2078, 46.5727, 47.2145, 50.5197, 50.7014];
% figure;
% plot(EsNidb, N./avg_sym_len.*bsc_capacity(ber), '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Measured Interference Mitigation BLE');
% hold on;
% 
% 
% 
% EsN0_dbs = [4];
% EsNidb = [35, 30, 25, 20, 15, 10, 5, 0, -5, -10, -15];
% ber = [0.0174735, 0.0133086, 0.0176309, 0.016871, 0.021875, 0.0279576, 0.0350167, 0.0513672, 0.0734954, 0.0976562, 0.119256];
% num_bits = [28672, 30720, 28416, 29696, 23040, 17920, 14336, 10240, 6912, 5376, 4352];
% num_errs = [501, 501, 501, 501, 504, 501, 502, 526, 508, 525, 519];
% rss_dbms = [-126.83, -126.829, -126.827, -126.802, -126.784, -126.853, -126.802, -126.784, -126.817, -126.819, -126.87];
% avg_sym_len = [32, 32, 32, 32, 32, 32, 32, 32, 32, 32, 32];
% plot(EsNidb, N./avg_sym_len.*bsc_capacity(ber), '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Wireless BPSK');
% 
% EsN0_dbs = [4];
% EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
% ber = [0.303571, 0.189918, 0.145744, 0.104768, 0.035831, 0.0101139, 0.00429216, 0.00083084, 0.000273947, 7.7985e-05, 5.59893e-05];
% num_bits = [3584, 5376, 7424, 9984, 28160, 99072, 233216, 1000192, 1000192, 1000192, 1000192];
% num_errs = [1088, 1021, 1082, 1046, 1009, 1002, 1001, 831, 274, 78, 56];
% rss_dbms = [-126.777, -126.742, -126.743, -126.773, -126.749, -126.775, -126.74, -126.73, -126.772, -126.727, -126.736];
% avg_sym_len = [12.9515, 19.9048, 25.6294, 26.1013, 31.9205, 32.3101, 31.5851, 32.9931, 34.0688, 32.2594, 33.439];
% plot(EsNidb, N./avg_sym_len.*bsc_capacity(ber), '-s', 'LineWidth', 2, 'MarkerSize', 6.5,  'DisplayName', 'Wireless OSLA');
% 
% % 
% % % Tavg = [1504.1969       1190.328      860.34286      485.53239      253.92744      145.69132       96.00819       77.98925      71.699002      70.142821      69.479396];
% % % plot(EsNidb, N./Tavg, '-o', 'LineWidth', 2, 'MarkerSize', 6.5, 'Color', 'g', 'DisplayName', 'ARQ, CRC-9, HD = 3');
% 
% % Add labels and title
% xlabel("EsNi (dB)");
% ylabel('Normalized Binary Capacity,BLE');
% % title('EsN0 = 4dB, mu = 5ms')
% legend('Location', 'best');
% grid on;

%% 5db

EsN0_dbs = [5];
EsNidb = [-15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35];
ber = [3.24351e-05, 2.58335e-05, 2.64826e-05, 1.9682e-05, 0, 0, 0, 0, 0, 0, 0];
num_bits = [6196992, 7780608, 7589888, 10212352, 0, 0, 0, 0, 0, 0, 0];
num_errs = [201, 201, 201, 201, 0, 0, 0, 0, 0, 0, 0];
rss_dbms = [-125.745, -125.734, -125.729, -125.694, 0, 0, 0, 0, 0, 0, 0];
avg_sym_len = [50.427, 47.8844, 45.3769, 42.2729, 0, 0, 0, 0, 0, 0, 0];
%% threshold sweep-wireless
figure();
EsN0_dbs = [0, 1, 2, 3, 4, 5, 6];
ber = [0.0752704, 0.054579, 0.0330078, 0.0213995, 0.00997383, 0.00555506, 0.00196687];
num_bits = [6656, 9216, 15360, 23552, 50432, 90368, 254720];
num_errs = [501, 503, 507, 504, 503, 502, 501];
rss_dbms = [-111.794, -110.803, -109.928, -108.915, -107.811, -106.86, -105.806] - 10*log10(50)-1.31;
% ber = [0.0476751, 0.0303113, 0.0207987, 0.0101299, 0.00531271, 0.00164361, 0.000578854];
% num_bits = [21248, 33024, 48128, 98816, 188416, 609024, 1729280];
% num_errs = [1013, 1001, 1001, 1001, 1001, 1001, 1001];
% rss_dbms = [-129.748, -128.768, -127.783, -126.791, -125.85, -124.762, -123.852];
semilogy(rss_dbms,ber,'-o','LineWidth', 2, 'MarkerSize', 6.5,'DisplayName',"Wireless BPSK, 18.6kbps");
hold on
grid on
legend('Location', 'southwest','Fontsize', fontsize)

ber = [0.0209704, 0.0090554, 0.00272091, 0.000800266, 7.63855e-05, 9.39988e-06, 2.99996e-07];
num_bits = [4864, 11264, 37120, 126208, 1322240, 10000128, 10000128];
num_errs = [102, 102, 101, 101, 101, 94, 3];
rss_dbms = [-128.821, -127.91, -126.865, -125.978, -124.765, -123.807, -122.833];
% semilogy(rss_dbms,ber,'-o','DisplayName',"OSLA, Threshold = 7500");
semilogy(rss_dbms,ber,'-o','LineWidth', 2, 'MarkerSize', 6.5,'DisplayName',"Wireless OSLA, 24.8kbps");

ber = [0.0186435, 0.00935683, 0.0025956, 0.000620332, 8.67292e-05, 1.11905e-05, 6.99991e-07];
num_bits = [5632, 11008, 38912, 162816, 1164544, 9025536, 10000128];
num_errs = [105, 103, 101, 101, 101, 101, 7];
rss_dbms = [-130.107, -129.14, -128.191, -127.171, -126.126, -125.116, -124.081];
% semilogy(rss_dbms,ber,'-o','DisplayName',"OSLA, Threshold = 10000");
semilogy(rss_dbms,ber,'-o','LineWidth', 2, 'MarkerSize', 6.5,'DisplayName',"Wireless OSLA, 18.6kbps");

ber = [0.0232077, 0.00885417, 0.0028059, 0.000651042, 9.54126e-05, 1.65846e-05, 4.99994e-07];
num_bits = [4352, 11520, 36352, 155136, 1058560, 6089984, 10000128];
num_errs = [101, 102, 102, 101, 101, 101, 5];
rss_dbms = [-131.244, -130.158, -129.206, -128.152, -127.082, -126.127, -125.143];
% semilogy(rss_dbms,ber,'-o','DisplayName',"OSLA, Threshold = 12500");
semilogy(rss_dbms,ber,'-o','LineWidth', 2, 'MarkerSize', 6.5,'DisplayName',"Wireless OSLA, 14.9kbps");

ber = [0.0232077, 0.00805166, 0.00260417, 0.00059507, 9.5022e-05, 1.02652e-05, 1.99997e-07];
num_bits = [4352, 12544, 39168, 169728, 1062912, 9839104, 10000128];
num_errs = [101, 101, 102, 101, 101, 101, 2];
rss_dbms = [-131.876, -130.891, -129.89, -128.892, -127.864, -126.836, -125.838];
% semilogy(rss_dbms,ber,'-o','DisplayName',"OSLA, Threshold = 15000");
semilogy(rss_dbms,ber,'-o','LineWidth', 2, 'MarkerSize', 6.5,'DisplayName',"Wireless OSLA, 12.4kbps");

ber = [0.0213816, 0.00917515, 0.00295139, 0.000683763, 0.000122335, 9.09988e-06, 6.99991e-07];
num_bits = [4864, 11008, 34560, 147712, 825600, 10000128, 10000128];
num_errs = [104, 101, 102, 101, 101, 91, 7];
rss_dbms = [-133.189, -132.077, -131.103, -130.153, -129.202, -128.082, -127.068];
% semilogy(rss_dbms,ber,'-o','DisplayName',"OSLA, Threshold = 20000");
semilogy(rss_dbms,ber,'-o','LineWidth', 2, 'MarkerSize', 6.5,'DisplayName',"Wireless OSLA, 9.3kbps");

% Set axis font sizes
ax = gca;
set(ax, 'FontSize', fontsize);  % or any value you prefer

% Set axis label font sizes explicitly
xlabel("Received Signal Power (dBm)", 'FontSize', fontsize);
ylabel("Bit Error Rate", 'FontSize', fontsize);

legend('FontSize', fontsize);

% create file------------------
filename = 'threshold_ber';
set(gcf, 'Units', 'normalized');
set(gcf, 'Position', [0.1, 0.1, 0.53, 0.73]);  % Make the figure window larger
set(gcf, 'PaperPositionMode', 'auto');  % Ensure full content fits
% Build full paths
png_path = fullfile(folder, [filename, '.png']);
svg_path = fullfile(folder, [filename, '.svg']);
% Export the figure
print(gcf, png_path, '-dpng', '-r600');         % High-res PNG
% exportgraphics(gcf, svg_path, 'ContentType', 'vector');  % Vector SVG% Build full paths
print(gcf, svg_path, '-dsvg');  % Export to SVG


%% Symbol length histogram

bin_size = 2;
figure()
EsN0s = [];
mean_lengths = [];

sym_lens = [0, 0, 0, 0, 0, 9, 40, 71, 129, 176, 236, 275, 332, 383, 419, 395, 399, 418, 384, 390, 387, 382, 376, 373, 345, 328, 319, 291, 297, 291, 282, 241, 239, 259, 254, 249, 196, 177, 171, 180, 156, 158, 155, 143, 150, 122, 109, 94, 129, 111, 94, 81, 77, 79, 88, 69, 55, 65, 59, 62, 44, 48, 53, 49, 48, 69, 48, 40, 41, 38, 38, 36, 30, 28, 23, 18, 21, 18, 22, 14, 14, 25, 17, 12, 18, 8, 13, 5, 21, 8, 8, 3, 6, 6, 9, 152];
EsN0s = [EsN0s, 0];
mean_lengths = [mean_lengths, sum(sym_lens/(sum(sym_lens)).*(1:length(sym_lens)))];
[n, sym_lens] = downsample_and_prepare_plot(sym_lens, bin_size);
sym_lens = sym_lens/sum(sym_lens);
plot(n,sym_lens, '--o','LineWidth', 2, 'MarkerSize', 6.5, DisplayName="0dB Es/N0")
% plot(n,sym_lens, '--o', DisplayName="0dB, threshold = 10000")
hold on

sym_lens = [0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 10, 25, 53, 68, 132, 172, 213, 276, 354, 355, 405, 435, 481, 515, 515, 486, 527, 510, 492, 548, 496, 464, 440, 402, 371, 354, 302, 281, 291, 274, 259, 213, 183, 173, 159, 148, 164, 128, 114, 114, 113, 80, 71, 59, 60, 51, 45, 30, 38, 38, 33, 33, 32, 20, 19, 16, 15, 13, 17, 10, 8, 8, 6, 12, 9, 5, 8, 7, 3, 4, 4, 6, 4, 3, 1, 4, 1, 3, 1, 2, 1, 0, 0, 1, 1, 2];
EsN0s = [EsN0s, 5];
mean_lengths = [mean_lengths, sum(sym_lens/(sum(sym_lens)).*(1:length(sym_lens)))];
[n, sym_lens] = downsample_and_prepare_plot(sym_lens, bin_size);
sym_lens = sym_lens/sum(sym_lens);
plot(n,sym_lens, '--o','LineWidth', 2, 'MarkerSize', 6.5, DisplayName="5dB Es/N0")
% plot(n,sym_lens, '--o', DisplayName="5dB, threshold = 10000")

sym_lens = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2, 6, 17, 33, 57, 129, 220, 308, 384, 488, 585, 702, 772, 860, 865, 865, 871, 801, 691, 615, 607, 457, 418, 351, 319, 237, 216, 199, 183, 111, 97, 76, 60, 41, 29, 34, 20, 22, 11, 9, 8, 5, 5, 3, 2, 1, 4, 1, 0, 0, 0, 0, 2, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
EsN0s = [EsN0s, 10];
mean_lengths = [mean_lengths, sum(sym_lens/(sum(sym_lens)).*(1:length(sym_lens)))];
[n, sym_lens] = downsample_and_prepare_plot(sym_lens, bin_size);
sym_lens = sym_lens/sum(sym_lens);
plot(n,sym_lens, '--o','LineWidth', 2, 'MarkerSize', 6.5, DisplayName="10dB Es/N0")
% plot(n,sym_lens, '--o', DisplayName="10dB, threshold = 10000")

% Set axis font sizes
ax = gca;
set(ax, 'FontSize', fontsize);  % or any value you prefer

% Set axis label font sizes explicitly
xlabel("Number of chips/symbol", 'FontSize', fontsize);
ylabel("Frequency", 'FontSize', fontsize);

% Already done: legend font size
legend('FontSize', fontsize);

display(EsN0s)
display(mean_lengths)

% create file------------------
filename = 'sym_len';
set(gcf, 'Units', 'normalized');
set(gcf, 'Position', [0.1, 0.1, 0.53, 0.73]);  % Make the figure window larger
set(gcf, 'PaperPositionMode', 'auto');  % Ensure full content fits
% Build full paths
png_path = fullfile(folder, [filename, '.png']);
svg_path = fullfile(folder, [filename, '.svg']);
% Export the figure
print(gcf, png_path, '-dpng', '-r600');         % High-res PNG
% exportgraphics(gcf, svg_path, 'ContentType', 'vector');  % Vector SVG% Build full paths
print(gcf, svg_path, '-dsvg');  % Export to SVG


%% Run combining

% Original data
EsN0_dbs = [6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7, 6, 7];
ber = [0.00567109, 0.00360936, 0.00445348, 0.501953, 0.00390253, 0.0115426, 0.504883, 0.00853588, 0.509766, 0.00576483, 0.0108271, 0.00468844, 0.491699, 0.49707];
num_bits = [177920, 299776, 224768, 2048, 268544, 96512, 2048, 117504, 2048, 185088, 94208, 213504, 2048, 2048];
num_errs = [1009, 1082, 1001, 1028, 1048, 1114, 1034, 1003, 1044, 1067, 1020, 1001, 1007, 1018];
rss_dbms = [-122.886, -120.569, -120.253, -121.632, -121.491, -120.724, -121.685, -121.8, -122.787, -120.514, -122.805, -121.752, -121.6, -120.75];
avg_sym_len = [32.349, 32.2878, 31.91, 44045.5, 32.0219, 733.947, 44511.6, 692.474, 44281.2, 187.656, 31.9172, 32.1106, 44445.6, 44684.8];

% Filter mask
valid_mask = avg_sym_len <= 40;

% Filtered data
EsN0_valid = EsN0_dbs(valid_mask);
ber_valid = ber(valid_mask);
num_bits_valid = num_bits(valid_mask);
num_errs_valid = num_errs(valid_mask);
rss_dbms_valid = rss_dbms(valid_mask);
avg_sym_len_valid = avg_sym_len(valid_mask);

% Group by EsN0 = 6 and 7
group_6_mask = EsN0_valid == 6;
group_7_mask = EsN0_valid == 7;

% Combine values for EsN0 = 6
data_6.ber = ber_valid(group_6_mask);
data_6.num_bits = num_bits_valid(group_6_mask);
data_6.num_errs = num_errs_valid(group_6_mask);
data_6.rss_dbms = rss_dbms_valid(group_6_mask);
data_6.avg_sym_len = avg_sym_len_valid(group_6_mask);

% Combine values for EsN0 = 7
data_7.ber = ber_valid(group_7_mask);
data_7.num_bits = num_bits_valid(group_7_mask);
data_7.num_errs = num_errs_valid(group_7_mask);
data_7.rss_dbms = rss_dbms_valid(group_7_mask);
data_7.avg_sym_len = avg_sym_len_valid(group_7_mask);

% Average and sum for EsN0 = 6
stats_6.avg_ber = mean(data_6.ber);
stats_6.total_bits = sum(data_6.num_bits);
stats_6.total_errs = sum(data_6.num_errs);
stats_6.avg_rss_dbm = mean(data_6.rss_dbms);
stats_6.avg_sym_len = mean(data_6.avg_sym_len);

% Average and sum for EsN0 = 7
stats_7.avg_ber = mean(data_7.ber);
stats_7.total_bits = sum(data_7.num_bits);
stats_7.total_errs = sum(data_7.num_errs);
stats_7.avg_rss_dbm = mean(data_7.rss_dbms);
stats_7.avg_sym_len = mean(data_7.avg_sym_len);

%% functions


function [x_downsampled, y_downsampled] = downsample_and_prepare_plot(sym_len, factor)
    % This function downsamples the sym_len vector by averaging neighboring points together
    % and prepares the x and y vectors for plotting.
    % Inputs:
    %   sym_len - vector indicating the counts of each number
    %   factor - the downsampling factor (how many points to average together)
    % Outputs:
    %   x_downsampled - x coordinates of the downsampled data
    %   y_downsampled - y coordinates of the downsampled data

    % Calculate the number of groups
    num_groups = ceil(length(sym_len) / factor);

    % Initialize the downsampled vector
    y_downsampled = zeros(1, num_groups);

    % Loop through each group to calculate the averaged value
    for i = 1:num_groups
        % Calculate the start and end indices for the current group
        start_idx = (i-1) * factor + 1;
        end_idx = min(i * factor, length(sym_len));
        
        % Compute the average of the current group
        y_downsampled(i) = mean(sym_len(start_idx:end_idx));
    end

    % Generate x vector for the downsampled data
    x_downsampled = linspace(1, length(sym_len), num_groups);
end

function c = bsc_capacity(ber)
    c = 1-(-(1-ber).*log2(1-ber)-ber.*log2(ber));
end