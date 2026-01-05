% --------- tx-gain = 25, lognormVar = 0 ---------
P_target_0 = [-101.23, -106.23, -111.23, -116.23, -121.23, -126.23, -131.23, ...
              -136.23, -141.23, -146.23, -151.23, -156.23];
P_measured_0 = [-101.14, -106.33, -111.32, -115.93, -120.36, -124.2, -126.55, ...
                -127.46, -127.88, -128.0, -128.4, -128.04];

% --------- tx-gain = 25, lognormVar = 5 ---------
P_target_5 = [-101.23, -106.23, -111.23, -116.23, -121.23, -131.23];
P_measured_5 = [-98.91, -103.87, -109.04, -114.06, -118.65, -125.23];

% --------- tx-gain = 30 ---------
P_target_30 = [-95, -100, -105, -110, -115, -120];
P_measured_30 = [-91.73, -96.31, -101.62, -106.63, -111.48, -116.24];

% --------- Plotting ---------
figure; hold on; grid on;

% Plot tx-gain = 25, lognormVar = 0
plot(P_target_0, P_measured_0, 'o-', 'LineWidth', 1.5, ...
    'DisplayName', 'tx-gain = 25, lognormVar = 0');

% Plot tx-gain = 25, lognormVar = 5
plot(P_target_5, P_measured_5, 's--', 'LineWidth', 1.5, ...
    'DisplayName', 'tx-gain = 25, lognormVar = 5');

% Plot tx-gain = 30
plot(P_target_30, P_measured_30, 'd-.', 'LineWidth', 1.5, ...
    'DisplayName', 'tx-gain = 30, lognormVar = 5');

% Ideal line (slope = 1)
all_targets = [P_target_0, P_target_5, P_target_30];
all_measured = [P_measured_0, P_measured_5, P_measured_30];
min_val = min([all_targets, all_measured]);
max_val = max([all_targets, all_measured]);
plot([min_val, max_val], [min_val, max_val], 'r:', 'LineWidth', 1.5, ...
    'DisplayName', 'ideal');
text(min_val + 2, min_val + 2, 'ideal', 'Color', 'r', 'FontSize', 10);

% Noise floor line
noise_floor = -128.03;
xlim_vals = [min(all_targets), max(all_targets)];
plot(xlim_vals, [noise_floor, noise_floor], 'k--', 'LineWidth', 1, ...
    'DisplayName', 'noise floor');
text(xlim_vals(2), noise_floor, '  \leftarrow noise floor', ...
    'VerticalAlignment', 'bottom', 'FontSize', 10);

% Labels and legend
xlabel('P_{target} (dBm)');
ylabel('P_{measured} (dBm)');
title('Measured vs Target Power for Different tx-gain and lognormVar');
legend('Location', 'SouthEast');


%%
% -------- BLE Calibration Data --------

% lognormVar = 0
P_target_ln0 = [-90, -95, -100, -105, -110, -115, -120];
P_measured_ln0 = [-89.16, -94.04, -99.08, -103.94, -108.85, -113.80, -118.56];

% lognormVar = 5 (including repeated values at -100)
P_target_ln5 = [-90, -92, -94, -95, -97, -99, ...
                -100, -100, -100, -100, -100, -100, ...
                -105, -110, -115, -120, -125, -130, -135];
P_measured_ln5 = [-86.2701, -88.75, -90.89, -91.4846, -93.1779, -95.693, ...
                  -96.6227, -96.8604, -97.1193, -96.8838, -96.8678, -96.6548, ...
                  -101.8635, -106.2634, -111.7995, -116.4474, ...
                  -120.9408, -124.3748, -126.5988];

% -------- Plot --------
figure;
hold on; grid on;

% Plot lognormVar = 0
plot(P_target_ln0, P_measured_ln0, 'o-', 'LineWidth', 1.5, ...
    'DisplayName', 'lognormVar = 0');

% Plot lognormVar = 5
plot(P_target_ln5, P_measured_ln5, 's--', 'LineWidth', 1.5, ...
    'DisplayName', 'lognormVar = 5');

% Ideal line
all_targets = [P_target_ln0, P_target_ln5];
all_measured = [P_measured_ln0, P_measured_ln5];
min_val = min([all_targets, all_measured]);
max_val = max([all_targets, all_measured]);
plot([min_val, max_val], [min_val, max_val], 'r:', 'LineWidth', 1.5, ...
    'DisplayName', 'ideal');
text(min_val + 2, min_val + 2, 'ideal', 'Color', 'r', 'FontSize', 10);

% Noise floor
noise_floor = -128.03;
xlim_vals = [min(all_targets), max(all_targets)];
plot(xlim_vals, [noise_floor, noise_floor], 'k--', 'LineWidth', 1, ...
    'DisplayName', 'noise floor');
text(xlim_vals(2), noise_floor, '  \leftarrow noise', ...
    'VerticalAlignment', 'bottom', 'FontSize', 10);

% Labels and title
xlabel('P_{target} (dBm)');
ylabel('P_{measured} (dBm)');
title('BLE calibration');
legend('Location', 'SouthEast');
