% ========================================================================
% PLOT_RESULTS.m - Run this AFTER your Simulink simulation finishes!
% ========================================================================
% This script extracts the data from the 'sim_out' workspace variable
% and plots it with proper, professional tags (legends) without disturbing
% your Simulink model.

disp('[*] Extracting Data and Tagging Graphs...');

% Ensure the simulation data exists
if ~exist('out', 'var') && ~exist('sim_out', 'var')
    error('Simulation data not found! Make sure you pressed PLAY in Simulink and the To Workspace block is named "sim_out" or outputs to "out".');
end

% Handle both 'sim_out' direct structure and 'out.sim_out' Dataset/Structure
if exist('out', 'var') && isfield(out, 'sim_out')
    data = out.sim_out;
elseif exist('sim_out', 'var')
    data = sim_out;
else
    error('Could not find sim_out data structure.');
end

time = data.time;
sigs = data.signals;

% Assuming standard Mux ordering based on the model:
% (You can adjust these indices if your Out_Mux is wired differently)
z_c_base   = sigs(1).values; % Passive Displacement
accel_base = sigs(2).values; % Passive Acceleration
z_c_active = sigs(3).values; % Active Displacement
accel_act  = sigs(4).values; % Active Acceleration

figure('Name', 'Cyber-Resilient Active Suspension Results', 'NumberTitle', 'off', 'Position', [100, 100, 1200, 600]);

% --- Plot 1: Body Acceleration (Ride Comfort) ---
subplot(1,2,1);
plot(time, accel_base, 'b', 'LineWidth', 1.2); hold on;
plot(time, accel_act, 'y', 'LineWidth', 1.5);
title('Body Acceleration (Ride Comfort)');
xlabel('Time (s)');
ylabel('Acceleration (m/s^2)');
legend('Passive Baseline (Blue)', 'LPV Active Suspension (Yellow)', 'Location', 'best');
grid on;

% --- Plot 2: Body Displacement ---
subplot(1,2,2);
plot(time, z_c_base, 'b', 'LineWidth', 1.2); hold on;
plot(time, z_c_active, 'y', 'LineWidth', 1.5);
title('Body Displacement (Chassis Bounce)');
xlabel('Time (s)');
ylabel('Displacement (m)');
legend('Passive Baseline (Blue)', 'LPV Active Suspension (Yellow)', 'Location', 'best');
grid on;

disp('[+] Professional Graphs generated successfully!');
