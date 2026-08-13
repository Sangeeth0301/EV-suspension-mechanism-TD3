% run_and_plot_simulink.m
clear; clc;
try
    disp('Loading parameters (running setup)...');
    % We don't want to rebuild the whole model to save time, just load variables.
    % So we will run create_simulink_model, but we will catch its output.
    create_simulink_model;
    
    disp('Simulating the model...');
    out = sim('CyberSuspension_MultiBlock', 'StopTime', '10');
    disp('Simulation finished. Plotting...');
    
    t = out.sim_out.time;
    d = out.sim_out.signals.values; 
    
    % Mux order: z_c_ours, z_c_base, accel_ours, accel_base, uf, rho_f, soc, V_lyap
    
    f = figure('Position', [100, 100, 1200, 800], 'Visible', 'off');
    
    subplot(2,2,1);
    plot(t, d(:,1), 'b', 'LineWidth', 1.5); hold on;
    plot(t, d(:,2), 'r--', 'LineWidth', 1);
    title('Body Displacement (z_c) [m]'); legend('Our LPV Project', 'Baseline Model'); grid on;
    
    subplot(2,2,2);
    plot(t, d(:,3), 'b', 'LineWidth', 1.5); hold on;
    plot(t, d(:,4), 'r--', 'LineWidth', 1);
    title('Body Acceleration [m/s^2]'); legend('Our LPV Project', 'Baseline Model'); grid on;
    
    subplot(2,2,3);
    plot(t, d(:,7), 'g', 'LineWidth', 1.5);
    title('Battery State of Charge (SoC)'); grid on;
    
    subplot(2,2,4);
    plot(t, d(:,5), 'm', 'LineWidth', 1.5);
    title('Active Suspension Force (U_f) [N]'); grid on;
    
    saveas(f, 'simulink_results.png');
    disp('Saved to simulink_results.png');
catch ME
    disp(ME.message);
end
exit;
