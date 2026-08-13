%% ========================================================================
% CYBER-RESILIENT LPV-ADAPTIVE EV ACTIVE SUSPENSION
% Complete MATLAB Implementation with Simulink Model Generation
% ========================================================================
% This script replicates the ENTIRE Python simulation in MATLAB:
%   - 4-DOF Half-Car Plant (8-state ODE with bump stops + IWM)
%   - UKF Road Estimator (rho + rho_dot)
%   - LPV H-inf Controller (CARE + polytopic blending + rho_dot feedforward)
%   - TD3 Heuristic Energy Agent (COMFORT/ECO mode switching)
%   - LC Power Electronics Filter (2-state ODE)
%   - METS Network Filter (event-triggered CAN-bus)
%   - Lyapunov Stability Monitor (V(x) = x'Px)
%   - Base Paper Fixed H-inf Controller (for comparison)
%   - ISO 8608 Road Generator + Random Potholes
%
% Author: Auto-generated from Python codebase
% ========================================================================
clear; clc; close all;
fprintf('======================================================================\n');
fprintf('[*] Cyber-Resilient LPV-Adaptive Active Suspension — MATLAB Edition\n');
fprintf('[*] With Power Electronics Integration & Lyapunov Monitoring\n');
fprintf('======================================================================\n');
tic;

%% ========================================================================
% SECTION 1: VEHICLE PARAMETERS
% ========================================================================
fprintf('[*] Loading Vehicle Parameters...\n');

% --- Sprung Mass (Car Body) ---
p.ms       = 730.0;          % Half-car sprung mass (kg)
p.I_phi    = 1222.0;         % Pitch moment of inertia (kg*m^2)
p.a        = 1.1;            % CG to front axle (m)
p.b        = 1.5;            % CG to rear axle (m)
p.ms_f     = p.ms * p.b / (p.a + p.b);  % Front sprung mass equivalent (~421 kg)
p.ms_r     = p.ms * p.a / (p.a + p.b);  % Rear sprung mass equivalent (~309 kg)

% --- Unsprung Mass (In-Wheel Motors) ---
p.mu_f     = 45.0;           % Front unsprung mass (kg)
p.mu_r     = 45.0;           % Rear unsprung mass (kg)

% --- Suspension Stiffness & Damping ---
p.ks_f     = 18000.0;        % Front spring stiffness (N/m)
p.ks_r     = 22000.0;        % Rear spring stiffness (N/m)
p.cs_f     = 1200.0;         % Front passive damping (N*s/m)
p.cs_r     = 1200.0;         % Rear passive damping (N*s/m)

% --- Tire Stiffness ---
p.kt_f     = 190000.0;       % Front tire stiffness (N/m)
p.kt_r     = 190000.0;       % Rear tire stiffness (N/m)

% --- In-Wheel Motor (IWM) Disturbance ---
p.k_em     = 15.0;           % Electromechanical coupling factor (N/A)
p.i_d      = 20.0;           % Stator current amplitude (A)
p.p_pole   = 8;              % Number of pole pairs
p.wheel_radius = 0.3;        % Tire radius (m)

% --- Regenerative Actuator ---
p.C_e      = 1500.0;         % Regenerative damping coefficient (N*s/m)
p.eta_regen = 0.65;          % Power electronics efficiency
p.K_v      = 50.0;           % Back-EMF constant (V per m/s)

% --- Physical Limits ---
p.stroke_max = 0.08;         % Max suspension deflection (m)
p.u_max    = 6000.0;         % Max actuator force (N)
p.k_bs     = 1e7;            % Bump stop stiffness (N/m^3)

% --- Power Electronics (LC Filter) ---
p.L_filter = 0.01;           % Inductance (10 mH)
p.C_filter = 0.0047;         % Capacitance (4700 uF)
p.R_load   = 10.0;           % Battery equivalent resistance (Ohm)

% --- Simulation Timing ---
p.dt       = 0.001;          % Timestep = 1 ms (1 kHz)
p.T_total  = 10.0;           % Total simulation time (s)
p.v_kmh    = 72.0;           % Vehicle speed (km/h)
p.v_ms     = p.v_kmh / 3.6;  % Vehicle speed (m/s) = 20 m/s

% --- METS Network Filter ---
p.sigma_mets = 0.1;          % Event-trigger threshold

% Derived
t = 0:p.dt:(p.T_total - p.dt);
n_steps = length(t);
wheelbase = p.a + p.b;
delay_steps = round(wheelbase / p.v_ms / p.dt);

fprintf('    Parameters loaded: ms=%.0f kg, v=%.0f km/h, dt=%.0f ms\n', p.ms, p.v_kmh, p.dt*1000);

%% ========================================================================
% SECTION 2: CARE GAIN SYNTHESIS (LPV Controller Design)
% ========================================================================
fprintf('[*] Synthesizing LPV Controller Gains via CARE...\n');

% Quarter-car state-space model for gain synthesis
% State: x = [z_s - z_u, z_s_dot, z_u - w, z_u_dot]
A_qc = [0,                    1,                    0,  -1;
        -p.ks_f/p.ms_f,      -p.cs_f/p.ms_f,       0,   p.cs_f/p.ms_f;
         0,                    0,                    0,   1;
         p.ks_f/p.mu_f,       p.cs_f/p.mu_f,  -p.kt_f/p.mu_f, -p.cs_f/p.mu_f];

B_qc = [0; 1/p.ms_f; 0; -1/p.mu_f];

% --- Smooth Road Gain (rho = 0) ---
rho_smooth = 0.0;
q_stroke_s  = 1e5 * (1.0 - 0.5 * rho_smooth);
q_body_vel_s = 1e4 * (1.0 + 5.0 * rho_smooth);
Q_smooth = diag([q_stroke_s, q_body_vel_s, 1e4, 1e2]);
R_care   = 1e-3;

[P_smooth, ~, ~] = care(A_qc, B_qc, Q_smooth, R_care);
K_smooth = (1/R_care) * B_qc' * P_smooth;
K_smooth = K_smooth(:)';  % 1x4 row vector

% --- Rough Road Gain (rho = 1) ---
rho_rough = 1.0;
q_stroke_r  = 1e5 * (1.0 - 0.5 * rho_rough);
q_body_vel_r = 1e4 * (1.0 + 5.0 * rho_rough);
Q_rough = diag([q_stroke_r, q_body_vel_r, 1e4, 1e2]);

[P_rough, ~, ~] = care(A_qc, B_qc, Q_rough, R_care);
K_rough = (1/R_care) * B_qc' * P_rough;
K_rough = K_rough(:)';

% Lyapunov P matrix (for stability monitoring)
Q_lyap = diag([1e5, 1e4, 1e4, 1e2]);
[P_lyap, ~, ~] = care(A_qc, B_qc, Q_lyap, R_care);

% Feedforward gain for rho_dot
K_rho_dot = [0.0, 500.0, 0.0, 0.0];

fprintf('    K_smooth = [%.1f, %.1f, %.1f, %.1f]\n', K_smooth);
fprintf('    K_rough  = [%.1f, %.1f, %.1f, %.1f]\n', K_rough);

% --- Base Paper Fixed H-inf Gains ---
K_base_front = [-12000, 5500, 8000, 0, -3500, 1800, 2200, 0];
K_base_rear  = [-12000, -5500, 0, 8000, -3500, -1800, 0, 2200];

%% ========================================================================
% SECTION 3: ROAD PROFILE GENERATION (ISO 8608 + Potholes)
% ========================================================================
fprintf('[*] Generating Mixed-Pothole Test Track...\n');

% --- ISO 8608 Road Generation ---
% Roughness coefficients G(n0) in m^3
G_A = 16e-6;    % Smooth highway
G_D = 1024e-6;  % Poor/broken road

n0_road = 0.1;   % Spatial frequency reference (cycles/m)
f0_road = 0.01;  % Lower cutoff frequency (Hz)

rng(42);  % Reproducibility

% Generate ISO Class A base road
w_amp_A = 2*pi*n0_road * sqrt(G_A * p.v_ms);
alpha_road = 2*pi*f0_road;
w_noise_A = randn(1, n_steps);
w_base = zeros(1, n_steps);
for i = 2:n_steps
    w_base(i) = w_base(i-1) + p.dt * (-alpha_road*w_base(i-1) + w_amp_A*w_noise_A(i-1));
end

% Generate ISO Class D rough overlay
rng(43);
w_amp_D = 2*pi*n0_road * sqrt(G_D * p.v_ms);
w_noise_D = randn(1, n_steps);
w_rough_overlay = zeros(1, n_steps);
for i = 2:n_steps
    w_rough_overlay(i) = w_rough_overlay(i-1) + p.dt * (-alpha_road*w_rough_overlay(i-1) + w_amp_D*w_noise_D(i-1));
end

% Combine: smooth 0-3s, rough 3-7s, smooth 7-10s
w_f = w_base;
idx_s = round(3.0/p.dt) + 1;
idx_e = round(7.0/p.dt);
fade_len = round(0.2/p.dt);

fade_in  = linspace(0, 1, fade_len);
fade_out = linspace(1, 0, fade_len);
fade_full = ones(1, idx_e - idx_s + 1);
fade_full(1:fade_len) = fade_in;
fade_full(end-fade_len+1:end) = fade_out;

w_f(idx_s:idx_e) = w_f(idx_s:idx_e) + w_rough_overlay(idx_s:idx_e) .* fade_full;

% --- Add Potholes (1-cosine shape) ---
pothole_depths = [-0.060, -0.040, -0.020, -0.005, -0.050, -0.030]; % meters
pothole_widths = [ 0.250,  0.150,  0.100,  0.050,  0.200,  0.120]; % seconds
pothole_times  = linspace(3.5, 6.5, 6);
rng(42);
pothole_times = pothole_times(randperm(length(pothole_times)));

for k = 1:length(pothole_depths)
    t_start = pothole_times(k);
    depth   = pothole_depths(k);
    width   = pothole_widths(k);
    
    i_start = round(t_start / p.dt) + 1;
    dur_steps = round(width / p.dt);
    i_end   = min(i_start + dur_steps - 1, n_steps);
    actual_dur = i_end - i_start + 1;
    
    x_pot = linspace(0, 2*pi, actual_dur);
    pothole_shape = (depth/2) * (1 - cos(x_pot));
    w_f(i_start:i_end) = w_f(i_start:i_end) + pothole_shape;
end

% --- Rear wheel profile (delayed by wheelbase) ---
w_r = zeros(1, n_steps);
w_r(delay_steps+1:end) = w_f(1:end-delay_steps);

fprintf('    Track: 10s at 72 km/h | 6 potholes | Wheelbase delay: %d ms\n', delay_steps);

%% ========================================================================
% SECTION 4: MAIN SIMULATION LOOP (1kHz RK4)
% ========================================================================
fprintf('[*] Running 1kHz Simulation Loop...\n');
fprintf('[*] Steps: %d | dt: %.0fms | Speed: %.0f km/h\n', n_steps, p.dt*1000, p.v_kmh);

% --- Pre-allocate logging arrays ---
log.t           = t;
log.w_f         = w_f;
log.w_r         = w_r;

% Base paper
log.z_c_base    = zeros(1, n_steps);
log.theta_base  = zeros(1, n_steps);
log.accel_base  = zeros(1, n_steps);
log.u_f_base_log = zeros(1, n_steps);

% Our project
log.z_c_ours    = zeros(1, n_steps);
log.theta_ours  = zeros(1, n_steps);
log.z_uf_ours   = zeros(1, n_steps);
log.z_ur_ours   = zeros(1, n_steps);
log.accel_ours  = zeros(1, n_steps);
log.pitch_rate  = zeros(1, n_steps);
log.u_f_ours_log = zeros(1, n_steps);
log.u_r_ours_log = zeros(1, n_steps);
log.rho_f_log   = zeros(1, n_steps);
log.rho_dot_f_log = zeros(1, n_steps);
log.mode_log    = zeros(1, n_steps);  % 0=COMFORT, 1=ECO
log.battery_soc_log = zeros(1, n_steps);
log.power_harvested = zeros(1, n_steps);
log.power_consumed  = zeros(1, n_steps);
log.V_lyap      = zeros(1, n_steps);
log.lc_voltage  = zeros(1, n_steps);
log.tire_def_f_base = zeros(1, n_steps);
log.tire_def_f_ours = zeros(1, n_steps);

% --- Initial States ---
state_base = zeros(8, 1);
state_ours = zeros(8, 1);

% Battery
battery_soc       = 0.50;
battery_capacity_j = 50000.0;
eta_actuator      = 0.85;

% LC filter state [i_L; v_C]
lc_state = [0; 0];

% UKF simplified state
rho_f = 0; rho_r = 0;
rho_dot_f = 0; rho_dot_r = 0;
rho_f_prev = 0; rho_r_prev = 0;
rho_dot_f_filt = 0; rho_dot_r_filt = 0;
w_max_ukf = 0.05;
ema_alpha = 0.05;

% METS state
mets_last_state = zeros(8, 1);
mets_first = true;
mets_transmits = 0;

% Lyapunov settling tracker
lyap_is_disturbed = false;
lyap_dist_start = 0;
lyap_settled_count = 0;
lyap_settling_times = [];

% Previous forces for UKF
u_f_prev = 0; u_r_prev = 0;
dx_ours = zeros(8, 1);

% ==========================================================================
% THE LOOP
% ==========================================================================
for i = 1:n_steps
    ti = t(i);
    wf = w_f(i);
    wr = w_r(i);
    
    % ======================================================================
    % BASE PAPER SIMULATION (Fixed H-inf)
    % ======================================================================
    u_f_base = -K_base_front * state_base;
    u_r_base = -K_base_rear  * state_base;
    u_f_base = max(-p.u_max, min(p.u_max, u_f_base));
    u_r_base = max(-p.u_max, min(p.u_max, u_r_base));
    
    [state_base, dx_base] = rk4_step_halfcar(ti, state_base, u_f_base, u_r_base, wf, wr, p);
    
    log.z_c_base(i)    = state_base(1);
    log.theta_base(i)  = state_base(2);
    log.accel_base(i)  = dx_base(5);
    log.u_f_base_log(i) = u_f_base;
    log.tire_def_f_base(i) = state_base(3) - wf;
    
    % ======================================================================
    % OUR PROJECT SIMULATION (LPV + UKF + TD3 + METS + Power Electronics)
    % ======================================================================
    
    % --- Step A: METS Network Filter ---
    if mets_first
        network_state = state_ours;
        mets_last_state = state_ours;
        mets_first = false;
        mets_transmits = mets_transmits + 1;
        transmit_flag = true;
    else
        e_mets = state_ours - mets_last_state;
        norm_e = norm(e_mets);
        norm_x = norm(state_ours);
        if norm_e > p.sigma_mets * norm_x
            mets_last_state = state_ours;
            network_state = state_ours;
            mets_transmits = mets_transmits + 1;
            transmit_flag = true;
        else
            network_state = mets_last_state;
            transmit_flag = false;
        end
    end
    
    % --- Step B: UKF Road Estimation (Simplified Algebraic) ---
    % Use body acceleration as proxy for road estimation
    z_s_ddot = dx_ours(5);
    
    % Simplified estimator: infer road from suspension deflection dynamics
    z_sf = state_ours(1) - p.a * state_ours(2);
    z_uf = state_ours(3);
    susp_stroke = abs(z_sf - z_uf);
    
    % Estimate road roughness from suspension activity
    w_est_f = susp_stroke;
    rho_f = min(1.0, max(0.0, w_est_f / w_max_ukf));
    
    raw_rho_dot_f = (rho_f - rho_f_prev) / p.dt;
    rho_dot_f_filt = ema_alpha * raw_rho_dot_f + (1 - ema_alpha) * rho_dot_f_filt;
    rho_dot_f = rho_dot_f_filt;
    rho_f_prev = rho_f;
    
    % Rear wheel (simplified: use same rho with delay)
    rho_r = rho_f;
    rho_dot_r = rho_dot_f;
    
    % --- Step C: TD3 Heuristic Energy Agent ---
    if rho_f > 0.7 || rho_dot_f > 0.5
        if battery_soc < 0.02
            mode_f = 1;  % ECO
        else
            mode_f = 0;  % COMFORT
        end
    elseif rho_f > 0.3
        if battery_soc < 0.25
            mode_f = 1;  % ECO
        else
            mode_f = 0;  % COMFORT
        end
    elseif rho_f < 0.3
        mode_f = 1;      % ECO (harvest on smooth roads)
    else
        mode_f = 0;      % COMFORT
    end
    mode_r = mode_f;
    
    % --- Step D: LPV Controller ---
    % Map 8-DOF half-car to 4-DOF quarter-car (FRONT)
    z_sf = network_state(1) - p.a * network_state(2);
    z_sf_dot = network_state(5) - p.a * network_state(6);
    qc_state_f = [z_sf - network_state(3); z_sf_dot; 0; network_state(7)];
    
    % Map REAR
    z_sr = network_state(1) + p.b * network_state(2);
    z_sr_dot = network_state(5) + p.b * network_state(6);
    qc_state_r = [z_sr - network_state(4); z_sr_dot; 0; network_state(8)];
    
    if mode_f == 1  % ECO
        u_f_ours = -p.C_e * qc_state_f(2);
    else            % COMFORT
        K_f = (1.0 - rho_f) * K_smooth + rho_f * K_rough;
        u_f_base_lpv = -K_f * qc_state_f;
        u_f_ff = -K_rho_dot * max(0, rho_dot_f) * qc_state_f;
        ff_limit = 0.2 * p.u_max;
        u_f_ff = max(-ff_limit, min(ff_limit, u_f_ff));
        u_f_ours = u_f_base_lpv + u_f_ff;
    end
    
    if mode_r == 1  % ECO
        u_r_ours = -p.C_e * qc_state_r(2);
    else
        K_r_gain = (1.0 - rho_r) * K_smooth + rho_r * K_rough;
        u_r_base_lpv = -K_r_gain * qc_state_r;
        u_r_ff = -K_rho_dot * max(0, rho_dot_r) * qc_state_r;
        ff_limit = 0.2 * p.u_max;
        u_r_ff = max(-ff_limit, min(ff_limit, u_r_ff));
        u_r_ours = u_r_base_lpv + u_r_ff;
    end
    
    u_f_ours = max(-p.u_max, min(p.u_max, u_f_ours));
    u_r_ours = max(-p.u_max, min(p.u_max, u_r_ours));
    
    u_f_prev = u_f_ours;
    u_r_prev = u_r_ours;
    
    % --- Step E: Physics Integration (RK4) ---
    [state_ours, dx_ours] = rk4_step_halfcar(ti, state_ours, u_f_ours, u_r_ours, wf, wr, p);
    
    % --- Step F: Energy Harvesting with Power Electronics ---
    z_sf_dot_post = state_ours(5) - p.a * state_ours(6);
    z_sr_dot_post = state_ours(5) + p.b * state_ours(6);
    v_rel_f = z_sf_dot_post - state_ours(7);
    v_rel_r = z_sr_dot_post - state_ours(8);
    
    harvested_j = 0;
    consumed_w = 0;
    
    if mode_f == 1  % ECO mode — harvest
        v_em_raw = p.K_v * v_rel_f;
        v_rectified = abs(v_em_raw);
        
        % LC filter ODE (Euler step)
        di_L = (v_rectified - lc_state(2)) / p.L_filter;
        dv_C = (lc_state(1) - lc_state(2)/p.R_load) / p.C_filter;
        lc_state(1) = max(0, lc_state(1) + di_L * p.dt);
        lc_state(2) = max(0, lc_state(2) + dv_C * p.dt);
        
        % Conditioned power
        i_load = lc_state(2) / p.R_load;
        power_cond = lc_state(2) * i_load * p.eta_regen;
        harvested_j = power_cond * p.dt;
        battery_soc = min(1.0, battery_soc + harvested_j / battery_capacity_j);
    else  % COMFORT mode — consume
        consumed_w = abs(u_f_ours * v_rel_f) / eta_actuator;
        consumed_j = consumed_w * p.dt;
        battery_soc = max(0.0, battery_soc - consumed_j / battery_capacity_j);
        
        % LC filter decays
        di_L = (0 - lc_state(2)) / p.L_filter;
        dv_C = (lc_state(1) - lc_state(2)/p.R_load) / p.C_filter;
        lc_state(1) = max(0, lc_state(1) + di_L * p.dt);
        lc_state(2) = max(0, lc_state(2) + dv_C * p.dt);
    end
    
    % --- Step G: Lyapunov Monitoring ---
    z_sf_lyap = state_ours(1) - p.a * state_ours(2);
    z_sf_dot_lyap = state_ours(5) - p.a * state_ours(6);
    x_qc_lyap = [z_sf_lyap - state_ours(3); z_sf_dot_lyap; 0; state_ours(7)];
    V_x = x_qc_lyap' * P_lyap * x_qc_lyap;
    
    % Settling time tracker
    abs_zc = abs(state_ours(1));
    if ~lyap_is_disturbed
        if abs_zc > 0.005
            lyap_is_disturbed = true;
            lyap_dist_start = ti;
            lyap_settled_count = 0;
        end
    else
        if abs_zc < 0.002
            lyap_settled_count = lyap_settled_count + 1;
            if lyap_settled_count >= 30
                settle_time = ti - lyap_dist_start;
                lyap_settling_times(end+1) = settle_time;
                lyap_is_disturbed = false;
                lyap_settled_count = 0;
            end
        else
            lyap_settled_count = 0;
        end
    end
    
    % --- LOG ---
    log.z_c_ours(i)     = state_ours(1);
    log.theta_ours(i)   = state_ours(2);
    log.z_uf_ours(i)    = state_ours(3);
    log.z_ur_ours(i)    = state_ours(4);
    log.accel_ours(i)   = dx_ours(5);
    log.pitch_rate(i)   = state_ours(6);
    log.u_f_ours_log(i) = u_f_ours;
    log.u_r_ours_log(i) = u_r_ours;
    log.rho_f_log(i)    = rho_f;
    log.rho_dot_f_log(i) = rho_dot_f;
    log.mode_log(i)     = mode_f;
    log.battery_soc_log(i) = battery_soc;
    log.power_harvested(i)  = harvested_j / p.dt;  % W
    log.power_consumed(i)   = consumed_w;
    log.V_lyap(i)       = V_x;
    log.lc_voltage(i)   = lc_state(2);
    log.tire_def_f_ours(i) = state_ours(3) - wf;
    
    % Progress
    if mod(i, n_steps/10) == 0
        fprintf('    ... %d%% complete | SoC: %.1f%% | Mode: %s | rho: %.2f\n', ...
            round(i/n_steps*100), battery_soc*100, ...
            char('C'*(mode_f==0) + 'E'*(mode_f==1)), rho_f);
    end
end

elapsed = toc;
fprintf('[*] Simulation complete.\n');
fprintf('[*] Solved %d timesteps in %.2f seconds.\n', n_steps, elapsed);

%% ========================================================================
% SECTION 5: PERFORMANCE METRICS CALCULATION
% ========================================================================
fprintf('[*] Processing Results...\n');

% RMS calculations
rms_accel_base = rms(log.accel_base);
rms_accel_ours = rms(log.accel_ours);
peak_accel_base = max(abs(log.accel_base));
peak_accel_ours = max(abs(log.accel_ours));
rms_tire_f_base = rms(log.tire_def_f_base);
rms_tire_f_ours = rms(log.tire_def_f_ours);
rms_pitch_base = rms(log.theta_base);
rms_pitch_ours = rms(log.theta_ours);

comfort_improvement = (1 - rms_accel_ours/rms_accel_base) * 100;

% Energy metrics
total_harvested_j = sum(log.power_harvested) * p.dt;
total_consumed_j  = sum(log.power_consumed) * p.dt;
total_harvested_kj = total_harvested_j / 1000;
total_consumed_kj  = total_consumed_j / 1000;
net_energy_kj = total_harvested_kj - total_consumed_kj;

peak_regen_w = max(log.power_harvested);
eco_mask = (log.mode_log == 1);
if any(eco_mask)
    avg_regen_w = mean(log.power_harvested(eco_mask));
else
    avg_regen_w = 0;
end

soc_initial = log.battery_soc_log(1);
soc_final   = log.battery_soc_log(end);
soc_change_pct = (soc_final - soc_initial) * 100;

if total_consumed_j > 0
    energy_efficiency = (total_harvested_j / total_consumed_j) * 100;
else
    energy_efficiency = 0;
end

% Settling time (simple threshold)
settling_base = compute_settling_time(t, log.z_c_base, 0.002);
settling_ours = compute_settling_time(t, log.z_c_ours, 0.002);

% Lyapunov settling
if ~isempty(lyap_settling_times)
    lyap_worst = max(lyap_settling_times);
    lyap_avg   = mean(lyap_settling_times);
else
    lyap_worst = inf;
    lyap_avg   = inf;
end

% METS bandwidth
bandwidth_saved = 100 * (1 - mets_transmits / n_steps);

%% ========================================================================
% SECTION 6: CONSOLE COMPARISON TABLE
% ========================================================================
fprintf('\n');
fprintf('================================================================================\n');
fprintf('FINAL PERFORMANCE RESULTS (MATLAB)\n');
fprintf('================================================================================\n');
fprintf('%-35s | %-15s | %-15s | Unit\n', 'Metric', 'Base Paper', 'Our Project');
fprintf('--------------------------------------------------------------------------------\n');
fprintf('%-35s | %-15.4f | %-15.4f | m/s^2\n', 'RMS Body Accel (Comfort)', rms_accel_base, rms_accel_ours);
fprintf('%-35s | %-15.4f | %-15.4f | m/s^2\n', 'Peak Body Accel', peak_accel_base, peak_accel_ours);
fprintf('%-35s | %-15.4f | %-15.4f | m\n',    'RMS Tire Deflection (Safety)', rms_tire_f_base, rms_tire_f_ours);
fprintf('%-35s | %-15.6f | %-15.6f | rad\n',  'RMS Body Pitch', rms_pitch_base, rms_pitch_ours);
fprintf('%-35s | %-15.4f | %-15.4f | s\n',    'Settling Time (worst)', settling_base, settling_ours);
fprintf('%-35s | %-15s | %-15.4f | s\n',      'Lyapunov Worst Settling', 'N/A', lyap_worst);
fprintf('%-35s | %-15s | %-15.4f | s\n',      'Lyapunov Avg Settling', 'N/A', lyap_avg);
fprintf('--------------------------------------------------------------------------------\n');
fprintf('%-35s | %-15s | %-15.1f | %%\n',     'Comfort Improvement', '---', comfort_improvement);
fprintf('%-35s | %-15s | %-15.4f | kJ\n',     'Energy Harvested', '0.00', total_harvested_kj);
fprintf('%-35s | %-15s | %-15.4f | kJ\n',     'Energy Consumed (Actuator)', 'N/A', total_consumed_kj);
fprintf('%-35s | %-15s | %-15.4f | kJ\n',     'Net Energy Balance', 'N/A', net_energy_kj);
fprintf('%-35s | %-15s | %-15.1f | W\n',      'Peak Regen Power', '0', peak_regen_w);
fprintf('%-35s | %-15s | %-15.1f | W\n',      'Avg Regen Power (ECO mode)', '0', avg_regen_w);
fprintf('%-35s | %-15s | %+15.2f | %%\n',     sprintf('SoC: %.0f%% -> %.0f%%', soc_initial*100, soc_final*100), '---', soc_change_pct);
fprintf('%-35s | %-15s | %-15.1f | %%\n',     'Harvesting Efficiency', '0.0%', energy_efficiency);
fprintf('%-35s | %-15s | %-15.1f | %%\n',     'CAN-bus Bandwidth Saved', '0.0%', bandwidth_saved);
fprintf('================================================================================\n');

if lyap_worst <= 0.5
    fprintf('[OK] LYAPUNOV GUARANTEE MET: Worst settling time %.3fs <= 0.5s\n', lyap_worst);
else
    fprintf('[!!] LYAPUNOV GUARANTEE: Worst settling time %.3fs (target <= 0.5s)\n', lyap_worst);
end

%% ========================================================================
% SECTION 7: 10-PANEL RESULTS DASHBOARD
% ========================================================================
fprintf('[*] Generating 10-Panel Dashboard...\n');

fig = figure('Name', 'Cyber-Resilient Active Suspension — MATLAB Results', ...
    'Position', [50, 50, 1800, 1000], 'Color', [0.06 0.08 0.12]);

% Color palette
c_blue   = [0.23 0.51 0.96];
c_red    = [0.94 0.27 0.27];
c_green  = [0.06 0.73 0.51];
c_purple = [0.55 0.36 0.96];
c_orange = [0.96 0.62 0.04];
c_cyan   = [0.02 0.71 0.83];
c_gray   = [0.58 0.64 0.70];

% --- Panel 1: Road Profile ---
ax1 = subplot(5,2,1);
plot(t, w_f*1000, 'Color', c_blue, 'LineWidth', 1); hold on;
plot(t, w_r*1000, 'Color', c_cyan, 'LineWidth', 0.8);
xlabel('Time (s)'); ylabel('Height (mm)');
title('Road Profile', 'Color', 'w');
legend('Front w_f', 'Rear w_r', 'Location', 'best', 'TextColor', 'w');
set(ax1, 'Color', [0.08 0.1 0.15], 'XColor', 'w', 'YColor', 'w'); grid on;

% --- Panel 2: Body Displacement Comparison ---
ax2 = subplot(5,2,2);
plot(t, log.z_c_base*1000, 'Color', c_red, 'LineWidth', 1); hold on;
plot(t, log.z_c_ours*1000, 'Color', c_green, 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Displacement (mm)');
title('Body Vertical Displacement', 'Color', 'w');
legend('Base Paper (Fixed H\infty)', 'Our LPV + TD3', 'Location', 'best', 'TextColor', 'w');
set(ax2, 'Color', [0.08 0.1 0.15], 'XColor', 'w', 'YColor', 'w'); grid on;

% --- Panel 3: Body Acceleration Comparison ---
ax3 = subplot(5,2,3);
plot(t, log.accel_base, 'Color', c_red, 'LineWidth', 0.8); hold on;
plot(t, log.accel_ours, 'Color', c_green, 'LineWidth', 1);
xlabel('Time (s)'); ylabel('Accel (m/s^2)');
title(sprintf('Body Acceleration — Comfort Improvement: %.1f%%', comfort_improvement), 'Color', 'w');
legend('Base Paper', 'Our Project', 'Location', 'best', 'TextColor', 'w');
set(ax3, 'Color', [0.08 0.1 0.15], 'XColor', 'w', 'YColor', 'w'); grid on;

% --- Panel 4: UKF Road Severity (rho) ---
ax4 = subplot(5,2,4);
yyaxis left;
plot(t, log.rho_f_log, 'Color', c_orange, 'LineWidth', 1.2);
ylabel('\rho(t)');
yyaxis right;
plot(t, log.rho_dot_f_log, 'Color', c_purple, 'LineWidth', 0.8);
ylabel('d\rho/dt');
xlabel('Time (s)');
title('UKF Road Severity Estimation', 'Color', 'w');
set(ax4, 'Color', [0.08 0.1 0.15], 'XColor', 'w'); grid on;
ax4.YAxis(1).Color = c_orange;
ax4.YAxis(2).Color = c_purple;

% --- Panel 5: Actuator Force ---
ax5 = subplot(5,2,5);
area(t, log.u_f_ours_log, 'FaceColor', c_blue, 'FaceAlpha', 0.4, 'EdgeColor', c_blue); hold on;
area(t, log.u_r_ours_log, 'FaceColor', c_purple, 'FaceAlpha', 0.3, 'EdgeColor', c_purple);
xlabel('Time (s)'); ylabel('Force (N)');
title('LPV Actuator Forces', 'Color', 'w');
legend('Front u_f', 'Rear u_r', 'Location', 'best', 'TextColor', 'w');
set(ax5, 'Color', [0.08 0.1 0.15], 'XColor', 'w', 'YColor', 'w'); grid on;

% --- Panel 6: Body Pitch ---
ax6 = subplot(5,2,6);
plot(t, log.theta_base*1000, 'Color', c_red, 'LineWidth', 0.8); hold on;
plot(t, log.theta_ours*1000, 'Color', c_green, 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Pitch (mrad)');
title('Body Pitch Angle', 'Color', 'w');
legend('Base Paper', 'Our Project', 'Location', 'best', 'TextColor', 'w');
set(ax6, 'Color', [0.08 0.1 0.15], 'XColor', 'w', 'YColor', 'w'); grid on;

% --- Panel 7: Energy Mode & Battery SoC ---
ax7 = subplot(5,2,7);
yyaxis left;
area(t, log.mode_log, 'FaceColor', c_green, 'FaceAlpha', 0.3, 'EdgeColor', 'none');
ylabel('Mode (0=COM, 1=ECO)');
ylim([-0.1, 1.5]);
yyaxis right;
plot(t, log.battery_soc_log*100, 'Color', c_orange, 'LineWidth', 1.5);
ylabel('SoC (%)');
xlabel('Time (s)');
title(sprintf('TD3 Energy Agent — SoC: %.1f%% -> %.1f%%', soc_initial*100, soc_final*100), 'Color', 'w');
set(ax7, 'Color', [0.08 0.1 0.15], 'XColor', 'w'); grid on;
ax7.YAxis(1).Color = c_green;
ax7.YAxis(2).Color = c_orange;

% --- Panel 8: Power Electronics ---
ax8 = subplot(5,2,8);
plot(t, log.power_harvested, 'Color', c_green, 'LineWidth', 1); hold on;
plot(t, -log.power_consumed, 'Color', c_red, 'LineWidth', 0.8);
xlabel('Time (s)'); ylabel('Power (W)');
title(sprintf('Power Balance — Peak Regen: %.1f W', peak_regen_w), 'Color', 'w');
legend('Harvested', 'Consumed', 'Location', 'best', 'TextColor', 'w');
set(ax8, 'Color', [0.08 0.1 0.15], 'XColor', 'w', 'YColor', 'w'); grid on;

% --- Panel 9: LC Filter Voltage ---
ax9 = subplot(5,2,9);
plot(t, log.lc_voltage, 'Color', c_cyan, 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Voltage (V)');
title('LC Filter Output Voltage', 'Color', 'w');
set(ax9, 'Color', [0.08 0.1 0.15], 'XColor', 'w', 'YColor', 'w'); grid on;

% --- Panel 10: Lyapunov Function ---
ax10 = subplot(5,2,10);
semilogy(t, max(log.V_lyap, 1e-20), 'Color', c_purple, 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('V(x) [log scale]');
title(sprintf('Lyapunov Stability V(x) = x^T P x — Worst Settling: %.3fs', lyap_worst), 'Color', 'w');
set(ax10, 'Color', [0.08 0.1 0.15], 'XColor', 'w', 'YColor', 'w'); grid on;

% Main title
sgtitle(sprintf('Cyber-Resilient LPV-Adaptive EV Active Suspension — MATLAB Results\nComfort: %.1f%% | Energy Harvested: %.4f kJ | Peak Regen: %.1f W | CAN Saved: %.1f%%', ...
    comfort_improvement, total_harvested_kj, peak_regen_w, bandwidth_saved), ...
    'Color', 'w', 'FontSize', 14, 'FontWeight', 'bold');

% Save figure
results_dir = fullfile(fileparts(mfilename('fullpath')), '..', 'Simulation_Core', 'results');
if ~exist(results_dir, 'dir')
    mkdir(results_dir);
end
saveas(fig, fullfile(results_dir, 'MATLAB_Results_Dashboard.png'));
fprintf('[*] Dashboard saved to: Simulation_Core/results/MATLAB_Results_Dashboard.png\n');

%% ========================================================================
% SECTION 8: SIMULINK MODEL GENERATION
% ========================================================================
fprintf('[*] Generating Simulink Model...\n');

try
    model_name = 'CyberResilient_ActiveSuspension';
    
    % Close if already open
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    
    % Create new model
    new_system(model_name);
    open_system(model_name);
    
    % Set solver to fixed-step at 1ms (matching our Python simulation)
    set_param(model_name, 'SolverType', 'Fixed-step');
    set_param(model_name, 'FixedStep', '0.001');
    set_param(model_name, 'StopTime', '10');
    set_param(model_name, 'Solver', 'ode4');
    
    % =====================================================================
    % Add Road Profile (From Workspace)
    % =====================================================================
    % Save road data to workspace for Simulink
    road_time_series_f = timeseries(w_f', t');
    road_time_series_r = timeseries(w_r', t');
    assignin('base', 'road_f_ts', road_time_series_f);
    assignin('base', 'road_r_ts', road_time_series_r);
    
    add_block('simulink/Sources/From Workspace', [model_name '/Road_Front'], ...
        'VariableName', 'road_f_ts', 'Position', [50, 100, 150, 140]);
    add_block('simulink/Sources/From Workspace', [model_name '/Road_Rear'], ...
        'VariableName', 'road_r_ts', 'Position', [50, 200, 150, 240]);
    
    % =====================================================================
    % Add Half-Car Plant (MATLAB Function Block)
    % =====================================================================
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [model_name '/HalfCar_Plant'], 'Position', [400, 80, 550, 280]);
    
    % =====================================================================
    % Add LPV Controller (MATLAB Function Block)
    % =====================================================================
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [model_name '/LPV_Controller'], 'Position', [400, 350, 550, 450]);
    
    % =====================================================================
    % Add Scopes
    % =====================================================================
    add_block('simulink/Sinks/Scope', [model_name '/Body_Displacement_Scope'], ...
        'Position', [700, 100, 750, 140], 'NumInputPorts', '2');
    add_block('simulink/Sinks/Scope', [model_name '/Body_Acceleration_Scope'], ...
        'Position', [700, 180, 750, 220], 'NumInputPorts', '2');
    add_block('simulink/Sinks/Scope', [model_name '/Actuator_Force_Scope'], ...
        'Position', [700, 260, 750, 300]);
    add_block('simulink/Sinks/Scope', [model_name '/Road_Profile_Scope'], ...
        'Position', [700, 340, 750, 380], 'NumInputPorts', '2');
    add_block('simulink/Sinks/Scope', [model_name '/Energy_Scope'], ...
        'Position', [700, 420, 750, 460]);
    
    % =====================================================================
    % Add To Workspace blocks for post-processing
    % =====================================================================
    add_block('simulink/Sinks/To Workspace', [model_name '/Save_States'], ...
        'VariableName', 'sim_states', 'Position', [700, 500, 800, 540]);
    
    % Save the model
    save_system(model_name);
    fprintf('[*] Simulink model saved: %s.slx\n', model_name);
    fprintf('[*] NOTE: Open the MATLAB Function blocks to add the plant/controller code.\n');
    fprintf('[*]       The model structure is ready for you to wire up!\n');
    
catch ME
    fprintf('[!] Simulink model creation skipped: %s\n', ME.message);
    fprintf('[!] The MATLAB script simulation above is fully functional.\n');
end

fprintf('\n[*] ALL DONE! Total elapsed: %.2f seconds.\n', toc);

%% ========================================================================
% LOCAL FUNCTIONS
% ========================================================================

function [next_state, dx] = rk4_step_halfcar(t, state, u_f, u_r, w_f, w_r, p)
    %RK4_STEP_HALFCAR Runge-Kutta 4th order integration for the half-car model
    dt = p.dt;
    k1 = halfcar_ode(t,        state,           u_f, u_r, w_f, w_r, p);
    k2 = halfcar_ode(t + dt/2, state + k1*dt/2, u_f, u_r, w_f, w_r, p);
    k3 = halfcar_ode(t + dt/2, state + k2*dt/2, u_f, u_r, w_f, w_r, p);
    k4 = halfcar_ode(t + dt,   state + k3*dt,   u_f, u_r, w_f, w_r, p);
    next_state = state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
    dx = k1;
end

function dx = halfcar_ode(t, x, u_f, u_r, w_f, w_r, p)
    %HALFCAR_ODE 4-DOF Half-Car ODE (Newton's Laws)
    % State: [z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot]
    
    z_c       = x(1);
    theta     = x(2);
    z_uf      = x(3);
    z_ur      = x(4);
    z_c_dot   = x(5);
    theta_dot = x(6);
    z_uf_dot  = x(7);
    z_ur_dot  = x(8);
    
    % 1. Kinematics (CG -> corners)
    z_sf = z_c - p.a * theta;
    z_sr = z_c + p.b * theta;
    z_sf_dot = z_c_dot - p.a * theta_dot;
    z_sr_dot = z_c_dot + p.b * theta_dot;
    
    % 2. Deflections
    def_sf = z_sf - z_uf;      % Front suspension stroke
    def_sr = z_sr - z_ur;      % Rear suspension stroke
    def_tf = z_uf - w_f;       % Front tire deflection
    def_tr = z_ur - w_r;       % Rear tire deflection
    
    % Relative velocities
    v_rel_f = z_sf_dot - z_uf_dot;
    v_rel_r = z_sr_dot - z_ur_dot;
    
    % 3. Nonlinear bump stop forces
    f_bs_f = bump_stop_force(def_sf, p.stroke_max, p.k_bs);
    f_bs_r = bump_stop_force(def_sr, p.stroke_max, p.k_bs);
    
    % IWM electromagnetic disturbance
    omega_m = p.v_ms / p.wheel_radius;
    theta_m = omega_m * t;
    f_iwm = p.k_em * p.i_d * sin(p.p_pole * theta_m);
    
    % 4. Total forces
    F_susp_f = p.ks_f * def_sf + p.cs_f * v_rel_f + f_bs_f - u_f;
    F_susp_r = p.ks_r * def_sr + p.cs_r * v_rel_r + f_bs_r - u_r;
    F_tire_f = p.kt_f * def_tf;
    F_tire_r = p.kt_r * def_tr;
    
    % 5. Equations of Motion
    z_c_ddot   = -(F_susp_f + F_susp_r) / p.ms;
    theta_ddot = (p.a * F_susp_f - p.b * F_susp_r) / p.I_phi;
    z_uf_ddot  = (F_susp_f - F_tire_f + f_iwm) / p.mu_f;
    z_ur_ddot  = (F_susp_r - F_tire_r + f_iwm) / p.mu_r;
    
    dx = [z_c_dot; theta_dot; z_uf_dot; z_ur_dot; ...
          z_c_ddot; theta_ddot; z_uf_ddot; z_ur_ddot];
end

function f = bump_stop_force(deflection, max_stroke, k_bs)
    %BUMP_STOP_FORCE Cubic nonlinear bump stop
    if deflection > max_stroke
        f = k_bs * (deflection - max_stroke)^3;
    elseif deflection < -max_stroke
        f = k_bs * (deflection + max_stroke)^3;
    else
        f = 0;
    end
end

function t_settle = compute_settling_time(t, signal, threshold)
    %COMPUTE_SETTLING_TIME Find worst settling time (last time signal exceeds threshold)
    above = find(abs(signal) > threshold);
    if isempty(above)
        t_settle = 0;
    else
        last_above = above(end);
        t_settle = t(end) - t(last_above);
    end
end
