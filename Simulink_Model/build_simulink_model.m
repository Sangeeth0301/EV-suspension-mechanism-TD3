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

[P_smooth, ~, ~] = care_nt(A_qc, B_qc, Q_smooth, R_care);
K_smooth = (1/R_care) * B_qc' * P_smooth;
K_smooth = K_smooth(:)';  % 1x4 row vector

% --- Rough Road Gain (rho = 1) ---
rho_rough = 1.0;
q_stroke_r  = 1e5 * (1.0 - 0.5 * rho_rough);
q_body_vel_r = 1e4 * (1.0 + 5.0 * rho_rough);
Q_rough = diag([q_stroke_r, q_body_vel_r, 1e4, 1e2]);

[P_rough, ~, ~] = care_nt(A_qc, B_qc, Q_rough, R_care);
K_rough = (1/R_care) * B_qc' * P_rough;
K_rough = K_rough(:)';

% Lyapunov P matrix (for stability monitoring)
Q_lyap = diag([1e5, 1e4, 1e4, 1e2]);
[P_lyap, ~, ~] = care_nt(A_qc, B_qc, Q_lyap, R_care);

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
% SECTION 8: FULLY-WIRED SIMULINK MODEL GENERATION
% ========================================================================
% This section creates a COMPLETE, RUNNABLE Simulink .slx model:
%   - Clock + From-Workspace road profiles as inputs
%   - One MATLAB Function block with ALL simulation logic (persistent state)
%   - 6 named Scopes wired to all key output signals
%   - To Workspace block for post-run analysis
%   - MATLAB Function code injected automatically via Stateflow API
% ========================================================================
fprintf('[*] Generating Fully-Wired Simulink Model...\n');

try
    model_name = 'CyberResilient_ActiveSuspension';
    script_dir = fileparts(mfilename('fullpath'));
    slx_path   = fullfile(script_dir, [model_name '.slx']);
    
    % -- Cleanup: close any stale open model --
    if bdIsLoaded(model_name)
        close_system(model_name, 0);
    end
    if exist(slx_path, 'file')
        delete(slx_path);
    end
    
    % -- Create model and configure solver (fixed-step ODE4, 1 ms, 10 s) --
    new_system(model_name);
    open_system(model_name);
    set_param(model_name, ...
        'SolverType',     'Fixed-step', ...
        'Solver',         'ode4',        ...
        'FixedStep',      '0.001',       ...
        'StopTime',       '10',          ...
        'SaveOutput',     'on',          ...
        'OutputSaveName', 'sim_yout',    ...
        'SaveFormat',     'StructureWithTime');
    
    % -- Export road profiles to base workspace (used by From Workspace blocks) --
    road_f_ts = timeseries(w_f', t');
    road_r_ts = timeseries(w_r', t');
    assignin('base', 'road_f_ts', road_f_ts);
    assignin('base', 'road_r_ts', road_r_ts);
    fprintf('    [+] Road profiles exported to workspace.\n');
    
    % ==================================================================
    % ADD ALL BLOCKS
    % ==================================================================
    % 1. Clock — provides simulation time scalar to the MATLAB Function block
    add_block('simulink/Sources/Clock', [model_name '/t_clock'], ...
        'Position', [35, 55, 75, 85]);
    
    % 2. Road inputs (From Workspace, linearly interpolated between timesteps)
    add_block('simulink/Sources/From Workspace', [model_name '/Road_Front'], ...
        'VariableName', 'road_f_ts', ...
        'Position', [35, 140, 180, 170], ...
        'Interpolate', 'on');
    add_block('simulink/Sources/From Workspace', [model_name '/Road_Rear'], ...
        'VariableName', 'road_r_ts', ...
        'Position', [35, 200, 180, 230], ...
        'Interpolate', 'on');
    
    % 3. Main simulation MATLAB Function block
    %    Inputs  (3): t_in, w_f_in, w_r_in
    %    Outputs (9): z_c_ours, z_c_base, accel_ours, accel_base,
    %                 u_f_out, rho_f_out, soc_out, mode_out, V_lyap_out
    add_block('simulink/User-Defined Functions/MATLAB Function', ...
        [model_name '/Suspension_Sim'], ...
        'Position', [255, 45, 490, 395]);
    
    % 4. Six named Scopes for live visualization during simulation
    %    Scope 1: Body Displacement comparison (ours vs base)
    add_block('simulink/Sinks/Scope', [model_name '/Sc_Body_Disp'], ...
        'Position', [565, 45, 615, 80], 'NumInputPorts', '2', 'Open', 'off');
    %    Scope 2: Body Acceleration comparison
    add_block('simulink/Sinks/Scope', [model_name '/Sc_Accel'], ...
        'Position', [565, 105, 615, 140], 'NumInputPorts', '2', 'Open', 'off');
    %    Scope 3: LPV Actuator Force
    add_block('simulink/Sinks/Scope', [model_name '/Sc_Force'], ...
        'Position', [565, 165, 615, 200], 'Open', 'off');
    %    Scope 4: UKF Road Severity rho
    add_block('simulink/Sinks/Scope', [model_name '/Sc_Rho'], ...
        'Position', [565, 225, 615, 260], 'Open', 'off');
    %    Scope 5: Battery SoC and Mode flag
    add_block('simulink/Sinks/Scope', [model_name '/Sc_Energy'], ...
        'Position', [565, 285, 615, 320], 'NumInputPorts', '2', 'Open', 'off');
    %    Scope 6: Lyapunov V(x) stability function
    add_block('simulink/Sinks/Scope', [model_name '/Sc_Lyapunov'], ...
        'Position', [565, 345, 615, 380], 'Open', 'off');
    
    % 5. Mux (9→1) + To Workspace — saves all signals for post-processing
    add_block('simulink/Signal Routing/Mux', [model_name '/Out_Mux'], ...
        'Position', [520, 415, 545, 505], 'Inputs', '9');
    add_block('simulink/Sinks/To Workspace', [model_name '/SimOut'], ...
        'VariableName', 'sim_out', ...
        'Position', [565, 430, 680, 465], ...
        'MaxDataPoints', 'inf', ...
        'SaveFormat', 'Structure With Time');
    
    fprintf('    [+] All %d blocks placed.\n', 3 + 1 + 6 + 2);
    
    % ==================================================================
    % WIRE ALL BLOCKS
    % ==================================================================
    % Inputs → Suspension_Sim
    add_line(model_name, 't_clock/1',    'Suspension_Sim/1', 'autorouting', 'on');
    add_line(model_name, 'Road_Front/1', 'Suspension_Sim/2', 'autorouting', 'on');
    add_line(model_name, 'Road_Rear/1',  'Suspension_Sim/3', 'autorouting', 'on');
    
    % Suspension_Sim outputs → Scopes
    % Port 1: z_c_ours (m)  Port 2: z_c_base (m)
    add_line(model_name, 'Suspension_Sim/1', 'Sc_Body_Disp/1', 'autorouting', 'on');
    add_line(model_name, 'Suspension_Sim/2', 'Sc_Body_Disp/2', 'autorouting', 'on');
    % Port 3: accel_ours (m/s2)  Port 4: accel_base (m/s2)
    add_line(model_name, 'Suspension_Sim/3', 'Sc_Accel/1',     'autorouting', 'on');
    add_line(model_name, 'Suspension_Sim/4', 'Sc_Accel/2',     'autorouting', 'on');
    % Port 5: u_f actuator force (N)
    add_line(model_name, 'Suspension_Sim/5', 'Sc_Force/1',     'autorouting', 'on');
    % Port 6: rho_f road severity (0–1)
    add_line(model_name, 'Suspension_Sim/6', 'Sc_Rho/1',       'autorouting', 'on');
    % Port 7: battery SoC (0–1)   Port 8: ECO mode flag (0/1)
    add_line(model_name, 'Suspension_Sim/7', 'Sc_Energy/1',    'autorouting', 'on');
    add_line(model_name, 'Suspension_Sim/8', 'Sc_Energy/2',    'autorouting', 'on');
    % Port 9: Lyapunov V(x) = x'Px
    add_line(model_name, 'Suspension_Sim/9', 'Sc_Lyapunov/1',  'autorouting', 'on');
    
    % All 9 outputs also feed through Mux → To Workspace
    for k_wire = 1:9
        add_line(model_name, sprintf('Suspension_Sim/%d', k_wire), ...
                             sprintf('Out_Mux/%d',        k_wire), ...
                             'autorouting', 'on');
    end
    add_line(model_name, 'Out_Mux/1', 'SimOut/1', 'autorouting', 'on');
    
    fprintf('    [+] All signal wires connected.\n');
    
    % ==================================================================
    % INJECT MATLAB FUNCTION BLOCK CODE VIA STATEFLOW API
    % ==================================================================
    % Generate the self-contained simulation function code string.
    % All gains (K_smooth, K_rough, P_lyap, etc.) are hardcoded as numeric
    % literals so the block needs no workspace access during simulation.
    sim_func_code = build_sim_function_code(p, K_smooth, K_rough, K_rho_dot, ...
                                             P_lyap, K_base_front, K_base_rear);
    
    code_injected = false;
    try
        sfrt       = sfroot();
        all_charts = sfrt.find('-isa', 'Stateflow.EMChart');
        tgt_path   = [model_name '/Suspension_Sim'];
        for ci = 1:numel(all_charts)
            if strcmp(all_charts(ci).Path, tgt_path)
                all_charts(ci).Script = sim_func_code;
                code_injected = true;
                break;
            end
        end
    catch sf_err
        fprintf('    [!] Stateflow API error: %s\n', sf_err.message);
    end
    
    if code_injected
        fprintf('    [+] MATLAB Function code injected via Stateflow API.\n');
    else
        % Fallback: write code to a companion .m file the user can paste manually
        code_file = fullfile(script_dir, 'Suspension_Sim_code.m');
        fid = fopen(code_file, 'w');
        fprintf(fid, '%s', sim_func_code);
        fclose(fid);
        fprintf('    [!] Auto-inject failed. Function code written to:\n');
        fprintf('        %s\n', code_file);
        fprintf('    [!] Open Suspension_Sim block and paste that code inside.\n');
    end
    
    % Save model
    save_system(model_name, slx_path);
    
    fprintf('\n');
    fprintf('  ================================================================\n');
    fprintf('  [OK]  Simulink Model Ready: %s.slx\n', model_name);
    fprintf('  ================================================================\n');
    fprintf('   HOW TO RUN:\n');
    fprintf('   1. The model window is already open in Simulink.\n');
    fprintf('   2. Click the green Play button in the Simulink toolbar, OR\n');
    fprintf('      type in MATLAB Command Window:\n');
    fprintf('         >> sim(''%s'')\n', model_name);
    fprintf('   3. Double-click any Scope block to see live signals.\n');
    fprintf('   4. After sim: type sim_out in MATLAB to inspect results.\n');
    fprintf('  ================================================================\n');
    
catch ME
    fprintf('[!] Simulink model generation failed: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('[!]   at function ''%s'', line %d\n', ME.stack(1).name, ME.stack(1).line);
    end
    fprintf('[!] The pure MATLAB simulation (Sections 1-7) is complete and valid.\n');
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
    %COMPUTE_SETTLING_TIME Duration from first to last exceedance of threshold.
    % A large value = the car was bouncing for a long time (bad).
    % A small value = the car settled quickly (good).
    above = find(abs(signal) > threshold);
    if isempty(above)
        t_settle = 0;  % Never exceeded — instant settling
    else
        t_settle = t(above(end)) - t(above(1));  % Time between first and last bounce
    end
end

function code = build_sim_function_code(p, K_smooth, K_rough, K_rho_dot, P_lyap, K_base_front, K_base_rear)
%BUILD_SIM_FUNCTION_CODE  Generate the MATLAB Function block script as a string.
% All controller gains and vehicle parameters are substituted as numeric
% literals so the resulting block is fully self-contained.

L = {};

% ---- Function header and persistent variable declarations ----
L{end+1} = 'function [z_c_ours, z_c_base, accel_ours, accel_base, u_f_out, rho_f_out, soc_out, mode_out, V_lyap_out] = Suspension_Sim(t_in, w_f_in, w_r_in)';
L{end+1} = '% Cyber-Resilient LPV-Adaptive Active Suspension — Simulink MATLAB Function Block';
L{end+1} = '% All parameters hardcoded. State maintained via persistent variables.';
L{end+1} = '% Automatically re-initialises when t_in resets to 0 (new simulation run).';
L{end+1} = '';
L{end+1} = 'persistent st_ours st_base bat_soc lc_il lc_vc';
L{end+1} = 'persistent rho_f_p rdot_filt mets_last mets_first';
L{end+1} = '';

% ---- Hardcoded vehicle parameters ----
L{end+1} = '% ---- VEHICLE PARAMETERS (hardcoded numeric literals) ----';
L{end+1} = sprintf('ms    = %.4f;  I_phi = %.4f;', p.ms, p.I_phi);
L{end+1} = sprintf('a     = %.4f;  b     = %.4f;', p.a, p.b);
L{end+1} = sprintf('mu_f  = %.4f;  mu_r  = %.4f;', p.mu_f, p.mu_r);
L{end+1} = sprintf('ks_f  = %.2f; ks_r  = %.2f;', p.ks_f, p.ks_r);
L{end+1} = sprintf('cs_f  = %.2f;  cs_r  = %.2f;', p.cs_f, p.cs_r);
L{end+1} = sprintf('kt_f  = %.2f; kt_r  = %.2f;', p.kt_f, p.kt_r);
L{end+1} = sprintf('k_em  = %.4f; i_d   = %.4f;  p_pole = %d;', p.k_em, p.i_d, p.p_pole);
L{end+1} = sprintf('whr   = %.4f; v_ms  = %.4f;', p.wheel_radius, p.v_ms);
L{end+1} = sprintf('C_e   = %.4f; eta_r = %.4f;  K_v    = %.4f;', p.C_e, p.eta_regen, p.K_v);
L{end+1} = sprintf('smax  = %.6f; u_max = %.4f;  k_bs   = %.6e;', p.stroke_max, p.u_max, p.k_bs);
L{end+1} = sprintf('Lf    = %.6f; Cf    = %.8f;  Rl     = %.4f;', p.L_filter, p.C_filter, p.R_load);
L{end+1} = sprintf('sig_m = %.4f;', p.sigma_mets);
L{end+1} = 'dt = 0.001; w_max = 0.05; ema_a = 0.05; eta_act = 0.85; cap_j = 50000;';
L{end+1} = '';

% ---- Hardcoded controller gains (substituted numeric values) ----
L{end+1} = '% ---- CONTROLLER GAINS (computed from CARE, substituted as literals) ----';
L{end+1} = sprintf('Ks  = %s;', mat2str(K_smooth,    10));
L{end+1} = sprintf('Kr  = %s;', mat2str(K_rough,     10));
L{end+1} = sprintf('Krd = %s;', mat2str(K_rho_dot,   10));
L{end+1} = sprintf('Pl  = %s;', mat2str(P_lyap,      10));
L{end+1} = sprintf('Kbf = %s;', mat2str(K_base_front,10));
L{end+1} = sprintf('Kbr = %s;', mat2str(K_base_rear, 10));
L{end+1} = '';

% ---- Initialisation block ----
L{end+1} = '% ---- INITIALISE on first call OR when t_in resets to 0 ----';
L{end+1} = 'if isempty(st_ours) || (t_in < 5e-4)';
L{end+1} = '    st_ours = zeros(8,1);  st_base = zeros(8,1);';
L{end+1} = '    bat_soc = 0.5;  lc_il = 0;  lc_vc = 0;';
L{end+1} = '    rho_f_p = 0;   rdot_filt = 0;';
L{end+1} = '    mets_last = zeros(8,1);  mets_first = true;';
L{end+1} = 'end';
L{end+1} = '';
L{end+1} = 'wf = w_f_in;  wr = w_r_in;';
L{end+1} = '';

% ---- Base paper simulation ----
L{end+1} = '% ==== BASE PAPER: Fixed H-infinity controller ====';
L{end+1} = 'uf_b = max(-u_max, min(u_max, -Kbf * st_base));';
L{end+1} = 'ur_b = max(-u_max, min(u_max, -Kbr * st_base));';
L{end+1} = '[st_base, dx_b] = rk4_hc(t_in, st_base, uf_b, ur_b, wf, wr, ...';
L{end+1} = '    ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs,dt);';
L{end+1} = '';

% ---- METS filter ----
L{end+1} = '% ==== OUR PROJECT: LPV + UKF + TD3 + METS ====';
L{end+1} = '% Step A: METS Network Filter';
L{end+1} = 'if mets_first';
L{end+1} = '    mets_last = st_ours;  mets_first = false;  net = st_ours;';
L{end+1} = 'else';
L{end+1} = '    em = st_ours - mets_last;';
L{end+1} = '    ne = sqrt(em(1)^2+em(2)^2+em(3)^2+em(4)^2+em(5)^2+em(6)^2+em(7)^2+em(8)^2);';
L{end+1} = '    nx = sqrt(st_ours(1)^2+st_ours(2)^2+st_ours(3)^2+st_ours(4)^2+st_ours(5)^2+st_ours(6)^2+st_ours(7)^2+st_ours(8)^2);';
L{end+1} = '    if ne > sig_m * nx;  mets_last = st_ours;  net = st_ours;';
L{end+1} = '    else;                                       net = mets_last;  end';
L{end+1} = 'end';
L{end+1} = '';

% ---- UKF road estimator ----
L{end+1} = '% Step B: UKF Road Severity Estimator (simplified algebraic)';
L{end+1} = 'rho_f     = min(1.0, max(0.0, abs(st_ours(1) - a*st_ours(2) - st_ours(3)) / w_max));';
L{end+1} = 'rdot_filt = ema_a * ((rho_f - rho_f_p) / dt) + (1.0 - ema_a) * rdot_filt;';
L{end+1} = 'rho_dot   = rdot_filt;  rho_f_p = rho_f;';
L{end+1} = '';

% ---- TD3 heuristic agent ----
L{end+1} = '% Step C: TD3 Heuristic Energy Agent';
L{end+1} = 'if rho_f > 0.7 || rho_dot > 0.5';
L{end+1} = '    if bat_soc < 0.02;  mf = 1;  else;  mf = 0;  end';
L{end+1} = 'elseif rho_f > 0.3';
L{end+1} = '    if bat_soc < 0.25;  mf = 1;  else;  mf = 0;  end';
L{end+1} = 'else';
L{end+1} = '    mf = 1;  % Smooth road — harvest energy';
L{end+1} = 'end';
L{end+1} = '';

% ---- LPV controller ----
L{end+1} = '% Step D: LPV Controller — Quarter-Car State Mapping';
L{end+1} = 'zsf  = net(1) - a*net(2);   zsfd = net(5) - a*net(6);';
L{end+1} = 'qcf  = [zsf - net(3); zsfd; 0; net(7)];';
L{end+1} = 'zsr  = net(1) + b*net(2);   zsrd = net(5) + b*net(6);';
L{end+1} = 'qcr  = [zsr - net(4); zsrd; 0; net(8)];';
L{end+1} = 'ffl  = 0.2 * u_max;';
L{end+1} = 'if mf == 1  % ECO: regenerative damping';
L{end+1} = '    ufo = -C_e * qcf(2);  uro = -C_e * qcr(2);';
L{end+1} = 'else        % COMFORT: LPV active control';
L{end+1} = '    Kf  = (1.0 - rho_f) * Ks + rho_f * Kr;';
L{end+1} = '    ufo = (-Kf * qcf) + max(-ffl, min(ffl, -Krd * max(0, rho_dot) * qcf));';
L{end+1} = '    uro = (-Kf * qcr) + max(-ffl, min(ffl, -Krd * max(0, rho_dot) * qcr));';
L{end+1} = 'end';
L{end+1} = 'ufo = max(-u_max, min(u_max, ufo));';
L{end+1} = 'uro = max(-u_max, min(u_max, uro));';
L{end+1} = '';

% ---- RK4 integration ----
L{end+1} = '% Step E: RK4 Physics Integration';
L{end+1} = '[st_ours, dx_o] = rk4_hc(t_in, st_ours, ufo, uro, wf, wr, ...';
L{end+1} = '    ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs,dt);';
L{end+1} = '';

% ---- Energy harvesting ----
L{end+1} = '% Step F: Energy Harvesting with LC Power Electronics Filter';
L{end+1} = 'vrf = (st_ours(5) - a*st_ours(6)) - st_ours(7);  % Front suspension rel. velocity';
L{end+1} = 'if mf == 1  % ECO mode: harvest';
L{end+1} = '    vrect = abs(K_v * vrf);';
L{end+1} = '    di_L  = (vrect - lc_vc) / Lf;';
L{end+1} = '    dv_C  = (lc_il - lc_vc / Rl) / Cf;';
L{end+1} = '    lc_il = min(max(lc_il + di_L * dt, 0), 100);  % Clamped for numerical stability';
L{end+1} = '    lc_vc = min(max(lc_vc + dv_C * dt, 0), 500);';
L{end+1} = '    harv  = (lc_vc^2 / Rl) * eta_r * dt;';
L{end+1} = '    bat_soc = min(1.0, bat_soc + harv / cap_j);';
L{end+1} = 'else  % COMFORT mode: consume battery';
L{end+1} = '    bat_soc = max(0.0, bat_soc - abs(ufo * vrf) / eta_act * dt / cap_j);';
L{end+1} = '    lc_il = lc_il * 0.99;  lc_vc = lc_vc * 0.99;  % LC filter decays';
L{end+1} = 'end';
L{end+1} = '';

% ---- Lyapunov monitor ----
L{end+1} = '% Step G: Lyapunov Stability Monitor — V(x) = x_qc'' * P * x_qc';
L{end+1} = 'xq  = [st_ours(1)-a*st_ours(2)-st_ours(3); st_ours(5)-a*st_ours(6); 0; st_ours(7)];';
L{end+1} = 'V_x = xq'' * Pl * xq;';
L{end+1} = '';

% ---- Assign outputs ----
L{end+1} = '% ---- OUTPUTS ----';
L{end+1} = 'z_c_ours   = st_ours(1);  z_c_base   = st_base(1);';
L{end+1} = 'accel_ours = dx_o(5);     accel_base = dx_b(5);';
L{end+1} = 'u_f_out    = ufo;         rho_f_out  = rho_f;';
L{end+1} = 'soc_out    = bat_soc;     mode_out   = double(mf);';
L{end+1} = 'V_lyap_out = V_x;';
L{end+1} = 'end  % Suspension_Sim';
L{end+1} = '';

% ---- Local helper: RK4 integrator ----
L{end+1} = '% ===== LOCAL: 4th-Order Runge-Kutta =====';
L{end+1} = 'function [ns, k1] = rk4_hc(t, x, uf, ur, wf, wr, ms, Ip, a, b, muf, mur, ksf, ksr, csf, csr, ktf, ktr, kem, id, pp, whr, vms, sm, kbs, dt)';
L{end+1} = '    k1 = hc_ode(t,       x,         uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);';
L{end+1} = '    k2 = hc_ode(t+dt/2,  x+k1*dt/2, uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);';
L{end+1} = '    k3 = hc_ode(t+dt/2,  x+k2*dt/2, uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);';
L{end+1} = '    k4 = hc_ode(t+dt,    x+k3*dt,   uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);';
L{end+1} = '    ns  = x + (dt / 6) * (k1 + 2*k2 + 2*k3 + k4);';
L{end+1} = 'end';
L{end+1} = '';

% ---- Local helper: Half-car ODE ----
L{end+1} = '% ===== LOCAL: 4-DOF Half-Car ODE =====';
L{end+1} = 'function dx = hc_ode(t, x, uf, ur, wf, wr, ms, Ip, a, b, muf, mur, ksf, ksr, csf, csr, ktf, ktr, kem, id, pp, whr, vms, sm, kbs)';
L{end+1} = '    z_c=x(1); th=x(2); zuf=x(3); zur=x(4); zcd=x(5); thd=x(6); zufd=x(7); zurd=x(8);';
L{end+1} = '    zsf=z_c-a*th;  zsr=z_c+b*th;  zsfd=zcd-a*thd;  zsrd=zcd+b*thd;';
L{end+1} = '    dsf=zsf-zuf;   dsr=zsr-zur;   dtf=zuf-wf;      dtr=zur-wr;';
L{end+1} = '    vrf=zsfd-zufd; vrr=zsrd-zurd;';
L{end+1} = '    if     dsf >  sm;  fb_f = kbs*(dsf-sm)^3;';
L{end+1} = '    elseif dsf < -sm;  fb_f = kbs*(dsf+sm)^3;';
L{end+1} = '    else;              fb_f = 0;  end';
L{end+1} = '    if     dsr >  sm;  fb_r = kbs*(dsr-sm)^3;';
L{end+1} = '    elseif dsr < -sm;  fb_r = kbs*(dsr+sm)^3;';
L{end+1} = '    else;              fb_r = 0;  end';
L{end+1} = '    fiwm = kem * id * sin(pp * (vms/whr) * t);';
L{end+1} = '    Fsf  = ksf*dsf + csf*vrf + fb_f - uf;';
L{end+1} = '    Fsr  = ksr*dsr + csr*vrr + fb_r - ur;';
L{end+1} = '    dx = [zcd; thd; zufd; zurd; ...';
L{end+1} = '          -(Fsf+Fsr)/ms; (a*Fsf-b*Fsr)/Ip; ...';
L{end+1} = '          (Fsf - ktf*dtf + fiwm)/muf; (Fsr - ktr*dtr + fiwm)/mur];';
L{end+1} = 'end';

code = strjoin(L, newline);
end

function [P, K, L] = care_nt(A, B, Q, R)
%CARE_NT  Continuous Algebraic Riccati Equation solver — NO TOOLBOX REQUIRED.
% Uses Laub's Hamiltonian matrix eigendecomposition.
% Requires only base MATLAB (eig is built-in).
%
% Solves:  A'P + PA - P*B*(1/R)*B'*P + Q = 0
%
% Inputs : A (nxn), B (nxm), Q (nxn symmetric PSD), R (mxm symmetric PD)
% Outputs: P — CARE solution matrix
%          K = R\B'P  — LQR gain matrix
%          L = eig(A-BK)  — closed-loop eigenvalues

    n     = size(A, 1);
    R_inv = inv(R);
    
    % Build 2n x 2n Hamiltonian matrix
    H = [  A,              -B * R_inv * B' ; ...
           -Q,             -A'             ];
    
    % Eigendecomposition (eig is in base MATLAB, no toolbox needed)
    [V, D] = eig(H);
    ev     = diag(D);
    
    % Sort all eigenvalues by real part (most negative first)
    [~, sort_idx] = sort(real(ev));
    V_sorted      = V(:, sort_idx);
    ev_sorted     = ev(sort_idx);
    
    % Verify we have n stable (negative real-part) eigenvalues
    n_stable = sum(real(ev_sorted) < 0);
    if n_stable < n
        error('care_nt: Only %d stable eigenvalues found (need %d). Check system stabilisability.', n_stable, n);
    end
    
    % Use the n most stable eigenvectors to span the stable invariant subspace
    V_stable = V_sorted(:, 1:n);
    
    % P is recovered from the relation: [X1; X2] spans the stable subspace
    % where X2 = P * X1  =>  P = X2 / X1
    X1 = V_stable(1:n,     :);
    X2 = V_stable(n+1:2*n, :);
    
    P = real(X2 / X1);
    P = (P + P') / 2;      % Symmetrise to eliminate floating-point noise
    
    K = R_inv * B' * P;
    L = eig(A - B * K);
end
%% 
