function [uf, ur, rho_f, mode_flag] = LPV_UKF_Ctrl(x_state, soc_in)
% LPV Gain-Scheduled Controller + Kalman Road-Severity Estimator + Supervisor
% Block 2 in the signal chain. Uses x_state from Unit Delay (1 ms behind plant).
%
% CONTENTS
%   Step A  Event-triggered network filter (bandwidth reduction)
%   Step B  Kalman road-severity estimator, WITH RATE LIMIT on rho
%   Step C  Energy supervisor, WITH HYSTERESIS + MINIMUM DWELL TIME
%   Step D  Polytopic LPV gain scheduling + feedforward
%   Step E  Actuator saturation
%
% STABILITY NOTE
%   The scheduled gain K(rho) = (1-rho)*K_s + rho*K_r makes the closed loop a
%   convex combination of two vertex closed loops. Both vertices are Hurwitz
%   (verified in build_simulink_model.m). Frozen-parameter stability does NOT
%   by itself imply stability under fast rho variation, so rho is explicitly
%   RATE LIMITED here. This is what makes the "rate-bounded LPV" description
%   accurate rather than aspirational.
% -------------------------------------------------------------------------
persistent rho_f_p rdot_filt mets_last mets_first kf_P mode_prev dwell_cnt

a = 1.10;  b = 1.50;
u_max = 6000;  sig_m = 0.10;
w_max = 0.05;  ema_a = 0.05;  dt = 0.001;

% --- Electromagnetic transducer: C_e is DERIVED, not chosen ---
k_e    = 50.0;  R_coil = 2.0;  R_load = 10.0;
C_e    = k_e^2 / (R_coil + R_load);   % = 208.33 N*s/m

% --- LPV vertex gains (CARE, R = 1e-4; identical to lpv_controller.py) ---
Ks  = [54965.745388 22300.813833 -14066.707352 -189.473296];
Kr  = [35141.321022 53959.884711 -49124.631637  390.336744];
Krd = [0 500 0 0];

% --- Supervisor switching guards ---
dwell_n = 100;      % 100 ms minimum hold between mode changes
rho_hi  = 0.35;     % enter COMFORT above this
rho_lo  = 0.25;     % return to ECO below this
rho_rate_max = 4.0; % max |d(rho)/dt| per second (rate bound for LPV validity)

% --- Kalman estimator tuning ---
kf_Q = 1e-3;        % process noise on the road-severity random walk
kf_R = 5e-2;        % measurement noise on the algebraic severity proxy

if isempty(rho_f_p)
    rho_f_p = 0; rdot_filt = 0; mets_last = zeros(8,1); mets_first = true;
    kf_P = 1.0; mode_prev = 1; dwell_cnt = dwell_n;
end

% ---- Step A: Event-Triggered Network Filter ----
if mets_first
    net = x_state; mets_last = x_state; mets_first = false;
else
    em = x_state - mets_last;
    nx = sqrt(sum(x_state.^2)); ne = sqrt(sum(em.^2));
    if ne > sig_m*nx; mets_last = x_state; net = x_state;
    else;             net = mets_last; end
end

% ---- Step B: Kalman Road-Severity Estimator (scalar, with rate limit) ----
% Measurement: normalised suspension deflection at the front corner.
rho_meas = min(1, max(0, abs(x_state(1) - a*x_state(2) - x_state(3)) / w_max));

kf_P     = kf_P + kf_Q;               % predict (random-walk model)
kf_K     = kf_P / (kf_P + kf_R);      % Kalman gain
rho_raw  = rho_f_p + kf_K*(rho_meas - rho_f_p);
kf_P     = (1 - kf_K) * kf_P;         % covariance update

% Rate limit keeps the scheduling parameter inside its assumed bound.
d_max    = rho_rate_max * dt;
rho_f    = min(rho_f_p + d_max, max(rho_f_p - d_max, rho_raw));
rho_f    = min(1, max(0, rho_f));

rdot_filt = ema_a*((rho_f - rho_f_p)/dt) + (1-ema_a)*rdot_filt;
rho_dot   = rdot_filt;  rho_f_p = rho_f;

% ---- Step C: Energy Supervisor (hysteresis + minimum dwell time) ----
mode_req = mode_prev;
if mode_prev == 1                                  % in ECO
    if rho_f > rho_hi || rho_dot > 0.5; mode_req = 0; end
else                                               % in COMFORT
    if rho_f < rho_lo && rho_dot < 0.5; mode_req = 1; end
end
if soc_in < 0.02
    mode_req = 1;                                  % flat battery: must harvest
elseif soc_in < 0.25 && rho_f < 0.7
    mode_req = 1;
end
if mode_req ~= mode_prev && dwell_cnt >= dwell_n
    mf = mode_req; dwell_cnt = 0;
else
    mf = mode_prev; dwell_cnt = dwell_cnt + 1;
end
mode_prev = mf;

% ---- Step D: LPV Controller — Polytopic Gain Scheduling ----
zsf = net(1)-a*net(2); zsfd = net(5)-a*net(6); qcf = [zsf-net(3); zsfd; 0; net(7)];
zsr = net(1)+b*net(2); zsrd = net(5)+b*net(6); qcr = [zsr-net(4); zsrd; 0; net(8)];
ffl = 0.2*u_max;
if mf == 1
    % ECO: Lenz reaction force of the harvesting coil. Acts ACROSS the
    % damper, so it is driven by RELATIVE velocity. Using the absolute body
    % velocity here would be a skyhook force, which a two-terminal
    % electromagnetic damper cannot produce, and would not match the v_rel
    % used in the energy balance in Energy_SoC.
    uf = -C_e * (qcf(2) - net(7));
    ur = -C_e * (qcr(2) - net(8));
else
    Kf = (1-rho_f)*Ks + rho_f*Kr;   % polytopic interpolation
    uf = (-Kf*qcf) + max(-ffl, min(ffl, -Krd*max(0,rho_dot)*qcf));
    ur = (-Kf*qcr) + max(-ffl, min(ffl, -Krd*max(0,rho_dot)*qcr));
end

% ---- Step E: Actuator Saturation ----
uf = max(-u_max, min(u_max, uf));
ur = max(-u_max, min(u_max, ur));
mode_flag = double(mf);
end
