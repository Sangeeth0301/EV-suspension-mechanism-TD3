function [soc_out, lc_voltage] = Energy_SoC(x_state, uf, mode_in)
% LC Filter Power Electronics + Regenerative Energy Harvesting
% Tracks battery State-of-Charge and LC filter capacitor voltage.
persistent bat_soc lc_il lc_vc

a=1.10; K_v=50.0; eta_r=0.65; eta_act=0.85;
Lf=0.0100; Cf=0.004700; Rl=10.0; cap_j=50000; dt=0.001;
N_t=10.0; % Lightweight Transformer/Boost step-up ratio

if isempty(bat_soc); bat_soc=0.5; lc_il=0; lc_vc=0; end

% Relative suspension velocity (front corner)
vrf = (x_state(5) - a*x_state(6)) - x_state(7);

if mode_in == 1  % ECO mode: harvest energy via back-EMF
    v_rect = abs(K_v * vrf) * N_t; % Voltage is boosted by transformer
    di_L   = (v_rect - lc_vc) / Lf;
    dv_C   = (lc_il - lc_vc/Rl) / Cf;
    lc_il  = min(max(lc_il + di_L*dt, 0), 100);  % Clamped
    lc_vc  = min(max(lc_vc + dv_C*dt, 0), 500);
    bat_soc = min(1.0, bat_soc + (lc_vc^2/Rl)*eta_r*dt / cap_j);
else             % COMFORT mode: active actuator consumes battery
    bat_soc = max(0.0, bat_soc - abs(uf*vrf)/eta_act*dt / cap_j);
    lc_il = lc_il*0.99;  lc_vc = lc_vc*0.99;  % LC filter decays
end

soc_out    = bat_soc;
lc_voltage = lc_vc;
end