function [uf, ur, rho_f, mode_flag] = LPV_UKF_Ctrl(x_state, soc_in)
% LPV-Adaptive H-inf Controller with UKF Road Estimator + TD3 Heuristic
% Block 2 in the signal chain. Uses x_state from Unit Delay (1ms behind plant).
%
% MATHEMATICAL EQUATIONS:
% 1. UKF Algebraic Proxy (Road Severity):
%    rho = |z_c - a*theta - z_uf| / w_max
%    rho_dot = EMA( d(rho)/dt ) = alpha * (d_rho) + (1 - alpha) * rho_dot_prev
% 2. TD3 Heuristic Energy Supervisor (Logic distilled from DRL):
%    if rho > 0.7 OR rho_dot > 0.5 -> COMFORT MODE (Unless SoC < 0.02)
%    if rho < 0.3 -> ECO MODE (Passive Damping for energy harvesting)
% 3. LPV H-infinity Gain Scheduling:
%    K(rho) = (1 - rho)*K_s + rho*K_r
%    u_active = -K(rho)*x_state - K_rd * max(0, rho_dot) * x_state
% 4. Saturation Limiter:
%    u_final = clamp(u_active, -u_max, u_max)
%
% -------------------------------------------------------------------------
persistent rho_f_p rdot_filt mets_last mets_first

a=1.10; b=1.50;
C_e=1500; u_max=6000; sig_m=0.10;
w_max=0.05; ema_a=0.05; dt=0.001;
Ks  = [2591.260282 2545.019204 -4409.593492 -22.6009354];
Kr  = [1339.079606 6792.410633 -15288.11257 39.41237591];
Krd = [0 500 0 0];

% Initialise persistent variables (auto-reset at start of each sim run)
if isempty(rho_f_p)
    rho_f_p=0; rdot_filt=0; mets_last=zeros(8,1); mets_first=true;
end

% ---- Step A: METS Network Filter ----
if mets_first
    net=x_state; mets_last=x_state; mets_first=false;
else
    em=x_state-mets_last;
    nx=sqrt(sum(x_state.^2)); ne=sqrt(sum(em.^2));
    if ne>sig_m*nx; mets_last=x_state; net=x_state;
    else;           net=mets_last; end
end

% ---- Step B: UKF Road Estimator (algebraic proxy) ----
rho_f     = min(1, max(0, abs(x_state(1)-a*x_state(2)-x_state(3)) / w_max));
rdot_filt = ema_a*((rho_f-rho_f_p)/dt) + (1-ema_a)*rdot_filt;
rho_dot   = rdot_filt;  rho_f_p = rho_f;

% ---- Step C: TD3 Heuristic Energy Agent ----
if rho_f>0.7 || rho_dot>0.5
    if soc_in<0.02; mf=1; else; mf=0; end   % COMFORT if SoC critical
elseif rho_f>0.3
    if soc_in<0.25; mf=1; else; mf=0; end
else
    mf=1;  % Smooth road — harvest energy (ECO)
end

% ---- Step D: LPV Controller — Polytopic Gain Scheduling ----
zsf=net(1)-a*net(2); zsfd=net(5)-a*net(6); qcf=[zsf-net(3);zsfd;0;net(7)];
zsr=net(1)+b*net(2); zsrd=net(5)+b*net(6); qcr=[zsr-net(4);zsrd;0;net(8)];
ffl = 0.2*u_max;
if mf==1  % ECO: passive regenerative damping
    uf=-C_e*qcf(2);  ur=-C_e*qcr(2);
else       % COMFORT: blended LPV H-inf + feedforward
    Kf = (1-rho_f)*Ks + rho_f*Kr;   % Polytopic interpolation
    uf = (-Kf*qcf) + max(-ffl, min(ffl, -Krd*max(0,rho_dot)*qcf));
    ur = (-Kf*qcr) + max(-ffl, min(ffl, -Krd*max(0,rho_dot)*qcr));
end
uf=max(-u_max,min(u_max,uf));  ur=max(-u_max,min(u_max,ur));
mode_flag = double(mf);
end