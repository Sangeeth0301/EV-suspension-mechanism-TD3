function [soc_out, lc_voltage, p_harv_w, v_bus] = Energy_SoC(x_state, uf, mode_in)
% Faraday Coil -> Rectifier -> LC Filter -> Boost Converter -> Battery
%
% This block implements the "Energy Recovery" branch of the system block
% diagram. The key modelling point is that the harvesting coil and the
% actuator are ONE physical device, so generation and reaction force are
% locked together by the same electromechanical constant k_e:
%
%   Faraday (generation) :  e = k_e * v_rel
%   Circuit             :  i = e / (R_coil + R_load)
%   Lenz  (reaction)    :  F = k_e * i
%   =>  C_e = k_e^2 / (R_coil + R_load)          <-- DERIVED, not chosen
%
% Consequently energy conservation is structural rather than clamped:
%
%   P_mech = F * v_rel = C_e * v_rel^2           (absorbed from suspension)
%   P_elec = i^2 * R_load                        (delivered to the load)
%   P_elec / P_mech = R_load / (R_coil + R_load) < 1     ALWAYS
%
% Power electronics chain:
%   Rectifier : full bridge, two diode forward drops
%   LC filter : output smoothing, integrated with semi-implicit (symplectic)
%               Euler. Forward Euler is unconditionally unstable for this
%               lightly damped LC (zeta ~ 0.07) and was the reason the old
%               version needed hard clamps on i_L and v_C.
%   Boost     : V_out = V_in / (1 - D); steps voltage up but can only scale
%               POWER by its efficiency, never increase it.
%
% Battery SoC by coulomb counting on the conditioned power.
% -------------------------------------------------------------------------
persistent bat_soc lc_il lc_vc

a = 1.10;

% --- Electromagnetic transducer ---
k_e     = 50.0;                        % V*s/m  (equivalently N/A)
R_coil  = 2.0;                         % Ohm, coil winding resistance
R_load  = 10.0;                        % Ohm, harvesting load
C_e     = k_e^2 / (R_coil + R_load);   % = 208.33 N*s/m  (DERIVED)

% --- Power electronics ---
V_diode  = 0.7;      % diode forward drop (V), two in the bridge path
Lf       = 0.0100;   % filter inductance (H)
Cf       = 0.004700; % filter capacitance + bank (F)
D_bst    = 0.60;     % boost duty cycle
eta_rect = 0.97;     % rectifier efficiency
eta_bst  = 0.92;     % boost converter efficiency
eta_act  = 0.85;     % actuator efficiency when motoring
cap_j    = 50000;    % battery buffer used for SoC scaling (J)
dt       = 0.001;

if isempty(bat_soc); bat_soc = 0.5; lc_il = 0; lc_vc = 0; end

% Relative suspension velocity at the front corner (across the damper)
vrf = (x_state(5) - a*x_state(6)) - x_state(7);

p_harv_w = 0;

if mode_in == 1   % ---- ECO: coil generates ----
    e_emf  = k_e * vrf;                              % Faraday
    v_rect = max(0, abs(e_emf) - 2*V_diode);         % full-bridge rectifier
    i_coil = v_rect / (R_coil + R_load);             % circuit current
    p_mech = C_e * vrf * vrf;                        % Lenz reaction * velocity
    p_elec = i_coil * i_coil * R_load;               % delivered to load

    % LC output filter, semi-implicit Euler (uses UPDATED current in dv_C)
    di_L  = (v_rect - lc_vc - lc_il*R_coil) / Lf;
    lc_il = lc_il + di_L * dt;
    dv_C  = (lc_il - lc_vc / R_load) / Cf;
    lc_vc = lc_vc + dv_C * dt;

    % Boost converter: voltage up, power scaled by efficiency only
    p_harv_w = min(p_elec * eta_rect * eta_bst, p_mech);
    bat_soc  = min(1.0, bat_soc + p_harv_w * dt / cap_j);

else              % ---- COMFORT: coil motors, draws from battery ----
    bat_soc = max(0.0, bat_soc - abs(uf*vrf)/eta_act * dt / cap_j);

    % Filter discharges through load / winding resistance
    lc_vc = lc_vc * exp(-dt / (R_load * Cf));
    lc_il = lc_il * exp(-dt * R_coil / Lf);
end

soc_out    = bat_soc;
lc_voltage = lc_vc;
v_bus      = lc_vc / (1 - D_bst);
end
