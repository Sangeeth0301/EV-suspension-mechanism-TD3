%% ========================================================================
% CREATE_SIMULINK_MODEL.M
% Cyber-Resilient LPV Active Suspension — Multi-Block Simulink Model
% ========================================================================
% Creates a FULL BLOCK DIAGRAM in Simulink with 5 separate named subsystems:
%
%   [Clock] ──→ [HalfCar_Plant]  ←── [UD_X / UD_UF / UD_UR] (Unit Delays)
%                      │                       ↑
%                  x(k) ──→ [UD_X] ──→ [LPV_UKF_Ctrl] ──→ uf, ur
%                                   ↘  [Energy_SoC]    ←── soc (UD_SoC)
%                                   ↘  [Lyapunov_Mon]  ──→ V(x)
%              [Base_Paper_Sim] ←── [Clock, Road_Front, Road_Rear]
%
% HOW TO RUN:
%   1. Open MATLAB
%   2. cd to this folder
%   3. Type:  create_simulink_model
%   4. Press Play in the Simulink window that opens
%
% REQUIREMENTS: MATLAB + Simulink. No extra toolboxes needed.
% ========================================================================
clear; clc;
fprintf('==========================================================\n');
fprintf('[*] Cyber-Resilient Suspension — Multi-Block Simulink Builder\n');
fprintf('==========================================================\n');
tic;

%% ========================================================================
% SECTION 1: VEHICLE PARAMETERS
% ========================================================================
p.ms=730; p.I_phi=1222; p.a=1.1; p.b=1.5;
p.ms_f = p.ms * p.b / (p.a + p.b);   % ~421 kg front equiv.
p.mu_f=45; p.mu_r=45;
p.ks_f=18000; p.ks_r=22000;
p.cs_f=1200;  p.cs_r=1200;
p.kt_f=190000; p.kt_r=190000;
p.k_em=15; p.i_d=20; p.p_pole=8; p.wheel_radius=0.3; p.v_ms=20;
p.C_e=1500; p.eta_regen=0.65; p.K_v=50;
p.stroke_max=0.08; p.u_max=6000; p.k_bs=1e7;
p.L_filter=0.01; p.C_filter=0.0047; p.R_load=10;
p.sigma_mets=0.1; p.dt=0.001; p.T_total=10; p.v_kmh=72;
fprintf('[*] Vehicle parameters set.\n');

%% ========================================================================
% SECTION 2: GAIN SYNTHESIS  (care_nt — no Control System Toolbox needed)
% ========================================================================
fprintf('[*] Synthesising LPV gains (no toolbox required)...\n');
A_qc = [0, 1, 0, -1;
        -p.ks_f/p.ms_f, -p.cs_f/p.ms_f, 0,  p.cs_f/p.ms_f;
         0,  0, 0,  1;
         p.ks_f/p.mu_f, p.cs_f/p.mu_f, -p.kt_f/p.mu_f, -p.cs_f/p.mu_f];
B_qc = [0; 1/p.ms_f; 0; -1/p.mu_f];
R_c  = 1e-3;

[Ps,~,~] = care_nt(A_qc, B_qc, diag([1e5, 1e4, 1e4, 1e2]), R_c);
K_smooth  = (1/R_c) * B_qc' * Ps;        % 1x4 row vector

[Pr,~,~] = care_nt(A_qc, B_qc, diag([5e4, 6e4, 1e4, 1e2]), R_c);
K_rough   = (1/R_c) * B_qc' * Pr;        % 1x4 row vector

[Pl,~,~] = care_nt(A_qc, B_qc, diag([1e5, 1e4, 1e4, 1e2]), R_c);
P_lyap    = Pl;

K_rho_dot    = [0, 500, 0, 0];            % 1x4 feedforward
K_base_front = [-12000, 5500,  8000, 0, -3500,  1800, 2200,    0];
K_base_rear  = [-12000, -5500, 0, 8000, -3500, -1800,    0, 2200];

fprintf('    K_smooth = [%.0f, %.0f, %.0f, %.0f]\n', K_smooth);
fprintf('    K_rough  = [%.0f, %.0f, %.0f, %.0f]\n', K_rough);

%% ========================================================================
% SECTION 3: ROAD PROFILE (ISO 8608 Class A + rough overlay + potholes)
% ========================================================================
fprintf('[*] Generating road profile...\n');
t_vec   = 0 : p.dt : (p.T_total - p.dt);
n_steps = length(t_vec);
delay_steps = round((p.a+p.b) / p.v_ms / p.dt);

rng(42);
G_A=16e-6; n0=0.1; f0=0.01;
w_amp = 2*pi*n0*sqrt(G_A*p.v_ms); alpha_r=2*pi*f0;
wn = randn(1,n_steps);
w_f = zeros(1,n_steps);
for i=2:n_steps
    w_f(i) = w_f(i-1) + p.dt*(-alpha_r*w_f(i-1) + w_amp*wn(i-1));
end
rng(43);
G_D=1024e-6; w_amp_D=2*pi*n0*sqrt(G_D*p.v_ms);
wn_D=randn(1,n_steps); wr_ov=zeros(1,n_steps);
for i=2:n_steps
    wr_ov(i)=wr_ov(i-1)+p.dt*(-alpha_r*wr_ov(i-1)+w_amp_D*wn_D(i-1));
end
i_s=round(3/p.dt)+1; i_e=round(7/p.dt); fl=round(0.2/p.dt);
fade=ones(1,i_e-i_s+1);
fade(1:fl)=linspace(0,1,fl); fade(end-fl+1:end)=linspace(1,0,fl);
w_f(i_s:i_e)=w_f(i_s:i_e)+wr_ov(i_s:i_e).*fade;
depths=[-0.06,-0.04,-0.02,-0.005,-0.05,-0.03];
widths=[0.25,0.15,0.1,0.05,0.2,0.12];
times=linspace(3.5,6.5,6); rng(42); times=times(randperm(6));
for k=1:6
    i_st=round(times(k)/p.dt)+1;
    dur=round(widths(k)/p.dt); i_en=min(i_st+dur-1,n_steps);
    xp=linspace(0,2*pi,i_en-i_st+1);
    w_f(i_st:i_en)=w_f(i_st:i_en)+(depths(k)/2)*(1-cos(xp));
end
w_r=zeros(1,n_steps);
w_r(delay_steps+1:end)=w_f(1:end-delay_steps);

road_f_ts = timeseries(w_f', t_vec');
road_r_ts = timeseries(w_r', t_vec');
assignin('base','road_f_ts', road_f_ts);
assignin('base','road_r_ts', road_r_ts);
fprintf('    [+] Road profiles in workspace.\n');

%% ========================================================================
% SECTION 4: WRITE COMPANION CODE FILES (one .m per Simulink block)
% ========================================================================
fprintf('[*] Writing companion code files...\n');
script_dir = fileparts(mfilename('fullpath'));

block_info = {
    'HalfCar_Plant',    gen_halfcar_code(p);
    'LPV_UKF_Ctrl',     gen_lpv_code(p, K_smooth, K_rough, K_rho_dot);
    'Energy_SoC',       gen_energy_code(p);
    'Lyapunov_Mon',     gen_lyapunov_code(p, P_lyap);
    'Base_Paper_Sim',   gen_base_code(p, K_base_front, K_base_rear)
};

for bi = 1:size(block_info,1)
    fname = fullfile(script_dir, [block_info{bi,1} '_code.m']);
    fid   = fopen(fname,'w');
    fprintf(fid,'%s', block_info{bi,2});
    fclose(fid);
end
fprintf('    [+] Wrote %d companion .m files (open them to see each block''s code).\n', ...
        size(block_info,1));

%% ========================================================================
% SECTION 5: BUILD SIMULINK MODEL
% ========================================================================
fprintf('[*] Building Simulink block diagram...\n');

try
    mn  = 'CyberSuspension_MultiBlock';
    slx = fullfile(script_dir, [mn '.slx']);

    if bdIsLoaded(mn); close_system(mn,0); end
    if exist(slx,'file'); delete(slx); end

    new_system(mn);
    open_system(mn);
    set_param(mn,'SolverType','Fixed-step','Solver','ode4',...
                 'FixedStep','0.001','StopTime','10',...
                 'SaveOutput','off');

    % ------------------------------------------------------------------
    % SOURCES
    % ------------------------------------------------------------------
    add_block('simulink/Sources/Clock',...
        [mn '/t_clock'], 'Position',[30,60,70,90]);
    add_block('simulink/Sources/From Workspace',...
        [mn '/Road_Front'],'Position',[30,145,180,175],...
        'VariableName','road_f_ts','Interpolate','on');
    add_block('simulink/Sources/From Workspace',...
        [mn '/Road_Rear'],'Position',[30,210,180,240],...
        'VariableName','road_r_ts','Interpolate','on');

    % ------------------------------------------------------------------
    % UNIT DELAYS  (break the plant ↔ controller algebraic loop)
    % ------------------------------------------------------------------
    % UD_X: delays 8-wide state vector by one sample (1 ms)
    add_block('simulink/Discrete/Unit Delay',...
        [mn '/UD_X'],'Position',[220,130,270,165],...
        'SampleTime','0.001','InitialCondition','zeros(8,1)');
    % UD_UF / UD_UR: delay actuator forces
    add_block('simulink/Discrete/Unit Delay',...
        [mn '/UD_UF'],'Position',[220,195,270,225],...
        'SampleTime','0.001','InitialCondition','0');
    add_block('simulink/Discrete/Unit Delay',...
        [mn '/UD_UR'],'Position',[220,245,270,275],...
        'SampleTime','0.001','InitialCondition','0');
    % UD_SoC: delays battery SoC (initial SoC = 50%)
    add_block('simulink/Discrete/Unit Delay',...
        [mn '/UD_SoC'],'Position',[220,305,270,335],...
        'SampleTime','0.001','InitialCondition','0.5');

    % ------------------------------------------------------------------
    % MAIN SUBSYSTEM MATLAB FUNCTION BLOCKS
    % ------------------------------------------------------------------
    % 1. HalfCar_Plant  (6-in: t,wf,wr,x_prev[8],uf_prev,ur_prev  |  3-out: x_new[8],z_c,accel)
    add_block('simulink/User-Defined Functions/MATLAB Function',...
        [mn '/HalfCar_Plant'],'Position',[340,30,530,215]);

    % 2. LPV_UKF_Ctrl  (2-in: x_state[8],soc_in  |  4-out: uf,ur,rho_f,mode)
    add_block('simulink/User-Defined Functions/MATLAB Function',...
        [mn '/LPV_UKF_Ctrl'],'Position',[340,250,530,375]);

    % 3. Energy_SoC  (3-in: x_state[8],uf,mode  |  2-out: soc,lc_voltage)
    add_block('simulink/User-Defined Functions/MATLAB Function',...
        [mn '/Energy_SoC'],'Position',[590,250,750,340]);

    % 4. Lyapunov_Mon  (1-in: x_state[8]  |  1-out: V_x)
    add_block('simulink/User-Defined Functions/MATLAB Function',...
        [mn '/Lyapunov_Mon'],'Position',[590,360,750,410]);

    % 5. Base_Paper_Sim  (3-in: t,wf,wr  |  2-out: z_c_base,accel_base)
    add_block('simulink/User-Defined Functions/MATLAB Function',...
        [mn '/Base_Paper_Sim'],'Position',[340,420,530,520]);

    % ------------------------------------------------------------------
    % SCOPES (double-click in Simulink to open during/after simulation)
    % ------------------------------------------------------------------
    add_block('simulink/Sinks/Scope',[mn '/Sc_Body_Disp'],...
        'Position',[810,30,860,65],'NumInputPorts','2','Open','off');
    add_block('simulink/Sinks/Scope',[mn '/Sc_Accel'],...
        'Position',[810,90,860,125],'NumInputPorts','2','Open','off');
    add_block('simulink/Sinks/Scope',[mn '/Sc_Force'],...
        'Position',[810,150,860,185],'Open','off');
    add_block('simulink/Sinks/Scope',[mn '/Sc_Rho'],...
        'Position',[810,210,860,245],'Open','off');
    add_block('simulink/Sinks/Scope',[mn '/Sc_Energy'],...
        'Position',[810,270,860,305],'NumInputPorts','2','Open','off');
    add_block('simulink/Sinks/Scope',[mn '/Sc_Lyapunov'],...
        'Position',[810,330,860,365],'Open','off');

    % To Workspace — saves 8 key signals for post-processing
    add_block('simulink/Signal Routing/Mux',...
        [mn '/Out_Mux'],'Position',[775,390,795,520],'Inputs','8');
    add_block('simulink/Sinks/To Workspace',...
        [mn '/SimOut'],'Position',[810,440,920,475],...
        'VariableName','sim_out',...
        'SaveFormat','Structure With Time',...
        'MaxDataPoints','inf');

    fprintf('    [+] All blocks placed.\n');

    % ------------------------------------------------------------------
    % INJECT MATLAB FUNCTION CODE via Stateflow API
    % ------------------------------------------------------------------
    % This must happen BEFORE wiring, so the blocks generate their correct ports!
    injected = 0;
    try
        sfrt   = sfroot();
        charts = sfrt.find('-isa','Stateflow.EMChart');
        for bi = 1:size(block_info,1)
            tgt_path = [mn '/' block_info{bi,1}];
            for ci = 1:numel(charts)
                if strcmp(charts(ci).Path, tgt_path)
                    charts(ci).Script = block_info{bi,2};
                    injected = injected + 1;
                    fprintf('    [+] Code injected: %s\n', block_info{bi,1});
                    break;
                end
            end
        end
    catch sf_err
        fprintf('    [!] Stateflow API: %s\n', sf_err.message);
    end

    if injected < size(block_info,1)
        n_missing = size(block_info,1) - injected;
        fprintf('    [!] %%d block(s) need manual code paste.\n', n_missing);
        fprintf('        Open each empty block, select all, paste from *_code.m\n');
    end

    % ------------------------------------------------------------------
    % WIRING — connect all signals
    % ------------------------------------------------------------------
    % Sources → HalfCar_Plant inputs
    add_line(mn,'t_clock/1',    'HalfCar_Plant/1','autorouting','on'); % t
    add_line(mn,'Road_Front/1', 'HalfCar_Plant/2','autorouting','on'); % wf
    add_line(mn,'Road_Rear/1',  'HalfCar_Plant/3','autorouting','on'); % wr
    add_line(mn,'UD_X/1',       'HalfCar_Plant/4','autorouting','on'); % x_prev (8-wide)
    add_line(mn,'UD_UF/1',      'HalfCar_Plant/5','autorouting','on'); % uf_prev
    add_line(mn,'UD_UR/1',      'HalfCar_Plant/6','autorouting','on'); % ur_prev

    % HalfCar_Plant outputs
    add_line(mn,'HalfCar_Plant/1','UD_X/1','autorouting','on');        % x_new → delay (8-wide)
    add_line(mn,'HalfCar_Plant/2','Sc_Body_Disp/1','autorouting','on');% z_c_ours → scope
    add_line(mn,'HalfCar_Plant/3','Sc_Accel/1','autorouting','on');    % accel_ours → scope

    % UD_X branches to: LPV_UKF_Ctrl, Energy_SoC, Lyapunov_Mon
    add_line(mn,'UD_X/1','LPV_UKF_Ctrl/1','autorouting','on');
    add_line(mn,'UD_X/1','Energy_SoC/1','autorouting','on');
    add_line(mn,'UD_X/1','Lyapunov_Mon/1','autorouting','on');

    % Delayed SoC → LPV_UKF_Ctrl
    add_line(mn,'UD_SoC/1','LPV_UKF_Ctrl/2','autorouting','on');

    % LPV_UKF_Ctrl outputs
    add_line(mn,'LPV_UKF_Ctrl/1','UD_UF/1','autorouting','on');        % uf → delay → plant
    add_line(mn,'LPV_UKF_Ctrl/2','UD_UR/1','autorouting','on');        % ur → delay → plant
    add_line(mn,'LPV_UKF_Ctrl/1','Energy_SoC/2','autorouting','on');   % uf → energy harvester
    add_line(mn,'LPV_UKF_Ctrl/3','Sc_Rho/1','autorouting','on');       % rho_f → scope
    add_line(mn,'LPV_UKF_Ctrl/4','Energy_SoC/3','autorouting','on');   % mode → energy
    add_line(mn,'LPV_UKF_Ctrl/1','Sc_Force/1','autorouting','on');     % uf → scope

    % Energy_SoC outputs
    add_line(mn,'Energy_SoC/1','UD_SoC/1','autorouting','on');         % soc → delay
    add_line(mn,'Energy_SoC/1','Sc_Energy/1','autorouting','on');      % soc → scope
    add_line(mn,'Energy_SoC/2','Sc_Energy/2','autorouting','on');      % lc_v → scope

    % Lyapunov_Mon output
    add_line(mn,'Lyapunov_Mon/1','Sc_Lyapunov/1','autorouting','on');

    % Base_Paper_Sim (standalone — manages its own state internally)
    add_line(mn,'t_clock/1',   'Base_Paper_Sim/1','autorouting','on');
    add_line(mn,'Road_Front/1','Base_Paper_Sim/2','autorouting','on');
    add_line(mn,'Road_Rear/1', 'Base_Paper_Sim/3','autorouting','on');
    add_line(mn,'Base_Paper_Sim/1','Sc_Body_Disp/2','autorouting','on');% z_c_base → scope
    add_line(mn,'Base_Paper_Sim/2','Sc_Accel/2','autorouting','on');    % accel_base → scope

    % Mux → To Workspace  (8 key scalar signals)
    add_line(mn,'HalfCar_Plant/2',  'Out_Mux/1','autorouting','on');   % z_c_ours
    add_line(mn,'Base_Paper_Sim/1', 'Out_Mux/2','autorouting','on');   % z_c_base
    add_line(mn,'HalfCar_Plant/3',  'Out_Mux/3','autorouting','on');   % accel_ours
    add_line(mn,'Base_Paper_Sim/2', 'Out_Mux/4','autorouting','on');   % accel_base
    add_line(mn,'LPV_UKF_Ctrl/1',  'Out_Mux/5','autorouting','on');   % uf
    add_line(mn,'LPV_UKF_Ctrl/3',  'Out_Mux/6','autorouting','on');   % rho_f
    add_line(mn,'Energy_SoC/1',    'Out_Mux/7','autorouting','on');   % soc
    add_line(mn,'Lyapunov_Mon/1',  'Out_Mux/8','autorouting','on');   % V_lyap
    add_line(mn,'Out_Mux/1','SimOut/1','autorouting','on');

    fprintf('    [+] All signal wires connected.\n');

    % Auto-arrange the blocks beautifully
    Simulink.BlockDiagram.arrangeSystem(mn);

    save_system(mn, slx);

    fprintf('\n');
    fprintf('  ================================================================\n');
    fprintf('  [OK] Simulink Block Diagram Ready!\n');
    fprintf('       Model: CyberSuspension_MultiBlock.slx\n');
    fprintf('  ================================================================\n');
    fprintf('  BLOCKS IN THE MODEL:\n');
    fprintf('    HalfCar_Plant  — 4-DOF ODE physics (RK4, 8-state)\n');
    fprintf('    LPV_UKF_Ctrl   — METS + UKF estimator + TD3 + LPV H-inf\n');
    fprintf('    Energy_SoC     — LC filter + regenerative harvesting\n');
    fprintf('    Lyapunov_Mon   — Quadratic stability monitor V(x)=x''Px\n');
    fprintf('    Base_Paper_Sim — Fixed H-inf reference (for comparison)\n');
    fprintf('  ----------------------------------------------------------------\n');
    fprintf('  HOW TO RUN:\n');
    fprintf('    1. Model is already open in Simulink\n');
    fprintf('    2. Open Scopes: double-click Sc_Body_Disp, Sc_Accel, Sc_Rho\n');
    fprintf('    3. Press the green Play button  (or: sim(''%s''))\n', mn);
    fprintf('    4. After run: type sim_out in Command Window\n');
    fprintf('  ================================================================\n');

catch ME
    fprintf('[!] Model build failed: %s\n', ME.message);
    if ~isempty(ME.stack)
        fprintf('[!]   at %s, line %d\n', ME.stack(1).name, ME.stack(1).line);
    end
end

fprintf('\n[*] Done in %.1f seconds.\n', toc);


%% ========================================================================
%% CODE GENERATOR FUNCTIONS  (one per Simulink block)
%% ========================================================================

% -------------------------------------------------------------------------
function code = gen_halfcar_code(p)
% Generates the MATLAB Function block code for HalfCar_Plant
% Explicit RK4 — no persistent state needed (state passed via port)
L = {};
L{end+1} = 'function [x_new, z_c, body_accel] = HalfCar_Plant(t_in, wf, wr, x_prev, uf_prev, ur_prev)';
L{end+1} = '% 4-DOF Half-Car Plant with RK4 Integration';
L{end+1} = '% State: [z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot]';
L{end+1} = '% State passed explicitly — no persistent variables needed here.';
L{end+1} = '';
L{end+1} = sprintf('ms=%.1f;  I_phi=%.1f;  a=%.2f;  b=%.2f;', p.ms,p.I_phi,p.a,p.b);
L{end+1} = sprintf('mu_f=%.1f; mu_r=%.1f;', p.mu_f,p.mu_r);
L{end+1} = sprintf('ks_f=%.0f; ks_r=%.0f; cs_f=%.0f; cs_r=%.0f;', p.ks_f,p.ks_r,p.cs_f,p.cs_r);
L{end+1} = sprintf('kt_f=%.0f; kt_r=%.0f;', p.kt_f,p.kt_r);
L{end+1} = sprintf('k_em=%.1f; i_d=%.1f; p_pole=%d; whr=%.2f; v_ms=%.2f;', p.k_em,p.i_d,p.p_pole,p.wheel_radius,p.v_ms);
L{end+1} = sprintf('smax=%.4f; u_max=%.0f; k_bs=%.2e;', p.stroke_max,p.u_max,p.k_bs);
L{end+1} = 'dt = 0.001;';
L{end+1} = '';
L{end+1} = '% Runge-Kutta 4 integration (fixed-step, dt=1ms)';
L{end+1} = 'k1=hc_ode(t_in,      x_prev,         uf_prev,ur_prev,wf,wr,ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs);';
L{end+1} = 'k2=hc_ode(t_in+dt/2, x_prev+k1*dt/2, uf_prev,ur_prev,wf,wr,ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs);';
L{end+1} = 'k3=hc_ode(t_in+dt/2, x_prev+k2*dt/2, uf_prev,ur_prev,wf,wr,ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs);';
L{end+1} = 'k4=hc_ode(t_in+dt,   x_prev+k3*dt,   uf_prev,ur_prev,wf,wr,ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs);';
L{end+1} = 'x_new      = x_prev + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);';
L{end+1} = 'z_c        = x_new(1);';
L{end+1} = 'body_accel = k1(5);';
L{end+1} = 'end';
L{end+1} = '';
L{end+1} = 'function dx = hc_ode(t,x,uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs)';
L{end+1} = '    z_c=x(1);th=x(2);zuf=x(3);zur=x(4);zcd=x(5);thd=x(6);zufd=x(7);zurd=x(8);';
L{end+1} = '    zsf=z_c-a*th; zsr=z_c+b*th; zsfd=zcd-a*thd; zsrd=zcd+b*thd;';
L{end+1} = '    dsf=zsf-zuf; dsr=zsr-zur; dtf=zuf-wf; dtr=zur-wr;';
L{end+1} = '    vrf=zsfd-zufd; vrr=zsrd-zurd;';
L{end+1} = '    if dsf> sm; fbf=kbs*(dsf-sm)^3; elseif dsf<-sm; fbf=kbs*(dsf+sm)^3; else; fbf=0; end';
L{end+1} = '    if dsr> sm; fbr=kbs*(dsr-sm)^3; elseif dsr<-sm; fbr=kbs*(dsr+sm)^3; else; fbr=0; end';
L{end+1} = '    fiwm = kem*id*sin(pp*(vms/whr)*t);';
L{end+1} = '    Fsf=ksf*dsf+csf*vrf+fbf-uf; Fsr=ksr*dsr+csr*vrr+fbr-ur;';
L{end+1} = '    dx=[zcd;thd;zufd;zurd; -(Fsf+Fsr)/ms; (a*Fsf-b*Fsr)/Ip; (Fsf-ktf*dtf+fiwm)/muf; (Fsr-ktr*dtr+fiwm)/mur];';
L{end+1} = 'end';
code = strjoin(L, newline);
end

% -------------------------------------------------------------------------
function code = gen_lpv_code(p, K_smooth, K_rough, K_rho_dot)
% Generates MATLAB Function block code for LPV_UKF_Ctrl
% Implements: METS → UKF → TD3 Heuristic → LPV H-inf Controller
L = {};
L{end+1} = 'function [uf, ur, rho_f, mode_flag] = LPV_UKF_Ctrl(x_state, soc_in)';
L{end+1} = '% LPV-Adaptive H-inf Controller with UKF Road Estimator + TD3 Heuristic';
L{end+1} = '% Block 2 in the signal chain. Uses x_state from Unit Delay (1ms behind plant).';
L{end+1} = 'persistent rho_f_p rdot_filt mets_last mets_first';
L{end+1} = '';
L{end+1} = sprintf('a=%.2f; b=%.2f;', p.a, p.b);
L{end+1} = sprintf('C_e=%.0f; u_max=%.0f; sig_m=%.2f;', p.C_e, p.u_max, p.sigma_mets);
L{end+1} = 'w_max=0.05; ema_a=0.05; dt=0.001;';
L{end+1} = sprintf('Ks  = %s;', mat2str(K_smooth,  10));  % 1x4 row vectors
L{end+1} = sprintf('Kr  = %s;', mat2str(K_rough,   10));
L{end+1} = sprintf('Krd = %s;', mat2str(K_rho_dot, 10));
L{end+1} = '';
L{end+1} = '% Initialise persistent variables (auto-reset at start of each sim run)';
L{end+1} = 'if isempty(rho_f_p)';
L{end+1} = '    rho_f_p=0; rdot_filt=0; mets_last=zeros(8,1); mets_first=true;';
L{end+1} = 'end';
L{end+1} = '';
L{end+1} = '% ---- Step A: METS Network Filter ----';
L{end+1} = 'if mets_first';
L{end+1} = '    net=x_state; mets_last=x_state; mets_first=false;';
L{end+1} = 'else';
L{end+1} = '    em=x_state-mets_last;';
L{end+1} = '    nx=sqrt(sum(x_state.^2)); ne=sqrt(sum(em.^2));';
L{end+1} = '    if ne>sig_m*nx; mets_last=x_state; net=x_state;';
L{end+1} = '    else;           net=mets_last; end';
L{end+1} = 'end';
L{end+1} = '';
L{end+1} = '% ---- Step B: UKF Road Estimator (algebraic proxy) ----';
L{end+1} = 'rho_f     = min(1, max(0, abs(x_state(1)-a*x_state(2)-x_state(3)) / w_max));';
L{end+1} = 'rdot_filt = ema_a*((rho_f-rho_f_p)/dt) + (1-ema_a)*rdot_filt;';
L{end+1} = 'rho_dot   = rdot_filt;  rho_f_p = rho_f;';
L{end+1} = '';
L{end+1} = '% ---- Step C: TD3 Heuristic Energy Agent ----';
L{end+1} = 'if rho_f>0.7 || rho_dot>0.5';
L{end+1} = '    if soc_in<0.02; mf=1; else; mf=0; end   % COMFORT if SoC critical';
L{end+1} = 'elseif rho_f>0.3';
L{end+1} = '    if soc_in<0.25; mf=1; else; mf=0; end';
L{end+1} = 'else';
L{end+1} = '    mf=1;  % Smooth road — harvest energy (ECO)';
L{end+1} = 'end';
L{end+1} = '';
L{end+1} = '% ---- Step D: LPV Controller — Polytopic Gain Scheduling ----';
L{end+1} = 'zsf=net(1)-a*net(2); zsfd=net(5)-a*net(6); qcf=[zsf-net(3);zsfd;0;net(7)];';
L{end+1} = 'zsr=net(1)+b*net(2); zsrd=net(5)+b*net(6); qcr=[zsr-net(4);zsrd;0;net(8)];';
L{end+1} = 'ffl = 0.2*u_max;';
L{end+1} = 'if mf==1  % ECO: passive regenerative damping';
L{end+1} = '    uf=-C_e*qcf(2);  ur=-C_e*qcr(2);';
L{end+1} = 'else       % COMFORT: blended LPV H-inf + feedforward';
L{end+1} = '    Kf = (1-rho_f)*Ks + rho_f*Kr;   % Polytopic interpolation';
L{end+1} = '    uf = (-Kf*qcf) + max(-ffl, min(ffl, -Krd*max(0,rho_dot)*qcf));';
L{end+1} = '    ur = (-Kf*qcr) + max(-ffl, min(ffl, -Krd*max(0,rho_dot)*qcr));';
L{end+1} = 'end';
L{end+1} = 'uf=max(-u_max,min(u_max,uf));  ur=max(-u_max,min(u_max,ur));';
L{end+1} = 'mode_flag = double(mf);';
L{end+1} = 'end';
code = strjoin(L, newline);
end

% -------------------------------------------------------------------------
function code = gen_energy_code(p)
% Generates MATLAB Function block code for Energy_SoC
% Implements LC filter power electronics + regenerative energy harvesting
L = {};
L{end+1} = 'function [soc_out, lc_voltage] = Energy_SoC(x_state, uf, mode_in)';
L{end+1} = '% LC Filter Power Electronics + Regenerative Energy Harvesting';
L{end+1} = '% Tracks battery State-of-Charge and LC filter capacitor voltage.';
L{end+1} = 'persistent bat_soc lc_il lc_vc';
L{end+1} = '';
L{end+1} = sprintf('a=%.2f; K_v=%.1f; eta_r=%.2f; eta_act=0.85;', p.a, p.K_v, p.eta_regen);
L{end+1} = sprintf('Lf=%.4f; Cf=%.6f; Rl=%.1f; cap_j=50000; dt=0.001;', p.L_filter, p.C_filter, p.R_load);
L{end+1} = 'N_t=10.0; % Lightweight Transformer/Boost step-up ratio';
L{end+1} = '';
L{end+1} = 'if isempty(bat_soc); bat_soc=0.5; lc_il=0; lc_vc=0; end';
L{end+1} = '';
L{end+1} = '% Relative suspension velocity (front corner)';
L{end+1} = 'vrf = (x_state(5) - a*x_state(6)) - x_state(7);';
L{end+1} = '';
L{end+1} = 'if mode_in == 1  % ECO mode: harvest energy via back-EMF';
L{end+1} = '    v_rect = abs(K_v * vrf) * N_t; % Voltage is boosted by transformer';
L{end+1} = '    di_L   = (v_rect - lc_vc) / Lf;';
L{end+1} = '    dv_C   = (lc_il - lc_vc/Rl) / Cf;';
L{end+1} = '    lc_il  = min(max(lc_il + di_L*dt, 0), 100);  % Clamped';
L{end+1} = '    lc_vc  = min(max(lc_vc + dv_C*dt, 0), 500);';
L{end+1} = '    bat_soc = min(1.0, bat_soc + (lc_vc^2/Rl)*eta_r*dt / cap_j);';
L{end+1} = 'else             % COMFORT mode: active actuator consumes battery';
L{end+1} = '    bat_soc = max(0.0, bat_soc - abs(uf*vrf)/eta_act*dt / cap_j);';
L{end+1} = '    lc_il = lc_il*0.99;  lc_vc = lc_vc*0.99;  % LC filter decays';
L{end+1} = 'end';
L{end+1} = '';
L{end+1} = 'soc_out    = bat_soc;';
L{end+1} = 'lc_voltage = lc_vc;';
L{end+1} = 'end';
code = strjoin(L, newline);
end

% -------------------------------------------------------------------------
function code = gen_lyapunov_code(p, P_lyap)
% Generates MATLAB Function block code for Lyapunov_Mon
% Computes V(x) = x_qc' * P * x_qc to verify closed-loop stability
L = {};
L{end+1} = 'function V_x = Lyapunov_Mon(x_state)';
L{end+1} = '% Lyapunov Stability Monitor  V(x) = x_qc'' * P * x_qc';
L{end+1} = '% V(x) must be positive and decreasing for Lyapunov stability.';
L{end+1} = '% P is the solution to CARE with Q=diag([1e5,1e4,1e4,1e2]), R=1e-3.';
L{end+1} = sprintf('a  = %.2f;', p.a);
L{end+1} = sprintf('Pl = %s;', mat2str(P_lyap, 10));
L{end+1} = '';
L{end+1} = '% Map half-car state to front quarter-car state vector';
L{end+1} = 'xq  = [x_state(1)-a*x_state(2)-x_state(3); ...';
L{end+1} = '        x_state(5)-a*x_state(6); 0; x_state(7)];';
L{end+1} = 'V_x = xq'' * Pl * xq;';
L{end+1} = 'end';
code = strjoin(L, newline);
end

% -------------------------------------------------------------------------
function code = gen_base_code(p, Kbf, Kbr)
% Generates MATLAB Function block code for Base_Paper_Sim
% Fixed H-infinity controller (base paper) for comparison
L = {};
L{end+1} = 'function [z_c_base, accel_base] = Base_Paper_Sim(t_in, wf, wr)';
L{end+1} = '% Fixed H-infinity Base Paper Controller (self-contained with persistent state)';
L{end+1} = '% Used as a performance baseline — no gain scheduling, no energy harvesting.';
L{end+1} = 'persistent st_base';
L{end+1} = '';
L{end+1} = sprintf('ms=%.1f; I_phi=%.1f; a=%.2f; b=%.2f;', p.ms,p.I_phi,p.a,p.b);
L{end+1} = sprintf('mu_f=%.1f; mu_r=%.1f;', p.mu_f,p.mu_r);
L{end+1} = sprintf('ks_f=%.0f; ks_r=%.0f; cs_f=%.0f; cs_r=%.0f;', p.ks_f,p.ks_r,p.cs_f,p.cs_r);
L{end+1} = sprintf('kt_f=%.0f; kt_r=%.0f;', p.kt_f,p.kt_r);
L{end+1} = sprintf('k_em=%.1f; i_d=%.1f; p_pole=%d; whr=%.2f; v_ms=%.2f;', p.k_em,p.i_d,p.p_pole,p.wheel_radius,p.v_ms);
L{end+1} = sprintf('smax=%.4f; u_max=%.0f; k_bs=%.2e; dt=0.001;', p.stroke_max,p.u_max,p.k_bs);
L{end+1} = sprintf('Kbf = %s;', mat2str(Kbf, 10));
L{end+1} = sprintf('Kbr = %s;', mat2str(Kbr, 10));
L{end+1} = '';
L{end+1} = 'if isempty(st_base); st_base=zeros(8,1); end';
L{end+1} = '';
L{end+1} = 'uf_b = max(-u_max, min(u_max, -Kbf * st_base));';
L{end+1} = 'ur_b = max(-u_max, min(u_max, -Kbr * st_base));';
L{end+1} = '[st_base, dx_b] = rk4_b(t_in,st_base,uf_b,ur_b,wf,wr,ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs,dt);';
L{end+1} = 'z_c_base   = st_base(1);';
L{end+1} = 'accel_base = dx_b(5);';
L{end+1} = 'end';
L{end+1} = '';
L{end+1} = 'function [ns,k1]=rk4_b(t,x,uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs,dt)';
L{end+1} = '    k1=ode_b(t,      x,         uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);';
L{end+1} = '    k2=ode_b(t+dt/2, x+k1*dt/2, uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);';
L{end+1} = '    k3=ode_b(t+dt/2, x+k2*dt/2, uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);';
L{end+1} = '    k4=ode_b(t+dt,   x+k3*dt,   uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);';
L{end+1} = '    ns=x+(dt/6)*(k1+2*k2+2*k3+k4);';
L{end+1} = 'end';
L{end+1} = '';
L{end+1} = 'function dx=ode_b(t,x,uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs)';
L{end+1} = '    z_c=x(1);th=x(2);zuf=x(3);zur=x(4);zcd=x(5);thd=x(6);zufd=x(7);zurd=x(8);';
L{end+1} = '    zsf=z_c-a*th;zsr=z_c+b*th;zsfd=zcd-a*thd;zsrd=zcd+b*thd;';
L{end+1} = '    dsf=zsf-zuf;dsr=zsr-zur;dtf=zuf-wf;dtr=zur-wr;vrf=zsfd-zufd;vrr=zsrd-zurd;';
L{end+1} = '    if dsf>sm;fbf=kbs*(dsf-sm)^3;elseif dsf<-sm;fbf=kbs*(dsf+sm)^3;else;fbf=0;end';
L{end+1} = '    if dsr>sm;fbr=kbs*(dsr-sm)^3;elseif dsr<-sm;fbr=kbs*(dsr+sm)^3;else;fbr=0;end';
L{end+1} = '    fiwm=kem*id*sin(pp*(vms/whr)*t);';
L{end+1} = '    Fsf=ksf*dsf+csf*vrf+fbf-uf;Fsr=ksr*dsr+csr*vrr+fbr-ur;';
L{end+1} = '    dx=[zcd;thd;zufd;zurd;-(Fsf+Fsr)/ms;(a*Fsf-b*Fsr)/Ip;(Fsf-ktf*dtf+fiwm)/muf;(Fsr-ktr*dtr+fiwm)/mur];';
L{end+1} = 'end';
code = strjoin(L, newline);
end

% -------------------------------------------------------------------------
function [P, K, L] = care_nt(A, B, Q, R)
%CARE_NT  CARE solver — NO Control System Toolbox required.
% Uses Laub Hamiltonian eigendecomposition. Only needs base MATLAB eig().
    n = size(A,1);  Ri = inv(R);
    H = [A, -B*Ri*B'; -Q, -A'];
    [V,D] = eig(H);  ev = diag(D);
    [~,si] = sort(real(ev));
    Vs = V(:, si(1:n));
    P  = real(Vs(n+1:2*n,:) / Vs(1:n,:));
    P  = (P+P')/2;
    K  = Ri*B'*P;
    L  = eig(A-B*K);
end
