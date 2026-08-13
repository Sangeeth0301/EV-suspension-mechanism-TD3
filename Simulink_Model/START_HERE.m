% ========================================================================
% START_HERE.m - Run this script first in MATLAB Online!
% ========================================================================
% This script loads the required variables (road profile) into the workspace
% and then opens the beautifully arranged Simulink model so you can press Play.

disp('==========================================================');
disp('[*] Cyber-Resilient Suspension   MATLAB Online Setup');
disp('==========================================================');

disp('[*] Generating Road Profiles (ISO 8608 Class A + Potholes)...');
p.dt=0.001; p.T_total=10; p.v_ms=20; p.a=1.1; p.b=1.5;
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

% Load to base workspace for Simulink to read
road_f_ts = timeseries(w_f', t_vec');
road_r_ts = timeseries(w_r', t_vec');
assignin('base','road_f_ts', road_f_ts);
assignin('base','road_r_ts', road_r_ts);
disp('    [+] road_f_ts and road_r_ts loaded successfully!');

disp('[*] Opening Simulink Model...');
open_system('CyberSuspension_MultiBlock.slx');

disp('==========================================================');
disp('   [READY] You can now press the green PLAY button in');
disp('           the Simulink window to run the simulation!');
disp('==========================================================');
