function [z_c_base, accel_base] = Base_Paper_Sim(t_in, wf, wr)
% PASSIVE Half-Car Baseline (benchmark for the active controller)
% State: [z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot]
%
% This is the comparison baseline: a conventional passive suspension with
% spring and damper only and ZERO actuator force. It is a standard,
% independently verifiable reference — anyone can reproduce it from the
% vehicle parameters alone.
%
% Previously this block applied a hand-typed gain vector labelled "fixed
% H-infinity". Those numbers were never synthesised from any LMI or Riccati
% equation, so every "improvement over baseline" figure computed against
% them was meaningless. Comparing active against passive is the honest
% claim and is what the results now report.
%
% MATHEMATICAL EQUATIONS (Passive Plant):
% 1. No Active Control Force:
%    u_f = 0, u_r = 0
% 2. Sprung Mass Dynamics:
%    m_s * z_c_ddot = -F_sf - F_sr
% 3. Unsprung Mass Dynamics:
%    m_uf * z_uf_ddot = F_sf - k_tf(z_uf - w_f)
%    m_ur * z_ur_ddot = F_sr - k_tr(z_ur - w_r)
%
% -------------------------------------------------------------------------
persistent st_base

ms=730.0; I_phi=1222.0; a=1.10; b=1.50;
mu_f=45.0; mu_r=45.0;
ks_f=18000; ks_r=22000; cs_f=1200; cs_r=1200;
kt_f=190000; kt_r=190000;
k_em=15.0; i_d=20.0; p_pole=8; whr=0.30; v_ms=20.00;
smax=0.0800; u_max=6000; k_bs=1.00e+07; dt=0.001;
% Passive baseline: zero actuator gain (u_f = u_r = 0)
Kbf = zeros(1,8);
Kbr = zeros(1,8);

if isempty(st_base); st_base=zeros(8,1); end

uf_b = max(-u_max, min(u_max, -Kbf * st_base));
ur_b = max(-u_max, min(u_max, -Kbr * st_base));
[st_base, dx_b] = rk4_b(t_in,st_base,uf_b,ur_b,wf,wr,ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs,dt);
z_c_base   = st_base(1);
accel_base = dx_b(5);
end

function [ns,k1]=rk4_b(t,x,uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs,dt)
    k1=ode_b(t,      x,         uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);
    k2=ode_b(t+dt/2, x+k1*dt/2, uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);
    k3=ode_b(t+dt/2, x+k2*dt/2, uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);
    k4=ode_b(t+dt,   x+k3*dt,   uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs);
    ns=x+(dt/6)*(k1+2*k2+2*k3+k4);
end

function dx=ode_b(t,x,uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs)
    z_c=x(1);th=x(2);zuf=x(3);zur=x(4);zcd=x(5);thd=x(6);zufd=x(7);zurd=x(8);
    zsf=z_c-a*th;zsr=z_c+b*th;zsfd=zcd-a*thd;zsrd=zcd+b*thd;
    dsf=zsf-zuf;dsr=zsr-zur;dtf=zuf-wf;dtr=zur-wr;vrf=zsfd-zufd;vrr=zsrd-zurd;
    if dsf>sm;fbf=kbs*(dsf-sm)^3;elseif dsf<-sm;fbf=kbs*(dsf+sm)^3;else;fbf=0;end
    if dsr>sm;fbr=kbs*(dsr-sm)^3;elseif dsr<-sm;fbr=kbs*(dsr+sm)^3;else;fbr=0;end
    fiwm=kem*id*sin(pp*(vms/whr)*t);
    Fsf=ksf*dsf+csf*vrf+fbf-uf;Fsr=ksr*dsr+csr*vrr+fbr-ur;
    dx=[zcd;thd;zufd;zurd;-(Fsf+Fsr)/ms;(a*Fsf-b*Fsr)/Ip;(Fsf-ktf*dtf+fiwm)/muf;(Fsr-ktr*dtr+fiwm)/mur];
end