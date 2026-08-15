function [x_new, z_c, body_accel] = HalfCar_Plant(t_in, wf, wr, x_prev, uf_prev, ur_prev)
% 4-DOF Half-Car Plant with RK4 Integration
% State: [z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot]
%
% MATHEMATICAL EQUATIONS:
% 1. Sprung Mass Dynamics:
%    m_s * z_c_ddot = -F_sf - F_sr
%    I_phi * theta_ddot = a*F_sf - b*F_sr
% 2. Unsprung Mass Dynamics:
%    m_uf * z_uf_ddot = F_sf - k_tf(z_uf - w_f) + F_iwm
%    m_ur * z_ur_ddot = F_sr - k_tr(z_ur - w_r) + F_iwm
% 3. Suspension Force (with Bump Stop):
%    F_sf = k_sf(z_sf - z_uf) + c_sf(z_sf_dot - z_uf_dot) + F_bs - u_f
%    F_bs = k_bs * (d_s - s_max)^3  (if d_s > s_max)
% 
% State passed explicitly — no persistent variables needed here.

ms=730.0;  I_phi=1222.0;  a=1.10;  b=1.50;
mu_f=45.0; mu_r=45.0;
ks_f=18000; ks_r=22000; cs_f=1200; cs_r=1200;
kt_f=190000; kt_r=190000;
k_em=15.0; i_d=20.0; p_pole=8; whr=0.30; v_ms=20.00;
smax=0.0800; u_max=6000; k_bs=1.00e+07;
dt = 0.001;

% Runge-Kutta 4 integration (fixed-step, dt=1ms)
k1=hc_ode(t_in,      x_prev,         uf_prev,ur_prev,wf,wr,ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs);
k2=hc_ode(t_in+dt/2, x_prev+k1*dt/2, uf_prev,ur_prev,wf,wr,ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs);
k3=hc_ode(t_in+dt/2, x_prev+k2*dt/2, uf_prev,ur_prev,wf,wr,ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs);
k4=hc_ode(t_in+dt,   x_prev+k3*dt,   uf_prev,ur_prev,wf,wr,ms,I_phi,a,b,mu_f,mu_r,ks_f,ks_r,cs_f,cs_r,kt_f,kt_r,k_em,i_d,p_pole,whr,v_ms,smax,k_bs);
x_new      = x_prev + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
z_c        = x_new(1);
body_accel = k1(5);
end

function dx = hc_ode(t,x,uf,ur,wf,wr,ms,Ip,a,b,muf,mur,ksf,ksr,csf,csr,ktf,ktr,kem,id,pp,whr,vms,sm,kbs)
    z_c=x(1);th=x(2);zuf=x(3);zur=x(4);zcd=x(5);thd=x(6);zufd=x(7);zurd=x(8);
    zsf=z_c-a*th; zsr=z_c+b*th; zsfd=zcd-a*thd; zsrd=zcd+b*thd;
    dsf=zsf-zuf; dsr=zsr-zur; dtf=zuf-wf; dtr=zur-wr;
    vrf=zsfd-zufd; vrr=zsrd-zurd;
    if dsf> sm; fbf=kbs*(dsf-sm)^3; elseif dsf<-sm; fbf=kbs*(dsf+sm)^3; else; fbf=0; end
    if dsr> sm; fbr=kbs*(dsr-sm)^3; elseif dsr<-sm; fbr=kbs*(dsr+sm)^3; else; fbr=0; end
    fiwm = kem*id*sin(pp*(vms/whr)*t);
    Fsf=ksf*dsf+csf*vrf+fbf-uf; Fsr=ksr*dsr+csr*vrr+fbr-ur;
    dx=[zcd;thd;zufd;zurd; -(Fsf+Fsr)/ms; (a*Fsf-b*Fsr)/Ip; (Fsf-ktf*dtf+fiwm)/muf; (Fsr-ktr*dtr+fiwm)/mur];
end