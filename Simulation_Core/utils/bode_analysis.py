import sys
import os
import numpy as np
import matplotlib.pyplot as plt
import control as ctrl

# Add project root to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from Simulation_Core.config.vehicle_params import VehicleParams

def main():
    p = VehicleParams()
    
    # We will use a front quarter-car model for this analysis
    ms = p.ms_f
    mu = p.mu_f
    ks = p.ks_f
    cs = p.cs_f
    kt = p.kt_f
    
    # State-Space Representation
    # State vector: x = [z_s, z_u, z_s_dot, z_u_dot]^T
    
    A = np.array([
        [0,           0,            1,       0],
        [0,           0,            0,       1],
        [-ks/ms,      ks/ms,       -cs/ms,   cs/ms],
        [ks/mu,      -(ks+kt)/mu,   cs/mu,  -cs/mu]
    ])
    
    # Input matrix for active control u
    B_u = np.array([
        [0],
        [0],
        [1/ms],
        [-1/mu]
    ])
    
    # Disturbance matrix for road input w
    B_w = np.array([
        [0],
        [0],
        [0],
        [kt/mu]
    ])
    
    # Output: we want to observe Sprung Mass Displacement (z_s)
    C_zs = np.array([[1, 0, 0, 0]])
    D_w = np.array([[0]])
    
    # Open-loop plant from road w to displacement z_s
    sys_open = ctrl.ss(A, B_w, C_zs, D_w)
    
    # LQR Design for Active Damping (Closed-Loop)
    # We penalize sprung mass displacement, velocity, and suspension deflection
    # Q matrix weights: [z_s, z_u, z_s_dot, z_u_dot]
    Q = np.diag([10000, 100, 100, 10])
    R = np.array([[0.001]])
    
    # Solve Algebraic Riccati Equation for LQR gain
    K, _, _ = ctrl.lqr(A, B_u, Q, R)
    
    # Closed-loop system matrix A_cl = A - B_u * K
    A_cl = A - B_u @ K
    
    # Closed-loop plant from road w to displacement z_s
    sys_closed = ctrl.ss(A_cl, B_w, C_zs, D_w)
    
    # Frequency vector for Bode plot (0.1 Hz to 100 Hz, converted to rad/s)
    # logspace from 10^-1 to 10^2 Hz -> 2*pi * 10^-1 to 2*pi * 10^2 rad/s
    omega = np.logspace(-1, 2, 500) * 2 * np.pi
    
    # Compute Bode responses
    mag_open, phase_open, omega_out = ctrl.freqresp(sys_open, omega)
    mag_closed, phase_closed, omega_out = ctrl.freqresp(sys_closed, omega)
    
    # Squeeze the 3D arrays to 1D arrays if it's a SISO system
    mag_open = np.squeeze(mag_open)
    phase_open = np.squeeze(phase_open)
    mag_closed = np.squeeze(mag_closed)
    phase_closed = np.squeeze(phase_closed)

    
    # Plotting
    plt.figure(figsize=(10, 8))
    
    # Magnitude Plot
    plt.subplot(2, 1, 1)
    # the function bode_response returns magnitude in absolute units
    # plt.loglog handles log-log scale. We also can plot in dB. Let's use dB.
    mag_open_db = 20 * np.log10(mag_open)
    mag_closed_db = 20 * np.log10(mag_closed)
    
    plt.semilogx(omega_out / (2 * np.pi), mag_open_db, label='Open Loop (Passive Mechanical Damping)', color='blue')
    plt.semilogx(omega_out / (2 * np.pi), mag_closed_db, label='Closed Loop (Active LQR Control)', color='red')
    plt.title('Bode Plot: Road Disturbance (w) to Sprung Mass Displacement (z_s)')
    plt.ylabel('Magnitude (dB)')
    plt.grid(True, which="both", ls="--")
    plt.legend()
    
    # Phase Plot
    plt.subplot(2, 1, 2)
    # Convert phase from radians to degrees
    phase_open_deg = np.degrees(phase_open)
    phase_closed_deg = np.degrees(phase_closed)
    
    plt.semilogx(omega_out / (2 * np.pi), phase_open_deg, label='Open Loop', color='blue')
    plt.semilogx(omega_out / (2 * np.pi), phase_closed_deg, label='Closed Loop', color='red')
    plt.xlabel('Frequency (Hz)')
    plt.ylabel('Phase (degrees)')
    plt.grid(True, which="both", ls="--")
    
    plt.tight_layout()
    output_path = os.path.join(current_dir, 'bode_plot.png')
    plt.savefig(output_path)
    print(f"Bode plot saved to: {output_path}")
    plt.close()

    # --- POLE-ZERO MAP ANALYSIS ---
    poles_open = ctrl.poles(sys_open)
    zeros_open = ctrl.zeros(sys_open)
    poles_closed = ctrl.poles(sys_closed)
    zeros_closed = ctrl.zeros(sys_closed)

    plt.figure(figsize=(8, 8))
    
    # Plot Open Loop Poles and Zeros
    plt.scatter(np.real(poles_open), np.imag(poles_open), marker='x', color='blue', s=100, label='Open Loop Poles')
    if len(zeros_open) > 0:
        plt.scatter(np.real(zeros_open), np.imag(zeros_open), marker='o', facecolors='none', edgecolors='blue', s=100, label='Open Loop Zeros')
        
    # Plot Closed Loop Poles and Zeros
    plt.scatter(np.real(poles_closed), np.imag(poles_closed), marker='x', color='red', s=100, label='Closed Loop Poles')
    if len(zeros_closed) > 0:
        plt.scatter(np.real(zeros_closed), np.imag(zeros_closed), marker='o', facecolors='none', edgecolors='red', s=100, label='Closed Loop Zeros')

    # Formatting the PZ Map
    plt.axvline(0, color='black', lw=2)
    plt.axhline(0, color='black', lw=1)
    plt.title('Pole-Zero Map: Control System Stability Analysis')
    plt.xlabel('Real Axis (Seconds^{-1})')
    plt.ylabel('Imaginary Axis (Seconds^{-1})')
    plt.grid(True, which='both', ls='--')
    plt.legend()
    
    pz_output_path = os.path.join(current_dir, 'pz_map.png')
    plt.savefig(pz_output_path)
    print(f"Pole-Zero Map saved to: {pz_output_path}")
    plt.close()

if __name__ == '__main__':
    main()
