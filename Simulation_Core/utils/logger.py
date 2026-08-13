# ==============================================================================
# Simulation_Core/utils/logger.py
# High-performance data logger for the simulation loop
# ==============================================================================
import numpy as np
import pandas as pd

class SimulationLogger:
    """
    Pre-allocates numpy arrays for extreme speed during the 1ms simulation loop,
    then converts to a Pandas DataFrame at the end for easy plotting/export.
    
    Enhanced to track:
    - Power electronics (raw/conditioned power, LC filter state)
    - Body pitch dynamics
    - Rear wheel data
    - COMFORT/ECO mode flag
    - Lyapunov function V(x)
    """
    def __init__(self, n_steps, dt):
        self.n_steps = n_steps
        self.dt = dt
        self.idx = 0
        
        # Pre-allocate arrays
        self.t = np.zeros(n_steps)
        self.w_f = np.zeros(n_steps)
        self.w_r = np.zeros(n_steps)
        
        # Base paper comparison
        self.base_z_c = np.zeros(n_steps)
        self.base_z_c_ddot = np.zeros(n_steps)
        self.base_z_uf = np.zeros(n_steps)
        self.base_theta = np.zeros(n_steps)
        
        # Our approach — body dynamics
        self.z_c = np.zeros(n_steps)
        self.z_c_ddot = np.zeros(n_steps)
        self.theta = np.zeros(n_steps)
        self.theta_dot = np.zeros(n_steps)
        
        # Unsprung mass (wheel) dynamics
        self.z_uf = np.zeros(n_steps)
        self.z_ur = np.zeros(n_steps)
        
        # Relative velocities
        self.v_rel_f = np.zeros(n_steps)
        self.v_rel_r = np.zeros(n_steps)
        
        # UKF estimation outputs
        self.rho_f = np.zeros(n_steps)
        self.rho_dot_f = np.zeros(n_steps)
        self.rho_r = np.zeros(n_steps)
        self.rho_dot_r = np.zeros(n_steps)
        
        # Actuator forces
        self.u_f = np.zeros(n_steps)
        self.u_r = np.zeros(n_steps)
        
        # Energy harvesting
        self.battery_soc = np.zeros(n_steps)
        self.energy_harvested_j = np.zeros(n_steps)
        self.power_consumed_w = np.zeros(n_steps)
        
        # Power electronics — LC filter state
        self.power_raw_w = np.zeros(n_steps)
        self.power_conditioned_w = np.zeros(n_steps)
        self.v_capacitor = np.zeros(n_steps)
        self.i_inductor = np.zeros(n_steps)
        
        # TD3 Mode flag (0 = COMFORT, 1 = ECO)
        self.mode_f = np.zeros(n_steps)
        self.mode_r = np.zeros(n_steps)
        
        # Lyapunov function
        self.lyapunov_v = np.zeros(n_steps)
        
    def log_step(self, t, w_f, w_r, base_state, base_dx, our_state, our_dx,
                 rho_f, rho_dot_f, rho_r, rho_dot_r,
                 u_f, u_r, soc, harvested_j, power_consumed,
                 power_raw, power_conditioned, v_cap, i_ind,
                 mode_f, mode_r, lyapunov_v):
        """Records data for a single timestep."""
        if self.idx >= self.n_steps:
            return
            
        i = self.idx
        p_a = 1.1  # CG to front axle distance
        p_b = 1.5  # CG to rear axle distance
        
        self.t[i] = t
        self.w_f[i] = w_f
        self.w_r[i] = w_r
        
        # Base paper
        self.base_z_c[i] = base_state[0]
        self.base_z_c_ddot[i] = base_dx[4]
        self.base_z_uf[i] = base_state[2]
        self.base_theta[i] = base_state[1]
        
        # Ours — body
        self.z_c[i] = our_state[0]
        self.z_c_ddot[i] = our_dx[4]
        self.theta[i] = our_state[1]
        self.theta_dot[i] = our_state[5]
        
        # Wheels
        self.z_uf[i] = our_state[2]
        self.z_ur[i] = our_state[3]
        
        # Relative velocities
        z_sf_dot = our_state[4] - p_a * our_state[5]
        z_sr_dot = our_state[4] + p_b * our_state[5]
        self.v_rel_f[i] = z_sf_dot - our_state[6]
        self.v_rel_r[i] = z_sr_dot - our_state[7]
        
        # UKF
        self.rho_f[i] = rho_f
        self.rho_dot_f[i] = rho_dot_f
        self.rho_r[i] = rho_r
        self.rho_dot_r[i] = rho_dot_r
        
        # Forces
        self.u_f[i] = u_f
        self.u_r[i] = u_r
        
        # Energy
        self.battery_soc[i] = soc
        self.energy_harvested_j[i] = harvested_j
        self.power_consumed_w[i] = power_consumed
        
        # Power electronics
        self.power_raw_w[i] = power_raw
        self.power_conditioned_w[i] = power_conditioned
        self.v_capacitor[i] = v_cap
        self.i_inductor[i] = i_ind
        
        # Mode
        self.mode_f[i] = 1.0 if mode_f == "ECO" else 0.0
        self.mode_r[i] = 1.0 if mode_r == "ECO" else 0.0
        
        # Lyapunov
        self.lyapunov_v[i] = lyapunov_v
        
        self.idx += 1
        
    def to_dataframe(self):
        """Converts logged data to a pandas DataFrame."""
        return pd.DataFrame({
            'time': self.t,
            'w_f': self.w_f,
            'w_r': self.w_r,
            'base_z_c': self.base_z_c,
            'base_z_c_ddot': self.base_z_c_ddot,
            'base_z_uf': self.base_z_uf,
            'base_theta': self.base_theta,
            'z_c': self.z_c,
            'z_c_ddot': self.z_c_ddot,
            'theta': self.theta,
            'theta_dot': self.theta_dot,
            'z_uf': self.z_uf,
            'z_ur': self.z_ur,
            'v_rel_f': self.v_rel_f,
            'v_rel_r': self.v_rel_r,
            'rho_f': self.rho_f,
            'rho_dot_f': self.rho_dot_f,
            'rho_r': self.rho_r,
            'rho_dot_r': self.rho_dot_r,
            'u_f': self.u_f,
            'u_r': self.u_r,
            'battery_soc': self.battery_soc,
            'energy_harvested_j': self.energy_harvested_j,
            'power_consumed_w': self.power_consumed_w,
            'power_raw_w': self.power_raw_w,
            'power_conditioned_w': self.power_conditioned_w,
            'v_capacitor': self.v_capacitor,
            'i_inductor': self.i_inductor,
            'mode_f': self.mode_f,
            'mode_r': self.mode_r,
            'lyapunov_v': self.lyapunov_v
        })
