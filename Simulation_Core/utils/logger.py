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
    """
    def __init__(self, n_steps, dt):
        self.n_steps = n_steps
        self.dt = dt
        self.idx = 0
        
        # Pre-allocate arrays
        self.t = np.zeros(n_steps)
        self.w_f = np.zeros(n_steps)
        
        # Base paper comparison
        self.base_z_c = np.zeros(n_steps)
        self.base_z_c_ddot = np.zeros(n_steps)
        self.base_z_uf = np.zeros(n_steps)
        
        # Our approach
        self.z_c = np.zeros(n_steps)
        self.z_c_ddot = np.zeros(n_steps)
        self.z_uf = np.zeros(n_steps)
        self.v_rel_f = np.zeros(n_steps)
        
        self.rho_f = np.zeros(n_steps)
        self.rho_dot_f = np.zeros(n_steps)
        self.u_f = np.zeros(n_steps)
        
        self.battery_soc = np.zeros(n_steps)
        self.energy_harvested_j = np.zeros(n_steps)
        
    def log_step(self, t, w_f, base_state, base_dx, our_state, our_dx, 
                 rho_f, rho_dot_f, u_f, soc, harvested_j):
        """Records data for a single timestep."""
        if self.idx >= self.n_steps:
            return
            
        self.t[self.idx] = t
        self.w_f[self.idx] = w_f
        
        # Base paper
        self.base_z_c[self.idx] = base_state[0]
        self.base_z_c_ddot[self.idx] = base_dx[4]
        self.base_z_uf[self.idx] = base_state[2]
        
        # Ours
        self.z_c[self.idx] = our_state[0]
        self.z_c_ddot[self.idx] = our_dx[4]
        self.z_uf[self.idx] = our_state[2]
        
        # Derived
        z_sf_dot = our_state[4] - 1.1 * our_state[5] # a=1.1
        self.v_rel_f[self.idx] = z_sf_dot - our_state[6]
        
        self.rho_f[self.idx] = rho_f
        self.rho_dot_f[self.idx] = rho_dot_f
        self.u_f[self.idx] = u_f
        
        self.battery_soc[self.idx] = soc
        self.energy_harvested_j[self.idx] = harvested_j
        
        self.idx += 1
        
    def to_dataframe(self):
        """Converts logged data to a pandas DataFrame."""
        return pd.DataFrame({
            'time': self.t,
            'w_f': self.w_f,
            'base_z_c': self.base_z_c,
            'base_z_c_ddot': self.base_z_c_ddot,
            'base_z_uf': self.base_z_uf,
            'z_c': self.z_c,
            'z_c_ddot': self.z_c_ddot,
            'z_uf': self.z_uf,
            'v_rel_f': self.v_rel_f,
            'rho_f': self.rho_f,
            'rho_dot_f': self.rho_dot_f,
            'u_f': self.u_f,
            'battery_soc': self.battery_soc,
            'energy_harvested_j': self.energy_harvested_j
        })
