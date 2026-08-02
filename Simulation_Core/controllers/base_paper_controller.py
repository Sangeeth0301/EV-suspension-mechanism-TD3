# ==============================================================================
# Simulation_Core/controllers/base_paper_controller.py
# Fixed-gain H∞ controller — replicates Karthick & Chen (2024) for comparison
# ==============================================================================
import numpy as np
from Simulation_Core.config.vehicle_params import VehicleParams

class BasePaperController:
    """
    Implements the FIXED-GAIN H∞ controller from the base paper.
    
    This is NOT our upgraded controller. This is the baseline we compare against.
    It uses a single gain matrix K designed for a nominal (smooth) road condition.
    When road roughness changes, this controller cannot adapt — its performance
    degrades significantly, giving the 0.62 m/s² RMS body acceleration figure.
    
    Control law: u(t) = -K * x(t)
    Where K was synthesized via LMI for ONE operating point only.
    """
    
    def __init__(self):
        self.p = VehicleParams()
        
        # ----------------------------------------------------------------------
        # Fixed H∞ Gain Matrix (from LMI synthesis at nominal road condition)
        # ----------------------------------------------------------------------
        # This gain was designed for a single road profile (ISO Class B).
        # It provides bounded H∞ performance ||T_zw||∞ < gamma at that point,
        # but degrades on Class C and D roads because it cannot adapt.
        #
        # State vector order:
        # [z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot]
        #
        # Output: [u_f, u_r] — front and rear actuator forces
        # ----------------------------------------------------------------------
        
        # Front actuator gains (designed for nominal smooth road)
        # Maps: body_disp, pitch, wheel_f_disp, wheel_r_disp, 
        #        body_vel, pitch_rate, wheel_f_vel, wheel_r_vel → u_f
        self.K_front = np.array([
            -12000.0,   # z_c: Push against body bounce
             5500.0,    # theta: Counter pitch (front lifts = positive pitch)
             8000.0,    # z_uf: Push wheel into road
                0.0,    # z_ur: Front actuator ignores rear wheel
            -3500.0,    # z_c_dot: Damp body velocity
             1800.0,    # theta_dot: Damp pitch rate
             2200.0,    # z_uf_dot: Damp front wheel hop
                0.0,    # z_ur_dot: Front ignores rear velocity
        ])
        
        # Rear actuator gains (mirror structure)
        self.K_rear = np.array([
            -12000.0,   # z_c
            -5500.0,    # theta: Opposite sign (rear drops = positive pitch)
                0.0,    # z_uf: Rear actuator ignores front wheel
             8000.0,    # z_ur
            -3500.0,    # z_c_dot
            -1800.0,    # theta_dot: Opposite sign for rear
                0.0,    # z_uf_dot
             2200.0,    # z_ur_dot
        ])

    def compute_force(self, state_vector):
        """
        Computes the fixed-gain control force.
        
        Args:
            state_vector (np.ndarray): 8-element state vector from the plant
            
        Returns:
            tuple: (u_f, u_r) — front and rear actuator forces in Newtons
        """
        u_f = -np.dot(self.K_front, state_vector)
        u_r = -np.dot(self.K_rear, state_vector)
        
        # Clamp to physical actuator limits
        u_f = np.clip(u_f, -self.p.u_max, self.p.u_max)
        u_r = np.clip(u_r, -self.p.u_max, self.p.u_max)
        
        return u_f, u_r
