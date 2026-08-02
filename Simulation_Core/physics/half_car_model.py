# ==============================================================================
# Simulation_Core/physics/half_car_model.py
# 4-DOF ODE (Newton's laws for car + wheels)
# ==============================================================================
import numpy as np
from Simulation_Core.config.vehicle_params import VehicleParams
from Simulation_Core.physics.iwm_disturbance import calculate_iwm_force
from Simulation_Core.physics.bump_stop import calculate_bump_stop_force

class HalfCarModel:
    """
    Rigorous 4-Degree-of-Freedom Half-Car Active Suspension Model.
    Models Body Bounce, Body Pitch, Front Unsprung Mass, and Rear Unsprung Mass.
    Includes IWM disturbance and nonlinear bump stop limits.
    """
    def __init__(self):
        self.p = VehicleParams()
        
    def get_state_derivative(self, t, x, u_f, u_r, w_f, w_r, v_ms):
        """
        Computes the state derivative dx/dt for the ODE solver.
        
        State vector x:
        [0] z_c        : Body CG vertical displacement (m)
        [1] theta      : Body pitch angle (rad)
        [2] z_uf       : Front unsprung mass displacement (m)
        [3] z_ur       : Rear unsprung mass displacement (m)
        [4] z_c_dot    : Body CG vertical velocity (m/s)
        [5] theta_dot  : Body pitch angular velocity (rad/s)
        [6] z_uf_dot   : Front unsprung mass velocity (m/s)
        [7] z_ur_dot   : Rear unsprung mass velocity (m/s)
        
        Inputs:
        u_f, u_r : Front and rear actuator control forces (N)
        w_f, w_r : Front and rear road height disturbances (m)
        v_ms     : Vehicle speed (m/s) for IWM disturbance calculation
        """
        z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot = x
        
        # 1. Kinematics (calculate corner movements from CG)
        # Displacement of sprung mass at front/rear axle locations
        z_sf = z_c - self.p.a * theta
        z_sr = z_c + self.p.b * theta
        
        # Velocity of sprung mass at front/rear axle locations
        z_sf_dot = z_c_dot - self.p.a * theta_dot
        z_sr_dot = z_c_dot + self.p.b * theta_dot
        
        # 2. Deflections
        def_sf = z_sf - z_uf      # Front suspension stroke
        def_sr = z_sr - z_ur      # Rear suspension stroke
        def_tf = z_uf - w_f       # Front tire deflection
        def_tr = z_ur - w_r       # Rear tire deflection
        
        # Relative velocities across suspension dampers
        v_rel_f = z_sf_dot - z_uf_dot
        v_rel_r = z_sr_dot - z_ur_dot
        
        # 3. Nonlinear constraints and disturbances
        # Bump stop forces (Hard mechanical limits preventing metal-on-metal impact)
        f_bs_f = calculate_bump_stop_force(def_sf, self.p.stroke_max)
        f_bs_r = calculate_bump_stop_force(def_sr, self.p.stroke_max)
        
        # IWM Electromagnetic Disturbance
        f_iwm_f = calculate_iwm_force(t, v_ms)
        f_iwm_r = calculate_iwm_force(t, v_ms)
        
        # 4. Total Forces
        # Suspension forces (Passive Spring + Damper + Bump Stop - Active Actuator)
        # Note: u_f is defined positive pulling the body DOWN
        F_susp_f = self.p.ks_f * def_sf + self.p.cs_f * v_rel_f + f_bs_f - u_f
        F_susp_r = self.p.ks_r * def_sr + self.p.cs_r * v_rel_r + f_bs_r - u_r
        
        # Tire forces
        F_tire_f = self.p.kt_f * def_tf
        F_tire_r = self.p.kt_r * def_tr
        
        # 5. Equations of Motion (Accelerations)
        # Body Bounce (Sum of vertical forces)
        z_c_ddot = -(F_susp_f + F_susp_r) / self.p.ms
        
        # Body Pitch (Sum of moments around CG)
        theta_ddot = (self.p.a * F_susp_f - self.p.b * F_susp_r) / self.p.I_phi
        
        # Front Unsprung Mass (Wheel + IWM)
        z_uf_ddot = (F_susp_f - F_tire_f + f_iwm_f) / self.p.mu_f
        
        # Rear Unsprung Mass (Wheel + IWM)
        z_ur_ddot = (F_susp_r - F_tire_r + f_iwm_r) / self.p.mu_r
        
        # Return state derivative vector
        dx = np.array([
            z_c_dot, theta_dot, z_uf_dot, z_ur_dot,
            z_c_ddot, theta_ddot, z_uf_ddot, z_ur_ddot
        ])
        
        return dx
