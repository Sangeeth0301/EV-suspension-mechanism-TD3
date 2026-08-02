# ==============================================================================
# Simulation_Core/controllers/lpv_controller.py
# Rate-Bounded LPV Controller with ρ̇ feedforward
# ==============================================================================
import numpy as np
import scipy.linalg
from Simulation_Core.config.vehicle_params import VehicleParams

class LPVController:
    """
    Novelty 1B: Rate-Bounded Linear Parameter-Varying (LPV) Controller.
    
    Instead of hardcoding gains, this controller solves the Algebraic Riccati 
    Equation (CARE) for the optimal LQR gain at two extreme operating points:
    - Smooth Road (ρ = 0)
    - Rough Road (ρ = 1)
    
    CRITICAL UPGRADE: It incorporates a feedforward term based on ρ̇ (rate of 
    road change) to pre-stiffen the suspension before a pothole fully impacts.
    """
    def __init__(self):
        self.p = VehicleParams()
        
        # Simplified Quarter-Car matrices for gain synthesis
        # State: x = [z_s - z_u, z_s_dot, z_u - w, z_u_dot]
        # For LQR design, we use a 4-state quarter car model.
        
        # A matrix (System dynamics)
        self.A = np.array([
            [0, 1, 0, -1],
            [-self.p.ks_f / self.p.ms_f, -self.p.cs_f / self.p.ms_f, 0, self.p.cs_f / self.p.ms_f],
            [0, 0, 0, 1],
            [self.p.ks_f / self.p.mu_f, self.p.cs_f / self.p.mu_f, -self.p.kt_f / self.p.mu_f, -self.p.cs_f / self.p.mu_f]
        ])
        
        # B matrix (Actuator input)
        self.B = np.array([
            [0],
            [1 / self.p.ms_f],
            [0],
            [-1 / self.p.mu_f]
        ])

        # Synthesize the vertex gains
        self.K_smooth = self._synthesize_gain(rho=0.0)
        self.K_rough = self._synthesize_gain(rho=1.0)
        
        # Feedforward gain for rho_dot (Pre-stiffening)
        # These weights apply extra damping force when the road is rapidly worsening
        self.K_rho_dot = np.array([0.0, 500.0, 0.0, 0.0])

    def _synthesize_gain(self, rho):
        """
        Solves the Continuous Algebraic Riccati Equation (CARE) to find the
        optimal LQR gain matrix K for a given road severity rho.
        """
        # Q matrix (State penalties)
        # As rho increases (rougher road), we penalize body acceleration/velocity more
        # to prioritize comfort, while relaxing suspension stroke penalties.
        q_stroke = 1e5 * (1.0 - 0.5 * rho)  # Less penalty on stroke on rough roads
        q_body_vel = 1e4 * (1.0 + 5.0 * rho) # High penalty on body movement on rough roads
        q_tire = 1e4                         # Constant penalty on tire deflection
        q_unsprung_vel = 1e2                 # Constant penalty on unsprung mass
        
        Q = np.diag([q_stroke, q_body_vel, q_tire, q_unsprung_vel])
        
        # R matrix (Actuator effort penalty)
        R = np.array([[1e-3]])
        
        # Solve CARE: A^T P + P A - P B R^-1 B^T P + Q = 0
        P = scipy.linalg.solve_continuous_are(self.A, self.B, Q, R)
        
        # Compute LQR gain: K = R^-1 B^T P
        K = np.linalg.inv(R) @ self.B.T @ P
        
        return K.flatten()

    def _map_half_car_state(self, state_vector, is_front):
        """
        Maps the 8-DOF Half-Car state vector to the 4-DOF Quarter-Car state 
        needed by the synthesized controller gains.
        """
        z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot = state_vector
        
        if is_front:
            z_s = z_c - self.p.a * theta
            z_s_dot = z_c_dot - self.p.a * theta_dot
            z_u = z_uf
            z_u_dot = z_uf_dot
        else:
            z_s = z_c + self.p.b * theta
            z_s_dot = z_c_dot + self.p.b * theta_dot
            z_u = z_ur
            z_u_dot = z_ur_dot
            
        # Quarter-car state: [stroke, body_vel, tire_deflection(approx 0 for feedback), unsprung_vel]
        # Note: Tire deflection is usually unmeasurable, we approximate it or rely on observer.
        # For this robust feedback law, we set tire deflection error to 0 and rely on stroke/velocities.
        return np.array([z_s - z_u, z_s_dot, 0.0, z_u_dot])

    def compute_force(self, state_vector, rho_f, rho_dot_f, rho_r, rho_dot_r, mode_f="COMFORT", mode_r="COMFORT"):
        """
        Computes the adaptive control force for both front and rear actuators.
        
        Args:
            state_vector: 8-DOF half-car state
            rho_f, rho_r: Road severity (0 to 1)
            rho_dot_f, rho_dot_r: Rate of change of road severity
            mode_f, mode_r: "COMFORT" or "ECO" (From DRL agent)
            
        Returns:
            (u_f, u_r): Actuator forces (N)
        """
        # Map states
        qc_state_f = self._map_half_car_state(state_vector, is_front=True)
        qc_state_r = self._map_half_car_state(state_vector, is_front=False)
        
        # 1. FRONT WHEEL
        if mode_f == "ECO":
            # In ECO mode, we disable active pushing and only use regenerative damping
            # Must be negative to resist motion! (u_f is subtracted in the ODE)
            u_f = -self.p.C_e * qc_state_f[1] # -C_e * (z_s_dot - z_u_dot)
        else:
            # Polytopic blending of gains based on rho
            K_f = (1.0 - rho_f) * self.K_smooth + rho_f * self.K_rough
            
            # Base LPV force
            u_f_base = -np.dot(K_f, qc_state_f)
            
            # Feedforward force (pre-stiffening based on rho_dot)
            # Only apply if road is rapidly worsening (> 0)
            u_f_ff = -np.dot(self.K_rho_dot * max(0, rho_dot_f), qc_state_f)
            
            u_f = u_f_base + u_f_ff
            
        # 2. REAR WHEEL
        if mode_r == "ECO":
            u_r = -self.p.C_e * qc_state_r[1]
        else:
            K_r = (1.0 - rho_r) * self.K_smooth + rho_r * self.K_rough
            u_r_base = -np.dot(K_r, qc_state_r)
            u_r_ff = -np.dot(self.K_rho_dot * max(0, rho_dot_r), qc_state_r)
            u_r = u_r_base + u_r_ff
            
        # Clamp to physical actuator limits
        u_f = np.clip(u_f, -self.p.u_max, self.p.u_max)
        u_r = np.clip(u_r, -self.p.u_max, self.p.u_max)
        
        return u_f, u_r
