# ==============================================================================
# Simulation_Core/controllers/ukf_estimator.py
# Unscented Kalman Filter for nonlinear road roughness estimation
# ==============================================================================
import numpy as np
from filterpy.kalman import UnscentedKalmanFilter, MerweScaledSigmaPoints
from Simulation_Core.config.vehicle_params import VehicleParams
from Simulation_Core.config.sim_config import SimConfig

class UKFRoadEstimator:
    """
    Novelty 1A: Unscented Kalman Filter (UKF) for road estimation.
    
    Unlike standard linear Kalman filters, the UKF uses sigma points to 
    propagate the state through the nonlinear suspension dynamics (e.g., 
    bump stop impacts). 
    
    It estimates a 3-state vector per corner:
    x = [susp_deflection, susp_velocity, road_disturbance]
    
    CRITICAL UPGRADE: It outputs both rho (roughness) AND rho_dot (rate of change).
    """
    def __init__(self, dt):
        self.p = VehicleParams()
        self.dt = dt
        
        # We need a separate UKF for the front and rear wheels
        self.ukf_f = self._init_ukf()
        self.ukf_r = self._init_ukf()
        
        # History for calculating the rate of change (rho_dot)
        self.rho_f_prev = 0.0
        self.rho_r_prev = 0.0
        
        # Maximum expected road disturbance (for normalizing rho to 0-1)
        self.w_max = 0.05  # 50mm

    def _init_ukf(self):
        """Initializes a 3-state UKF for one corner of the car."""
        # State: [z_s - z_u, z_s_dot - z_u_dot, w_road]
        # Measurement: [z_s_ddot] (Body acceleration from accelerometer)
        
        # 1. Sigma Points generation (Merwe Scaled)
        # alpha, beta, kappa are standard tuning parameters for UKF
        points = MerweScaledSigmaPoints(n=3, alpha=0.1, beta=2., kappa=0)
        
        # 2. Create the filter
        ukf = UnscentedKalmanFilter(dim_x=3, dim_z=1, dt=self.dt, 
                                   fx=self._state_transition, 
                                   hx=self._measurement_function, 
                                   points=points)
        
        # 3. Initial State [deflection, velocity, road]
        ukf.x = np.array([0., 0., 0.])
        
        # 4. State Covariance Matrix P (Uncertainty in initial state)
        ukf.P *= 0.1
        
        # 5. Process Noise Q
        # We trust our kinematics, but road disturbance is a random walk
        ukf.Q = np.diag([1e-4, 1e-4, 1e-2])
        
        # 6. Measurement Noise R (Accelerometer noise)
        ukf.R = np.array([[0.05]])
        
        return ukf

    def _state_transition(self, x, dt, **kwargs):
        """
        Nonlinear state transition function fx(x, dt).
        x = [deflection, relative_velocity, road_disturbance]
        """
        deflection, rel_vel, w_road = x
        
        # Active force (passed via kwargs)
        u = kwargs.get('u', 0.0)
        
        # Calculate passive forces (linear for now in the estimator, but UKF 
        # allows us to easily add the nonlinear bump stop here if needed)
        f_spring = self.p.ks_f * deflection
        f_damper = self.p.cs_f * rel_vel
        
        # Body acceleration (Simplified quarter-car logic for the estimator)
        z_s_ddot = -(f_spring + f_damper - u) / self.p.ms_f
        
        # Unsprung mass acceleration (Simplified)
        f_tire = self.p.kt_f * (-w_road) # Approximation
        z_u_ddot = (f_spring + f_damper - f_tire) / self.p.mu_f
        
        # Euler integration for the next state
        next_deflection = deflection + rel_vel * dt
        next_rel_vel = rel_vel + (z_s_ddot - z_u_ddot) * dt
        
        # Assume road disturbance is a random walk (stays same, noise added by Q)
        next_w_road = w_road 
        
        return np.array([next_deflection, next_rel_vel, next_w_road])

    def _measurement_function(self, x, **kwargs):
        """
        Measurement function hx(x). 
        Maps the state vector to the expected sensor measurement (body acceleration).
        """
        deflection, rel_vel, _ = x
        u = kwargs.get('u', 0.0)
        
        f_spring = self.p.ks_f * deflection
        f_damper = self.p.cs_f * rel_vel
        
        # Expected body acceleration
        expected_z_s_ddot = -(f_spring + f_damper - u) / self.p.ms_f
        return np.array([expected_z_s_ddot])

    def estimate(self, z_s_ddot_f, z_s_ddot_r, u_f, u_r):
        """
        Runs one predict/update cycle of the UKF.
        
        Args:
            z_s_ddot_f, z_s_ddot_r: Measured front/rear body accelerations
            u_f, u_r: Current actuator forces
            
        Returns:
            tuple: (rho_f, rho_dot_f, rho_r, rho_dot_r)
        """
        # --- FRONT WHEEL ---
        # Predict step
        self.ukf_f.predict(u=u_f)
        # Update step with sensor measurement
        self.ukf_f.update(np.array([z_s_ddot_f]), u=u_f)
        
        # Extract estimated road disturbance magnitude and normalize to [0, 1]
        w_road_est_f = abs(self.ukf_f.x[2])
        rho_f = np.clip(w_road_est_f / self.w_max, 0.0, 1.0)
        
        # Calculate rate of change (rho_dot)
        rho_dot_f = (rho_f - self.rho_f_prev) / self.dt
        self.rho_f_prev = rho_f


        # --- REAR WHEEL ---
        self.ukf_r.predict(u=u_r)
        self.ukf_r.update(np.array([z_s_ddot_r]), u=u_r)
        
        w_road_est_r = abs(self.ukf_r.x[2])
        rho_r = np.clip(w_road_est_r / self.w_max, 0.0, 1.0)
        rho_dot_r = (rho_r - self.rho_r_prev) / self.dt
        self.rho_r_prev = rho_r

        return rho_f, rho_dot_f, rho_r, rho_dot_r
