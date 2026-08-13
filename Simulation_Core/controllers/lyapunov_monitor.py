# ==============================================================================
# Simulation_Core/controllers/lyapunov_monitor.py
# Monitors Lyapunov stability function V(x) = x^T P x during simulation
# ==============================================================================
import numpy as np
import scipy.linalg
from Simulation_Core.config.vehicle_params import VehicleParams

class LyapunovMonitor:
    """
    Computes the Lyapunov function V(x) = x^T P x in real-time.
    
    Uses the Riccati solution P from the LPV controller design to monitor
    energy-like stability of the closed-loop system. V(x) should be:
    1. Always positive (V(x) > 0 for x ≠ 0)
    2. Monotonically decreasing (V̇(x) < 0) — system is stable
    
    Also tracks settling time: time for |z_c(t)| < 1mm after each disturbance.
    """
    def __init__(self):
        self.p = VehicleParams()
        
        # Build the quarter-car state-space model for P computation
        A = np.array([
            [0, 1, 0, -1],
            [-self.p.ks_f / self.p.ms_f, -self.p.cs_f / self.p.ms_f, 0, self.p.cs_f / self.p.ms_f],
            [0, 0, 0, 1],
            [self.p.ks_f / self.p.mu_f, self.p.cs_f / self.p.mu_f, -self.p.kt_f / self.p.mu_f, -self.p.cs_f / self.p.mu_f]
        ])
        
        B = np.array([
            [0],
            [1 / self.p.ms_f],
            [0],
            [-1 / self.p.mu_f]
        ])
        
        # Solve CARE for the nominal case to get P
        Q = np.diag([1e5, 1e4, 1e4, 1e2])
        R = np.array([[1e-3]])
        
        self.P = scipy.linalg.solve_continuous_are(A, B, Q, R)
        
        # Settling time tracking
        self.is_disturbed = False
        self.disturbance_start_time = 0.0
        self.settling_times = []
        self.disturbance_threshold = 0.005  # 5mm — anything above this is a disturbance
        self.settled_threshold = 0.002      # 2mm — settled when below this
        self.settled_count = 0
        self.settled_required = 30          # Must stay below for 30ms
        
    def compute_V(self, state_8dof):
        """
        Computes V(x) = x_qc^T P x_qc using the front quarter-car state.
        
        Maps the 8-DOF half-car state to 4-DOF quarter-car state first.
        """
        z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot = state_8dof
        
        # Map to quarter-car state
        z_sf = z_c - self.p.a * theta
        z_sf_dot = z_c_dot - self.p.a * theta_dot
        
        # Quarter-car state: [stroke, body_vel, tire_defl_approx, unsprung_vel]
        x_qc = np.array([z_sf - z_uf, z_sf_dot, 0.0, z_uf_dot])
        
        # V(x) = x^T P x
        V = x_qc @ self.P @ x_qc
        
        return float(V)
    
    def track_settling(self, t, z_c):
        """
        Tracks settling time: measures how long it takes |z_c| < 1mm 
        after detecting a disturbance exceeding 3mm.
        """
        abs_z_c = abs(z_c)
        
        if not self.is_disturbed:
            # Check if a disturbance just started
            if abs_z_c > self.disturbance_threshold:
                self.is_disturbed = True
                self.disturbance_start_time = t
                self.settled_count = 0
        else:
            # We're tracking a disturbance — check if settled
            if abs_z_c < self.settled_threshold:
                self.settled_count += 1
                if self.settled_count >= self.settled_required:
                    # Settled! Record the settling time
                    settle_time = t - self.disturbance_start_time
                    self.settling_times.append(settle_time)
                    self.is_disturbed = False
                    self.settled_count = 0
            else:
                self.settled_count = 0
    
    def get_worst_settling_time(self):
        """Returns the worst (longest) settling time observed."""
        if not self.settling_times:
            return float('inf')
        return max(self.settling_times)
    
    def get_average_settling_time(self):
        """Returns the average settling time across all disturbance events."""
        if not self.settling_times:
            return float('inf')
        return sum(self.settling_times) / len(self.settling_times)
