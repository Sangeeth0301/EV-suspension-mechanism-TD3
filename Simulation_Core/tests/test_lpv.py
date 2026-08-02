# ==============================================================================
# Simulation_Core/tests/test_lpv.py
# Pytest assertions for the LPV Controller
# ==============================================================================
import numpy as np
from Simulation_Core.controllers.lpv_controller import LPVController

def test_lpv_gain_scheduling():
    """
    Test that the controller dynamically adjusts its force based on rho.
    Rough roads (rho=1) should result in a different force than smooth roads (rho=0).
    """
    lpv = LPVController()
    
    # Fake state: car body is bouncing up (positive velocity)
    # [z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot]
    test_state = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    
    # Smooth road calculation
    u_f_smooth, _ = lpv.compute_force(test_state, rho_f=0.0, rho_dot_f=0.0, rho_r=0.0, rho_dot_r=0.0, mode_f="COMFORT")
    
    # Rough road calculation
    u_f_rough, _ = lpv.compute_force(test_state, rho_f=1.0, rho_dot_f=0.0, rho_r=1.0, rho_dot_r=0.0, mode_f="COMFORT")
    
    assert u_f_smooth != u_f_rough, "Controller failed to schedule gains; output is identical for rho=0 and rho=1"
    
def test_eco_mode_override():
    """
    Test that when ECO mode is triggered by the TD3 agent, the controller
    disables active pushing and only uses the C_e regenerative damping.
    """
    lpv = LPVController()
    test_state = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0])
    
    u_f_eco, _ = lpv.compute_force(test_state, rho_f=0.0, rho_dot_f=0.0, rho_r=0.0, rho_dot_r=0.0, mode_f="ECO")
    
    # In ECO mode, u_f = -C_e * (z_s_dot - z_u_dot)
    # C_e = 1500.0, z_s_dot = 1.0, z_u_dot = 0.0 -> Expected = -1500.0 (restoring force)
    assert np.isclose(u_f_eco, -1500.0), f"ECO mode failed to apply exact restoring regenerative damping. Got {u_f_eco}"
