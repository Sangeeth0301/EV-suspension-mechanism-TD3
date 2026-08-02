# ==============================================================================
# Simulation_Core/tests/test_ukf.py
# Pytest assertions for the Unscented Kalman Filter
# ==============================================================================
import numpy as np
from Simulation_Core.controllers.ukf_estimator import UKFRoadEstimator

def test_ukf_road_estimation():
    """
    Test that the UKF properly calculates rho (severity) and rho_dot (rate).
    If we feed it large body accelerations, it should output a high rho.
    """
    dt = 0.001
    ukf = UKFRoadEstimator(dt)
    
    # Step 1: Smooth road (Zero acceleration)
    rho_f, rho_dot_f, _, _ = ukf.estimate(z_s_ddot_f=0.0, z_s_ddot_r=0.0, u_f=0.0, u_r=0.0)
    
    assert 0.0 <= rho_f <= 0.1, "Rho should be near 0 for smooth road"
    
    # Step 2: Sudden pothole (Massive 20 m/s^2 body acceleration spike)
    # Check that it reacts within 2 milliseconds!
    reacted = False
    for i in range(2): 
        rho_f, rho_dot_f, _, _ = ukf.estimate(z_s_ddot_f=20.0, z_s_ddot_r=20.0, u_f=0.0, u_r=0.0)
        if rho_dot_f > 0.0:
            reacted = True
            break
            
    # ASSERTIONS: The UKF must have detected the pothole instantly
    assert reacted, "UKF failed to detect road worsening within 2 milliseconds!"
    assert rho_f > 0.1, f"Rho must increase when hitting a pothole, got {rho_f}"
