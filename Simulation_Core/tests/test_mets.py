# ==============================================================================
# Simulation_Core/tests/test_mets.py
# Pytest assertions for the Memory Event-Triggered Scheme (METS)
# ==============================================================================
import numpy as np
from Simulation_Core.network.mets_filter import METSFilter

def test_mets_bandwidth_saving():
    """
    Test that the METS filter actually saves bandwidth by dropping packets
    when the state change is small (e.g. driving on a smooth highway).
    """
    mets = METSFilter(sigma=0.10)
    
    # Tiny highway vibration (almost zero state)
    # [z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot]
    flat_road_state = np.array([0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    tx_count = 0
    for _ in range(1000): # Simulate 1,000 milliseconds
        # Add tiny noise to simulate flat road vibration
        noisy_state = flat_road_state + np.random.normal(0, 0.00005, 8)
        
        if mets.should_transmit(noisy_state)[0]:
            tx_count += 1

    # ASSERTION: On a smooth road, the bouncer MUST drop at least 40% of the packets!
    transmission_rate = tx_count / 1000.0
    assert transmission_rate < 0.60, f"METS failed to save bandwidth! Transmitted {transmission_rate*100}%"
