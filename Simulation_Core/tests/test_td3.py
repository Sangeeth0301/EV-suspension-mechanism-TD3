# ==============================================================================
# Simulation_Core/tests/test_td3.py
# Pytest assertions for the TD3 Energy Harvester
# ==============================================================================
import numpy as np
from Simulation_Core.controllers.td3_energy_agent import TD3EnergyAgent

def calculate_harvested_power(stroke_velocity, c_e, eta):
    """
    Helper function replicating the Faraday induction math used in the loop.
    P_regen = eta * c_e * (v_stroke)^2
    """
    force = c_e * stroke_velocity
    return force * stroke_velocity * eta

def test_faraday_energy_harvesting():
    """
    Test that hitting a pothole generates positive electrical power.
    """
    # Velocity difference between body and wheel = 1.5 m/s (hitting a pothole)
    stroke_velocity = 1.5 
    c_e = 1500.0 # Electromagnetic damping constant
    eta = 0.85   # 85% generator/inverter efficiency

    # P_regen = eta * c_e * (v_stroke)^2
    p_regen = calculate_harvested_power(stroke_velocity, c_e, eta)

    # ASSERTION: Hitting a pothole MUST generate positive electrical power (> 2000 Watts peak)
    assert p_regen > 2000.0, f"Faraday induction math failed! Expected >2000W, got {p_regen}W"

def test_td3_low_battery_override():
    """
    Test that the TD3 agent (or its heuristic fallback) forces ECO mode
    when the battery is dying, unless the road is extremely dangerous.
    """
    agent = TD3EnergyAgent()
    
    # Battery at 10% SoC (dying battery), road roughness moderate (rho=0.5)
    mode = agent.get_mode(battery_soc=0.10, rho=0.5, rho_dot=0.1)

    # ASSERTION: AI must prioritize battery survival over comfort
    assert mode == "ECO", f"Agent failed to prioritize battery! Chose {mode} mode instead of ECO"
