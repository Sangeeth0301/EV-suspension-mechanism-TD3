# ==============================================================================
# Simulation_Core/tests/test_physics.py
# Pytest assertions for the Half-Car Model physics engine
# ==============================================================================
import numpy as np
import pytest

from Simulation_Core.config.vehicle_params import VehicleParams
from Simulation_Core.physics.bump_stop import calculate_bump_stop_force
from Simulation_Core.physics.iwm_disturbance import calculate_iwm_force
from Simulation_Core.physics.half_car_model import HalfCarModel

def test_bump_stop_force():
    """
    Test that bump stop returns 0 when inside bounds, and > 0 when outside.
    """
    # Inside bounds (e.g. 50mm deflection, limit is 80mm)
    force = calculate_bump_stop_force(0.05, max_stroke=0.08)
    assert force == 0.0, f"Expected 0 force inside bounds, got {force}"
    
    # Outside upper bounds (e.g. 90mm deflection)
    force = calculate_bump_stop_force(0.09, max_stroke=0.08)
    assert force > 0.0, "Expected positive restoring force when hitting upper limit"
    
    # Outside lower bounds (e.g. -90mm deflection)
    force = calculate_bump_stop_force(-0.09, max_stroke=0.08)
    assert force < 0.0, "Expected negative restoring force when hitting lower limit"

def test_iwm_disturbance_magnitude():
    """
    Test that the IWM disturbance oscillates and has the correct peak amplitude.
    F_em(t) = k_em * i_d * sin(p * theta_m)
    """
    p = VehicleParams()
    expected_peak = p.k_em * p.i_d # 15.0 * 20.0 = 300 N
    
    # Calculate across a range of times to find peak
    forces = []
    for t in np.linspace(0, 1.0, 1000):
        forces.append(calculate_iwm_force(t, v_ms=20.0))
        
    peak_force = max(forces)
    assert abs(peak_force - expected_peak) < 1.0, f"Expected peak force ~{expected_peak}N, got {peak_force}N"

def test_half_car_model_equilibrium():
    """
    Test that the car stays perfectly still if there is zero road disturbance 
    and zero actuator force.
    """
    model = HalfCarModel()
    x_zero = np.zeros(8)
    
    # No actuator force, no road input
    dx = model.get_state_derivative(t=0.0, x=x_zero, u_f=0.0, u_r=0.0, w_f=0.0, w_r=0.0, v_ms=0.0)
    
    # Acceleration should be zero
    assert np.allclose(dx, np.zeros(8)), f"Expected zero acceleration in equilibrium, got {dx}"
