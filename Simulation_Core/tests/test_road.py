# ==============================================================================
# Simulation_Core/tests/test_road.py
# Pytest assertions for the Road Generation module
# ==============================================================================
import numpy as np
from Simulation_Core.config.sim_config import SimConfig
from Simulation_Core.config.vehicle_params import VehicleParams
from Simulation_Core.road.iso_profiles import generate_iso_road
from Simulation_Core.road.road_generator import RoadGenerator

def test_iso_road_classes():
    """
    Test that a Class D (Poor) road generates a higher RMS roughness
    than a Class A (Smooth) road.
    """
    cfg = SimConfig()
    t = cfg.t_vector
    v_ms = cfg.v_ms
    
    # Generate roads
    road_A = generate_iso_road(t, v_ms, iso_class='A', seed=42)
    road_D = generate_iso_road(t, v_ms, iso_class='D', seed=42)
    
    # RMS values
    rms_A = np.sqrt(np.mean(np.square(road_A)))
    rms_D = np.sqrt(np.mean(np.square(road_D)))
    
    assert rms_D > rms_A, "Class D road should be rougher than Class A road"
    
def test_pothole_delay():
    """
    Test that the rear wheel hits the exact same pothole as the front wheel,
    but delayed by (wheelbase / velocity) seconds.
    """
    cfg = SimConfig()
    p = VehicleParams()
    gen = RoadGenerator(cfg, p)
    
    w_f, w_r = gen.generate_mixed_pothole_track()
    
    # Check that w_r is exactly w_f, shifted by delay_steps
    delay_steps = gen.delay_steps
    
    assert np.allclose(w_r[delay_steps:], w_f[:-delay_steps]), "Rear wheel road profile must perfectly match delayed front profile"
