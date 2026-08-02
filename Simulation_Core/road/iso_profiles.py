# ==============================================================================
# Simulation_Core/road/iso_profiles.py
# Generates standard ISO 8608 road roughness profiles
# ==============================================================================
import numpy as np

def generate_iso_road(t_vector, v_ms, iso_class='A', seed=None):
    """
    Generates a stochastic road profile according to the ISO 8608 standard.
    This creates the realistic "background noise" of the road before we add potholes.
    
    Args:
        t_vector (numpy.ndarray): Time vector
        v_ms (float): Vehicle speed in m/s
        iso_class (str): 'A' (Smooth), 'B' (Good), 'C' (Average), 'D' (Poor)
        seed (int): Random seed for reproducibility
        
    Returns:
        numpy.ndarray: Road height w(t) in meters
    """
    if seed is not None:
        np.random.seed(seed)
        
    # ISO 8608 Road Roughness Coefficients G(n0) in m^3
    roughness_map = {
        'A': 16e-6,   # Smooth highway
        'B': 64e-6,   # Good road
        'C': 256e-6,  # Average road
        'D': 1024e-6  # Poor/broken urban road
    }
    
    G_n0 = roughness_map.get(iso_class.upper(), 16e-6)
    
    # White noise passing through a first-order shaping filter
    # dx_r/dt + 2*pi*f0 * x_r = 2*pi*n0 * sqrt(G_n0 * v) * w(t)
    dt = t_vector[1] - t_vector[0]
    n0 = 0.1      # Spatial frequency reference (cycles/m)
    f0 = 0.01     # Lower cutoff frequency (Hz)
    
    n_steps = len(t_vector)
    w_noise = np.random.normal(0, 1, n_steps)
    
    # Precompute filter coefficients
    w_amplitude = 2 * np.pi * n0 * np.sqrt(G_n0 * v_ms)
    alpha = 2 * np.pi * f0
    
    road_height = np.zeros(n_steps)
    for i in range(1, n_steps):
        # Euler integration for the shaping filter
        road_height[i] = road_height[i-1] + dt * (-alpha * road_height[i-1] + w_amplitude * w_noise[i-1])
        
    return road_height
