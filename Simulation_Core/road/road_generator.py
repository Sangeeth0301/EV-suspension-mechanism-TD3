# ==============================================================================
# Simulation_Core/road/road_generator.py
# Generates the custom 10-second mixed-pothole test track
# ==============================================================================
import numpy as np
from Simulation_Core.road.iso_profiles import generate_iso_road

class RoadGenerator:
    """
    Generates the full test track for the simulation.
    Combines a base ISO A highway with dynamically generated random potholes
    of varying severities (small, medium, large, giant).
    """
    def __init__(self, sim_config, vehicle_params):
        self.cfg = sim_config
        self.p = vehicle_params
        
        # Calculate time delay between front and rear wheels hitting the same bump
        # Delay = wheelbase / velocity
        self.wheelbase = self.p.a + self.p.b
        self.delay_sec = self.wheelbase / self.cfg.v_ms
        self.delay_steps = int(self.delay_sec / self.cfg.dt)

    def generate_mixed_pothole_track(self):
        """
        Creates a 10s track:
        0s-3s: Smooth ISO A
        3s-7s: Broken road with random potholes
        7s-10s: Smooth ISO A
        
        Returns:
            w_f (ndarray): Front wheel road height profile
            w_r (ndarray): Rear wheel road height profile
        """
        np.random.seed(self.cfg.road_seed)
        t = self.cfg.t_vector
        
        # 1. Start with a baseline ISO Class A (Smooth) road
        w_base = generate_iso_road(t, self.cfg.v_ms, iso_class='A', seed=self.cfg.road_seed)
        w_f = w_base.copy()
        
        # 2. Add the rough road section (3s to 7s)
        # We overlay an ISO D profile on this section to simulate generally broken asphalt
        idx_start = int(3.0 / self.cfg.dt)
        idx_end = int(7.0 / self.cfg.dt)
        
        w_rough = generate_iso_road(t, self.cfg.v_ms, iso_class='D', seed=self.cfg.road_seed+1)
        
        # Fade in/out to prevent impossible vertical steps
        fade = np.ones(idx_end - idx_start)
        fade_len = int(0.2 / self.cfg.dt)
        fade[:fade_len] = np.linspace(0, 1, fade_len)
        fade[-fade_len:] = np.linspace(1, 0, fade_len)
        
        w_f[idx_start:idx_end] += w_rough[idx_start:idx_end] * fade
        
        # 3. Add explicit random potholes within the rough section
        # Pothole types: (depth_meters, width_seconds)
        potholes = [
            (-0.060, 0.25),  # Giant, wide dip
            (-0.040, 0.15),  # Large, sharp hole
            (-0.020, 0.10),  # Medium hole
            (-0.005, 0.05),  # Small crack
            (-0.050, 0.20),  # Large dip
            (-0.030, 0.12),  # Medium/Large hole
        ]
        
        # Randomly scatter these potholes between 3.5s and 6.5s
        pothole_times = np.linspace(3.5, 6.5, len(potholes))
        np.random.shuffle(pothole_times)
        
        for p_time, (depth, width) in zip(pothole_times, potholes):
            self._add_pothole(w_f, t, p_time, depth, width)
            
        # 4. Generate rear wheel profile (exact copy of front, delayed by wheelbase)
        w_r = np.zeros_like(w_f)
        w_r[self.delay_steps:] = w_f[:-self.delay_steps]
        
        return w_f, w_r
        
    def _add_pothole(self, road_array, t_vector, start_time, depth, width):
        """
        Injects a single smooth pothole (1-cosine shape) into the road array.
        """
        start_idx = int(start_time / self.cfg.dt)
        duration_steps = int(width / self.cfg.dt)
        end_idx = start_idx + duration_steps
        
        if end_idx >= len(road_array):
            end_idx = len(road_array) - 1
            duration_steps = end_idx - start_idx
            
        # 1-cosine shape for a realistic pothole dip
        x = np.linspace(0, 2*np.pi, duration_steps)
        pothole_profile = (depth / 2) * (1 - np.cos(x))
        
        road_array[start_idx:end_idx] += pothole_profile
