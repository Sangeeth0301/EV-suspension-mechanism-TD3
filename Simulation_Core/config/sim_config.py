# ==============================================================================
# Simulation_Core/config/sim_config.py
# Global configuration settings for the simulation run
# ==============================================================================

import numpy as np

class SimConfig:
    """
    Simulation timing, road parameters, and DRL training configuration.
    """
    
    # --------------------------------------------------------------------------
    # Timing Parameters
    # --------------------------------------------------------------------------
    dt = 0.001          # Simulation timestep (1 ms matches CAN-bus frequency)
    T_total = 10.0      # Total simulation duration (seconds)
    
    # Pre-calculated time vector for convenience
    t_vector = np.arange(0, T_total, dt)
    n_steps = len(t_vector)
    
    # Vehicle Speed
    v_kmh = 72.0                # Vehicle speed (km/h)
    v_ms = v_kmh / 3.6          # Vehicle speed (m/s) = 20 m/s
    
    # --------------------------------------------------------------------------
    # Road Generation Parameters
    # --------------------------------------------------------------------------
    road_seed = 42      # For reproducible random potholes
    
    # --------------------------------------------------------------------------
    # TD3 Training Parameters
    # --------------------------------------------------------------------------
    # (These are used by train_td3.py, not the main run_simulation loop)
    total_timesteps = 100_000   # Total steps for DRL agent to learn
    batch_size = 256
    learning_rate = 3e-4
    gamma = 0.99                # Discount factor for future rewards
    
    # Domain Randomization (for Sim-to-Real transfer)
    # The agent will face variations in mass and actuator delay during training
    mass_variation_pct = 0.20   # +/- 20% variation in passenger payload
    max_actuator_delay = 0.040  # Up to 40 ms random delay to simulate real STM32
