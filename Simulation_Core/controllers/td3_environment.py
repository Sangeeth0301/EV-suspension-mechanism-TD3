# ==============================================================================
# Simulation_Core/controllers/td3_environment.py
# Gymnasium environment wrapper for training the TD3 Energy Agent
# ==============================================================================
import numpy as np
import gymnasium as gym
from gymnasium import spaces
from Simulation_Core.config.vehicle_params import VehicleParams
from Simulation_Core.physics.half_car_model import HalfCarModel
from Simulation_Core.road.iso_profiles import generate_iso_road

class SuspensionEnv(gym.Env):
    """
    Custom Environment that follows gymnasium interface.
    This allows us to train the TD3 Deep Reinforcement Learning agent.
    
    The Agent observes the state and outputs an action [0, 1] which dictates 
    the blend between COMFORT (active pushing) and ECO (energy harvesting).
    """
    def __init__(self, dt=0.001):
        super(SuspensionEnv, self).__init__()
        self.dt = dt
        self.p = VehicleParams()
        self.model = HalfCarModel()
        
        # Action space: Continuous value between 0.0 (COMFORT) and 1.0 (ECO)
        self.action_space = spaces.Box(low=0.0, high=1.0, shape=(1,), dtype=np.float32)
        
        # Observation space: 
        # [battery_soc (0-1), rho_front (0-1), rho_dot_front (-1 to 1)]
        self.observation_space = spaces.Box(
            low=np.array([0.0, 0.0, -1.0]), 
            high=np.array([1.0, 1.0, 1.0]), 
            dtype=np.float32
        )
        
        self.state_vector = np.zeros(8)
        self.battery_soc = 0.5  # Start at 50%
        self.battery_capacity_joules = 50000.0 # Small buffer for simulation
        
        # Road generation for training episodes
        self.time = 0.0
        self.max_time = 5.0
        
    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        self.state_vector = np.zeros(8)
        
        # Randomize starting battery to teach agent all scenarios
        self.battery_soc = np.random.uniform(0.1, 0.9)
        self.time = 0.0
        
        # Domain Randomization (Sim-to-Real transfer)
        # Randomize vehicle mass by +/- 20% during training so the agent learns
        # a robust policy that works even if the car is loaded with passengers.
        mass_variation = np.random.uniform(0.8, 1.2)
        self.p.ms *= mass_variation
        self.p.ms_f *= mass_variation
        self.p.ms_r *= mass_variation
        
        # Generate a random road profile for this episode (mix of A, B, C, D)
        iso_classes = ['A', 'B', 'C', 'D']
        self.current_road_class = np.random.choice(iso_classes)
        
        obs = np.array([self.battery_soc, 0.0, 0.0], dtype=np.float32)
        return obs, {}

    def step(self, action):
        """
        Takes a step in the simulation based on the agent's action.
        """
        action_val = np.clip(action[0], 0.0, 1.0) # 0 = Comfort, 1 = Eco
        
        # Fake a road disturbance and UKF estimation for training purposes
        w_f = np.random.normal(0, 0.02) if self.current_road_class in ['C', 'D'] else 0.0
        w_r = np.random.normal(0, 0.02) if self.current_road_class in ['C', 'D'] else 0.0
        
        # Very simplified UKF mock for training speed
        rho_f = min(abs(w_f) / 0.05, 1.0)
        rho_dot_f = (rho_f - 0.0) / self.dt # simplified
        
        # Calculate forces
        # If action is 1 (ECO), we use regenerative damping (C_e)
        # If action is 0 (COMFORT), we use LPV active force (simplified here for training)
        z_sf_dot = self.state_vector[4] - self.p.a * self.state_vector[5]
        z_uf_dot = self.state_vector[6]
        v_rel_f = z_sf_dot - z_uf_dot
        
        force_eco = -self.p.C_e * v_rel_f
        force_comfort = -10000 * (self.state_vector[0] - self.p.a * self.state_vector[1] - self.state_vector[2]) # Simplified LPV
        
        u_f = (1.0 - action_val) * force_comfort + action_val * force_eco
        u_r = 0.0 # Rear wheel passive during this specific training phase
        
        # Integrate Physics
        v_ms = 20.0
        dx = self.model.get_state_derivative(self.time, self.state_vector, u_f, u_r, w_f, w_r, v_ms)
        self.state_vector += dx * self.dt
        self.time += self.dt
        
        # Calculate Energy Harvested (Power = Force * Velocity)
        # We only harvest energy when the actuator resists the suspension movement
        power_watts = 0.0
        if action_val > 0.5:
            power_watts = abs(u_f * v_rel_f) * self.p.eta_regen
            
        joules_harvested = power_watts * self.dt
        self.battery_soc += joules_harvested / self.battery_capacity_joules
        self.battery_soc = np.clip(self.battery_soc, 0.0, 1.0)
        
        # Calculate Reward (Multi-objective)
        # Penalty for body acceleration (comfort)
        comfort_penalty = abs(dx[4]) * 0.1 
        
        # Reward for harvesting energy, heavily weighted if battery is low
        battery_urgency = max(0, 0.3 - self.battery_soc) * 10.0
        energy_reward = power_watts * 0.001 * (1.0 + battery_urgency)
        
        # If road is terrible (rho > 0.8), severely penalize ECO mode
        safety_penalty = 0.0
        if rho_f > 0.8 and action_val > 0.5:
            safety_penalty = 10.0
            
        reward = energy_reward - comfort_penalty - safety_penalty
        
        obs = np.array([self.battery_soc, rho_f, rho_dot_f], dtype=np.float32)
        terminated = self.time >= self.max_time
        truncated = False
        
        return obs, reward, terminated, truncated, {}
