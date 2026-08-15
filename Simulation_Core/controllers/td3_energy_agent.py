# ==============================================================================
# Simulation_Core/controllers/td3_energy_agent.py
# TD3 Inference Agent for deciding Comfort vs ECO mode
# ==============================================================================
import os
import numpy as np
from stable_baselines3 import TD3

class TD3EnergyAgent:
    """
    Novelty 2: Deep Reinforcement Learning (TD3) Energy Harvester.
    
    This agent monitors the battery State of Charge (SoC), road roughness (rho),
    and rate of road change (rho_dot). It decides in real-time whether to:
    1. Output "COMFORT" (spend battery energy to push against the bump)
    2. Output "ECO" (turn actuator into a generator to harvest the bump's kinetic energy)
    
    If battery is critically low (<15%), it forces ECO mode unless the road
    is dangerously rough.
    """
    def __init__(self, model_path=None):
        self.is_loaded = False
        self.model = None
        
        # Hard limits
        self.critical_soc = 0.15
        
        # In a real scenario we load the trained .zip model
        if model_path and os.path.exists(model_path):
            try:
                self.model = TD3.load(model_path)
                self.is_loaded = True
            except:
                print("Warning: Could not load TD3 model. Using heuristic fallback.")
                
    def get_mode(self, battery_soc, rho, rho_dot):
        """
        Infers the optimal action from the TD3 policy.
        
        Returns:
            str: "COMFORT" or "ECO"
        """
        # 1. Hardware Override: If battery is dead, we physically cannot push.
        if battery_soc <= 0.02:
            return "ECO"
            
        # 2. Use trained policy if available
        if self.is_loaded:
            obs = np.array([battery_soc, rho, rho_dot], dtype=np.float32)
            action, _ = self.model.predict(obs, deterministic=True)
            # Action > 0.5 means ECO, < 0.5 means COMFORT
            return "ECO" if action[0] > 0.5 else "COMFORT"
            
        # 3. Heuristic Fallback (If no model is trained yet)
        # This mimics the intelligent policy the agent learns:
        # - Harvest on smooth/moderate roads (safe to passively damp)
        # - Active control only when road is rough (safety critical)
        
        # If road is extremely rough or rapidly worsening, prioritize safety
        if rho > 0.7 or rho_dot > 0.5:
            # UNLESS battery is critically low
            if battery_soc < self.critical_soc:
                return "ECO"
            return "COMFORT"
        
        # If road is moderate (0.3 < rho < 0.7), use COMFORT for safety
        if rho > 0.3:
            if battery_soc < 0.25:
                return "ECO"  # Low battery overrides moderate road
            return "COMFORT"
            
        # If road is smooth (rho < 0.05), opportunistically harvest energy
        # This is the key insight: small bumps on smooth roads can be 
        # passively damped while the actuator harvests their kinetic energy.
        if rho < 0.05:
            return "ECO"
            
        # Default to comfort
        return "COMFORT"
