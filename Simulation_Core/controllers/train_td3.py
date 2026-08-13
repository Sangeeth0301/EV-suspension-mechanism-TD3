# ==============================================================================
# Simulation_Core/controllers/train_td3.py
# Trains the TD3 Deep Reinforcement Learning Energy Harvesting Agent
# ==============================================================================
import os
import sys
import numpy as np

# Add project root
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from stable_baselines3 import TD3
from stable_baselines3.common.noise import NormalActionNoise
from stable_baselines3.common.callbacks import EvalCallback

from Simulation_Core.controllers.td3_environment import SuspensionEnv
from Simulation_Core.config.sim_config import SimConfig

def train():
    """
    Trains the TD3 agent to learn optimal COMFORT vs ECO mode switching.
    
    The agent learns to:
    1. Harvest energy when bumps are small and battery is low
    2. Use active control when bumps are dangerous (ρ > 0.7)
    3. Balance comfort vs energy recovery across all SoC levels
    """
    cfg = SimConfig()
    
    print("=" * 60)
    print("[*] TD3 Energy Harvester — Training Pipeline")
    print("=" * 60)
    
    # 1. Create the training environment
    env = SuspensionEnv(dt=cfg.dt)
    
    # 2. Create evaluation environment (for periodic testing)
    eval_env = SuspensionEnv(dt=cfg.dt)
    
    # 3. Define exploration noise
    # TD3 uses deterministic policy + noise for exploration
    n_actions = env.action_space.shape[-1]
    action_noise = NormalActionNoise(
        mean=np.zeros(n_actions),
        sigma=0.2 * np.ones(n_actions)  # 20% exploration noise
    )
    
    # 4. Create the TD3 agent
    model = TD3(
        "MlpPolicy",
        env,
        action_noise=action_noise,
        learning_rate=cfg.learning_rate,
        batch_size=cfg.batch_size,
        gamma=cfg.gamma,
        buffer_size=100_000,
        learning_starts=1000,
        train_freq=(1, "step"),
        gradient_steps=1,
        policy_kwargs=dict(net_arch=[256, 256]),
        verbose=1,
        seed=42
    )
    
    # 5. Set up evaluation callback
    results_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "results")
    os.makedirs(results_dir, exist_ok=True)
    
    eval_callback = EvalCallback(
        eval_env,
        best_model_save_path=results_dir,
        log_path=results_dir,
        eval_freq=5000,
        n_eval_episodes=5,
        deterministic=True,
        render=False,
        verbose=1
    )
    
    # 6. Train!
    print(f"\n[*] Training for {cfg.total_timesteps} timesteps...")
    print(f"    Learning Rate: {cfg.learning_rate}")
    print(f"    Batch Size: {cfg.batch_size}")
    print(f"    Discount Factor (γ): {cfg.gamma}")
    print(f"    Mass Variation: ±{cfg.mass_variation_pct*100:.0f}%")
    
    model.learn(
        total_timesteps=cfg.total_timesteps,
        callback=eval_callback,
        progress_bar=True
    )
    
    # 7. Save the final model
    model_path = os.path.join(results_dir, "td3_energy_model")
    model.save(model_path)
    print(f"\n[*] Final model saved to: {model_path}.zip")
    
    # 8. Quick evaluation
    print("\n[*] Running quick evaluation (10 episodes)...")
    rewards = []
    for ep in range(10):
        obs, _ = eval_env.reset()
        total_reward = 0.0
        done = False
        while not done:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = eval_env.step(action)
            total_reward += reward
            done = terminated or truncated
        rewards.append(total_reward)
    
    print(f"[*] Average Episode Reward: {np.mean(rewards):.2f} ± {np.std(rewards):.2f}")
    print("[*] Training complete!")

if __name__ == "__main__":
    train()
