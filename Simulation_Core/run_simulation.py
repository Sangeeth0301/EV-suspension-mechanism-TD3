# ==============================================================================
# Simulation_Core/run_simulation.py
# MASTER RUNNER - Connects the road, physics, estimator, and AI controller
# ==============================================================================
import numpy as np
import time
import sys
import os

# Ensure the root directory is in the Python path so absolute imports work
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

# Import Configurations
from Simulation_Core.config.sim_config import SimConfig
from Simulation_Core.config.vehicle_params import VehicleParams

# Import Core Physics
from Simulation_Core.physics.half_car_model import HalfCarModel

# Import Generators & Networks
from Simulation_Core.road.road_generator import RoadGenerator
from Simulation_Core.network.mets_filter import METSFilter

# Import Controllers
from Simulation_Core.controllers.base_paper_controller import BasePaperController
from Simulation_Core.controllers.ukf_estimator import UKFRoadEstimator
from Simulation_Core.controllers.lpv_controller import LPVController
from Simulation_Core.controllers.td3_energy_agent import TD3EnergyAgent

# Import Utils
from Simulation_Core.utils.logger import SimulationLogger
from Simulation_Core.utils.metrics import calculate_performance_metrics
from Simulation_Core.utils.plotting import generate_dashboard

def run():
    print("="*60)
    print("[*] Starting Cyber-Resilient LPV-Adaptive Active Suspension Sim")
    print("="*60)
    
    start_time_real = time.time()
    
    # 1. Initialize configurations
    cfg = SimConfig()
    p = VehicleParams()
    
    print(f"[*] Initializing Test Track (10 seconds, {cfg.v_kmh} km/h)...")
    road_gen = RoadGenerator(cfg, p)
    w_f_track, w_r_track = road_gen.generate_mixed_pothole_track()
    
    # 2. Initialize Models & Controllers
    print("[*] Booting Physics Engine and Controllers...")
    base_model = HalfCarModel()     # Physical car for base paper comparison
    our_model = HalfCarModel()      # Physical car for our project
    
    base_controller = BasePaperController()
    ukf = UKFRoadEstimator(cfg.dt)
    lpv = LPVController()
    td3_agent = TD3EnergyAgent()    # Fallback to heuristic if no model loaded
    mets = METSFilter(sigma=0.1)    # 10% tolerance for bandwidth saving
    
    # 3. Initialize Logger
    logger = SimulationLogger(cfg.n_steps, cfg.dt)
    
    # 4. Initial States
    # [z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot]
    state_base = np.zeros(8)
    state_ours = np.zeros(8)
    
    battery_soc = 0.50 # Start at 50% battery
    battery_capacity_j = 50000.0 
    
    print("[*] Running 1kHz Simulation Loop...")
    
    # ==========================================================================
    # MAIN SIMULATION LOOP (Runs every 1 millisecond)
    # ==========================================================================
    for i in range(cfg.n_steps):
        t = cfg.t_vector[i]
        
        # 1. Read the road
        w_f = w_f_track[i]
        w_r = w_r_track[i]
        
        # --- BASE PAPER SIMULATION (Fixed H∞) ---
        u_f_base, u_r_base = base_controller.compute_force(state_base)
        dx_base = base_model.get_state_derivative(t, state_base, u_f_base, u_r_base, w_f, w_r, cfg.v_ms)
        state_base += dx_base * cfg.dt
        
        # --- OUR PROJECT SIMULATION (LPV + UKF + TD3 + METS) ---
        
        # Step A: METS Network Filter
        # Only process control if the network transmits the packet
        transmit_flag, network_state = mets.should_transmit(state_ours)
        
        # Step B: UKF Road Estimation (Reads noisy body acceleration)
        # Using previous step's state derivative for acceleration
        if i == 0:
            dx_ours = np.zeros(8)
        
        rho_f, rho_dot_f, rho_r, rho_dot_r = ukf.estimate(
            z_s_ddot_f=dx_ours[4], # Approx body accel 
            z_s_ddot_r=dx_ours[4],
            u_f=0.0, u_r=0.0
        )
        
        # Step C: TD3 DRL Energy Agent Decision
        mode_f = td3_agent.get_mode(battery_soc, rho_f, rho_dot_f)
        mode_r = td3_agent.get_mode(battery_soc, rho_r, rho_dot_r)
        
        # Step D: LPV Control Calculation (using the network-received state)
        u_f_ours, u_r_ours = lpv.compute_force(
            network_state, 
            rho_f, rho_dot_f, rho_r, rho_dot_r, 
            mode_f, mode_r
        )
        
        # Step E: Physics Integration
        dx_ours = our_model.get_state_derivative(t, state_ours, u_f_ours, u_r_ours, w_f, w_r, cfg.v_ms)
        state_ours += dx_ours * cfg.dt
        
        # Step F: Energy Harvesting Calculation
        z_sf_dot = state_ours[4] - p.a * state_ours[5]
        z_uf_dot = state_ours[6]
        v_rel_f = z_sf_dot - z_uf_dot
        
        harvested_j = 0.0
        if mode_f == "ECO" and (u_f_ours * v_rel_f) > 0:
            power_w = u_f_ours * v_rel_f * p.eta_regen
            harvested_j = power_w * cfg.dt
            battery_soc += harvested_j / battery_capacity_j
            battery_soc = min(battery_soc, 1.0) # Cap at 100%
            
        # Log data
        logger.log_step(
            t, w_f, state_base, dx_base, state_ours, dx_ours,
            rho_f, rho_dot_f, u_f_ours, battery_soc, harvested_j
        )
        
        # Print progress
        if i % (cfg.n_steps // 10) == 0 and i > 0:
            print(f"    ... {int((i/cfg.n_steps)*100)}% complete")
            
    # ==========================================================================
    
    print("[*] Simulation complete.")
    elapsed = time.time() - start_time_real
    print(f"[*] Solved {cfg.n_steps} timesteps in {elapsed:.2f} seconds.")
    
    # 5. Extract Metrics & Generate Dashboard
    print("[*] Processing Results...")
    df = logger.to_dataframe()
    df.to_csv("Simulation_Core/results/simulation_data.csv", index=False)
    
    metrics = calculate_performance_metrics(df)
    bandwidth_saved = mets.get_bandwidth_saved_percentage()
    
    # Print Final Comparison Table
    print("\n" + "="*70)
    print("FINAL PERFORMANCE RESULTS")
    print("="*70)
    print(f"{'Metric':<30} | {'Base Paper':<15} | {'Our Project':<15}")
    print("-" * 70)
    print(f"{'RMS Body Accel (Comfort)':<30} | {metrics['rms_accel_base']:<15.4f} | {metrics['rms_accel_ours']:<15.4f} (m/s²)")
    print(f"{'RMS Tire Deflection (Safety)':<30} | {metrics['rms_tire_f_base']:<15.4f} | {metrics['rms_tire_f_ours']:<15.4f} (m)")
    print(f"{'Energy Harvested':<30} | {'0.00':<15} | {metrics['total_harvested_kj']:<15.2f} (kJ)")
    print(f"{'Harvesting Efficiency':<30} | {'0.0%':<15} | {metrics['energy_efficiency_pct']:<15.1f} (%)")
    print(f"{'CAN-bus Bandwidth Saved':<30} | {'0.0%':<15} | {bandwidth_saved:<15.1f} (%)")
    print("="*70)
    
    generate_dashboard(df)

if __name__ == "__main__":
    run()
