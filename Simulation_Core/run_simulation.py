# ==============================================================================
# Simulation_Core/run_simulation.py
# MASTER RUNNER - Connects the road, physics, estimator, and AI controller
# with full power electronics integration and Lyapunov monitoring
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
from Simulation_Core.physics.power_electronics import PowerConditioner

from Simulation_Core.road.road_generator import RoadGenerator, RealRoadGenerator
from Simulation_Core.network.mets_filter import METSFilter

# Import Controllers
from Simulation_Core.controllers.base_paper_controller import BasePaperController
from Simulation_Core.controllers.ukf_estimator import UKFRoadEstimator
from Simulation_Core.controllers.lpv_controller import LPVController
from Simulation_Core.controllers.td3_energy_agent import TD3EnergyAgent
from Simulation_Core.controllers.lyapunov_monitor import LyapunovMonitor

# Import Utils
from Simulation_Core.utils.logger import SimulationLogger
from Simulation_Core.utils.metrics import calculate_performance_metrics
from Simulation_Core.utils.plotting import generate_dashboard

def rk4_step(model, t, state, dt, u_f, u_r, w_f, w_r, v_ms):
    """Runge-Kutta 4th Order numerical integration step."""
    k1 = model.get_state_derivative(t, state, u_f, u_r, w_f, w_r, v_ms)
    k2 = model.get_state_derivative(t + dt/2, state + k1 * dt/2, u_f, u_r, w_f, w_r, v_ms)
    k3 = model.get_state_derivative(t + dt/2, state + k2 * dt/2, u_f, u_r, w_f, w_r, v_ms)
    k4 = model.get_state_derivative(t + dt, state + k3 * dt, u_f, u_r, w_f, w_r, v_ms)
    next_state = state + (dt / 6.0) * (k1 + 2*k2 + 2*k3 + k4)
    return next_state, k1 # Return k1 (dx at t) for logging

def run():
    print("=" * 70)
    print("[*] Cyber-Resilient LPV-Adaptive Active Suspension Simulation")
    print("[*] With Power Electronics Integration & Lyapunov Monitoring")
    print("=" * 70)
    
    start_time_real = time.time()
    
    # 1. Initialize configurations
    cfg = SimConfig()
    p = VehicleParams()
    
    print(f"[*] Initializing Test Track (10 seconds, {cfg.v_kmh} km/h)...")
    
    # Try real road CSV, fallback to synthetic
    csv_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 
                            "data", "real_road_profile.csv")
    if os.path.exists(csv_path):
        print(f"[*] Loading real-world road data from CSV...")
        road_gen = RealRoadGenerator(cfg, p, csv_path=csv_path)
    else:
        print(f"[*] Using synthetic mixed-pothole track...")
        road_gen = RoadGenerator(cfg, p)
    
    w_f_track, w_r_track = road_gen.generate_mixed_pothole_track()
    
    # 2. Initialize Models & Controllers
    print("[*] Booting Physics Engine and Controllers...")
    base_model = HalfCarModel()     # Physical car for base paper comparison
    our_model = HalfCarModel()      # Physical car for our project
    
    base_controller = BasePaperController()
    ukf = UKFRoadEstimator(cfg.dt)
    lpv = LPVController()
    
    # Load trained TD3 model if available, otherwise use heuristic
    td3_model_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 
                                   "results", "td3_energy_model.zip")
    td3_agent = TD3EnergyAgent(model_path=td3_model_path)
    if td3_agent.is_loaded:
        print("[*] TD3 Agent: Loaded trained model [OK]")
    else:
        print("[*] TD3 Agent: Using heuristic fallback (no trained model found)")
    
    mets = METSFilter(sigma=0.1)    # 10% tolerance for bandwidth saving
    lyapunov = LyapunovMonitor()    # Stability monitor
    
    # Initialize Power Electronics (LC filter for energy conditioning)
    power_conditioner = PowerConditioner(L=p.inductance_L, C=p.capacitance_C, R_load=10.0)
    lc_state = np.array([0.0, 0.0])  # [i_L, v_C] initial state
    K_v = 50.0  # Back-EMF proportionality constant (V per m/s)
    
    # 3. Initialize Logger
    logger = SimulationLogger(cfg.n_steps, cfg.dt)
    
    # 4. Initial States
    # [z_c, theta, z_uf, z_ur, z_c_dot, theta_dot, z_uf_dot, z_ur_dot]
    state_base = np.zeros(8)
    state_ours = np.zeros(8)
    
    battery_soc = 0.50      # Start at 50% battery
    battery_capacity_j = 50000.0 
    eta_actuator = 0.85     # Actuator efficiency when consuming power (COMFORT mode)
    
    # Previous step's forces (for UKF feedback)
    u_f_prev = 0.0
    u_r_prev = 0.0
    
    print("[*] Running 1kHz Simulation Loop...")
    print(f"[*] Steps: {cfg.n_steps} | dt: {cfg.dt*1000:.0f}ms | Speed: {cfg.v_kmh} km/h")
    
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
        state_base, dx_base = rk4_step(base_model, t, state_base, cfg.dt, u_f_base, u_r_base, w_f, w_r, cfg.v_ms)
        
        # --- OUR PROJECT SIMULATION (LPV + UKF + TD3 + METS + Power Electronics) ---
        
        # Step A: METS Network Filter
        transmit_flag, network_state = mets.should_transmit(state_ours)
        
        # Step B: UKF Road Estimation (FIX: pass actual actuator forces)
        if i == 0:
            dx_ours = np.zeros(8)
        
        rho_f, rho_dot_f, rho_r, rho_dot_r = ukf.estimate(
            z_s_ddot_f=dx_ours[4],
            z_s_ddot_r=dx_ours[4],
            u_f=u_f_prev,  # FIX: Using previous step's actual forces
            u_r=u_r_prev
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
        
        # Store for next iteration's UKF
        u_f_prev = u_f_ours
        u_r_prev = u_r_ours
        
        # Step E: Physics Integration (RK4)
        state_ours, dx_ours = rk4_step(our_model, t, state_ours, cfg.dt, u_f_ours, u_r_ours, w_f, w_r, cfg.v_ms)
        
        # Step F: Energy Harvesting with Power Electronics
        # Calculate relative velocities at each corner
        z_sf_dot = state_ours[4] - p.a * state_ours[5]
        z_sr_dot = state_ours[4] + p.b * state_ours[5]
        z_uf_dot = state_ours[6]
        z_ur_dot = state_ours[7]
        v_rel_f = z_sf_dot - z_uf_dot
        v_rel_r = z_sr_dot - z_ur_dot
        
        harvested_j = 0.0
        power_consumed = 0.0
        power_raw = 0.0
        power_conditioned = 0.0
        
        if mode_f == "ECO":
            # --- REGENERATION MODE ---
            # Back-EMF voltage from electromagnetic actuator
            v_em_raw = K_v * v_rel_f
            v_rectified = abs(v_em_raw)  # Full-bridge rectifier
            
            # Raw power before conditioning
            power_raw = v_rectified * abs(v_rel_f) * p.C_e / K_v  # Approximate raw power
            
            # Integrate LC filter ODE for one timestep (Euler for speed at 1kHz)
            lc_deriv = power_conditioner.get_state_derivative(lc_state, v_rectified)
            lc_state = lc_state + lc_deriv * cfg.dt
            
            # Clamp to physical limits
            lc_state[0] = max(0.0, lc_state[0])  # Current can't be negative in this topology
            lc_state[1] = max(0.0, lc_state[1])  # Capacitor voltage can't be negative after rectifier
            
            # Conditioned power delivered to battery
            i_load = lc_state[1] / power_conditioner.R_load
            power_conditioned = lc_state[1] * i_load * p.eta_regen  # V * I * η
            
            # Energy harvested this timestep
            harvested_j = power_conditioned * cfg.dt
            battery_soc += harvested_j / battery_capacity_j
            battery_soc = min(battery_soc, 1.0)
        else:
            # --- COMFORT MODE ---
            # Active pushing consumes battery energy
            # Power consumed = |Force × velocity| / actuator_efficiency
            power_consumed = abs(u_f_ours * v_rel_f) / eta_actuator
            consumed_j = power_consumed * cfg.dt
            battery_soc -= consumed_j / battery_capacity_j
            battery_soc = max(battery_soc, 0.0)
            
            # LC filter decays naturally when not being fed
            lc_deriv = power_conditioner.get_state_derivative(lc_state, 0.0)
            lc_state = lc_state + lc_deriv * cfg.dt
            lc_state[0] = max(0.0, lc_state[0])
            lc_state[1] = max(0.0, lc_state[1])
        
        # Step G: Lyapunov Monitoring
        V_x = lyapunov.compute_V(state_ours)
        lyapunov.track_settling(t, state_ours[0])
        
        # Log data
        logger.log_step(
            t, w_f, w_r, state_base, dx_base, state_ours, dx_ours,
            rho_f, rho_dot_f, rho_r, rho_dot_r,
            u_f_ours, u_r_ours, battery_soc, harvested_j, power_consumed,
            power_raw, power_conditioned, lc_state[1], lc_state[0],
            mode_f, mode_r, V_x
        )
        
        # Print progress
        if i % (cfg.n_steps // 10) == 0 and i > 0:
            print(f"    ... {int((i/cfg.n_steps)*100)}% complete | SoC: {battery_soc*100:.1f}% | Mode: {mode_f} | rho: {rho_f:.2f}")
            
    # ==========================================================================
    
    print("[*] Simulation complete.")
    elapsed = time.time() - start_time_real
    print(f"[*] Solved {cfg.n_steps} timesteps in {elapsed:.2f} seconds.")
    
    # 5. Extract Metrics & Generate Dashboard
    print("[*] Processing Results...")
    df = logger.to_dataframe()
    
    results_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
    os.makedirs(results_dir, exist_ok=True)
    df.to_csv(os.path.join(results_dir, "simulation_data.csv"), index=False)
    
    metrics = calculate_performance_metrics(df)
    bandwidth_saved = mets.get_bandwidth_saved_percentage()
    
    # Lyapunov settling time results
    worst_settling = lyapunov.get_worst_settling_time()
    avg_settling = lyapunov.get_average_settling_time()
    
    # Print Final Comparison Table
    print("\n" + "=" * 80)
    print("FINAL PERFORMANCE RESULTS")
    print("=" * 80)
    print(f"{'Metric':<35} | {'Base Paper':<15} | {'Our Project':<15} | Unit")
    print("-" * 80)
    print(f"{'RMS Body Accel (Comfort)':<35} | {metrics['rms_accel_base']:<15.4f} | {metrics['rms_accel_ours']:<15.4f} | m/s²")
    print(f"{'Peak Body Accel':<35} | {metrics['peak_accel_base']:<15.4f} | {metrics['peak_accel_ours']:<15.4f} | m/s²")
    print(f"{'RMS Tire Deflection (Safety)':<35} | {metrics['rms_tire_f_base']:<15.4f} | {metrics['rms_tire_f_ours']:<15.4f} | m")
    print(f"{'RMS Body Pitch':<35} | {metrics['rms_pitch_base']:<15.6f} | {metrics['rms_pitch_ours']:<15.6f} | rad")
    print(f"{'Settling Time (worst)':<35} | {metrics['settling_time_base']:<15.4f} | {metrics['settling_time_ours']:<15.4f} | s")
    print(f"{'Lyapunov Worst Settling':<35} | {'N/A':<15} | {worst_settling:<15.4f} | s")
    print(f"{'Lyapunov Avg Settling':<35} | {'N/A':<15} | {avg_settling:<15.4f} | s")
    print("-" * 80)
    print(f"{'Comfort Improvement':<35} | {'---':<15} | {metrics['comfort_improvement_pct']:<15.1f} | %")
    print(f"{'Energy Harvested':<35} | {'0.00':<15} | {metrics['total_harvested_kj']:<15.4f} | kJ")
    print(f"{'Energy Consumed (Actuator)':<35} | {'N/A':<15} | {metrics['total_consumed_kj']:<15.4f} | kJ")
    print(f"{'Net Energy Balance':<35} | {'N/A':<15} | {metrics['net_energy_kj']:<15.4f} | kJ")
    print(f"{'Peak Regen Power':<35} | {'0':<15} | {metrics['peak_regen_power_w']:<15.1f} | W")
    print(f"{'Avg Regen Power (ECO mode)':<35} | {'0':<15} | {metrics['avg_regen_power_w']:<15.1f} | W")
    soc_label = 'SoC: {0:.0f}% -> {1:.0f}%'.format(metrics['soc_initial']*100, metrics['soc_final']*100)
    print(f"{soc_label:<35} | {'---':<15} | {metrics['soc_change_pct']:<+15.2f} | %")
    print(f"{'Harvesting Efficiency':<35} | {'0.0%':<15} | {metrics['energy_efficiency_pct']:<15.1f} | %")
    print(f"{'CAN-bus Bandwidth Saved':<35} | {'0.0%':<15} | {bandwidth_saved:<15.1f} | %")
    print("=" * 80)
    
    # Stability verdict
    if worst_settling <= 0.5:
        print(f"[OK] LYAPUNOV GUARANTEE MET: Worst settling time {worst_settling:.3f}s <= 0.5s")
    else:
        print(f"[!!] LYAPUNOV GUARANTEE: Worst settling time {worst_settling:.3f}s (target <= 0.5s)")
    
    generate_dashboard(df)

if __name__ == "__main__":
    run()
