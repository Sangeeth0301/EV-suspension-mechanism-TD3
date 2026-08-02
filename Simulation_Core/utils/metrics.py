# ==============================================================================
# Simulation_Core/utils/metrics.py
# Calculates final performance metrics (RMS, Energy, Bandwidth)
# ==============================================================================
import numpy as np

def calculate_rms(signal):
    """Calculates the Root Mean Square of a signal."""
    return np.sqrt(np.mean(np.square(signal)))

def calculate_performance_metrics(sim_log_df):
    """
    Takes the full simulation log DataFrame and extracts the key metrics
    used for the final comparison table.
    """
    metrics = {}
    
    # 1. Ride Comfort (RMS Body Acceleration)
    metrics['rms_accel_base'] = calculate_rms(sim_log_df['base_z_c_ddot'])
    metrics['rms_accel_ours'] = calculate_rms(sim_log_df['z_c_ddot'])
    
    # 2. Road Holding (RMS Tire Deflection)
    metrics['rms_tire_f_base'] = calculate_rms(sim_log_df['base_z_uf'] - sim_log_df['w_f'])
    metrics['rms_tire_f_ours'] = calculate_rms(sim_log_df['z_uf'] - sim_log_df['w_f'])
    
    # 3. Energy Harvesting
    # Total energy consumed vs harvested
    total_harvested_j = sim_log_df['energy_harvested_j'].sum()
    metrics['total_harvested_kj'] = total_harvested_j / 1000.0
    
    # Calculate percentage of kinetic bump energy recovered
    # (Approximation based on actuator work)
    actuator_work_j = np.sum(np.abs(sim_log_df['u_f'] * sim_log_df['v_rel_f']) * 0.001) # dt=0.001
    if actuator_work_j > 0:
        metrics['energy_efficiency_pct'] = (total_harvested_j / actuator_work_j) * 100.0
    else:
        metrics['energy_efficiency_pct'] = 0.0
        
    return metrics
