# ==============================================================================
# Simulation_Core/utils/metrics.py
# Calculates final performance metrics (RMS, Energy, Settling Time, Pitch)
# ==============================================================================
import numpy as np

def calculate_rms(signal):
    """Calculates the Root Mean Square of a signal."""
    return np.sqrt(np.mean(np.square(signal)))

def calculate_settling_time(time_vector, signal, threshold_m=0.001, event_start=3.0, event_end=7.0):
    """
    Calculates the settling time after the worst disturbance event.
    
    Finds the peak displacement within the event window, then measures
    how long it takes for the signal to return within ±threshold (1mm default).
    
    Returns:
        float: Settling time in seconds (from peak to within threshold)
    """
    dt = time_vector[1] - time_vector[0]
    
    # Find indices of the event window
    event_mask = (time_vector >= event_start) & (time_vector <= event_end)
    if not np.any(event_mask):
        return 0.0
    
    # Find the time of peak displacement within the event
    event_signal = np.abs(signal[event_mask])
    peak_idx_in_event = np.argmax(event_signal)
    peak_time_idx = np.where(event_mask)[0][peak_idx_in_event]
    
    # From peak, find when signal drops below threshold
    for i in range(peak_time_idx, len(signal)):
        if abs(signal[i]) < threshold_m:
            # Check if it stays below for at least 50ms (50 samples at 1kHz)
            window_end = min(i + 50, len(signal))
            if np.all(np.abs(signal[i:window_end]) < threshold_m):
                settling_time = (i - peak_time_idx) * dt
                return settling_time
    
    # If never settles, return remaining time
    return (len(signal) - peak_time_idx) * dt

def calculate_performance_metrics(sim_log_df):
    """
    Takes the full simulation log DataFrame and extracts the key metrics
    used for the final comparison table.
    """
    metrics = {}
    dt = 0.001  # 1ms timestep
    
    # 1. Ride Comfort (RMS Body Acceleration)
    metrics['rms_accel_base'] = calculate_rms(sim_log_df['base_z_c_ddot'])
    metrics['rms_accel_ours'] = calculate_rms(sim_log_df['z_c_ddot'])
    
    # 2. Peak Body Acceleration
    metrics['peak_accel_base'] = float(np.max(np.abs(sim_log_df['base_z_c_ddot'])))
    metrics['peak_accel_ours'] = float(np.max(np.abs(sim_log_df['z_c_ddot'])))
    
    # 3. Road Holding (RMS Tire Deflection)
    metrics['rms_tire_f_base'] = calculate_rms(sim_log_df['base_z_uf'] - sim_log_df['w_f'])
    metrics['rms_tire_f_ours'] = calculate_rms(sim_log_df['z_uf'] - sim_log_df['w_f'])
    
    # 4. Body Pitch (Angular Comfort)
    metrics['rms_pitch_base'] = calculate_rms(sim_log_df['base_theta'])
    metrics['rms_pitch_ours'] = calculate_rms(sim_log_df['theta'])
    
    # 5. Settling Time
    time = sim_log_df['time'].values
    metrics['settling_time_base'] = calculate_settling_time(
        time, sim_log_df['base_z_c'].values, threshold_m=0.001)
    metrics['settling_time_ours'] = calculate_settling_time(
        time, sim_log_df['z_c'].values, threshold_m=0.001)
    
    # 6. Energy Harvesting
    total_harvested_j = sim_log_df['energy_harvested_j'].sum()
    metrics['total_harvested_kj'] = total_harvested_j / 1000.0
    
    # 7. Total Energy Consumed (COMFORT mode actuator cost)
    total_consumed_j = sim_log_df['power_consumed_w'].sum() * dt
    metrics['total_consumed_kj'] = total_consumed_j / 1000.0
    
    # 8. Net Energy Balance
    metrics['net_energy_kj'] = metrics['total_harvested_kj'] - metrics['total_consumed_kj']
    
    # 9. Energy Efficiency (% of bump kinetic energy recovered)
    actuator_work_j = np.sum(np.abs(sim_log_df['u_f'] * sim_log_df['v_rel_f']) * dt)
    if actuator_work_j > 0:
        metrics['energy_efficiency_pct'] = (total_harvested_j / actuator_work_j) * 100.0
    else:
        metrics['energy_efficiency_pct'] = 0.0
    
    # 10. Peak Regenerated Power
    metrics['peak_regen_power_w'] = float(sim_log_df['power_conditioned_w'].max())
    
    # 11. Average Regenerated Power (only during ECO mode)
    eco_mask = sim_log_df['mode_f'] == 1.0
    if eco_mask.any():
        metrics['avg_regen_power_w'] = float(sim_log_df.loc[eco_mask, 'power_conditioned_w'].mean())
    else:
        metrics['avg_regen_power_w'] = 0.0
    
    # 12. SoC Change
    metrics['soc_initial'] = float(sim_log_df['battery_soc'].iloc[0])
    metrics['soc_final'] = float(sim_log_df['battery_soc'].iloc[-1])
    metrics['soc_change_pct'] = (metrics['soc_final'] - metrics['soc_initial']) * 100.0
    
    # 13. CAN Bandwidth (from METS — passed separately)
    # This will be added by the caller
    
    # 14. Comfort Improvement %
    if metrics['rms_accel_base'] > 0:
        metrics['comfort_improvement_pct'] = (
            (metrics['rms_accel_base'] - metrics['rms_accel_ours']) / metrics['rms_accel_base'] * 100.0
        )
    else:
        metrics['comfort_improvement_pct'] = 0.0
        
    return metrics
