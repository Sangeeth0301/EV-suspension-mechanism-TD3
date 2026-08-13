import pandas as pd
import json
import os
import numpy as np

def downsample_peak_preserving(csv_path, json_path, target_points=1000):
    """
    Reads the simulation CSV and performs a Peak-Preserving Downsampling.
    Instead of blindly taking every Nth row (which might miss a 3ms spike),
    it splits the data into buckets and selects the row with the maximum 
    actuator force/severity within that bucket.
    
    Enhanced to include: power electronics, pitch, energy balance, mode flags.
    """
    print(f"[*] Reading {csv_path}...")
    df = pd.read_csv(csv_path)
    
    total_points = len(df)
    print(f"[*] Total rows found: {total_points}")
    
    step = max(1, total_points // target_points)
    print(f"[*] Bucket size: {step} rows. Using Peak-Preserving decimation...")
    
    # Realistic EV Physics: Suspension powers the 48V Auxiliary Battery
    # Generator Efficiency is ~60%. Peak regeneration is capped at ~500W physically.
    df['instant_power_w'] = df['power_conditioned_w'].clip(upper=500.0)
    
    # Assume a 1.5 kWh 48V Aux Battery = 5,400,000 Joules
    INITIAL_BATTERY_J = 1.5 * 3.6e6
    AUX_DRAIN_W = 250.0  # Constant drain from car systems
    
    df['true_harvested_j'] = df['energy_harvested_j'].cumsum()
    df['aux_drained_j'] = df['time'] * AUX_DRAIN_W
    df['net_battery_j'] = INITIAL_BATTERY_J - df['aux_drained_j'] + df['true_harvested_j']
    df['net_battery_kwh'] = df['net_battery_j'] / 3.6e6
    
    CONSUMPTION_J_PER_KM = 540000.0
    df['remaining_range_km'] = (50.0 * 3.6e6 - (df['time'] * 10800.0) + df['true_harvested_j']) / CONSUMPTION_J_PER_KM
    df['harvested_range_m'] = df['true_harvested_j'] / (CONSUMPTION_J_PER_KM / 1000.0)
    
    # Cumulative energy for frontend charts
    dt = 0.001
    df['cum_harvested_kj'] = df['energy_harvested_j'].cumsum() / 1000.0
    df['cum_consumed_kj'] = (df['power_consumed_w'] * dt).cumsum() / 1000.0
    df['net_energy_kj'] = df['cum_harvested_kj'] - df['cum_consumed_kj']
    
    # Pitch in milliradians for frontend
    df['theta_mrad'] = df['theta'] * 1000.0
    df['base_theta_mrad'] = df['base_theta'] * 1000.0
    
    # Synthetic Cyber Attack boolean for the UI
    df['cyber_attack'] = (df['time'] > 4.5) & (df['time'] < 5.0) | (df['time'] > 7.5) & (df['time'] < 7.8)
    
    final_rows = []
    
    for i in range(0, total_points, step):
        bucket = df.iloc[i:i+step]
        if bucket.empty:
            continue
        max_force_idx = bucket['u_f'].abs().idxmax()
        best_row = bucket.loc[max_force_idx].copy()
        final_rows.append(best_row)
        
    df_downsampled = pd.DataFrame(final_rows)
    df_downsampled = df_downsampled.round(4)
    
    columns_to_keep = [
        'time', 
        'w_f',
        'w_r',
        'rho_f', 
        'rho_dot_f', 
        'u_f',
        'u_r',
        'base_z_c',
        'base_z_c_ddot',
        'base_theta_mrad',
        'z_c',
        'z_c_ddot',
        'z_uf',
        'z_ur',
        'theta_mrad',
        'v_rel_f',
        'battery_soc', 
        'energy_harvested_j',
        'instant_power_w',
        'power_raw_w',
        'power_conditioned_w',
        'power_consumed_w',
        'v_capacitor',
        'mode_f',
        'lyapunov_v',
        'cum_harvested_kj',
        'cum_consumed_kj',
        'net_energy_kj',
        'net_battery_kwh',
        'remaining_range_km',
        'harvested_range_m',
        'cyber_attack'
    ]
    
    # Only keep columns that exist (graceful handling)
    available_cols = [c for c in columns_to_keep if c in df_downsampled.columns]
    df_final = df_downsampled[available_cols]
    data = df_final.to_dict(orient='records')
    
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    
    print(f"[*] Writing {len(data)} peak-preserved points to {json_path}...")
    with open(json_path, 'w') as f:
        json.dump(data, f)
        
    print(f"[*] Dataset complete! {len(available_cols)} fields per point.")
    print(f"[*] New fields: power_raw_w, power_conditioned_w, power_consumed_w, theta_mrad,")
    print(f"    v_capacitor, mode_f, lyapunov_v, cum_harvested_kj, cum_consumed_kj, net_energy_kj")

if __name__ == "__main__":
    csv_in = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 
                          "Simulation_Core", "results", "simulation_data.csv")
    json_out = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                            "public", "sim_data.json")
    
    downsample_peak_preserving(csv_in, json_out)
