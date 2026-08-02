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
    """
    print(f"[*] Reading {csv_path}...")
    df = pd.read_csv(csv_path)
    
    total_points = len(df)
    print(f"[*] Total rows found: {total_points}")
    
    # We want a 60fps smooth scrubber, so 1000 points is perfect for a 10s simulation.
    step = max(1, total_points // target_points)
    print(f"[*] Bucket size: {step} rows. Using Peak-Preserving decimation...")
    
    # Realistic EV Physics: The suspension doesn't power the main traction motors.
    # It powers the 48V Auxiliary Battery (which runs the computers, A/C, and sensors).
    # Generator Efficiency is ~60%. Peak regeneration is capped at ~500W physically.
    df['instant_power_w'] = ((df['u_f'] * df['v_rel_f']).abs() * 0.6).clip(upper=500.0)

    # Assume a 1.5 kWh 48V Aux Battery = 5,400,000 Joules
    INITIAL_BATTERY_J = 1.5 * 3.6e6
    # Constant drain from car computers, displays, and sensors (250 Watts)
    AUX_DRAIN_W = 250.0
    
    # Recalculate true harvested energy based on the capped and efficient instant power
    # cumulative trapezoidal integration of instant_power_w
    df['true_harvested_j'] = df['instant_power_w'].cumsum() * 0.001  # dt=0.001s
    
    df['aux_drained_j'] = df['time'] * AUX_DRAIN_W
    df['net_battery_j'] = INITIAL_BATTERY_J - df['aux_drained_j'] + df['true_harvested_j']
    df['net_battery_kwh'] = df['net_battery_j'] / 3.6e6
    
    # 540 J/m is the traction consumption. We show how much MAIN battery range was saved
    # by letting the suspension power the aux systems instead of the main battery doing it.
    CONSUMPTION_J_PER_KM = 540000.0
    df['remaining_range_km'] = (50.0 * 3.6e6 - (df['time'] * 10800.0) + df['true_harvested_j']) / CONSUMPTION_J_PER_KM
    df['harvested_range_m'] = df['true_harvested_j'] / (CONSUMPTION_J_PER_KM / 1000.0)
    
    # Generate a synthetic Cyber Attack boolean for the UI (active when rho_f is very high and random noise)
    # This simulates a DoS attack during a critical bump.
    df['cyber_attack'] = (df['time'] > 4.5) & (df['time'] < 5.0) | (df['time'] > 7.5) & (df['time'] < 7.8)
    
    final_rows = []
    
    # Process in buckets
    for i in range(0, total_points, step):
        bucket = df.iloc[i:i+step]
        if bucket.empty:
            continue
            
        # Find the index of the row with the maximum absolute force in this bucket
        max_force_idx = bucket['u_f'].abs().idxmax()
        best_row = bucket.loc[max_force_idx].copy()
        
        final_rows.append(best_row)
        
    df_downsampled = pd.DataFrame(final_rows)
    df_downsampled = df_downsampled.round(4)
    
    columns_to_keep = [
        'time', 
        'w_f', 
        'rho_f', 
        'rho_dot_f', 
        'u_f', 
        'base_z_c', 
        'z_c',
        'z_uf',
        'battery_soc', 
        'energy_harvested_j',
        'instant_power_w',
        'net_battery_kwh',
        'remaining_range_km',
        'harvested_range_m',
        'cyber_attack'
    ]
    
    df_final = df_downsampled[columns_to_keep]
    data = df_final.to_dict(orient='records')
    
    os.makedirs(os.path.dirname(json_path), exist_ok=True)
    
    print(f"[*] Writing {len(data)} peak-preserved points to {json_path}...")
    with open(json_path, 'w') as f:
        json.dump(data, f)
        
    print("[*] Optimal Dataset complete! Instant Power & Cyber Attack flags included.")

if __name__ == "__main__":
    csv_in = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), 
                          "Simulation_Core", "results", "simulation_data.csv")
    json_out = os.path.join(os.path.dirname(os.path.dirname(__file__)), 
                            "public", "sim_data.json")
    
    downsample_peak_preserving(csv_in, json_out)
