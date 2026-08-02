# ==============================================================================
# Simulation_Core/utils/plotting.py
# Generates the 5-panel scientific dashboard
# ==============================================================================
import matplotlib.pyplot as plt
import os

def generate_dashboard(df, save_path="Simulation_Core/results/Phase5_Upgraded_Results.png"):
    """
    Creates the 5-panel visual dashboard showing the simulation results.
    """
    # Create the results directory if it doesn't exist
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    # Set up a beautiful, dark-themed or professional matplotlib style
    plt.style.use('seaborn-v0_8-darkgrid')
    fig, axs = plt.subplots(5, 1, figsize=(14, 16), sharex=True)
    fig.suptitle('Cyber-Resilient LPV-Adaptive Active Suspension (IWM-EV)', fontsize=18, fontweight='bold', y=0.92)
    
    t = df['time']
    
    # --------------------------------------------------------------------------
    # Panel 1: Road Profile
    # --------------------------------------------------------------------------
    axs[0].plot(t, df['w_f'] * 1000, color='#34495e', linewidth=1.5)
    axs[0].set_ylabel('Road Profile\n(mm)', fontweight='bold')
    axs[0].set_title('Mixed-Pothole Track (ISO A + Random Drops)', loc='left', fontsize=12)
    axs[0].fill_between(t, df['w_f'] * 1000, 0, alpha=0.2, color='#95a5a6')
    
    # --------------------------------------------------------------------------
    # Panel 2: UKF Output (Road Severity ρ)
    # --------------------------------------------------------------------------
    axs[1].plot(t, df['rho_f'], color='#e74c3c', linewidth=2, label='ρ (Severity)')
    # Scale rho_dot for visibility
    axs[1].plot(t, df['rho_dot_f'] * 0.1, color='#e67e22', linestyle='--', linewidth=1.5, label='0.1 * ρ̇ (Rate of Change)')
    axs[1].set_ylabel('UKF Estimator\nOutput', fontweight='bold')
    axs[1].set_title('Novelty 1A: UKF Road Estimation & Early Warning', loc='left', fontsize=12)
    axs[1].legend(loc='upper right')
    axs[1].set_ylim([-0.5, 1.2])
    
    # --------------------------------------------------------------------------
    # Panel 3: Actuator Force
    # --------------------------------------------------------------------------
    axs[2].plot(t, df['u_f'], color='#9b59b6', linewidth=1.5)
    axs[2].set_ylabel('Actuator Force\n(N)', fontweight='bold')
    axs[2].set_title('Novelty 1B: LPV Controller Force Output', loc='left', fontsize=12)
    
    # --------------------------------------------------------------------------
    # Panel 4: Body Acceleration (The core comfort metric)
    # --------------------------------------------------------------------------
    axs[3].plot(t, df['base_z_c_ddot'], color='#7f8c8d', alpha=0.7, linewidth=1.5, label='Base Paper (Fixed H∞)')
    axs[3].plot(t, df['z_c_ddot'], color='#2980b9', linewidth=2.0, label='Our Project (Adaptive LPV)')
    axs[3].set_ylabel('Body Accel\n(m/s²)', fontweight='bold')
    axs[3].set_title('Ride Comfort Comparison (Lower is Better)', loc='left', fontsize=12)
    axs[3].legend(loc='upper right')
    
    # --------------------------------------------------------------------------
    # Panel 5: Energy Harvesting (Battery SoC)
    # --------------------------------------------------------------------------
    ax5 = axs[4]
    ax5_twin = ax5.twinx()
    
    # Plot power harvested as green bars (using Joules per dt = Watts)
    power_watts = df['energy_harvested_j'] / (t.iloc[1] - t.iloc[0])
    ax5.fill_between(t, power_watts, 0, color='#2ecc71', alpha=0.6, label='Harvested Power (W)')
    ax5.set_ylabel('Power (W)', color='#27ae60', fontweight='bold')
    
    # Plot Battery SoC on twin axis
    soc_pct = df['battery_soc'] * 100.0
    ax5_twin.plot(t, soc_pct, color='#f1c40f', linewidth=2.5, label='Battery SoC (%)')
    ax5_twin.set_ylabel('Battery SoC (%)', color='#f39c12', fontweight='bold')
    ax5_twin.set_ylim([min(soc_pct)-0.1, max(soc_pct)+0.1])
    
    ax5.set_title('Novelty 2: TD3 Deep Reinforcement Learning Energy Harvester', loc='left', fontsize=12)
    ax5.set_xlabel('Time (Seconds)', fontweight='bold')
    
    # Combine legends for panel 5
    lines_1, labels_1 = ax5.get_legend_handles_labels()
    lines_2, labels_2 = ax5_twin.get_legend_handles_labels()
    ax5.legend(lines_1 + lines_2, labels_1 + labels_2, loc='center left')
    
    plt.tight_layout()
    plt.savefig(save_path, dpi=300, bbox_inches='tight')
    print(f"\n[*] Dashboard saved to: {save_path}")
    plt.close()
