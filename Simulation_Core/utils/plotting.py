# ==============================================================================
# Simulation_Core/utils/plotting.py
# Generates the 8-panel scientific dashboard
# ==============================================================================
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
import numpy as np
import os

def generate_dashboard(df, save_path="Simulation_Core/results/Phase5_Upgraded_Results.png"):
    """
    Creates the 8-panel visual dashboard showing the complete simulation results.
    
    Panels:
    1. Road Profile (front + rear)
    2. UKF Estimation (ρ, ρ̇)
    3. Actuator Forces (front + rear)
    4. Body Acceleration Comparison
    5. Body Pitch Angle Comparison
    6. Power Electronics (raw vs conditioned)
    7. Energy Balance (consumed vs harvested)
    8. Battery SoC with COMFORT/ECO mode overlay
    """
    # Create the results directory if it doesn't exist
    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    
    plt.style.use('seaborn-v0_8-darkgrid')
    fig = plt.figure(figsize=(18, 28))
    gs = gridspec.GridSpec(8, 1, hspace=0.35)
    fig.suptitle('Cyber-Resilient LPV-Adaptive Active Suspension — Complete Results', 
                 fontsize=20, fontweight='bold', y=0.995)
    
    t = df['time']
    
    # Color scheme
    c_road = '#64748b'
    c_road_rear = '#94a3b8'
    c_rho = '#ef4444'
    c_rho_dot = '#f59e0b'
    c_force_f = '#8b5cf6'
    c_force_r = '#a78bfa'
    c_base = '#7f8c8d'
    c_ours = '#3b82f6'
    c_raw = '#f97316'
    c_conditioned = '#10b981'
    c_consumed = '#ef4444'
    c_harvested = '#10b981'
    c_soc = '#eab308'
    c_comfort_bg = 'rgba(59, 130, 246, 0.15)'
    c_eco_bg = 'rgba(16, 185, 129, 0.15)'
    
    # --------------------------------------------------------------------------
    # Panel 1: Road Profile
    # --------------------------------------------------------------------------
    ax1 = fig.add_subplot(gs[0])
    ax1.plot(t, df['w_f'] * 1000, color=c_road, linewidth=1.2, label='Front wheel')
    ax1.plot(t, df['w_r'] * 1000, color=c_road_rear, linewidth=0.8, alpha=0.7, label='Rear wheel (delayed)')
    ax1.fill_between(t, df['w_f'] * 1000, 0, alpha=0.15, color=c_road)
    ax1.set_ylabel('Height (mm)', fontweight='bold')
    ax1.set_title('1. Road Profile — Mixed-Pothole Track', loc='left', fontsize=13, fontweight='bold')
    ax1.legend(loc='upper right', fontsize=9)
    ax1.axhspan(-80, 0, alpha=0.03, color='red', label='Pothole zone')
    
    # --------------------------------------------------------------------------
    # Panel 2: UKF Output (Road Severity ρ)
    # --------------------------------------------------------------------------
    ax2 = fig.add_subplot(gs[1])
    ax2.plot(t, df['rho_f'], color=c_rho, linewidth=2, label='ρ (Severity)')
    ax2.plot(t, df['rho_dot_f'] * 0.1, color=c_rho_dot, linestyle='--', linewidth=1.5, label='0.1 × ρ̇ (Rate)')
    ax2.set_ylabel('UKF Output', fontweight='bold')
    ax2.set_title('2. Novelty 1A: UKF Road Estimation & Early Warning', loc='left', fontsize=13, fontweight='bold')
    ax2.legend(loc='upper right', fontsize=9)
    ax2.set_ylim([-0.5, 1.2])
    ax2.axhline(y=0.7, color='red', linestyle=':', alpha=0.5, label='Danger threshold')
    
    # --------------------------------------------------------------------------
    # Panel 3: Actuator Force (Front + Rear)
    # --------------------------------------------------------------------------
    ax3 = fig.add_subplot(gs[2])
    ax3.plot(t, df['u_f'], color=c_force_f, linewidth=1.2, label='Front u_f', alpha=0.9)
    ax3.plot(t, df['u_r'], color=c_force_r, linewidth=0.8, label='Rear u_r', alpha=0.7)
    ax3.fill_between(t, df['u_f'], 0, alpha=0.1, color=c_force_f)
    ax3.set_ylabel('Force (N)', fontweight='bold')
    ax3.set_title('3. Novelty 1B: LPV Adaptive Controller Output', loc='left', fontsize=13, fontweight='bold')
    ax3.legend(loc='upper right', fontsize=9)
    ax3.axhline(y=6000, color='red', linestyle=':', alpha=0.3, label='Max force')
    ax3.axhline(y=-6000, color='red', linestyle=':', alpha=0.3)
    
    # --------------------------------------------------------------------------
    # Panel 4: Body Acceleration (Comfort comparison)
    # --------------------------------------------------------------------------
    ax4 = fig.add_subplot(gs[3])
    ax4.plot(t, df['base_z_c_ddot'], color=c_base, alpha=0.6, linewidth=1.2, label='Base Paper (Fixed H∞)')
    ax4.plot(t, df['z_c_ddot'], color=c_ours, linewidth=1.5, label='Our Project (Adaptive LPV)')
    ax4.set_ylabel('Body Accel (m/s²)', fontweight='bold')
    ax4.set_title('4. Ride Comfort — Body Acceleration Comparison (Lower = Better)', loc='left', fontsize=13, fontweight='bold')
    ax4.legend(loc='upper right', fontsize=9)
    
    # --------------------------------------------------------------------------
    # Panel 5: Body Pitch Angle
    # --------------------------------------------------------------------------
    ax5 = fig.add_subplot(gs[4])
    ax5.plot(t, df['base_theta'] * 1000, color=c_base, alpha=0.6, linewidth=1.2, label='Base Paper')
    ax5.plot(t, df['theta'] * 1000, color=c_ours, linewidth=1.5, label='Our Project')
    ax5.set_ylabel('Pitch (mrad)', fontweight='bold')
    ax5.set_title('5. Angular Comfort — Body Pitch Comparison', loc='left', fontsize=13, fontweight='bold')
    ax5.legend(loc='upper right', fontsize=9)
    
    # --------------------------------------------------------------------------
    # Panel 6: Power Electronics (Raw vs Conditioned)
    # --------------------------------------------------------------------------
    ax6 = fig.add_subplot(gs[5])
    ax6.plot(t, df['power_raw_w'], color=c_raw, alpha=0.5, linewidth=0.8, label='Raw Regen Power')
    ax6.plot(t, df['power_conditioned_w'], color=c_conditioned, linewidth=1.5, label='LC-Conditioned Power')
    ax6.fill_between(t, df['power_conditioned_w'], 0, alpha=0.2, color=c_conditioned)
    ax6.set_ylabel('Power (W)', fontweight='bold')
    ax6.set_title('6. Power Electronics — LC Filter Stabilization (L=10mH, C=4700μF)', loc='left', fontsize=13, fontweight='bold')
    ax6.legend(loc='upper right', fontsize=9)
    
    # Add capacitor voltage on twin axis
    ax6_twin = ax6.twinx()
    ax6_twin.plot(t, df['v_capacitor'], color='#06b6d4', linewidth=1, alpha=0.5, label='V_cap (V)')
    ax6_twin.set_ylabel('V_cap (V)', color='#06b6d4', fontweight='bold')
    ax6_twin.tick_params(axis='y', labelcolor='#06b6d4')
    
    # --------------------------------------------------------------------------
    # Panel 7: Energy Balance (Consumed vs Harvested)
    # --------------------------------------------------------------------------
    ax7 = fig.add_subplot(gs[6])
    
    # Cumulative energy
    dt = t.iloc[1] - t.iloc[0] if len(t) > 1 else 0.001
    cum_harvested = np.cumsum(df['energy_harvested_j']) / 1000.0  # kJ
    cum_consumed = np.cumsum(df['power_consumed_w'] * dt) / 1000.0  # kJ
    
    ax7.fill_between(t, cum_harvested, 0, alpha=0.3, color=c_harvested, label='Cumulative Harvested')
    ax7.fill_between(t, -cum_consumed, 0, alpha=0.3, color=c_consumed, label='Cumulative Consumed')
    ax7.plot(t, cum_harvested, color=c_harvested, linewidth=2)
    ax7.plot(t, -cum_consumed, color=c_consumed, linewidth=2)
    ax7.plot(t, cum_harvested - cum_consumed, color='white', linewidth=2, linestyle='--', label='Net Balance')
    ax7.axhline(y=0, color='white', alpha=0.3, linewidth=0.5)
    ax7.set_ylabel('Energy (kJ)', fontweight='bold')
    ax7.set_title('7. Novelty 2: Energy Balance — Harvested vs Consumed', loc='left', fontsize=13, fontweight='bold')
    ax7.legend(loc='upper left', fontsize=9)
    
    # --------------------------------------------------------------------------
    # Panel 8: Battery SoC with Mode Overlay
    # --------------------------------------------------------------------------
    ax8 = fig.add_subplot(gs[7])
    
    soc_pct = df['battery_soc'] * 100.0
    ax8.plot(t, soc_pct, color=c_soc, linewidth=2.5, label='Battery SoC (%)')
    
    # Shade COMFORT vs ECO regions
    mode_changes = df['mode_f'].diff().fillna(0)
    eco_regions = df['mode_f'] == 1.0
    
    # Fill ECO regions with green, COMFORT with blue
    ax8.fill_between(t, soc_pct.min() - 1, soc_pct.max() + 1, 
                     where=eco_regions, alpha=0.08, color='#10b981', label='ECO Mode')
    ax8.fill_between(t, soc_pct.min() - 1, soc_pct.max() + 1, 
                     where=~eco_regions, alpha=0.05, color='#3b82f6', label='COMFORT Mode')
    
    ax8.axhline(y=15, color='red', linestyle=':', alpha=0.5, label='Critical SoC (15%)')
    ax8.set_ylabel('SoC (%)', fontweight='bold')
    ax8.set_xlabel('Time (Seconds)', fontweight='bold', fontsize=12)
    ax8.set_title('8. TD3 DRL Battery Management — SoC with Mode Overlay', loc='left', fontsize=13, fontweight='bold')
    ax8.legend(loc='upper right', fontsize=9)
    ax8.set_ylim([max(0, soc_pct.min() - 2), min(100, soc_pct.max() + 2)])
    
    plt.savefig(save_path, dpi=300, bbox_inches='tight', facecolor='white')
    print(f"\n[*] 8-Panel Dashboard saved to: {save_path}")
    plt.close()
