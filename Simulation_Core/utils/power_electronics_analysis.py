import sys
import os
import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

# Add project root to sys.path
current_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.abspath(os.path.join(current_dir, '..', '..'))
if project_root not in sys.path:
    sys.path.insert(0, project_root)

from Simulation_Core.config.vehicle_params import VehicleParams
from Simulation_Core.physics.power_electronics import PowerConditioner

def main():
    p = VehicleParams()
    
    # Calculate System Weight
    total_added_weight = p.get_power_electronics_weight()
    print(f"Total Added Weight for Power Electronics (Half-Car): {total_added_weight:.2f} kg")
    
    # Instantiate the LC filter
    power_conditioner = PowerConditioner(L=p.inductance_L, C=p.capacitance_C, R_load=10.0)
    
    # Simulate a bumpy road which generates erratic raw voltage
    time_span = [0, 2.0]  # 2 seconds
    t_eval = np.linspace(time_span[0], time_span[1], 2000)
    
    # A realistic suspension v_rel would be a mix of frequencies (e.g. body bounce 1-2 Hz, wheel hop 10-12 Hz)
    def v_rel_fake(t):
        return 0.5 * np.sin(2 * np.pi * 1.5 * t) + 1.2 * np.sin(2 * np.pi * 11.0 * t) + np.random.normal(0, 0.1)
        
    # The electromagnetic actuator generates a voltage proportional to v_rel
    # V_em = K_em * v_rel (using C_e as a proxy for the back-EMF constant for simplicity)
    k_v = 50.0 # Proportionality constant
    
    def power_electronics_ode(t, state):
        v_in_raw = k_v * v_rel_fake(t)
        v_rect = abs(v_in_raw) # Rectifier turns AC to DC
        return power_conditioner.get_state_derivative(state, v_rect)

    # Initial state [i_L, v_C]
    initial_state = [0.0, 0.0]
    
    # Solve ODE
    solution = solve_ivp(power_electronics_ode, time_span, initial_state, t_eval=t_eval, method='RK45')
    
    t_out = solution.t
    i_L_out = solution.y[0]
    v_C_out = solution.y[1]
    
    # Calculate input and output currents
    v_in_raw_arr = np.array([k_v * v_rel_fake(t) for t in t_out])
    v_rect_arr = np.abs(v_in_raw_arr)
    
    # The output current to the battery is v_C / R_load
    i_load_out = v_C_out / power_conditioner.R_load
    
    # Assuming input raw current magnitude is approximately v_rect / R_load for plotting scale comparison
    i_raw_equivalent = v_rect_arr / power_conditioner.R_load
    
    # Plotting
    plt.figure(figsize=(12, 6))
    
    plt.plot(t_out, i_raw_equivalent, label='Raw Rectified Current (Unstable)', color='orange', alpha=0.6)
    plt.plot(t_out, i_load_out, label='Conditioned Current (Stable DC to Battery)', color='green', linewidth=2)
    
    plt.title(f'Power Electronics Stabilization\n(Added System Weight: {total_added_weight:.2f} kg)')
    plt.xlabel('Time (s)')
    plt.ylabel('Current (Amps)')
    plt.legend()
    plt.grid(True)
    
    output_path = os.path.join(current_dir, 'power_stability_plot.png')
    plt.savefig(output_path)
    print(f"Plot saved to: {output_path}")

if __name__ == '__main__':
    main()
