# ==============================================================================
# Simulation_Core/physics/power_electronics.py
# Models the electrical circuit (Shift Transformer, Inductor, Capacitor)
# for stabilizing harvested regenerative energy.
# ==============================================================================
import numpy as np

class PowerConditioner:
    """
    Models an LC Low-Pass Filter used in power electronics to smooth 
    rectified AC voltage/current into stable DC for battery charging.
    """
    def __init__(self, L, C, R_load=10.0):
        self.L = L  # Inductance (Henries)
        self.C = C  # Capacitance (Farads)
        self.R_load = R_load  # Equivalent battery load resistance (Ohms)
        
    def get_state_derivative(self, state, v_in):
        """
        Calculates the derivatives for the LC filter ODE.
        
        State vector:
        [0] i_L : Current through inductor (Amps)
        [1] v_C : Voltage across capacitor (Volts)
        
        Inputs:
        v_in : Rectified input voltage from the regenerative actuator (Volts)
        """
        i_L = state[0]
        v_C = state[1]
        
        # di_L/dt = (v_in - v_C) / L
        di_L_dt = (v_in - v_C) / self.L
        
        # dv_C/dt = (i_L - i_load) / C
        # i_load = v_C / R_load
        dv_C_dt = (i_L - (v_C / self.R_load)) / self.C
        
        return np.array([di_L_dt, dv_C_dt])
