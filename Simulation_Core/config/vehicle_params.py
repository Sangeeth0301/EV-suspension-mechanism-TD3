# ==============================================================================
# Simulation_Core/config/vehicle_params.py
# Physical constants for the In-Wheel Motor (IWM) EV Half-Car Model
# ==============================================================================

class VehicleParams:
    """
    Physical parameters of the half-car active suspension model.
    These values represent a premium EV sedan with In-Wheel Motors.
    """
    
    # --------------------------------------------------------------------------
    # Sprung Mass (Car Body)
    # --------------------------------------------------------------------------
    ms = 730.0          # Half-car sprung mass (kg)
    I_phi = 1222.0      # Pitch moment of inertia (kg*m^2)
    
    a = 1.1             # Distance from CG to front axle (m)
    b = 1.5             # Distance from CG to rear axle (m)
    
    # Equivalent sprung masses per corner (calculated from geometry)
    ms_f = ms * b / (a + b)  # Front sprung mass equivalent (~421 kg)
    ms_r = ms * a / (a + b)  # Rear sprung mass equivalent (~309 kg)

    # --------------------------------------------------------------------------
    # Unsprung Mass (In-Wheel Motors)
    # --------------------------------------------------------------------------
    # Note: Normal car wheels are ~25 kg. IWM wheels are significantly heavier.
    mu_f = 45.0         # Front unsprung mass (kg) - contains IWM
    mu_r = 45.0         # Rear unsprung mass (kg) - contains IWM

    # --------------------------------------------------------------------------
    # Suspension Stiffness & Damping (Passive baselines)
    # --------------------------------------------------------------------------
    ks_f = 18000.0      # Front suspension spring stiffness (N/m)
    ks_r = 22000.0      # Rear suspension spring stiffness (N/m)
    
    cs_f = 1200.0       # Front passive damping coefficient (N*s/m)
    cs_r = 1200.0       # Rear passive damping coefficient (N*s/m)
    
    # --------------------------------------------------------------------------
    # Tire Stiffness
    # --------------------------------------------------------------------------
    kt_f = 190000.0     # Front tire stiffness (N/m)
    kt_r = 190000.0     # Rear tire stiffness (N/m)
    
    # --------------------------------------------------------------------------
    # In-Wheel Motor (IWM) Electromagnetic Disturbance Parameters
    # --------------------------------------------------------------------------
    # F_em(t) = k_em * i_d * sin(p * theta_m)
    k_em = 15.0         # Electromechanical coupling factor (N/A)
    i_d = 20.0          # Stator current amplitude (A)
    p_pole = 8          # Number of pole pairs
    
    # --------------------------------------------------------------------------
    # Regenerative Electromagnetic Actuator Parameters
    # --------------------------------------------------------------------------
    # Power harvested = C_e * (z_s_dot - z_u_dot)^2
    C_e = 1500.0        # Equivalent regenerative damping coefficient (N*s/m)
    eta_regen = 0.65    # Efficiency of the power electronics/battery converter
    
    # --------------------------------------------------------------------------
    # Physical Limits (Safety Constraints)
    # --------------------------------------------------------------------------
    stroke_max = 0.08   # Maximum suspension deflection before hitting bump stop (m)
    u_max = 6000.0      # Maximum actuator force available (N)

    # --------------------------------------------------------------------------
    # Power Electronics (Shift Transformer & LC Filter)
    # --------------------------------------------------------------------------
    # Weights per wheel (kg)
    mass_transformer = 2.5
    mass_inductor = 1.5
    mass_capacitor = 0.5
    
    # Electrical properties
    inductance_L = 0.01      # 10 mH
    capacitance_C = 0.0047   # 4700 uF
    
    def get_power_electronics_weight(self):
        """Returns the total added weight of power electronics for the half-car (2 wheels)"""
        weight_per_wheel = self.mass_transformer + self.mass_inductor + self.mass_capacitor
        return weight_per_wheel * 2.0
