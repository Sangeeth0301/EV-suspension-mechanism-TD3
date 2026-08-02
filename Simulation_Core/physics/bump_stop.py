# ==============================================================================
# Simulation_Core/physics/bump_stop.py
# Simulates the hard mechanical limits of the suspension stroke.
# ==============================================================================

def calculate_bump_stop_force(deflection, max_stroke=0.08):
    """
    Calculates the nonlinear restoring force when the suspension hits the mechanical limit.
    This replaces the linear assumption with a realistic stiffening spring.
    
    Args:
        deflection (float): Current suspension stroke (z_s - z_u) in meters
        max_stroke (float): Maximum physical stroke before metal-on-metal impact
        
    Returns:
        float: Additional restoring force in Newtons
    """
    # Extremely stiff spring coefficient representing the rubber/polyurethane bump stop
    k_bs = 1e7
    
    if deflection > max_stroke:
        # Hitting upper limit (rebound stop)
        return k_bs * (deflection - max_stroke)**3
    elif deflection < -max_stroke:
        # Hitting lower limit (jounce stop)
        return k_bs * (deflection + max_stroke)**3
    else:
        # Normal operation range - no bump stop interference
        return 0.0
