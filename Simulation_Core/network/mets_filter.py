# ==============================================================================
# Simulation_Core/network/mets_filter.py
# Memory Event-Triggered Scheme (METS) for CAN-bus bandwidth reduction
# ==============================================================================
import numpy as np

class METSFilter:
    """
    Implements the Memory Event-Triggered Scheme (METS) from the base paper.
    Instead of transmitting sensor data every 1ms (time-triggered), this filter
    only transmits when the state error exceeds a dynamic threshold.
    
    This reduces CAN-bus load by 40-60% on flat roads, freeing up bandwidth
    and making cyber-attacks (like DoS floods) easier to detect.
    """
    def __init__(self, sigma=0.1):
        """
        Args:
            sigma (float): Sensitivity threshold (0 < sigma < 1).
                           Lower = more packets transmitted.
                           Higher = more bandwidth saved, but controller accuracy drops.
                           Base paper standard is typically 0.1 to 0.15.
        """
        self.sigma = sigma
        self.x_k = None          # Last transmitted state vector
        
        # Telemetry for the final results table
        self.total_checks = 0
        self.total_transmits = 0

    def should_transmit(self, current_state):
        """
        Evaluates the METS rule: ||e(t)|| > sigma * ||x(t)||
        
        Args:
            current_state (np.ndarray): The current 8-DOF state vector from the sensors
            
        Returns:
            tuple: (transmit_flag (bool), transmitted_state (np.ndarray))
        """
        self.total_checks += 1
        
        # First packet is always transmitted
        if self.x_k is None:
            self.x_k = current_state.copy()
            self.total_transmits += 1
            return True, self.x_k
            
        # Calculate error between current state and last transmitted state
        e_t = current_state - self.x_k
        
        # Calculate Euclidean norms (L2 norms)
        norm_e = np.linalg.norm(e_t)
        norm_x = np.linalg.norm(current_state)
        
        # The Event-Trigger Rule
        if norm_e > self.sigma * norm_x:
            # Threshold exceeded -> Update memory and transmit
            self.x_k = current_state.copy()
            self.total_transmits += 1
            return True, self.x_k
        else:
            # Threshold not met -> Do not transmit, controller holds last state
            return False, self.x_k
            
    def get_bandwidth_saved_percentage(self):
        """Returns the % of packets that were NOT transmitted."""
        if self.total_checks == 0:
            return 0.0
        return 100.0 * (1.0 - (self.total_transmits / self.total_checks))
