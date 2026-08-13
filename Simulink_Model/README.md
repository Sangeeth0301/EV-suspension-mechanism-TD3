# MATLAB/Simulink Implementation

## Quick Start

1. Open MATLAB
2. Navigate to the `Simulink_Model/` folder
3. Run:
   ```matlab
   build_simulink_model
   ```

## What It Does

The script `build_simulink_model.m` is a **single-file, self-contained implementation** of the entire Cyber-Resilient Active Suspension project. It:

1. **Defines** all 25+ vehicle parameters (matching `vehicle_params.py`)
2. **Solves** the Continuous Algebraic Riccati Equation (CARE) for the LPV controller gains
3. **Generates** the ISO 8608 road profile with 6 random potholes
4. **Runs** a 1kHz RK4 simulation loop with ALL subsystems:
   - 4-DOF Half-Car Plant (8-state ODE with bump stops + IWM)
   - UKF Road Estimator (rho + rho_dot)
   - LPV H-infinity Controller (polytopic blending + feedforward)
   - TD3 Heuristic Energy Agent (COMFORT/ECO switching)
   - LC Power Electronics Filter (2-state ODE)
   - METS Network Filter (event-triggered CAN-bus)
   - Lyapunov Stability Monitor (V(x) = x'Px)
   - Base Paper Fixed H-infinity Controller (comparison baseline)
5. **Prints** a detailed performance comparison table
6. **Generates** a 10-panel results dashboard figure
7. **Creates** a Simulink `.slx` model (if Simulink is installed)

## Requirements

- **MATLAB R2020b or later**
- **Control System Toolbox** (for the `care` function)
- **Simulink** (optional, for `.slx` model generation)

## Output

- `Simulation_Core/results/MATLAB_Results_Dashboard.png` — 10-panel figure
- `CyberResilient_ActiveSuspension.slx` — Simulink model (if Simulink available)
- Console printout with performance metrics

## Cross-Validation with Python

| Metric | Python | MATLAB (expected) |
|--------|--------|-------------------|
| Comfort Improvement | ~85% | ~85% |
| Peak Regen Power | ~1924 W | ~1900 W |
| CAN Bandwidth Saved | ~25% | ~25% |
