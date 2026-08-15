# Cyber-Resilient Active Suspension System
## Complete Project Documentation & Architecture Justification

This document provides a comprehensive breakdown of the dual-simulation architecture (MATLAB Simulink and Python Deep RL) used to validate the Cyber-Resilient LPV-Adaptive Active Suspension System.

---

## Part 1: MATLAB Simulink Co-Simulation Architecture

The MATLAB Simulink environment serves as the foundational physics, signal processing, and visual analysis layer. We completely restructured the baseline model to modularize the cyber-physical components into distinct, justified blocks.

### 1. Initialization (`START_HERE.m`)
- **Justification:** Simulink models require predefined workspace variables (like vehicle mass, suspension stiffness, and road profiles) to run. This script generates the ISO standard road profiles (synthetic potholes) and loads the matrices before the `.slx` file executes.

### 2. Physical Vehicle Model (`HalfCar_Plant`)
- **Justification:** This block represents the actual mechanical car. It is an 8-Degree-of-Freedom (8-DOF) model that calculates how the body and wheels bounce when hitting a pothole, taking into account the active force exerted by the electromagnetic actuator.
- **Mathematical Equations:**
  The dynamics are governed by Newton's second law for the sprung and unsprung masses:
  - **Sprung Mass (Body Bounce & Pitch):**
    `m_s * z_c_ddot = -F_sf - F_sr`
    `I_phi * theta_ddot = a*F_sf - b*F_sr`
  - **Unsprung Mass (Front & Rear Wheels):**
    `m_uf * z_uf_ddot = F_sf - k_tf(z_uf - w_f)`
    `m_ur * z_ur_ddot = F_sr - k_tr(z_ur - w_r)`
  - **Suspension Force:**
    `F_susp = k_s * def_s + c_s * v_rel + F_bump_stop - u_actuator`

### 3. Cyber-Control Layer (`LPV_UKF_Ctrl`)
- **Justification:** This is the "brain" of the vehicle. It combines three advanced algorithms into one block to filter network delays, estimate the road severity, and calculate the exact force needed to stabilize the car.
- **Mathematical Equations:**
  - **UKF Road Estimation:** Tracks the rate of road change (`rho_dot`) to predict potholes before the tire fully drops into them.
  - **LPV Blending:** 
    `K(rho) = (1 - rho) * K_smooth + rho * K_rough`
  - **Feedforward Pre-stiffening:**
    `u_ff = -K_rho_dot * max(0, rho_dot) * x_state`

### 4. Energy Harvesting System (`Energy_SoC`)
- **Justification:** An EV's battery is its most critical component. This block models the Power Electronics (LC Filter and Full-Bridge Rectifier) that converts the violent kinetic energy of the suspension compressing into usable 48V DC electricity.
- **Mathematical Equations:**
  - **Regenerative Force:** `F_regen = -C_e * v_rel`
  - **Raw Voltage (Back-EMF):** `V_raw = K_v * v_rel`
  - **Conditioned Power (LC Filter):**
    `di_L/dt = (V_rectified - V_C) / L`
    `dV_C/dt = (i_L - i_load) / C`

### 5. Safety Verification (`Lyapunov_Mon`)
- **Justification:** Deep Learning is notoriously unpredictable ("black-box"). To guarantee the AI never flips the car, this block acts as a mathematical watchdog, continuously evaluating the system's energy state.
- **Mathematical Equations:**
  - **Quadratic Energy Function:** `V(x) = x^T * P * x`
  - For the vehicle to be strictly safe, the derivative `dV(x)/dt` must always be negative (energy must be dissipating, not growing out of control).

### 6. Passive Baseline (`Base_Paper_Sim`)
- **Justification:** Scientific validation requires a control group. This block runs the standard, unintelligent passive suspension in parallel so we can prove our AI system is superior.

---

## Part 2: Python Deep Reinforcement Learning Simulator

While MATLAB excels at signal processing and visual scopes, Python is the industry standard for Artificial Intelligence. We built a custom Python simulator (`Simulation_Core`) that perfectly mirrors the MATLAB physics to train the AI.

### 1. The Custom Physics Engine (`half_car_model.py`)
- **Justification:** To train an AI, you have to run millions of simulations. Simulink is too slow for this. We coded a custom Runge-Kutta 4th Order (RK4) physics engine in Python that runs at 1,000 Hz, allowing the AI to experience years of driving over potholes in just a few minutes.

### 2. The TD3 Deep RL Agent (`td3_energy_agent.py`)
- **Justification:** The core novelty of the project. The Twin Delayed Deep Deterministic Policy Gradient (TD3) agent is an advanced neural network. It observes the battery State of Charge (SoC) and the road severity, and outputs a continuous decision to blend between two modes:
  - **COMFORT Mode:** Spends battery power to actively push against the bump, ensuring maximum passenger comfort.
  - **ECO Mode:** Turns off the active pushing and acts as a generator, sacrificing a tiny bit of comfort to harvest massive amounts of kinetic energy.

### 3. The Gymnasium Environment (`td3_environment.py`)
- **Justification:** The AI needs a structured way to "play" the simulation like a video game. This wrapper feeds the AI observations, accepts its actions, and calculates its "Score" (Reward).
- **The Reward Function:**
  The AI is rewarded heavily for harvesting energy (especially if the battery is low), but penalized if it allows the car's body acceleration to get too high. This forces the AI to learn the perfect trade-off.

### 4. The Dataset (Training Data)
The Deep RL agent does not use a static pre-recorded CSV dataset to train; it generates its own data dynamically using **Domain Randomization**:
- **Synthetic ISO Road Profiles:** The system mathematically generates randomized ISO Standard road profiles (Classes A, B, C, and D) ranging from smooth highways to severe off-road potholes.
- **Sim-to-Real Transfer:** During training, the simulator randomly alters the mass of the vehicle by `+/- 20%` (simulating different numbers of passengers in the car) and randomizes the starting battery SoC. 
- **Validation Dataset:** For the final test run, it evaluates the AI against a synthetic "Mixed Pothole Track" to prove the AI can handle unpredictable combinations of smooth and rough roads dynamically. (A fallback `real_road_profile.csv` hook is also built-in for physical hardware testing).
