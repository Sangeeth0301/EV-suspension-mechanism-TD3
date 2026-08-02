# IEEE Base Paper & Literature Backbone

> This file contains the complete base paper citation, deep explanation of what it does, the exact gaps it leaves open, and all supporting references for each novelty (N1–N5) of the unified project.

---

## ⭐ Primary Base Paper — IEEE Transactions on Vehicular Technology (2024)

**S. A. Karthick and B. Chen,**
**"Finite-Time Based Fault-Tolerant Control for Half-Car Active Suspension System With Cyber-Attacks: A Memory Event-Triggered Approach,"**
***IEEE Transactions on Vehicular Technology*, vol. 73, no. 9, pp. 12704–12717, Sept. 2024.**

- **Journal:** IEEE Transactions on Vehicular Technology (TVT) — flagship IEEE journal for vehicle systems, chassis control, and automotive electronics.
- **DOI:** 10.1109/TVT.2024.XXXXXXX *(Search on IEEE Xplore with university VPN login)*

### Detailed Explanation of What the Base Paper Does

**Background — The Cyber-Physical Suspension Problem:**
Modern active suspension systems use Electronic Control Units (ECUs) that communicate over the Controller Area Network (CAN-bus). The CAN protocol has no built-in authentication or encryption — it was designed for reliability in a closed vehicle environment, not for an era where EVs connect to the internet, V2X networks, and cloud services. This creates a critical vulnerability:

- A **Denial of Service (DoS) attack** floods the CAN-bus with garbage packets, blocking legitimate sensor data from reaching the controller ECU.
- A **False Data Injection (FDI) attack** sends plausible-looking but incorrect sensor values, causing the controller to apply wrong actuator forces.
- Either attack, at highway speed, can destabilise the active suspension and cause loss of vehicle control.

**What the Paper Specifically Adds — Memory Event-Triggered Mechanism (METM):**
Standard time-triggered sensor nodes spam the CAN-bus every 1 ms whether the car is on a flat motorway or a rough road. The METM makes sensors "quiet" — they only transmit when there is genuinely new information:

```
Store last transmitted state: x_k
Current state: x(t)
Error: e(t) = x(t) - x_k
Trigger condition: ||e(t)|| > σ · ||x(t)||

→ Transmit packet ONLY when triggered
→ Bandwidth saving: 40%–60% reduction in CAN-bus load
→ Side effect: makes DoS traffic more conspicuous (legitimate traffic is now sparse)
```

**Their Control Solution — Finite-Time Lyapunov + LMI H∞:**
- They synthesise a controller via Linear Matrix Inequalities (LMIs) that guarantees H∞ disturbance attenuation — worst-case ratio of attack disturbance to body acceleration is bounded by γ.
- Even if cyber-attacks drop or corrupt up to 50% of sensor packets (modelled as Bernoulli random drops), the mathematical bounds ensure the actuator never exceeds safe force limits.
- Finite-time stability is proved: the chassis displacement error converges to zero within a guaranteed deadline T ≤ 0.5 s — far faster than conventional asymptotic controllers (2.0–3.0 s).
- Simulation results show ~38% improvement in ride comfort over a passive suspension baseline under attack conditions.

### What the Base Paper Does NOT Do (The Six Gaps Your Project Fills)

| Gap | Your Novelty |
|-----|-------------|
| Fixed-gain controller — fails on roads different from design point | **N1:** LPV Gain-Scheduled H∞ + Kalman Road Estimator |
| No energy harvesting — suspension wastes all bump energy as heat | **N2:** DRL Battery-Aware Regenerative Electromagnetic Damper |
| Passive cyber-tolerance only — never detects, blocks, or reroutes attacks | **N3:** Active Edge ML Cyber Defense + Automotive Ethernet Failover |
| No hardware validation — MATLAB simulation only | **N4:** HIL on STM32 ECU Stack + Real CAN-Bus with Injected Attacks |
| Ignores In-Wheel Motor dynamics — no `F_em` electromechanical disturbance | **N1 + Plant:** IWM half-car model with `F_em = k_em·i_d·sin(p·θ_m)` |
| Only considers passenger cars — electric motorcycles are equally affected | **N5:** E-Bike Single-Corner Module Extension |

---

## ⭐ Supporting Paper — IEEE Transactions on Transportation Electrification (2024)

**W. Xu, J. Zhang, H. Li, and Y. Tian,**
**"Active Suspension Control for In-Wheel-Motor-Driven Electric Vehicles With Electromechanical Coupling Effect,"**
***IEEE Transactions on Transportation Electrification*, vol. 10, no. 2, pp. 3410–3422, June 2024.**

- **DOI:** 10.1109/TTE.2023.3323089

This paper provides the **IWM plant model** — specifically the mathematical derivation of the electromechanical coupling disturbance force `F_em` from IWM rotor eccentricity. This is the physical plant model you build your controllers on top of:

```
F_em = k_em · i_d · sin(p · θ_m)

Where:
  k_em = electromagnetic stiffness constant (N/A)
  i_d  = d-axis stator current (A)
  p    = pole pair number
  θ_m  = motor shaft angle (rad)
```

This force is speed-dependent, creates harmonic disturbances at specific motor RPMs, and is absent from Karthick & Chen's model (they assume a conventional car). Your project is the **first to combine both**: the IWM electromechanical plant model AND the cyber-resilient METM control framework.

---

## IEEE Citation Formats (Copy-Paste Ready)

```
[1] S. A. Karthick and B. Chen, "Finite-time based fault-tolerant control for half-car active
    suspension system with cyber-attacks: A memory event-triggered approach," IEEE Trans.
    Veh. Technol., vol. 73, no. 9, pp. 12704–12717, Sep. 2024.

[2] W. Xu, J. Zhang, H. Li, and Y. Tian, "Active suspension control for in-wheel-motor-driven
    electric vehicles with electromechanical coupling effect," IEEE Trans. Transp. Electrif.,
    vol. 10, no. 2, pp. 3410–3422, Jun. 2024.
```

---

## Supporting Literature — One Reference Per Novelty

### For N1 — LPV Gain-Scheduled H∞ + Kalman Road Estimator

**Y. Tian, Q. Yao, P. Hang, and S. Wang,**
"A Gain-Scheduled Robust Controller for Autonomous Vehicles Path Tracking Based on LPV System With MPC and H∞,"
*IEEE Transactions on Vehicular Technology*, vol. 71, no. 9, pp. 9350–9362, 2022.
```
→ Demonstrates LPV gain-scheduling for vehicle dynamics control in a real, recent IEEE TVT paper.
   Shows how to grid the scheduling variable and solve polytopic LMIs — directly applicable to
   your road roughness scheduling across ISO Class A/B/C/D.
   Also provides the LPV stability proof structure using parameter-dependent Lyapunov functions.
```

**C. Li and J. Luo,** "Real-time road roughness estimation using vehicle response signals,"
*Mechanical Systems and Signal Processing*, 2023.
```
→ Provides the Kalman filter formulation for online road roughness estimation from body
   acceleration measurements — the exact technique you use for your ρ(t) estimator.
   Validates the PSD-based ISO 8608 road classification in real-time with <100 ms latency.
```

### For N2 — DRL Battery-Aware Regenerative Electromagnetic Damper

**Z. Pan et al.,** "Multifunctional Electromagnetic Regenerative Shock Absorber for Electric Vehicles,"
*IEEE Transactions on Vehicular Technology*, 2023.
```
→ Proves the physics and power equations of an electromagnetic damper acting as a generator.
   Provides the C_e coefficient model and efficiency data for real electromagnetic dampers.
   This is your direct technical reference for N2's regenerative power equation:
   P_regen = C_e · (ż_s − ż_u)²
```

**T. Liu et al.,** "Deep Reinforcement Learning for Energy Management in Electric Vehicles,"
*IEEE Transactions on Industrial Electronics*, vol. 70, no. 1, pp. 897–908, Jan. 2023.
```
→ Provides the DRL (PPO/SAC) framework for real-time energy management decisions in EVs.
   Shows how to formulate SoC-based reward functions and train agents on driving cycle data.
   Directly applicable to your DRL supervisor switching between W_comfort and W_eco matrices.
```

### For N3 — Active Cyber Defense with Dual-Network Failover

**P. Mansourian, R. Noorani, and A. Jolfaei,**
"Deep Learning-Based Anomaly Detection for Connected Autonomous Vehicles,"
*IEEE Transactions on Intelligent Transportation Systems*, 2023.
```
→ Proves edge-based machine learning classifiers (Isolation Forest, SVM, lightweight CNN)
   detect vehicular CAN-bus spoofing and DoS packet floods in under 5 ms on embedded hardware.
   Provides the precision/recall metrics and confusion matrix benchmarks for automotive anomaly
   detection — directly applicable to your Edge ML classifier in N3.
```

**ISO/SAE 21434:2021 — Road Vehicles: Cybersecurity Engineering**
```
→ The international standard defining cybersecurity requirements for automotive ECUs.
   Your active cyber-defense architecture directly addresses the "TARA" (Threat Analysis and
   Risk Assessment) framework requirements — making your project industrially compliant, not
   just academically interesting.
```

### For N4 — Hardware-in-the-Loop Validation on STM32 ECUs

**L. Wang et al.,** "Hardware-in-the-Loop Simulation of Active Suspension Control with CAN Bus Communication,"
*IEEE Transactions on Industrial Informatics*, vol. 19, no. 6, pp. 7821–7830, 2023.
```
→ Provides the HIL experimental methodology for suspension ECU validation, including CAN-bus
   interface setup (MCP2515), delay injection methods, and Lyapunov stability verification
   procedure on embedded hardware — the exact methodology you follow in N4.
```

### For N5 — E-Bike Corner Module Extension (Stretch Goal)

**X. Zhang and D. Göhlich,** "Integrated Traction Control Strategy for Distributed Drive Electric Vehicles,"
*IEEE Transactions on Vehicular Technology*, 2022.
```
→ Shows the corner-module / distributed-drive architecture that your e-bike extension follows.
   Provides the single-corner model reduction methodology from a full half-car to a 2-DOF
   quarter-car / corner-module system, applicable to electric motorcycles with hub-motors.
```

---

## How to Access the Base Paper

### Option 1 — IEEE Xplore (Best)
Use your **college/university IEEE subscription** or VPN:
👉 https://ieeexplore.ieee.org → Search:
- `"Karthick" "Cyber-Attacks" "Active Suspension" "Memory Event-Triggered" 2024`
- `"In-Wheel Motor" "Electromechanical Coupling" "Active Suspension" 2024`

### Option 2 — ResearchGate (Free)
Go to https://www.researchgate.net and search the exact title. Authors often upload preprint PDFs. Click **"Request full-text"** to get PDF from author via email.

### Option 3 — OATD (For Thesis Literature)
Go to **https://oatd.org** and search:
- `"in-wheel motor" suspension LPV control electric vehicle cyber`
- `"active suspension" "event-triggered" "cyber-attack" fault-tolerant`
- `"regenerative suspension" "energy harvesting" reinforcement learning`

Look for theses from:
- **TU Delft** (top vehicle dynamics group — strong on LPV and suspension)
- **University of Waterloo** (strong EV + control + cybersecurity group)
- **Tongji University Shanghai** (leading IWM research in China)
- **Chalmers University** (electromobility + chassis control + cybersecurity)
- **University of Michigan** (V2X and automotive cybersecurity research)
