# Project Proposal — Full Technical Document

## Title
**Cyber-Resilient LPV-Adaptive Event-Triggered Fault-Tolerant Controller for Regenerative In-Wheel Motor EV Suspensions**

*(Slide title: "Solving the IWM Suspension Triple Problem: Road-Adaptive, Self-Charging, and Hack-Proof EV Ride Control")*

---

## 1. The Base Paper — What Exists Already

### ⭐ Anchor Paper (IEEE Transactions on Vehicular Technology, 2024)
**S. A. Karthick and B. Chen,**
*"Finite-Time Based Fault-Tolerant Control for Half-Car Active Suspension System With Cyber-Attacks: A Memory Event-Triggered Approach,"*
**IEEE Transactions on Vehicular Technology, vol. 73, no. 9, pp. 12704–12717, Sept. 2024.**
- **DOI:** 10.1109/TVT.2024.XXXXXXX

### ⭐ Supporting Paper (IEEE Transactions on Transportation Electrification, 2024)
**W. Xu, J. Zhang, H. Li, and Y. Tian,**
*"Active Suspension Control for In-Wheel-Motor-Driven Electric Vehicles With Electromechanical Coupling Effect,"*
**IEEE Transactions on Transportation Electrification, vol. 10, no. 2, pp. 3410–3422, June 2024.**
- **DOI:** 10.1109/TTE.2023.3323089

### What the Base Paper Does (Plain English Explanation)
The base paper (Karthick & Chen 2024) addresses a real but underexplored problem: the control loop of a vehicle's active suspension runs over the CAN-bus network. This network has no built-in encryption or authentication. A malicious actor — or even a faulty node — can flood the bus with garbage packets. The base paper:
1. **Models the CAN-bus cyber-threat mathematically** — using Bernoulli random variable distributions to represent probabilistic packet drops (DoS) and false data injection (FDI) attacks.
2. **Designs a Memory Event-Triggered Mechanism (METM)** — sensors only transmit data when the error `||e(t)|| > σ||x(t)||` exceeds a threshold, reducing network load by 40–60%.
3. **Synthesises a Finite-Time H∞ Fault-Tolerant Controller** — via Linear Matrix Inequalities (LMIs), guaranteeing that the chassis settles within a strict deadline T ≤ 0.5 s, even if cyber-attacks drop or corrupt sensor packets.
4. **Proves stability via Lyapunov theory** — in MATLAB simulation only, for a half-car passive hydraulic suspension model.

### What System Does the Base Paper Use?
| Component | Base Paper Choice |
|-----------|-------------------|
| Vehicle model | Half-car (bounce + pitch), passive hydraulic suspension |
| Actuator | Active force input (ideal, linear) |
| Controller type | Finite-Time Lyapunov + LMI H∞ Fault-Tolerant |
| Network mechanism | METM (Memory Event-Triggered) |
| Cyber-attack model | Bernoulli random DoS + FDI |
| Road disturbance | Sinusoidal / step input |
| Energy consideration | None — suspension consumes power, no recovery |
| Road adaptation | None — fixed-gain controller |
| IWM modelling | None — conventional car assumed |
| Validation | MATLAB simulation only — no hardware |

### What is the Core Gap in the Base Paper?
The base paper proves the fault-tolerant finite-time math works in simulation with:
- A **fixed controller** that never adapts when road roughness changes from smooth to rough
- **No energy harvesting** — every bump forces energy is wasted as heat
- **No In-Wheel Motor dynamics** — ignores the electromechanical coupling disturbance `F_em` from IWM eccentricity
- **No real hardware validation** — the gap between MATLAB simulation and a real microcontroller ECU is never closed
- **No active cyber-defense** — only mathematically tolerates attacks; never detects or blocks them
- **No extension to e-bikes** — IWM is equally problematic for electric motorcycles

> **Your project closes ALL six of these gaps.**

---

## 2. Problem Statement

### In Simple English
A premium Electric Vehicle uses In-Wheel Motors (IWM) — the motor lives inside the wheel hub. This makes the vehicle more efficient (no drivetrain loss) but creates a cascade of problems:

**Mechanical Problems:**
- The wheel becomes much heavier (unsprung mass increases by 30–50%)
- Heavy wheels bounce around on bumps instead of staying stuck to the road
- Vibration is transmitted to the car body (passenger discomfort, ISO 2631 violation)
- The IWM rotor eccentricity creates a rhythmic disturbance force `F_em = k_em · i_d · sin(p · θ_m)` at motor speed
- A fixed-gain controller designed for smooth roads fails on rough urban roads

**Digital / Cyber Problems:**
- The ECU controller receives sensor data over a **CAN-bus** — a shared, unauthenticated network
- Hackers can inject DoS packets or false data, causing the controller to apply wrong actuator forces at highway speed
- Standard time-triggered communication spams the network every 1 ms even when the car is on a flat road

**Energy Problems:**
- Active suspension actuators **drain the EV battery** every time they push against a bump
- No existing controller decides intelligently between "should I spend battery to fight this bump?" or "should I harvest this bump's energy back into the battery?"

### The One-Line Problem Statement
> *Design a road-adaptive, energy-recovering, cyber-resilient, network-efficient, hardware-validated active suspension controller specifically for In-Wheel Motor EVs — and prove it holds against real cyber-attacks on embedded hardware.*

---

## 3. Control System Part — What Control We Use and Why

### Control Strategy: LPV H∞ + METM + Finite-Time Lyapunov + DRL Supervisor

#### Pillar A: Why H∞ Control?
H∞ control is the gold standard for suspension systems because it provides a **mathematical guarantee**: the worst-case ratio of road disturbance (input) to body acceleration (output) is bounded by a value γ. In engineering terms:
- It handles model uncertainty (suspension parameters change with load)
- It handles actuator limits
- It handles cyber-attack-induced signal distortions (treats attack-corrupted packets as bounded disturbances)
- It is solved systematically via Linear Matrix Inequalities (LMIs) — no trial-and-error tuning

#### Pillar B: Why LPV (Gain Scheduling) on top of H∞?
The base paper uses a **fixed** H∞ controller — designed for one road condition. In reality, road roughness varies massively:
- Smooth highway (ISO Class A): minimise actuator energy
- Broken urban road (ISO Class D): maximise body isolation

An LPV controller automatically adjusts its gains as road roughness changes. The **scheduling variable ρ(t)** is estimated in real-time by a **Kalman Filter** running on the onboard ECU using body acceleration signals.

```
ρ(t) = Road Roughness (Kalman-estimated online, 0 = smooth Class A, 1 = rough Class D)

K(ρ) = Gain matrix that changes smoothly with ρ(t)

u(t) = K(ρ(t)) · x(t)     ← the LPV actuator force command

Stability Proof: Exists a parameter-dependent Lyapunov function V(x, ρ) > 0
such that dV/dt < 0 for all ρ(t) ∈ [0,1] and all attack realisations
```

#### Pillar C: Memory Event-Triggered Mechanism (METM)
Standard time-triggered sensors spam the CAN-bus every 1 ms — wasting bandwidth on flat roads. The METM suppresses transmission unless the error crosses a threshold:

```
e(t) = x(t) − x_k          (error since last transmitted state)
||e(t)|| > σ · ||x(t)||    (METM trigger condition)

→ Transmit packet ONLY when triggered
→ Saves 40–60% of CAN-bus bandwidth
→ Also makes DoS attacks harder to mount (less predictable transmission pattern)
```

#### Pillar D: Finite-Time Lyapunov Stabilization
Unlike asymptotic controllers that take 2–3 s to settle, our controller guarantees settling within T ≤ 0.5 s:

```
Lyapunov function: V(x,ρ) = xᵀ P(ρ) x > 0
Finite-time bound:  V(x(T)) = 0 for all x(0), for T ≤ 0.5 s
Even under: DoS drops up to 50% of packets, FDI corrupts signals by ±δ
```

#### Pillar E: DRL Supervisory Agent (Energy Mode Switcher)
A Deep Reinforcement Learning (DRL) agent monitors Battery State-of-Charge (SoC) and dynamically adjusts the H∞ weighting matrices:

```
SoC ≥ 15% → W_comfort active → Controller prioritises smooth ride
SoC  < 15% → W_eco active    → Controller stiffens damping, harvests maximum bump energy
```

#### Pillar F: Active Cyber Defense (Edge ML + Physical Failover)
Beyond mathematical tolerance, an Edge ML classifier monitors CAN-bus traffic in real-time:

```
Normal CAN load: < 60% bus utilisation → Primary CAN active
DoS detected   : > 90% spike in < 5 ms → Hardware switch: CAN → Automotive Ethernet
FDI detected   : Statistical anomaly in sensor packet values → Packet rejection + last-valid-state hold
```

---

## 4. System Design — Architecture and Components

### Physical Plant: IWM Half-Car EV Model
```
          Road Profile w(t) — ISO 8608 Class A/B/C/D
                   │
                   ▼
  ┌────────────────────────────────────────────┐
  │  Half-Car EV Plant (MATLAB/Simulink)       │
  │                                            │
  │   [Sprung Mass — Car Body: m_s]            │
  │        │ z_s (bounce)                      │
  │        │ θ (pitch)                         │
  │        │                                   │
  │   [Electromagnetic Actuator — Front/Rear]  │
  │        │ F_active  (push) — Active Mode    │
  │        │ P_regen = C_e·(ż_s−ż_u)² — Regen │
  │        │                                   │
  │   [Unsprung Mass — Wheel + IWM: m_u]       │
  │        │ z_u (wheel displacement)           │
  │        │ F_em = k_em·i_d·sin(p·θ_m)        │
  │        │ (IWM eccentricity disturbance)     │
  └────────────────────────────────────────────┘
         ▲
    Road input z_r(t)
```

### State Variables
| Symbol | Meaning |
|--------|---------|
| `z_s` | Car body vertical displacement (bounce) |
| `ż_s` | Car body vertical velocity |
| `θ` | Car body pitch angle |
| `θ̇` | Car body pitch rate |
| `z_u1, z_u2` | Front/rear wheel (unsprung mass) displacement |
| `ż_u1, ż_u2` | Front/rear wheel vertical velocity |
| `ρ(t)` | Road roughness scheduling variable (Kalman-estimated) |
| `SoC(t)` | Battery State-of-Charge (%) — monitored by DRL agent |

### 5-Layer ECU Network Architecture
```
[LAYER 1: PHYSICAL PLANT]
  Wheel Sensors (Accelerometers + Potentiometers on STM32 Node A)
       │  Reads: z_u, z_s, ż_s, ż_u, θ, θ̇
       │
[LAYER 2: METM FILTER — STM32 Node A]
       │  Evaluates: ||e(t)|| > σ||x(t)||
       │  → Transmits only on trigger (saves 40–60% bandwidth)
       │
       ├──────────────────── Primary CAN Bus (500 kbps) ──────────────────
       │                         │  ← DoS / FDI Attack injection point
       │
[LAYER 3: CYBER-DEFENSE LAYER — STM32 Node B (Edge ML)]
       │  ML Anomaly Detector: monitors CAN bus-load %
       │  → DoS detected? Hardware switch: CAN → Automotive Ethernet
       │
[LAYER 4: CENTRAL CONTROLLER ECU — Raspberry Pi / STM32H7]
       │  Kalman Road Estimator → ρ(t)
       │  LPV H∞ Controller → K(ρ(t)) · x(t) = u*(t)
       │  DRL Supervisor → W_comfort or W_eco
       │  Mode Decision: ACTIVE (large bump) or REGENERATE (small vibration)
       │
[LAYER 5: ACTUATOR NODE — STM32 Node C]
       │  Drives: Electromagnetic linear actuator
       │  Monitors: Battery SoC → feeds back to DRL Supervisor
```

### Electromagnetic Actuator — Dual Mode Operation
| Mode | Condition | What Happens |
|------|-----------|--------------|
| **Active Control** | Large road bump, \|F*\| > threshold OR SoC ≥ 15% | Motor draws battery power, applies counter-force |
| **Regeneration** | Small vibration, \|F*\| < threshold OR SoC < 15% | Motor acts as generator, charges battery |
| **Failover (Cyber-Attack)** | CAN DoS detected | Control rerouted over Automotive Ethernet, LMI H∞ bounds constrain actuator |

---

## 5. The Novelties — Five Contributions Over the Base Paper

### N1 — LPV Gain-Scheduled H∞ with Kalman Road Estimator
**What the base paper does:** One fixed H∞ controller — designed for one road condition. Fails on roads different from design point.
**What you add:** A Kalman Filter runs on the ECU estimating road roughness in real-time. The scheduling variable ρ(t) shifts the controller's LPV H∞ weighting matrices automatically:
- On Class A (smooth): Weighting favours energy saving → less aggressive actuation
- On Class D (rough): Weighting favours ride comfort → maximum body isolation
- **Result:** Controller is always at the Pareto-optimal point for ANY road condition

### N2 — DRL Battery-Aware Regenerative Electromagnetic Damper
**What the base paper does:** Actuator only pushes (consumes power). Energy from bumps is wasted as heat.
**What you add:** A DRL supervisory agent monitors battery SoC and switches H∞ weighting matrices:
- Regenerative power model: `P_regen = C_e · (ż_s − ż_u)²`
- Comfort Mode (SoC ≥ 15%): maximise passenger comfort
- Eco Mode (SoC < 15%): sacrifice 5% comfort to recover 12–18% of actuator energy
- **Result:** ~2% EV range extension over a drive cycle; SoC-aware intelligent energy management

### N3 — Active Cyber Defense with Dual-Network Failover
**What the base paper does:** Mathematically tolerates cyber-attacks via LMI bounds — but never detects, never blocks, never reroutes.
**What you add:** An onboard Edge ML classifier (SVM / Isolation Forest) monitors CAN-bus traffic. On attack detection:
- CAN DoS detected in < 5 ms (bus-load spike > 90%)
- Hardware switch physically isolates CAN bus
- Control traffic rerouted to Automotive Ethernet backbone
- LMI H∞ bounds remain active throughout failover → zero loss of stability
- **Result:** First fail-operational active suspension system — the vehicle remains stable even during live cyber-attacks

### N4 — Hardware-in-the-Loop Validation on Networked STM32 ECUs with Injected Attacks
**What the base paper does:** Proved only in MATLAB simulation (ideal, delay-free, no real hardware).
**What you add:** The complete LPV controller + METM + Edge ML is ported to STM32 microcontrollers. Connected via real CAN-bus hardware. Network delays of 5 ms, 10 ms, 20 ms AND live DoS attack scripts are injected. Controller is proven stable via Lyapunov theory AND experimentally.
- **Result:** First hardware validation of a combined LPV H∞ + cyber-resilient IWM suspension controller

### N5 — E-Bike Corner Module Extension (Stretch Goal)
**What the base paper does:** Half-car (4-wheel car) model only.
**What you add:** Extend the mathematical framework to a single-corner module, applicable to an electric motorcycle with a hub-motor. E-bikes have an even worse unsprung mass problem.
- **Result:** Second publishable result from the same framework — applicable to Zero Motorcycles, Energica, Ola Electric S1

---

## 6. Implementation Plan (Simulation → Hardware)

| Phase | Week | Deliverable |
|-------|------|-------------|
| **SIM** | 1–2 | Derive IWM half-car EOM with `F_em` electromechanical coupling + Bernoulli cyber-attack model. Build Simulink plant. |
| **SIM** | 3–4 | Baseline: Passive → Fixed H∞ (reproduce base paper results). Confirm METM bandwidth savings match paper. |
| **SIM** | 5–6 | Add LPV scheduling + Kalman road roughness estimator. Tune across ISO Class A/B/C/D. Validate ρ(t) convergence. |
| **SIM** | 7 | Integrate Regenerative Damper model + DRL SoC supervisor. Plot energy Pareto front (comfort vs. energy). |
| **SIM** | 8 | Inject Bernoulli DoS + FDI attacks in simulation. Validate LMI H∞ fault tolerance. Train Edge ML classifier on attack traffic. |
| **HW** | 9–10 | Build quarter-car HIL rig: linear actuator, springs, IMU, potentiometers on aluminium frame. |
| **HW** | 11 | Wire CAN-bus ECU stack: Sensor (STM32 Node A) → Edge ML (STM32 Node B) → Controller (RPi / STM32H7) → Actuator (STM32 Node C). |
| **HW** | 12 | Port LPV + METM controller to embedded C. Validate under injected 5/10/20 ms network delays. |
| **HW** | 13 | Inject live DoS attack scripts on CAN-bus. Test Edge ML detection + Ethernet failover. Measure failover latency. |
| **HW** | 14 | E-bike corner module extension (N5): single-corner model + hardware test (stretch goal). |
| **WRITE** | 15–17 | Full data analysis, Pareto plots, IEEE-format paper draft, viva presentation. |

---

## 7. Experiments & Expected Results

| Metric | Passive Baseline | Base Paper (Karthick 2024) | Your Full System (N1+N2+N3+N4) |
|--------|------------------|-----------------------------|--------------------------------|
| Body Accel RMS (ISO 2631) | 1.00 (normalised) | 0.62 | **0.44** |
| Tyre Deflection RMS | 1.00 (normalised) | 0.75 | **0.58** |
| Suspension Travel | 1.00 (normalised) | 0.80 | **0.65** |
| Energy Harvested | 0% | 0% | **12–18%** |
| Controller adapts to road class | No | No | **Yes (LPV)** |
| Cyber-attack detected actively | No | No | **Yes (<5 ms)** |
| Failover to Ethernet | No | No | **Yes (hardware switch)** |
| Hardware validated | No | No | **Yes (HIL STM32)** |
| Settling time guarantee | ~2.5 s | ≤ 0.5 s | **≤ 0.5 s (even under attack)** |

**Headline Plots:**
1. **Pareto Front** — Body Acceleration RMS vs. Energy Harvested across ISO Class A–D: Your LPV system stays near the Pareto frontier; the fixed-gain base paper degrades significantly on rough roads.
2. **Attack Scenario Timeline** — Body displacement during DoS attack: Base paper's controller loses stability; your system maintains ≤ 0.5 s settling with Ethernet failover.
3. **Kalman Estimator Convergence** — ρ(t) estimate vs. true road class over a simulated urban drive cycle.

---

## 8. Risks & Mitigations

| Risk | Mitigation |
|------|------------|
| LPV LMI synthesis may be computationally heavy | Use polytopic LPV with 4 design points (ISO Class A/B/C/D) instead of continuous scheduling |
| `F_em` electromagnetic coupling hard to model exactly | Use manufacturer motor data; validate coupling coefficient `k_em` on a bench motor test |
| Edge ML classifier may produce false positives (normal CAN spikes misread as attacks) | Train on diverse CAN traffic logs; add hysteresis to attack confirmation (require 3 consecutive anomaly detections) |
| DRL agent training convergence | Use PPO/SAC with curriculum learning — start with SoC fixed, then enable dynamic SoC variation |
| HIL rig actuator nonlinearity (dead-band, saturation) | Identify experimentally; add saturation block in Simulink plant model |
| Ethernet failover latency may exceed 5 ms on real hardware | Pre-route Ethernet path; use dedicated switch; test with hardware logic analyser |
| E-bike extension adds timeline risk | Treat N5 as stretch goal — N1+N2+N3+N4 alone is already a complete, strong IEEE TVT paper |

---

## 9. Deliverables Checklist

- [ ] IWM half-car plant model (MATLAB/Simulink) with `F_em` electromechanical coupling
- [ ] Bernoulli cyber-attack model (DoS + FDI) in Simulink
- [ ] Fixed H∞ + METM baseline (reproduce base paper results)
- [ ] Kalman road roughness estimator — ρ(t) convergence validated
- [ ] LPV gain-scheduled H∞ controller with polytopic LMI synthesis (YALMIP/MOSEK)
- [ ] DRL battery-aware supervisor (PyTorch / Stable-Baselines3)
- [ ] Regenerative electromagnetic damper model + SoC tracking
- [ ] Edge ML CAN-bus anomaly classifier trained and tested
- [ ] CAN-bus HIL rig with STM32 ECU stack (4 nodes)
- [ ] Physical Automotive Ethernet failover path wired and validated
- [ ] Delay robustness validation (5 ms / 10 ms / 20 ms)
- [ ] Live DoS attack injection and failover latency measurement
- [ ] E-bike corner-module extension (stretch)
- [ ] IEEE Transactions format paper + Pareto plots + attack-scenario plots + viva slides
