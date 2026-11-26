---
title: "Reaction Wheel Inverted Pendulum"
subtitle: "Custom STM32H7 control board and testbed for non-linear and optimal control (energy-based swing-up and PID working; LQR → NMPC next)"
tier: "featured"
track: "Control Systems"
tags: ["STM32H7", "FDCAN", "VESC", "SEGGER RTT"]
rank: 1
status: "In Progress — Swing-Up & PID Working"
outcome: "Energy-based swing-up and PID + wheel-velocity-damping upright stabilization run on the V1.1 board with disturbance rejection (Nov 25, 2026); system identification, LQR and acados NMPC are not done yet."
date: "October 2026 – Present"
specs:
  - label: "Working on HW"
    value: "Swing-up + PID"
  - label: "MCU"
    value: "STM32H743VIT6 @ 480 MHz"
  - label: "Actuator"
    value: "4006 380Kv BLDC + Flipsky Mini VESC 6.7 Pro"
  - label: "Loop Rates"
    value: "1 kHz sensor / 200 Hz control"
  - label: "Bus"
    value: "FDCAN @ 500 kbps"
  - label: "Encoders"
    value: "2× AS5047P (ABI motor / SPI joint)"
techStack: ["STM32H743", "FreeRTOS", "FDCAN", "VESC (FOC)", "SEGGER RTT", "C", "KiCad", "SOLIDWORKS"]
githubUrl: "https://github.com/arthur-wirjo/reaction-wheel-inverted-pendulum"
thumbnail: "/assets/projects/rwip/rwip-thumbnail.jpg"
previewVideo: "/assets/projects/rwip/rwip-swing-up.mp4"
gallery:
  - src: "/assets/projects/rwip/rwip-swing-up.mp4"
    caption: "Autonomous Lyapunov energy-shaping swing-up with hand-off to the upright stabilizing controller (Control Board V1.1)."
    type: "video"
  - src: "/assets/projects/rwip/rwip-disturbance-rejection.mp4"
    caption: "Upright stabilization (PID + wheel velocity damping) rejecting external disturbances."
    type: "video"
  - src: "/assets/projects/rwip/rwip-v1-1-setup.jpg"
    caption: "RWIP testbed with Control Board V1.1 and the Flipsky Mini VESC 6.7 Pro mounted on the pendulum arm."
    type: "image"
  - src: "/assets/projects/rwip/rwip-cad.png"
    caption: "SOLIDWORKS model of the 3D-printed chassis, pendulum arm and reaction wheel."
    type: "image"
  - src: "/assets/projects/rwip/rwip-pcb-v1-1.jpg"
    caption: "Control Board V1.1 (STM32H743, 60V LMR16030 buck, SMBJ26CA TVS) assembled."
    type: "image"
  - src: "/assets/projects/rwip/control-board-v1-1-layout.png"
    caption: "Control Board V1.1 PCB layout (KiCad)."
    type: "image"
  - src: "/assets/projects/rwip/rwip-v1-0-setup.jpg"
    caption: "Earlier test setup with Control Board V1.0."
    type: "image"
  - src: "/assets/projects/rwip/rwip-pcb-v1-0.jpg"
    caption: "Control Board V1.0 (28V TPS54302 buck stage), which failed during the regenerative-braking test described below."
    type: "image"
  - src: "/assets/projects/rwip/control-board-v1-0-layout.png"
    caption: "Control Board V1.0 PCB layout (KiCad)."
    type: "image"
results: []
---

A custom Reaction Wheel Inverted Pendulum (RWIP) designed as a physical testbed for non-linear and optimal control theory. The project features a custom STM32H743 control board, dual 14-bit magnetic encoders and real-time FDCAN communication. **Energy-based swing-up and PID upright stabilization are working on hardware**; LQR and `acados` NMPC are planned.

> **Current status:** On the V1.1 board, a Lyapunov energy-shaping swing-up and a full-state PID + wheel-velocity-damping controller are implemented and verified on the physical testbed (autonomous swing-up, indefinite upright stabilization and disturbance rejection, Nov 25, 2026). **Not done yet:** system identification, MATLAB/Simulink modeling, LQR and `acados` NMPC (Phases 4–5). See [Current Status & Roadmap](#current-status--roadmap).

## Overview

Following my DIY FOC Motor Controller project, I shifted my focus from power electronics to **control theory, state estimation, and real-time embedded software**. The reaction wheel inverted pendulum is a classic underactuated, non-linear control problem requiring continuous, precise reaction torque commands ($\tau = -J_w \alpha_w$) to swing up and stabilize at its unstable upright equilibrium.

Rather than using off-the-shelf development boards, I designed a custom **STM32H743VIT6** control board to handle the floating-point computational demands of real-time NMPC. High-level control runs on the STM32H7, which commands $q$-axis motor current ($I_q$) and receives rotor telemetry over a 500 kbps FDCAN bus connected to a Flipsky Mini VESC 6.7 Pro executing low-level Field-Oriented Control (FOC).

## Engineering Challenges & Post-Mortem Debugging

Designing hardware that interfaces high-speed digital logic with dynamic electromechanical actuators inevitably exposes real-world power electronics edge cases.

### Regenerative Braking Transients & Buck Converter Redesign (Control Board V1.0 → V1.1)

During initial 200 Hz closed-loop testing on Control Board V1.0, an aggressive torque reversal near the 180° upright equilibrium point caused a failure on the 5V and 3.3V power rails. SEGGER RTT telemetry captured the exact system state milliseconds before the failure:

```text
State:2 | Angle:176 deg | TopErr:-320 cDeg | CmdI:-3796 mA | VescRPM:-2210 | VescCur: 4100 mA
State:2 | Angle:178 deg | TopErr:-173 cDeg | CmdI:-2162 mA | VescRPM:-2297 | VescCur: 2500 mA
State:2 | Angle:179 deg | TopErr:  -2 cDeg | CmdI: -158 mA | VescRPM:-2342 | VescCur: -400 mA
State:2 | Angle:181 deg | TopErr: 177 cDeg | CmdI: 2056 mA | VescRPM:-2315 | VescCur:-1500 mA
```

**Root cause analysis:**

1. While the reaction wheel was spinning at `-2342 RPM`, the pendulum crossed the 180° setpoint (`179°` → `181°`), commanding an immediate torque reversal (`+2056 mA`).
2. Commanding opposing torque against a fast-spinning inertial load forced the 4-quadrant VESC into **regenerative braking**, dumping kinetic energy ($E = \frac{1}{2}J_w\omega_w^2$) back onto the 24V DC bus.
3. Because the system was powered by a 1-quadrant 24V bench supply incapable of sinking reverse current, the regenerated charge spiked the 24V bus voltage above the **28V absolute maximum rating** of the V1.0 **TI TPS54302** buck converter.

**Control Board V1.1 hardware & firmware mitigations:**

- **60V-Rated Buck Stage:** Replaced the 28V TPS54302 with a **TI LMR16030SDDAR** (60V max input, providing 36V of headroom above the 24V nominal bus).
- **TVS Clamping & Input Isolation:** Added a **Littelfuse SMBJ26CA** bidirectional TVS diode ($V_{RWM} = 26\text{V}$, $V_C = 42.1\text{V}$), an **SS56 (5A, 60V)** series blocking Schottky diode, and a **100µF electrolytic bulk capacitor** ahead of the buck stage.
- **Software & VESC Safeguards:** Configured VESC regenerative current/voltage cutoffs and implemented a firmware overspeed cutoff (`MAX_WHEEL_RPM = 2500`) that commands `0.0 A` (freewheel coasting) rather than active braking if safe speed bounds are exceeded.

**Lesson learned:** Never size DC-DC converter voltage ratings based solely on nominal supply voltage in motor-driven systems; always account for regenerative braking energy and 1-quadrant power supply behavior with ample voltage headroom, TVS clamping, and bulk capacitance. The V1.1 board has since been used for the swing-up and upright stabilization results above.

## Key Features

- **High-Performance Compute:** Custom control board built around a 480 MHz STM32H743VIT6 (Cortex-M7 with double-precision FPU), architected for real-time optimization solvers (`acados` NMPC, planned).
- **Non-Linear Swing-Up & Stabilization (working on hardware):** A Lyapunov energy-shaping swing-up controller pumps mechanical energy into the pendulum and hands off to an upright stabilizing controller (PID + wheel velocity damping). LQR and NMPC are in progress.
- **Deterministic RTOS Firmware:** CMSIS-RTOS v2 (FreeRTOS) architecture separating 1 kHz non-blocking SPI DMA sensor acquisition and 200 Hz FDCAN control loops from background SEGGER RTT telemetry.
- **Continuous 360° Rotation:** Mechanical design utilizes a 10A slip ring and degreased 608ZZ bearings (re-lubricated with light machine oil) to allow unrestricted 360° swing-up orbits with minimal Coulomb friction.
- **Dual 14-Bit Magnetic Encoders:** Custom AS5047P encoder PCBs, running in ABI mode on the motor for high-speed FOC commutation and 16-bit SPI mode on the pendulum joint for wrap-free absolute angle tracking.

## Hardware Architecture

### Custom Electronics

- **Main Control Board (Rev 1.1):**
  - **MCU:** STM32H743VIT6 (LQFP100, 480 MHz, 2 MB Flash, 1 MB RAM).
  - **Transient-Hardened Power Stage:** TI LMR16030 (60V-rated 24V-to-5V buck) and ME6211 (5V-to-3.3V LDO), protected by a Littelfuse SMBJ26CA bidirectional TVS diode, SS56 (5A, 60V) reverse-blocking Schottky diode, and 100µF bulk electrolytic capacitor to withstand regenerative braking voltage spikes on a 1-quadrant bench supply.
  - **Connectivity & Peripherals:** TJA1051T/3 CAN transceiver, hardware RC-debounced user button, status LEDs, USB-C, and SWD interface supporting non-intrusive OpenOCD Live Expressions and SEGGER RTT telemetry.
- **Modular Encoder Boards:**
  - Custom 2-layer PCBs for the AMS AS5047P 14-bit magnetic rotary sensor with hardware solder jumpers to select between ABI and SPI interfaces.

### Electromechanical Setup

- **Actuation:** 4006 380Kv BLDC pancake motor (14 pole pairs) driven by a Flipsky Mini VESC 6.7 Pro.
- **Chassis:** Custom 3D-printed structural frame and reaction wheel.

## Firmware Architecture & STM32CubeMX Configuration

The firmware is built on STM32CubeHAL and **FreeRTOS (CMSIS_V2)**, configured to maximize determinism and eliminate CPU blocking during sensor and bus I/O.

### Clock Tree & Peripherals (STM32CubeMX)

- **System Clock:** 25 MHz external crystal (HSE) feeding `PLL1` (`SYSCLK = 480 MHz`, `HCLK = 240 MHz`, APB timers at `240 MHz`, `TIM6` dedicated as HAL timebase).
- **SPI1 (Joint Encoder):** Full-Duplex Master, 16-bit Motorola frame, Mode 1 (`CPOL = Low`, `CPHA = 2 Edge`), clocked at **7.5 MHz** (`PLL1Q = 60 MHz`, Prescaler `/8`). Configured with `DMA1_Stream0` (RX) and `DMA1_Stream1` (TX) for zero-CPU-overhead angle reads (`0xFFFF` read command with hardware even-parity verification).
- **FDCAN1 (VESC Link):** Classic CAN 2.0B mode (29-bit Extended IDs) running at **500 kbps** (`PLL2Q = 40 MHz`, Prescaler `5`, `Seg1 = 13`, `Seg2 = 2`, 87.5% sample point). Configured with RX FIFO 0 interrupts (`CAN_PACKET_STATUS` at 200+ Hz) and an 8-element TX FIFO queue (`CAN_PACKET_SET_CURRENT`).
- **Real-Time Telemetry:** SEGGER RTT over SWD (`SWDIO`/`SWCLK`) via OpenOCD (`localhost:9090`), enabling zero-latency logging without USB COM port resets during flashing.

### RTOS Task Partitioning & Finite State Machine (FSM)

1. **`ControlTask` (`osPriorityRealtime`, 1 kHz / 200 Hz):**
   - **1 kHz Sensor Loop:** Triggers non-blocking 16-bit SPI DMA reads of the AS5047P joint encoder. In the DMA completion ISR (`HAL_SPI_TxRxCpltCallback`), raw 14-bit counts (`0–16383`) are converted into wrap-safe signed upright error $\theta_{\text{err}} \in [-180^{\circ}, +180^{\circ})$, low-pass filtered angular velocity ($\dot{\theta}_p$, $\alpha = 0.15$), and normalized mechanical energy ($E \in [0, 2]$).
   - **200 Hz Actuation Loop:** Executes the active control law and transmits current commands ($I_q$) to the VESC over FDCAN, guarded by tilt ($\pm 25^\circ$) and wheel overspeed ($\pm 2500\text{ RPM}$) safety cutoffs.
2. **`TelemetryTask` (`osPriorityNormal`, 20 Hz):**
   - Handles button edge detection, drives LED state indicators, and streams system state over SEGGER RTT at 10 Hz.
3. **4-State Operational FSM:**
   - `UNCALIBRATED` (Green LED Off): Motor disarmed (`0.0 A`). Pressing the button captures a 200-sample wrap-safe average of the downward hanging position ($0^\circ$).
   - `READY` (Green LED 2 Hz Blink): Armed standby. Pressing the button while holding the pendulum near $180^\circ$ ($\pm 15^\circ$) transitions directly to `RUNNING`. Pressing the button while hanging at $0^\circ$ triggers `SWING_UP`.
   - `SWING_UP` (Green LED 10 Hz Fast Blink): Executes Lyapunov energy-shaping control, then automatically hands off to `RUNNING` once the pendulum enters the top capture region ($|\theta_{\text{err}}| < 18^\circ$, $|\dot{\theta}_p| < 300^\circ/\text{s}$):

     $$
     u_{\text{swing}} = -\text{dir} \cdot K_{\text{swing}}(E_{\text{target}} - E)\tanh\!\left(\frac{\dot{\theta}_p}{25}\right) + K_{w,\text{swing}}\omega_w
     $$

   - `RUNNING` (Green LED Solid On): Stabilizes the pendulum upright and rejects external disturbances using full-state feedback (PID on pendulum state + proportional damping $K_w$ on reaction wheel RPM to prevent actuator saturation).

## Current Status & Roadmap

The hardware, RTOS firmware, energy-based swing-up, and upright stabilization (PID + wheel velocity damping) are complete and verified on the physical testbed. Upcoming work focuses on formal system identification, MATLAB state-space modeling, LQR, and `acados` NMPC deployment. **None of the Phase 4–5 items below have been implemented yet.**

- [x] **Phase 1: Hardware & Mechanical Design**
  - [x] Control Board (V1.0 & V1.1) and Modular Encoder schematics, PCB layout, and assembly.
  - [x] 3D-printed chassis, slip ring integration, and low-friction bearing preparation.
- [x] **Phase 2: Firmware & Hardware Bring-Up**
  - [x] STM32H743 clock tree (480 MHz), FreeRTOS, and SEGGER RTT / OpenOCD telemetry.
  - [x] 1 kHz non-blocking AS5047P SPI DMA driver with hardware parity checking and wrap-safe zero calibration.
  - [x] 500 kbps FDCAN driver for VESC current/velocity control and real-time `CAN_STATUS_1` feedback.
- [x] **Phase 3: Baseline Control & Non-Linear Swing-Up**
  - [x] 4-state operational FSM (`UNCALIBRATED`, `READY`, `SWING_UP`, `RUNNING`).
  - [x] Full-state PID + Wheel Velocity Damping ($K_w$) controller for indefinite upright stabilization and disturbance rejection.
  - [x] Lyapunov energy-shaping swing-up controller with automatic upright capture hand-off.
- [ ] **Phase 4: System Identification & Modern Control (Upcoming)**
  - [ ] Experimental system ID (free-swing logarithmic decrement for $J_p, b_p$ and current step tests for $J_w, b_w$).
  - [ ] MATLAB/Simulink non-linear modeling, linearization, and LQR ($Q, R$) gain synthesis.
- [ ] **Phase 5: Advanced Optimal Control (Upcoming)**
  - [ ] Real-time Non-linear Model Predictive Control (NMPC) C-code generation via `acados` and deployment on the STM32H7.

## Project Timeline

| Date (2026) | Milestone |
|---|---|
| Oct 7 | Project conceptualization and control theory research began. |
| Oct 12 | Schematic capture for STM32H7 control board started. |
| Oct 15 | Control board and modular encoder board schematics completed; BOM finalized and sourced. |
| Oct 16 | Mechanical CAD finalized. Encoder board PCB layout completed. |
| Oct 17 | Control board V1.0 PCB layout completed and ordered. |
| Oct 28 | V1.0 PCBs and components arrived. |
| Oct 29 | Control board V1.0 and encoder board PCBs soldered. |
| Nov 8 | VESC FOC current and velocity control verified via VESC Tool with ABI encoder. |
| Nov 10 | STM32H7 CubeMX configuration completed; verified SPI DMA encoder reads, LEDs, button, and SEGGER RTT. |
| Nov 12 | Verified 500 kbps FDCAN closed-loop communication with VESC. Encountered 24V regenerative braking transient during initial upright test; diagnosed V1.0 buck converter overvoltage failure via RTT logs. |
| Nov 14 | Redesigned Control Board V1.1 with 60V LMR16030 buck, SMBJ26CA TVS diode, SS56 Schottky diode, and 100µF bulk cap. Ordered V1.1 PCBs. |
| Nov 20 | Control Board V1.1 PCBs and power protection components arrived. |
| Nov 21 | Control Board V1.1 soldered and power rails verified. |
| Nov 25 | Implemented 4-state FSM, PID + wheel velocity damping ($K_w$), and Lyapunov energy-shaping swing-up. Achieved autonomous swing-up and indefinite upright stabilization with disturbance rejection. |
