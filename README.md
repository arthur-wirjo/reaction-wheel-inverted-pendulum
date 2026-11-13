# Reaction Wheel Inverted Pendulum: Advanced Control Systems Testbed

<img width="300" alt="control_board_PCB_V1" src="https://github.com/user-attachments/assets/416d7393-27cb-4652-adee-f799d5f0c942" />
<img width="300" alt="solidworks_mechanical_setup" src="https://github.com/user-attachments/assets/7b6c8edf-8ffc-41e8-991d-190b54d3e69b" />
<img width="300" alt="IMG_20261029_080815_2" src="https://github.com/user-attachments/assets/cca72489-3b4f-4ce4-a304-28c06302b1cf" />
<img width="300" alt="IMG_20261101_163136" src="https://github.com/user-attachments/assets/a9174aa8-f36a-4378-affa-efb8ecdf4ad2" />

A custom Reaction Wheel Inverted Pendulum designed as a physical testbed for advanced modern control theory. This project features a custom-designed STM32H7 control board, dual high-resolution magnetic encoders, and implements complex control strategies ranging from LQR to Non-linear Model Predictive Control (NMPC).

## Overview

Following the development of my DIY FOC Motor Controller Project, I wanted to shift my focus from power electronics to **advanced control theory and system dynamics**. The inverted pendulum is a classic non-linear control problem. By using a reaction wheel, the system is underactuated and requires continuous, precise torque commands to maintain balance.

Instead of using off-the-shelf development boards, I designed a custom STM32H7-based control board to handle the heavy computational requirements of real-time NMPC. The board communicates via FDCAN with a Flipsky Mini VESC 6.7 Pro, which handles the low-level Field-Oriented Control (FOC) of the reaction wheel. 

## Key Features

* **High-Performance Compute:** Custom control board built around the 480MHz STM32H743VIT6, providing the necessary floating-point performance for real-time model predictive control.
* **Advanced Control Algorithms:** Designed to run and compare PID, Linear Quadratic Regulator (LQR), and Acados-generated Non-linear Model Predictive Control (NMPC).
* **Robust CAN Bus Communication:** TJA1051T/3 transceiver enables high-speed, noise-immune torque commanding and telemetry between the STM32H7 and the VESC.
* **Continuous 360° Rotation:** Mechanical design utilizes a 10A slip ring and degreased 608ZZ bearings (lubricated with light machine oil) to allow infinite rotation with minimal friction.
* **Dual High-Res Encoders:** Custom AS5047P magnetic encoder PCBs. The motor encoder runs in ABI mode for fast commutation, while the joint encoder runs over SPI for absolute position tracking without wrapping bugs.

## Hardware Architecture

### Custom Electronics
* **Main Control Board (V1.1):** 
  * **MCU:** STM32H743VIT6 (Chosen for extreme computational power required by NMPC).
  * **Power Management:** Onboard TI LMR16030 (60V-rated 24V-to-5V buck) and ME6211 (5V-to-3.3V LDO), protected by a Littelfuse SMBJ26CA TVS diode, B360A 3A/60V reverse-polarity Schottky diode, and 100µF electrolytic bulk capacitance to safely power the logic alongside regenerative motor loads.
  * **Connectivity:** USB-C for telemetry, FDCAN for VESC communication, and SWD (with SEGGER RTT) for real-time debugging.
* **Modular Encoder Boards:** 
  * Custom 2-layer PCBs housing the AMS AS5047P 14-bit magnetic encoders. 
  * Designed with solder jumpers to easily switch between ABI (for the motor) and SPI (for the joint) configurations.

### Electromechanical Setup
* **Actuation:** Generic 4006 380Kv Drone BLDC motor driven by a Flipsky Mini VESC 6.7 Pro.
* **Mechanical Structure:** Fully custom 3D-printed chassis.
* **Friction Mitigation:** Standard 608ZZ skate bearings, stripped of their heavy factory grease and re-lubricated with light machine oil to minimize non-linear friction disturbances in the plant model.

## Control Strategies & Software

The core objective of this project is to model the system in MATLAB/Simulink and deploy increasingly complex control strategies to the STM32H7 in C/C++.

1. **System Identification & PID:** 
   * Finding the total mass, moment of inertia, and center of mass.
   * Establishing a baseline PID controller to validate sensor reads and CAN bus latency.
2. **LQR (Linear Quadratic Regulator):** 
   * Building a non-linear plant model and linearizing it in MATLAB to extract the state-space \( A \) and \( B \) matrices.
   * Tuning \( Q \) and \( R \) cost matrices in simulation to achieve fast stabilization without saturating the motor's maximum torque.
3. **Energy-Based Swing-Up:** 
   * Implementing a non-linear energy-shaping controller to pump energy into the pendulum, swinging it from a resting downward position into the linear capture region of the LQR controller.
4. **NMPC (Non-linear Model Predictive Control):** 
   * Formulating the optimal control problem in Python/MATLAB.
   * Generating highly optimized C-code using **acados**, deployed directly to the STM32H7 to predict and optimize the pendulum's trajectory in real-time.

## Challenges and Lessons Learned

Designing hardware that interfaces high-speed digital logic with dynamic electromechanical actuators inevitably exposes real-world power electronics edge cases.

### Regenerative Braking Transients and Buck Converter Headroom (Control Board V1.0)
After successfully verifying SPI DMA encoder reads and 500 kbps FDCAN communication with the VESC, I implemented a 200 Hz PID control loop to test initial upright stabilization. During an aggressive direction reversal around the 180° upright equilibrium point, a spark occurred on the V1.0 control board, taking down the 5V and 3.3V power rails and shorting the 3.3V rail to GND.

Fortunately, SEGGER RTT telemetry captured the exact state of the system in the final milliseconds before the failure:

    State:2 | Angle:176 deg | TopErr:-320 cDeg | CmdI:-3796 mA | VescRPM:-2210 | VescCur: 4100 mA
    State:2 | Angle:178 deg | TopErr:-173 cDeg | CmdI:-2162 mA | VescRPM:-2297 | VescCur: 2500 mA
    State:2 | Angle:179 deg | TopErr:  -2 cDeg | CmdI: -158 mA | VescRPM:-2342 | VescCur: -400 mA
    State:2 | Angle:181 deg | TopErr: 177 cDeg | CmdI: 2056 mA | VescRPM:-2315 | VescCur:-1500 mA

Analyzing the telemetry revealed the root cause:
1. While the reaction wheel had accumulated high negative angular velocity (`-2342 RPM`), the pendulum crossed the 180° upright setpoint (`179°` to `181°`), causing the controller to command an immediate torque reversal (`+2056 mA`).
2. Applying opposing torque against a fast-spinning inertial load forced the VESC into **regenerative braking**, dumping kinetic energy from the reaction wheel back onto the 24V DC bus.
3. Because the system was powered by a 24V bench power supply (a 1-quadrant source incapable of sinking reverse current), the regenerated energy had nowhere to go, causing the 24V bus voltage to spike violently.
4. On the V1.0 PCB, I had selected the **TI TPS54302** for the 24V-to-5V buck stage, which has an absolute maximum input voltage rating of only **28V** (leaving a razor-thin 4V safety margin on a 24V nominal bus) and lacked input TVS clamping. The regenerative voltage spike exceeded the TPS54302's breakdown voltage, causing the buck stage to fail catastrophically and cascade into the 3.3V rail.

For **Control Board V1.1**, I redesigned the power input stage to handle dynamic regenerative loads:
* **Wide-Input Buck Converter:** Replaced the 28V-rated TPS54302 with a **TI LMR16030SDDAR**, expanding the maximum input voltage rating to **60V** (providing 36V of transient headroom above the 24V nominal rail).
* **Transient Voltage Suppression:** Added a **Littelfuse SMBJ26CA** bidirectional TVS diode ($V_{RWM} = 26\text{V}$, $V_C = 42.1\text{V}$) at the power entry to clamp inductive and regenerative voltage spikes well below the LMR16030's 60V limit.
* **Reverse-Polarity & Bus Hold-Up Filtering:** Added a **SS56 (5A, 60V)** series Schottky diode followed by a **100µF electrolytic bulk capacitor** feeding the buck stage to prevent reverse current flow and isolate the logic supply from motor bus voltage dips.

**Lesson learned:** Never size DC-DC converter voltage ratings based solely on nominal supply voltage in motor-driven systems; always account for regenerative braking energy and 1-quadrant power supply behavior with ample voltage headroom, TVS clamping, and bulk capacitance.

## Current Status & Roadmap

The project is currently in active development. Hardware V1.0 bring-up exposed a power stage transient limitation during initial closed-loop tests, and Control Board V1.1 is currently being spun.

- [x] **Phase 1: Hardware & PCB Design**
  - [x] Schematics (Updated to V1.1 with 60V buck & TVS protection)
  - [x] PCB layout
  - [x] Component sourcing
  - [x] PCB soldering/assembly
  - [x] Mechanical design 
- [x] **Phase 2: Hardware Bring-up** 
  - [x] Mechanical assembly
  - [x] Flash and verify STM32H7 custom board.
  - [x] Test VESC FOC torque/current control via VESC Tool.
  - [x] Validate AS5047P SPI joint encoder (ensure 360° continuous reading).
  - [x] Establish STM32-to-VESC CAN communication.
- [ ] **Phase 3: System ID & Baseline Control**
  - [ ] Assemble Control Board V1.1
  - [ ] PID implementation & tuning
  - [ ] System identification 
- [ ] **Phase 4: Modern Control** (MATLAB simulation, LQR implementation, Energy-based swing-up).
- [ ] **Phase 5: Advanced Control** (Acados NMPC generation and deployment).

## Project Timeline

| Date (2026) | Milestone |
|---|---|
| Oct 7 | Project conceptualization and control theory research began. |
| Oct 12 | Schematic capture for STM32H7 control board started. |
| Oct 15 | Control board and modular encoder board schematics completed; BOM finalized and sourced. |
| Oct 16 | Mechanical CAD finalized. Encoder board PCB layout completed. |
| Oct 17 | Control board PCB layout completed. PCBs ordered. |
| Oct 28 | PCB and Components Arrived. |
| Oct 29 | Control board and encoder board PCBs soldered. |
| Nov 8 | VESC FOC torque/current and velocity control via VESC Tool tested. |
| Nov 10 | STM32H7 Firmware configuration done and wrote simple firmware to test SPI encoder, LEDs, and buttons. |
| Nov 12 | STM32H7 FDCAN communication with VESC working. Velocity and torque/current command with RPM and motor current feedback successful. |
| Nov 12 | Encountered 24V regenerative braking voltage spike during initial closed-loop PID testing, destroying the V1.0 28V-rated buck converter. Initiated Control Board V1.1 redesign with 60V-rated TI LMR16030 buck, SMBJ26CA TVS diode, B360A Schottky diode, and 100µF bulk cap. |
