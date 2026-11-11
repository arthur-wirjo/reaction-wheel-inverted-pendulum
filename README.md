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
* **Main Control Board:** 
  * **MCU:** STM32H743VIT6 (Chosen for extreme computational power required by NMPC).
  * **Power Management:** Onboard TI TPS54302 (24V-to-5V buck) and ME6211 (5V-to-3.3V LDO) to safely power the logic from a 24V bench supply.
  * **Connectivity:** USB-C for telemetry, FDCAN for VESC communication, and SWD for debugging.
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

## Current Status & Roadmap

The project is currently in active development. Hardware design is complete, and parts are being manufactured/sourced.

- [x] **Phase 1: Hardware & PCB Design** (Schematics, PCB layout, component sourcing).
- [ ] **Phase 2: Hardware Bring-up** 
  - [x] Flash and verify STM32H7 custom board.
  - [x] Test VESC FOC torque/current control via VESC Tool.
  - [x] Validate AS5047P SPI joint encoder (ensure 360° continuous reading).
  - [ ] Establish STM32-to-VESC CAN communication.
- [ ] **Phase 3: System ID & Baseline Control** (Mechanical assembly, System ID, PID implementation).
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
| Nov 10 | STM32H7 Firmware configuration done and wrote simple firmware to test SPI encoder, LEDs, and buttons |
