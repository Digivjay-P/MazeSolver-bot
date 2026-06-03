# eYRC-1687 — Autonomous FPGA Maze-Solving Robot

> **e-Yantra Robotics Competition 2025–26 | Team ID: 1687**

## Overview

This repository contains the complete RTL design and firmware for an **autonomous maze-solving robot** built as part of the e-Yantra Robotics Competition (eYRC) 2024–25. The robot navigates an unknown **9×9 grid maze** entirely in hardware, using a custom multi-module Verilog architecture implemented on an FPGA.

The system implements two distinct navigation strategies in hardware: a reactive **Wall-Following FSM** for initial exploration, and a full **Trémaux Algorithm Brain** for deterministic maze-solving with backtracking. Three ultrasonic sensors provide real-time distance feedback, dual quadrature encoders enable closed-loop turning, and a hierarchical FSM pipeline coordinates every decision from high-level pathfinding down to individual motor PWM signals — all without a CPU.

---

## 🎥 Demo

[![eYRC-1687 Submission Demo](https://img.shields.io/badge/YouTube-Watch%20Demo-red?style=for-the-badge&logo=youtube)](https://www.youtube.com/watch?v=37qrg9yEstA&feature=youtu.be)

---

## Table of Contents

1. [Key Features](#key-features)
2. [Tech Stack & Hardware](#tech-stack--hardware)
3. [System Architecture](#system-architecture)
4. [Algorithm Design](#algorithm-design)
5. [RTL Module Breakdown](#rtl-module-breakdown)
6. [Challenges & Solutions](#challenges--solutions)
7. [Repository Structure](#repository-structure)

---

## Key Features

- **Dual-Algorithm Navigation:** The robot ships with two complete, independently verified navigation brains. The Wall-Following FSM handles reactive corridor navigation, while the Trémaux Brain handles full graph-theoretic maze solving with visited-path memory — both implemented purely in synthesized RTL.

- **Encoder-Closed-Loop Turning:** Turns are not time-based. A quadrature encoder continuously reports wheel displacement, and the Turn state exits only when a calibrated encoder tick threshold (≈3,400 counts for 90°) is reached. This makes turns repeatable regardless of battery voltage or surface friction.

- **Trémaux Mark Memory:** The maze-solving brain maintains a `mark_mem[81][4]` register array in hardware — one 2-bit mark per cell per direction across the full 9×9 grid. The brain increments marks on departure and entry, and at every junction picks the direction with the lowest mark value, guaranteeing complete maze exploration with no repeated dead-ends.

- **Hierarchical Brain–Motor Decoupling:** Decision-making and actuation are cleanly separated. The Trémaux Brain FSM issues high-level commands (`LEFT`, `RIGHT`, `FORWARD`, `U-TURN`) over a simple handshake interface to the Motor Driver FSM, which handles all timing, stopping, and PWM control independently. Neither layer needs to know the internals of the other.

- **Encoder-Corrected Turning with Wall-Follow Recovery:** Encoder-Corrected Turning with Wall-Follow Recovery: Turn completion is not time or counter based — the robot exits the turn state only when the encoder displacement crosses a calibrated tick threshold. If the turn overshoots or undershoots, the wall-following algorithm is re-engaged immediately after, using live ultrasonic feedback to self-correct alignment before proceeding. This makes physical turns robust to surface slip and motor variance.

- **PWM Speed Control with Dynamic Adjustment:** A frequency-scaled PWM generator drives both motors at runtime-tunable duty cycles. During wall-following, left/right ultrasonic distance differential dynamically adjusts the two channels to keep the robot centered in the corridor without any microcontroller involvement.

---

## Tech Stack & Hardware

### Hardware

| Component | Details |
|-----------|---------|
| **Target Board** | Altera DE0 nano FPGA (programmed via `.jic` bitstream) |
| **Navigation Sensors** | 3× HC-SR04 Ultrasonic — Front (`dF`), Left (`dL`), Right (`dR`) |
| **Obstacle Detection** | 3× IR sensors — Front, Left, Right (debounced in RTL) |
| **Odometry** | 2× Quadrature encoders — Left and Right wheels |
| **Actuation** | Dual DC motors via H-bridge (IN1–IN4 direction + ENA/ENB PWM) |
| **Clock** | 50 MHz onboard oscillator |

### RTL Tools

| Category | Tools |
|----------|-------|
| **RTL Design** | Verilog HDL |
| **Synthesis & P&R** | Intel Quartus Prime |
| **Algorithm Prototype** | C++ (`tremaux_in_C++.cpp`) |
| **Bitstream** | `'name'.jic` (JTAG Indirect Configuration File) |

---

## System Architecture

The system is organized as a three-layer hierarchy. Each layer communicates through clean signal interfaces, keeping logic modular and testable.

```
┌──────────────────────────────────────────────────┐
│              BRAIN LAYER                         │
│  tremaux_brain.v  /  brain_WF.v                  │
│  • Trémaux mark memory (81 cells × 4 dirs)       │
│  • Junction decision logic                       │
│  • Grid coordinate tracking (x, y, current_cell) │
│  • Output: cmd_out → {LEFT, RIGHT, FWD, UTURN}   │
└────────────────────┬─────────────────────────────┘
                     │ start_in / need_decision / event_out handshake
┌────────────────────▼─────────────────────────────┐
│              MOTOR DRIVER LAYER                  │
│  motoring.v  /  FSM.v                            │
│  • 7-state FSM: BOOT→FOLLOW→FWD_BEFORE→          │
│    STOP_BEFORE→TURN→STOP_AFTER→FWD_AFTER         │
│  • Encoder-gated turn completion                 │
│  • Wall-following PD (ultrasonic diff → PWM)     │
│  • Fires need_decision when junction reached     │
└────────────────────┬─────────────────────────────┘
                     │ IN1–IN4, ENA/ENB PWM
┌────────────────────▼─────────────────────────────┐
│              PERIPHERAL LAYER                    │
│  distance.v · encoder_DP · pwm_generator · ir.v  │
│  • Ultrasonic echo timing → 16-bit cm distance   │
│  • Encoder pulse counting → 20-bit tick counter  │
│  • Frequency-scaled PWM (50MHz → 3.125MHz base)  │
│  • IR signal debounce                            │
└──────────────────────────────────────────────────┘
```

---

## Algorithm Design

### Wall-Following FSM (`FSM.v` / `WF_using_US.v`)

The initial navigation strategy. A 7-state Mealy FSM drives the robot through the maze by hugging the nearest wall, making turn decisions based purely on real-time sensor readings.

| State | Behaviour |
|-------|-----------|
| `S_BOOT` | 2-second initialization delay before motion begins to get stable sensor readings |
| `S_FOLLOW` | Active wall-following; adjusts PWM duty cycle based on `dL`–`dR` differential to stay centered |
| `S_FORWARD_BEFORE` | Creeps forward until IR confirms wall contact before committing to a turn or based on calibrated encoder count |
| `S_STOP_BEFORE` | Hard motor brake; latches current encoder counts as turn reference (90 degree turn) |
| `S_TURN` | Spot-turns until encoder displacement exceeds `turn_param` (≈3,400 ticks) (tuned for ~90 degree turn); direction selected by sensor state at junction |
| `S_STOP_AFTER` | Second hard brake after turn completion |
| `S_FORWARD_AFTER` | Drives forward based on encoder count / front IR readings, then returns to `S_FOLLOW` |

**Turn Direction Logic (at junction):**
```
!obst_l && (obst_r || !obst_r)               →  Turn LEFT  ie no obstacle at left or at a junction
 obst_l && !obst_r                           →  Turn RIGHT ie no obstacle at right
 obst_l &&  obst_r && (dF < tuned_parameter) →  U-TURN ie blocked from all three sides
```

---

### Trémaux Algorithm Brain (`tremaux_brain.v`)

The full maze-solving strategy, capable of guaranteeing exit from any simply-connected maze. The brain maintains a persistent mark array in hardware registers and communicates with the motor driver via a two-signal handshake (`need_decision` / `start_out`).

**Mark Memory:** `mark_mem[81][4]` — 81 cells (9×9 grid), 4 directions each (N/E/S/W), 2-bit values (0, 1, 2).

**Brain FSM States:**

| State | Action |
|-------|--------|
| `BOOT` | Waits for `robot_run`; initializes position to cell 76 (grid coordinate 4,8) and zeroes all marks |
| `WAIT_FOR_REQ` | Monitors `need_decision`; while in follow, updates `current_cell` and increments marks whenever encoder displacement exceeds cell threshold (≈3,400 ticks) |
| `DECIDE` | Reads mark values for all open directions; selects direction with lowest mark; classifies junction type (dead-end / path / junction) |
| `SEND_CMD` | Raises `start_out`; waits for motor driver to acknowledge; commits mark increments, updates `current_cell`, `current_dir`, and `(x, y)` |

**Junction Decision (Trémaux Rules):**
```
At any junction, evaluate open paths:
  markF, markL, markR  (unavailable paths get mark = 3)

Select min-mark direction. Ties broken Left > Forward > Right.
If all marks = 0  → prefer Left (unvisited path priority)
If all marks ≥ 1  → return via U-turn (backtrack)
```

**Mark Update Protocol (on cell crossing):**
```
Departing cell C via direction D:
  mark_mem[C][D]++

Entering cell C' from direction D:
  mark_mem[C'][opposite(D)]++
```

---

## RTL Module Breakdown

| File | Role |
|------|------|
| `tremaux_brain.v` | Top-level Trémaux maze solver — brain FSM + mark memory + coordinate tracking |
| `brain_WF.v` | Wall-following brain variant with turn-count tracking |
| `motoring.v` | Mid-layer motor driver; interfaces brain commands to low-level PWM and direction registers |
| `Motor_driver.v` | H-bridge direction and enable logic (IN1–IN4, ENA, ENB) |
| `WF_using_US.v` | Ultrasonic-only wall follower variant |
| `logical_WF_Turning.v` | Logical turn decision module for wall-follow mode |
| `distance.v` | HC-SR04 ultrasonic driver — generates trig pulse, times echo, outputs 16-bit cm distance |
| `encoder` | Quadrature encoder counter — outputs 20-bit cumulative tick count |
| `callibarated.v` | Sensor calibration and offset correction |
| `ddsmd.v` | Direct drive speed/motor driver |
| `tremaux_verilog.v` | Standalone Trémaux RTL prototype |
| `tremaux_in_C++.cpp` | C++ simulation used to verify algorithm logic before RTL implementation |
| `wall_following.v` | Earlier standalone wall-follower iteration |
| `67_points_attempt2.v` | Competition scoring attempt variant |
| `hard_code1.v` / `hardcode2.v` | Fixed route programs for specific maze configurations |
| `2004.jic` | Final FPGA bitstream for programming via Quartus |

---

## Challenges & Solutions

| Challenge | Solution |
|-----------|---------|
| **Incorrect turn angles causing navigation drift** | Time-based and IR-based turn exits were unreliable on physical hardware. Replaced with encoder-gated turn completion, then re-engaged the wall-following algorithm immediately post-turn so the robot uses live ultrasonic distance feedback to correct any residual misalignment before continuing |
| **Imprecise time-based turning** | Replaced fixed-time turn delays with encoder-gated turn exits. The FSM leaves `S_TURN` only when measured wheel displacement crosses `turn_param ≈ 3,400` ticks, making 90° turns repeatable across battery states |
| **Wall-follow sensor noise** | Added a `DEAD_BAND = 30` (3 cm) threshold below which left/right distance differential is ignored, preventing the robot from over-correcting in straight corridors |
| **Cell boundary tracking in Trémaux** | Encoder displacement since last cell commit (`R_diff = encoder_counter_R - R_ref`) is compared against `cell_threshold = 3,400` to trigger mark updates and cell increments without GPS or external positioning |
| **Brain–Motor synchronisation** | Designed a two-signal handshake: motor driver asserts `need_decision` when it reaches a junction; brain asserts `start_out` with `cmd_out` valid; motor driver clears `need_decision` on acceptance; brain then commits state in `SEND_CMD` |
| **Synthesizer dead-code elimination** | Applied `(* DONT_TOUCH = "true" *)` and `(* keep *)` attributes to critical wires and registers that Quartus was incorrectly pruning as unreachable during optimization |

---

## Repository Structure

```
MazeSolver-bot/
│
├── tremaux_brain.v          # Trémaux Algorithm — top-level brain FSM
├── tremaux_verilog.v        # Trémaux RTL prototype
├── tremaux_in_C++.cpp       # C++ algorithm verification
│
├── brain_WF.v               # Wall-following brain
├── WF_using_US.v            # Ultrasonic-based wall follower
├── wall_following.v         # Wall-following base module
├── logical_WF_Turning.v     # Turn logic for wall-follow mode
│
├── motoring.v               # Mid-layer motor interface
├── Motor_driver.v           # H-bridge driver (IN1–IN4, ENA, ENB)
├── ddsmd.v                  # Direct drive motor module
│
├── distance.v               # HC-SR04 ultrasonic driver
├── encoder                  # Quadrature encoder counter
├── callibarated.v           # Sensor calibration
│
├── 67_points_attempt2.v     # Competition run variant
├── hard_code1.v             # Fixed-route program A
├── hardcode2.v              # Fixed-route program B
│
├── 2004.jic                 # FPGA bitstream (Quartus JTAG programming file)
├── 2004.map                 # Quartus pin/resource map
│
└── README.md
```

---

*e-Yantra Robotics Competition 2024–25 · Team 1687*
