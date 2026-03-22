# Opt_Robot

## Overview

This project optimises the design of a 6 degrees-of-freedom (6DOF) robot arm based on performance objectives defined in Cartesian space. The goal is to use a robot model to parameterise the physical attributes of the arm (link lengths, joint limits, etc.) and apply a **Genetic Algorithm (GA)** to search the design space, producing families of robot arm configurations that are each fitted to specific performance objectives.

## Problem Statement

Designing a robot arm for a given task involves choosing geometric and mechanical parameters that maximise performance in the workspace the robot must operate in. Rather than hand-tuning these parameters, this project frames the design problem as an optimisation problem:

- **Decision variables** — the robot arm's structural parameters (e.g. DH parameters: link lengths, offsets, twist angles, joint limits).
- **Objective functions** — Cartesian-space performance metrics such as workspace volume, dexterity, reachability, or task-specific measures.
- **Optimisation algorithm** — a Genetic Algorithm that evolves a population of robot arm designs across generations, selecting and recombining high-performing individuals.

## Approach

1. **Robot model** — a kinematic model (likely using Denavit-Hartenberg convention) encodes the robot's geometry and computes forward/inverse kinematics and Jacobian-based metrics.
2. **Performance evaluation** — each candidate design is evaluated in Cartesian space against one or more objective functions.
3. **Genetic Algorithm** — a population of designs is evolved through selection, crossover, and mutation to converge on designs that satisfy the performance objectives.
4. **Design families** — by targeting different objective functions (or Pareto fronts in multi-objective settings), the GA produces distinct families of optimised robot arm designs.

## Physics Engine: MuJoCo

[MuJoCo](https://mujoco.org) is used as the physics simulation backend. It evaluates candidate solutions produced by the optimisation algorithms by simulating robot dynamics, computing forward kinematics, Jacobians, and contact physics.

MuJoCo is isolated behind the `IPhysicsEngine` interface (`src/interfaces/IPhysicsEngine.h`), so it can be swapped for another engine without touching the optimisation or problem layers.

### Installing MuJoCo

Download the prebuilt binary for Linux and extract it:

```bash
wget https://github.com/google-deepmind/mujoco/releases/download/3.2.3/mujoco-3.2.3-linux-x86_64.tar.gz
tar -xf mujoco-3.2.3-linux-x86_64.tar.gz -C ~/.local/
```

Add the shared library to your runtime path (add to `~/.bashrc` to make permanent):

```bash
export LD_LIBRARY_PATH=$HOME/.local/mujoco-3.2.3/lib:$LD_LIBRARY_PATH
```

## Build

```bash
cmake -B build
cmake --build build
./build/Main
```

If MuJoCo is installed somewhere other than `~/.local/mujoco-3.2.3`, pass its path at configure time:

```bash
cmake -B build -DMUJOCO_DIR=/path/to/mujoco-3.2.3
```

## Dependencies

- C++17 or later
- CMake 3.18+
- MuJoCo 3.2.3 (see above)
