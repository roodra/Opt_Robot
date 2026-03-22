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

## Dependencies

- C++17 or later
- CMake 3.18+
