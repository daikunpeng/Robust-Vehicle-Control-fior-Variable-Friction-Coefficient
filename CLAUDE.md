# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

This is a research project focused on developing **Reinforcement Learning-based robust path following control algorithms** for variable road-tire friction coefficient scenarios. The project aims to improve autonomous vehicle control performance under changing road conditions.

## Project Status

**Early-stage research project** - The repository is currently in its initial phase with minimal structure. Key components and architecture are yet to be implemented.

## Architecture

This project uses an integrated architecture with MATLAB as the primary environment:

- **TruckSim** - High-fidelity vehicle dynamics simulation for truck models (interfaces via S-functions)
- **MATLAB/Simulink** - Control system design, simulation environment, and RL implementation
- **MATLAB Reinforcement Learning Toolbox** - RL algorithms (DQN, DDPG, PPO, SAC, TD3) for control policy training
- **Variable friction coefficient modeling** - Simulation of different road surface conditions (dry, wet, icy, etc.)

The typical workflow involves:
1. Design RL environment in Simulink with TruckSim S-function blocks
2. Train RL agents using MATLAB Reinforcement Learning Toolbox
3. Implement and test control policies directly in Simulink
4. Validate robust controllers under varying friction conditions

## Development Guidelines

### Technology Stack

**Primary Platform (All-in-One):**
- **MATLAB/Simulink** - Control system design, simulation, and RL environment
- **MATLAB Reinforcement Learning Toolbox** - RL algorithms and training framework
- **TruckSim** - Vehicle dynamics simulation (requires TruckSim installation with Simulink S-function support)

**Supported RL Algorithms (MATLAB Toolbox):**
- **DQN** (Deep Q-Network) - Discrete action spaces
- **DDPG** (Deep Deterministic Policy Gradient) - Continuous action spaces
- **PPO** (Proximal Policy Optimization) - Both discrete and continuous
- **SAC** (Soft Actor-Critic) - Continuous action spaces, sample-efficient
- **TD3** (Twin Delayed DDPG) - Improved DDPG for continuous control

**Optional (for advanced/custom needs):**
- **Python** - For custom algorithm development or data analysis (if needed)
- **PyTorch/TensorFlow** - For implementing custom RL algorithms not available in MATLAB

### Suggested Directory Structure (Future)

As the project grows, consider organizing with:
- `models/` - Simulink model files (.slx, .mdl)
- `matlab/` - MATLAB scripts, functions, and RL training code
- `trucksim/` - TruckSim configuration files (.sim, .par) and vehicle models
- `data/` - Training data, logged simulations, and datasets
- `results/` - Experiment results, trained agents, and performance metrics
- `utils/` - Helper functions and utilities
- `tests/` - Unit and integration tests
- `docs/` - Documentation and technical reports

**Optional (if Python is needed):**
- `python/` - Python scripts for data analysis or custom algorithms
- `notebooks/` - Jupyter notebooks for visualization and analysis

## License

This project is licensed under **Apache License 2.0**. All contributions must comply with this license.