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

### Documentation Standards

**IMPORTANT:** This project requires strict documentation standards to maintain code clarity and system understanding.

#### Folder Documentation Requirements

**Every folder MUST contain a README.md with:**

1. **Architecture Description** (maximum 3 lines) - Brief overview of the folder's purpose and role in the system
2. **Files Section** - Listing each file with:
   - Name
   - Status (e.g., "In development", "Complete", "Experimental")
   - Function/Purpose

3. **Update Reminder** - At the bottom:
   ```markdown
   **Last Updated:** [Date]
   **Please update this document if any changes occur to this folder.**
   ```

#### File Header Requirements

**Every code file MUST start with:**

**For MATLAB files (.m):**
```matlab
% Please update this document if any changes occur to the folder it belongs to.
% Once this file is updated, its header comments and the MD document of its parent folder must be updated accordingly.
%
% File Input: [What external dependencies this file relies on]
% File Output: [What this file provides to external components]
% File Pos: [This file's position in the local system architecture]
```

**For Python files (.py):**
```python
# Please update this document if any changes occur to the folder it belongs to.
# Once this file is updated, its header comments and the MD document of its parent folder must be updated accordingly.
#
# File Input: [What external dependencies this file relies on]
# File Output: [What this file provides to external components]
# File Pos: [This file's position in the local system architecture]
```

#### Cross-Documentation Update Rule

**CRITICAL:** When you complete any work that affects:
- Functions and their interfaces
- System architecture or component relationships
- Code organization or file structure

You MUST update:
1. The affected file's header comments
2. The parent folder's README.md documentation
3. The main CLAUDE.md (if architectural changes occur)

This ensures documentation stays synchronized with code changes.

## License

This project is licensed under **Apache License 2.0**. All contributions must comply with this license.