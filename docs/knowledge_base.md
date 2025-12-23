# Knowledge Base

**Document Status:** Active - Continuously updated with insights
**Last Updated:** 2025-12-23
**Please update this document if any changes occur to the folder it belongs to.**
**Once this file is updated, its header comments and the MD document if its parent folder must be updated accordingly.**

---

## Purpose

This knowledge base captures working configurations, lessons learned, debugging tips, and best practices discovered during development. It serves as a practical reference to avoid repeating mistakes and to accelerate future work.

---

## Table of Contents

1. [Working Configurations](#working-configurations)
2. [Failed Approaches (and why)](#failed-approaches-and-why)
3. [Performance Benchmarks](#performance-benchmarks)
4. [Debugging Tips](#debugging-tips)
5. [Useful Patterns](#useful-patterns)
6. [TruckSim Specifics](#trucksim-specifics)
7. [MATLAB RL Tips](#matlab-rl-tips)

---

## Working Configurations

### Simulation Parameters

#### Successful Vehicle Configuration
```matlab
% TruckSim vehicle parameters that work well
% Configuration ID: config_001
% Date: TBD
% Use case: Basic path following

vehicle_params = struct();
vehicle_params.mass = 5000;  % kg
vehicle_params.wheelbase = 4.5;  % m
vehicle_params.steering_ratio = 20:1;
```

#### Successful RL Training Configuration
```matlab
% RL training parameters that converged
% Configuration ID: rl_config_001
% Date: TBD
% Algorithm: SAC

agent_options = struct();
agent_options.learning_rate = 1e-4;
agent_options.batch_size = 64;
agent_options.gamma = 0.99;
agent_options.tau = 0.005;
```

*To be populated as we discover working configurations*

---

## Failed Approaches (and Why)

### Section Template

```markdown
#### [Approach Name]
**Attempted:** [Date]
**Phase:** [1-5]
**Idea:** [What we tried and why]

**What Didn't Work:**
- [Specific failure mode 1]
- [Specific failure mode 2]

**Root Cause Analysis:**
- [Why it failed technically]

**Lessons for Future:**
- [What this teaches us]
- [Alternative approaches to consider instead]
```

### Failed Approaches Log

*No failed approaches documented yet. This section will grow as we experiment.*

---

## Performance Benchmarks

### Baseline Performance

| Controller | Friction | Path | RMS Error (m) | Max Error (m) | Compute Time (ms) |
|------------|----------|------|---------------|---------------|-------------------|
| PID        | 0.8      | Straight | TBD | TBD | TBD |
| PID        | 0.4      | Straight | TBD | TBD | TBD |
| DDPG       | 0.8      | Straight | TBD | TBD | TBD |
| SAC        | 0.8      | Straight | TBD | TBD | TBD |

*To be populated as experiments are conducted*

### Best Known Configurations

| Task | Best Algorithm | Best Config | Performance |
|------|----------------|-------------|-------------|
| Single friction | TBD | TBD | TBD |
| Variable friction | TBD | TBD | TBD |
| Sudden friction change | TBD | TBD | TBD |

---

## Debugging Tips

### Common Issues

#### Issue 1: Training Diverges
**Symptoms:** Loss explodes, NaN values
**Possible Causes:**
- Learning rate too high
- Reward function not normalized
- State observations not scaled

**Solutions to Try:**
1. Reduce learning rate by factor of 10
2. Normalize rewards to [-1, 1]
3. Add observation normalization layer
4. Check for NaN in environment

#### Issue 2: Agent Not Learning
**Symptoms:** No improvement over episodes
**Possible Causes:**
- Reward function misaligned
- Exploration insufficient
- Episode too short
- Exploration noise too low

**Solutions to Try:**
1. Visualize reward distribution
2. Increase exploration noise
3. Extend episode length
4. Add reward shaping

#### Issue 3: TruckSim Integration Fails
**Symptoms:** S-function errors, crashes
**Possible Causes:**
- TruckSim not installed correctly
- Path issues
- License problems

**Solutions to Try:**
1. Verify TruckSim installation: `trucksim -v`
2. Check MATLAB path includes TruckSim bin directory
3. Test with simple TruckSim example first
4. Check license validity

### MATLAB Debugging Commands

```matlab
% Useful debugging commands

% Check for NaN in signals
any(isnan(signal(:)))

% Visualize training progress
plot(trainingInfo.EnvironmentExperience)

% Inspect agent
inspect(agent)

% Test environment step by step
[obs, reward, done, info] = step(env, action)

% Profile performance
profile on
% run code
profile viewer
```

---

## Useful Patterns

### Reward Function Pattern

```matlab
function reward = compute_reward(state, desired_state)
    % Pattern for computing balanced rewards

    % Tracking error component (primary)
    error = compute_tracking_error(state, desired_state);
    reward_tracking = -error;  % Negative because we minimize error

    % Smoothness component (secondary)
    jerk = compute_jerk(state);
    reward_smoothness = -0.1 * jerk;  % Weighted less

    % Control effort penalty
    control_effort = compute_control_effort(state);
    reward_effort = -0.05 * control_effort;

    % Combine
    reward = reward_tracking + reward_smoothness + reward_effort;
end
```

### State Normalization Pattern

```matlab
function norm_state = normalize_state(raw_state, state_bounds)
    % Normalize states to [-1, 1] for neural network

    norm_state = zeros(size(raw_state));

    for i = 1:length(raw_state)
        lower = state_bounds(i, 1);
        upper = state_bounds(i, 2);
        norm_state(i) = 2 * (raw_state(i) - lower) / (upper - lower) - 1;
    end
end
```

### Training Loop Pattern

```matlab
function train_agent_with_logging(agent, env, num_episodes)
    % Pattern for training with progress logging

    for episode = 1:num_episodes
        % Reset
        obs = reset(env);
        episode_reward = 0;

        % Episode loop
        while ~isdone(env)
            % Action selection with noise
            action = getActionWithNoise(agent, obs);

            % Step environment
            [next_obs, reward, done, info] = step(env, action);

            % Learning
            experience = [obs, action, reward, next_obs, done];
            agent = learn(agent, experience);

            % Logging
            episode_reward = episode_reward + reward;
            obs = next_obs;
        end

        % Episode logging
        fprintf('Episode %d: Reward = %.2f\n', episode, episode_reward);
        log_to_file(episode, episode_reward);
    end
end
```

---

## TruckSim Specifics

### Common Variables

| Variable | TruckSim Name | Description | Typical Range |
|----------|---------------|-------------|---------------|
| Friction | `MU` | Road-tire friction coefficient | 0.1-1.0 |
| Steering | `IMP_STEER_SW` | Steering wheel angle (deg) | -500 to 500 |
| Velocity | `VX` | Longitudinal velocity (m/s) | 0-40 |
| Lateral Accel | `AY` | Lateral acceleration (m/s²) | -10 to 10 |
| Yaw Rate | `YAW_RATE` | Yaw rate (deg/s) | -50 to 50 |

### S-Function Integration

```matlab
% Pattern for TruckSim S-function in Simulink
% Block: TruckSim S-Function

% Parameters to set:
% - Simfile: Path to .sim file
% - Parfile: Path to .par file
% - Runfile: Path to .run file
% - Outputs: Select variables to output
% - Inputs: IMP_STEER_SW, IMP_THROTTLE, IMP_BRAKE
```

---

## MATLAB RL Tips

### Agent Creation

```matlab
% DDPG Agent Creation Pattern
obsInfo = rlNumericSpec([N_obs 1]);
actInfo = rlNumericSpec([N_act 1], 'LowerLimit', -1, 'UpperLimit', 1);

agent = rlDDPGAgent(obsInfo, actInfo);
```

### Environment Creation

```matlab
% Simulink Environment Pattern
env = rlSimulinkEnv('model_name', 'agent_block', obsInfo, actInfo);

% Set simulation parameters
env.ResetFcn = @(in) resetFunction(in);
```

### Plotting Training Progress

```matlab
% After training
plot(trainingInfo)

% Or manually
figure;
plot(trainingInfo.EpisodeReward);
xlabel('Episode');
ylabel('Total Reward');
title('Training Progress');
```

---

## Quick Reference

### File Locations
- Models: `models/`
- Scripts: `matlab/`
- TruckSim configs: `trucksim/`
- Data: `data/`
- Results: `results/`
- Tests: `tests/`

### Git Commands
```bash
git status                          # Check status
git add -A                          # Stage all
git commit -m "message"             # Commit
git push                            # Push to remote
```

### MATLAB Commands
```matlab
addpath('matlab')                   # Add scripts to path
addpath('utils')                    # Add utilities
open_system('model_name')           # Open Simulink model
sim('model_name')                   # Run simulation
```

---

**Last Updated:** 2025-12-23
**Next Review:** After each experiment phase
**Contributors:** [Add names as team works on project]
