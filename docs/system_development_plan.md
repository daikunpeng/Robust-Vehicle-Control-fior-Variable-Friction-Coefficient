# System Development Plan (TDD Approach)

**Document Status:** Active Development
**Last Updated:** 2025-12-23
**Please update this document if any changes occur to the folder it belongs to.**
**Once this file is updated, its header comments and the MD document of its parent folder must be updated accordingly.**

---

## 1. Project Overview

This document outlines the **Test-Driven Development (TDD)** approach for developing a Reinforcement Learning-based robust path following control system for vehicles under variable friction conditions.

### 1.1 Research Project Nature

This is an **exploratory research project** with the following characteristics:
- **Uncertain requirements** - Full system specification cannot be determined upfront
- **Experimental validation needed** - Hypotheses must be tested through simulation
- **Iterative refinement** - Design evolves based on empirical results
- **Knowledge discovery** - Learning occurs throughout the development process

### 1.2 Why Test-Driven Development?

TDD is particularly suitable for this research project because:

1. **Concrete Validation** - Each hypothesis is tested before implementation
2. **Small Iterations** - Breaks complex problems into manageable experiments
3. **Documentation through Tests** - Tests serve as executable specifications
4. **Confidence in Changes** - Regression tests ensure previous gains are maintained
5. **Empirical Progress** - Development advances through measurable improvements

---

## 2. TDD Methodology for Research

### 2.1 Core TDD Cycle

For this project, we follow an adapted TDD cycle:

```
┌─────────────────────────────────────────┐
│  1. Formulate Hypothesis/Question        │
│     "What are we trying to learn?"       │
└──────────────┬───────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│  2. Design Test                         │
│     "How do we validate this?"           │
│     - Define success criteria            │
│     - Design experiment                  │
│     - Specify metrics                    │
└──────────────┬───────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│  3. Write Test (Failing)                │
│     "Prove we don't know it yet"         │
│     - Implement test that fails          │
│     - Documents expected behavior        │
└──────────────┬───────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│  4. Implement Minimum Solution          │
│     "Make it work, simply"               │
│     - Minimal code to pass test          │
│     - Focus on core idea                 │
└──────────────┬───────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│  5. Run Experiment/Validate             │
│     "Did it work?"                       │
│     - Collect data                       │
│     - Analyze results                    │
└──────────────┬───────────────────────────┘
               │
               ▼
┌─────────────────────────────────────────┐
│  6. Reflect & Refine                    │
│     "What did we learn?"                 │
│     - Document findings                  │
│     - Decide next steps                  │
│     - Update knowledge base              │
└──────────────┬───────────────────────────┘
               │
               ▼
         (Back to step 1 or 7)

┌─────────────────────────────────────────┐
│  7. Integrate & Archive                 │
│     "Preserve what works"                │
│     - Add to regression suite            │
│     - Document lessons learned           │
└─────────────────────────────────────────┘
```

### 2.2 Test Categories

We use three categories of tests:

1. **Hypothesis Tests** - Validate research questions
   - "Does domain randomization improve robustness?"
   - "Can SAC outperform DDPG for this task?"

2. **Component Tests** - Validate individual modules
   - "Does the friction estimator converge?"
   - "Is the path tracking error within bounds?"

3. **Integration Tests** - Validate system behavior
   - "Can the vehicle complete the test track?"
   - "Does the controller handle friction changes?"

---

## 3. Development Phases

### Phase 1: Foundation (Weeks 1-2)
**Goal:** Establish baseline infrastructure and validation tools

**Tests to Write:**
- [ ] Test: Can we load TruckSim vehicle model in Simulink?
- [ ] Test: Can we run a simple simulation with predefined inputs?
- [ ] Test: Can we log vehicle states (position, velocity, acceleration)?
- [ ] Test: Can we vary friction coefficient parameter?
- [ ] Test: Can we implement a simple PID controller?
- [ ] Test: Can the PID controller follow a straight path at μ=0.8?

**Deliverables:**
- Working TruckSim + Simulink integration
- Data logging framework
- Baseline PID controller
- Test track definition

**Success Criteria:**
- All tests pass
- Baseline controller achieves < 2m RMS tracking error on dry asphalt

### Phase 2: Single Friction RL (Weeks 3-4)
**Goal:** Train RL agent for known friction conditions

**Tests to Write:**
- [ ] Test: Can we define RL environment in Simulink?
- [ ] Test: Can we create DDPG agent in MATLAB?
- [ ] Test: Can we train agent for 100 episodes without crashing?
- [ ] Test: Can trained agent outperform PID on dry asphalt (μ=0.8)?
- [ ] Test: Can trained agent follow path at constant velocity?

**Experiments:**
- Vary: RL algorithm (DDPG vs SAC)
- Vary: Network architecture
- Vary: Reward function design
- Vary: Training duration

**Success Criteria:**
- RL agent achieves < 1m RMS error on training friction
- Training converges within 5000 episodes

### Phase 3: Domain Randomization (Weeks 5-6)
**Goal:** Train robust agent using dynamics randomization

**Tests to Write:**
- [ ] Test: Does randomization during training improve test performance?
- [ ] Test: Can agent handle friction range [0.3, 0.9]?
- [ ] Test: Does performance degrade gracefully at unseen friction?
- [ ] Test: Can agent adapt to sudden friction changes?

**Experiments:**
- Vary: Randomization range (narrow vs wide)
- Vary: Randomization strategy (uniform vs Gaussian)
- Vary: Number of randomization parameters (friction only vs friction+mass+velocity)
- Compare: With/without domain randomization

**Success Criteria:**
- Randomized agent achieves < 1.5m RMS error across friction range [0.4, 0.8]
- Performance at μ=0.5 is within 50% of training friction performance

### Phase 4: Adaptive Friction Estimation (Weeks 7-8)
**Goal:** Incorporate friction estimation into control loop

**Tests to Write:**
- [ ] Test: Can we estimate friction from vehicle states?
- [ ] Test: Does estimator converge within 5 seconds?
- [ ] Test: Can RL agent use estimated friction as input?
- [ ] Test: Does adaptive control outperform fixed controller?

**Experiments:**
- Compare: Kalman filter vs neural network estimator
- Vary: Estimator update rate
- Test: Sudden friction changes (dry to wet)

**Success Criteria:**
- Estimation error < 0.1 within 3 seconds
- Adaptive controller achieves < 1.2m RMS on variable friction

### Phase 5: Validation & Comparison (Weeks 9-10)
**Goal:** Comprehensive evaluation against baselines

**Tests to Write:**
- [ ] Test: Does RL controller outperform PID on all friction levels?
- [ ] Test: Does RL controller outperform MPC (if implemented)?
- [ ] Test: Can controller handle complex paths (curves, S-bends)?
- [ ] Test: What is computational overhead of RL controller?

**Evaluation:**
- Test on: 5 different friction levels (0.2, 0.4, 0.6, 0.8, 1.0)
- Test on: 3 different path types
- Measure: Tracking error, smoothness, computation time
- Compare: Against PID and MPC baselines

**Success Criteria:**
- RL agent achieves best or tie performance across all scenarios
- Real-time computation < 10ms per control step

---

## 4. Test Organization

### 4.1 Directory Structure

```
tests/
├── hypothesis/          # Research hypothesis tests
│   ├── test_domain_randomization_effectiveness.m
│   ├── test_sac_vs_ddpg.m
│   └── test_friction_adaptation.m
├── components/          # Component unit tests
│   ├── test_friction_estimator.m
│   ├── test_reward_function.m
│   └── test_state_normalization.m
├── integration/         # System integration tests
│   ├── test_full_track_navigation.m
│   └── test_friction_transition.m
└── baselines/          # Baseline performance tests
    ├── test_pid_controller.m
    └── test_mpc_controller.m
```

### 4.2 Test Template

```matlab
% Please update this document if any changes occur to the folder it belongs to.
% Once this file is updated, its header comments and the MD document of its parent folder must be updated accordingly.
%
% File Input: Test framework, simulation environment, trained agents
% File Output: Test results, performance metrics, pass/fail status
% File Pos: tests/hypothesis/ - Validates research questions

function [results] = test_hypothesis_name()
% TEST_HYPOTHESIS_NAME Test if [hypothesis description]
%
%   Hypothesis: [Clear statement of what we're testing]
%
%   Success Criteria:
%   - [Specific, measurable criterion 1]
%   - [Specific, measurable criterion 2]
%
%   Method:
%   1. [Setup procedure]
%   2. [Test execution]
%   3. [Measurement approach]
%
%   Results:
%   - Struct with test metrics and pass/fail determination

    % Test implementation
    ...
end
```

---

## 5. Continuous Documentation

### 5.1 Experiment Log Template

For each test/experiment, maintain a log entry:

```markdown
## Experiment Log: [Test Name]

**Date:** YYYY-MM-DD
**Hypothesis:** [What we're testing]
**Test File:** `tests/category/test_name.m`

### Setup
- Simulation parameters: [config]
- Training configuration: [config]
- Test conditions: [friction, velocity, path]

### Results
- Metric 1: [value] (pass/fail)
- Metric 2: [value] (pass/fail)
- Observation: [what happened]

### Analysis
- Interpretation: [what results mean]
- Comparison: [vs baseline or previous]
- Surprise: [unexpected outcomes]

### Next Steps
- [ ] Action item 1
- [ ] Action item 2

### Lessons Learned
- [Key insight 1]
- [Key insight 2]
```

### 5.2 Knowledge Base

Maintain `docs/knowledge_base.md` with:
- Working parameter configurations
- Failed approaches (and why)
- Performance benchmarks
- Debugging tips
- Useful MATLAB/Simulink patterns

---

## 6. Progress Tracking

### 6.1 Metrics Dashboard

Track these metrics throughout development:

| Phase | Tests Written | Tests Passing | RMS Error (m) | Training Time | Notes |
|-------|--------------|---------------|---------------|---------------|-------|
| 1     | 6/6         | 6/6          | 1.8          | N/A          | Baseline established |
| 2     | 5/5         | 3/5          | TBD          | TBD          | In progress |
| 3     | 0/4         | 0/4          | TBD          | TBD          | Not started |

### 6.2 Decision Log

Record all major decisions in `docs/decision_log.md`:

```markdown
## Decision #[N]: [Decision Title]

**Date:** YYYY-MM-DD
**Context:** [Problem or question]
**Options:**
1. [Option A with pros/cons]
2. [Option B with pros/cons]

**Decision:** [Option chosen]
**Rationale:** [Why]
**Consequences:** [What this means for future work]

**Revisit Date:** [When to reconsider]
```

---

## 7. Risk Management

### 7.1 Identified Risks

| Risk | Probability | Impact | Mitigation |
|------|------------|--------|------------|
| Training instability | High | High | Start with simple environments, use SAC |
| Overfitting to training friction | High | Medium | Domain randomization |
| Computational cost | Medium | Medium | Start with shorter training, use GPU |
| TruckSim integration issues | Medium | High | Verify early in Phase 1 |

### 7.2 Exit Criteria

Project phases can only be considered complete when:
- All tests in phase pass consistently
- Results are documented and reproducible
- Code is committed and pushed
- Dependencies are documented
- Next phase is clearly defined

---

## 8. Tools and Infrastructure

### 8.1 Required Tools

- **MATLAB R2023b+** with Reinforcement Learning Toolbox
- **Simulink** with TruckSim S-function blocks
- **TruckSim** - Vehicle dynamics simulation
- **Git** - Version control
- **MATLAB Unit Test Framework** - Test execution

### 8.2 Test Automation

Create `tests/run_all_tests.m`:

```matlab
function results = run_all_tests()
% RUN_ALL_TESTS Execute full test suite
    results = struct();
    results.passed = 0;
    results.failed = 0;
    results.details = {};

    % Add test suite execution code
    ...
end
```

---

## 9. Next Actions (Immediate)

**This Week:**
1. [ ] Set up TruckSim integration in Simulink
2. [ ] Create first test: `test_load_trucksim_model.m`
3. [ ] Implement simple test track in Simulink
4. [ ] Log first simulation data
5. [ ] Document setup in experiment log

**Week 1-2 Goal:** All Phase 1 tests passing

---

## 10. References

- Ahmic et al. 2023 - "Reinforcement Learning-Based Path Following Control with Dynamics Randomization for Parametric Uncertainties"
- MATLAB RL Toolbox Documentation
- TruckSim User Manual

---

**Document Maintainer:** Development Team
**Review Cycle:** Weekly
**Version:** 1.0
