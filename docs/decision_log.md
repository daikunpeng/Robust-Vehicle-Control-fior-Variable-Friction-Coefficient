# Decision Log

**Document Status:** Active - Updated with major project decisions
**Last Updated:** 2025-12-23
**Please update this document if any changes occur to the folder it belongs to.**
**Once this file is updated, its header comments and the MD document of its parent folder must be updated accordingly.**

---

## Purpose

This document maintains a record of significant decisions made during the project. It captures the context, options considered, final decision, rationale, and consequences. This provides transparency and allows future reconsideration of decisions.

---

## Decision Template

```markdown
## Decision #[N]: [Decision Title]

**Date:** YYYY-MM-DD
**Category:** [Technical/Process/Tool/Priority]
**Status:** [Active/Superseded/Deprecated]
**Made By:** [Name]

### Context
[What problem or question led to this decision?]

### Options Considered
1. **[Option A]**
   - Pros: [advantages]
   - Cons: [disadvantages]
   - Effort: [implementation difficulty]

2. **[Option B]**
   - Pros: [advantages]
   - Cons: [disadvantages]
   - Effort: [implementation difficulty]

3. **[Option C - if applicable]**
   - Pros: [advantages]
   - Cons: [disadvantages]
   - Effort: [implementation difficulty]

### Decision
**Chosen:** [Option A/B/C]

### Rationale
- [Primary reason 1]
- [Primary reason 2]
- [Why this option best fits project goals]

### Consequences
**Positive:**
- [Expected benefit 1]
- [Expected benefit 2]

**Negative:**
- [Expected downside or cost]
- [Limitation introduced]

**Dependencies:**
- [What this decision enables or requires]

### Implementation Notes
- [How the decision was implemented]
- [Files modified]
- [Configuration changes]

### Re-evaluation Criteria
Review this decision if:
- [Condition 1]
- [Condition 2]

**Revisit Date:** [Specific date or "Never" / "As needed"]

---
```

---

## Decision Log

### Decision #1: Adopt Test-Driven Development Approach

**Date:** 2025-12-23
**Category:** Process
**Status:** Active
**Made By:** Project Team

#### Context
This is a research project with uncertain requirements and many unknowns. Traditional waterfall or upfront design approaches are not suitable because:
- Full system specification cannot be determined in advance
- Many parameters and algorithms need empirical validation
- The optimal solution path is unknown

#### Options Considered

1. **Traditional Design-First Approach**
   - Pros: Clear roadmap, predictable timeline
   - Cons: Risk of going down wrong path, inflexible to new discoveries
   - Effort: High upfront design effort

2. **Exploratory/Ad-hoc Approach**
   - Pros: Maximum flexibility, low initial structure
   - Cons: Hard to track progress, easy to lose direction, minimal documentation
   - Effort: Low initial effort

3. **Test-Driven Development (TDD)**
   - Pros: Empirical validation, small iterations, executable documentation, confidence in changes
   - Cons: Requires discipline, slower initial progress
   - Effort: Moderate sustained effort

#### Decision
**Chosen:** Option 3 - Test-Driven Development

#### Rationale
- TDD aligns perfectly with research methodology: hypothesis → test → validate → learn
- Each experiment builds on previous validated knowledge
- Tests serve as both validation and documentation
- Prevents regression as complexity increases
- Provides concrete progress metrics
- Forces thinking about success criteria before implementation

#### Consequences
**Positive:**
- Solid foundation of validated components
- Clear demonstration of what works
- Reproducible experiments
- Easy to demonstrate progress

**Negative:**
- Slower initial progress (writing tests before implementation)
- Requires maintaining test suite
- Needs discipline to follow cycle

**Dependencies:**
- Requires MATLAB Unit Test Framework
- Requires clear test templates and documentation
- Requires commitment to testing culture

#### Implementation Notes
- Created `docs/system_development_plan.md` with TDD methodology
- Created test directory structure: `tests/hypothesis/`, `tests/components/`, `tests/integration/`
- Created test templates for MATLAB files
- Established 5-phase development plan with test criteria

#### Re-evaluation Criteria
Review this decision if:
- Testing overhead becomes too great (>50% of development time)
- Experiments show TDD is slowing progress significantly
- Team finds alternative approach more effective

**Revisit Date:** After Phase 2 (approximately 4 weeks)

---

### Decision #2: MATLAB as Primary Development Environment

**Date:** 2025-12-23
**Category:** Tool/Technical
**Status:** Active
**Made By:** Project Team

#### Context
Project requires integration with TruckSim (vehicle simulation) and Simulink (control system design). Need to choose primary development environment for RL implementation and control algorithms.

#### Options Considered

1. **Python (PyTorch/TensorFlow) + MATLAB Co-simulation**
   - Pros: Rich ML ecosystem, more RL algorithms available, larger community
   - Cons: Complex integration with Simulink/TruckSim, co-simulation overhead, two languages
   - Effort: High integration effort

2. **MATLAB Reinforcement Learning Toolbox**
   - Pros: Native Simulink integration, seamless TruckSim interface, single language
   - Cons: Smaller ecosystem, fewer cutting-edge algorithms, licensing cost
   - Effort: Low to moderate

3. **C++ + Custom RL Framework**
   - Pros: High performance, deployment-ready
   - Cons: Extremely high development effort, no TruckSim integration, reinventing wheel
   - Effort: Very high

#### Decision
**Chosen:** Option 2 - MATLAB RL Toolbox

#### Rationale
- Native Simulink integration eliminates co-simulation complexity
- Direct access to TruckSim S-functions without external interfaces
- RL Toolbox has all necessary algorithms (DDPG, SAC, PPO, TD3)
- Control-focused toolset aligns with project goals
- Single language reduces complexity and debugging
- Industry standard for vehicle control applications

#### Consequences
**Positive:**
- Simpler architecture (all in MATLAB/Simulink)
- Faster development due to integrated tools
- Better debugging capabilities
- Industry-recognized approach

**Negative:**
- License cost for MATLAB and RL Toolbox
- Less flexibility for custom algorithms (but still possible)
- Smaller open-source community compared to Python

**Dependencies:**
- Requires valid MATLAB license with RL Toolbox
- Requires TruckSim installation with Simulink support
- Team needs MATLAB proficiency

#### Implementation Notes
- Set up `matlab/` directory for all MATLAB code
- Use `models/` for Simulink models
- Use `trucksim/` for TruckSim configuration files
- Python optional for data analysis only (`python/` directory)
- Documentation in CLAUDE.md reflects MATLAB-first approach

#### Re-evaluation Criteria
Review this decision if:
- MATLAB RL Toolbox lacks critical algorithm needed
- Performance issues arise that Python could solve
- Budget constraints make MATLAB license prohibitive

**Revisit Date:** After Phase 2 (if major algorithmic limitations encountered)

---

### Decision #3: Five-Phase Development Plan

**Date:** 2025-12-23
**Category:** Process
**Status:** Active
**Made By:** Project Team

#### Context
Need structured approach to develop robust RL controller with domain randomization. Many possible approaches and orderings of work.

#### Options Considered

1. **Build Complete System First**
   - Pros: Big picture visible early
   - Cons: High risk, hard to debug, late validation
   - Effort: High upfront, high risk of failure

2. **Incremental Feature Addition**
   - Pros: Steady progress
   - Cons: May not address key uncertainties early, potential for rework
   - Effort: Medium

3. **Five-Phase Validation-Gated Approach**
   - Pros: Each phase validates critical questions, builds incrementally, clear exit criteria
   - Cons: May seem slower initially, requires discipline
   - Effort: Medium with clear milestones

#### Decision
**Chosen:** Option 3 - Five-Phase Plan (as detailed in system_development_plan.md)

#### Rationale
- Each phase answers a critical research question
- Builds from simple to complex
- Validates foundations before building on them
- Clear "definition of done" for each phase
- Natural checkpoints for re-evaluation
- Aligns with TDD methodology

#### Consequences
**Positive:**
- Reduced risk (validate early)
- Clear progress tracking
- Easy to adjust plan based on learnings
- Natural stopping points if needed

**Negative:**
- May take longer to see "complete" system
- Requires following discipline (not jumping ahead)
- Might need to revisit phases if new questions emerge

**Dependencies:**
- Requires commitment to exit criteria
- Requires updating plan as learnings emerge
- Depends on successful completion of previous phases

#### Implementation Notes
Phases:
1. Foundation (TruckSim integration, PID baseline)
2. Single Friction RL (RL agent on known friction)
3. Domain Randomization (robustness to friction variation)
4. Adaptive Friction Estimation (online adaptation)
5. Validation & Comparison (comprehensive evaluation)

Each phase has specific tests and success criteria defined in system_development_plan.md.

#### Re-evaluation Criteria
Review this decision if:
- A phase takes 2x longer than estimated
- Results suggest different ordering would be better
- New research suggests alternative approach

**Revisit Date:** After each phase completion

---

### Decision #4: Documentation Standards

**Date:** 2025-12-23
**Category:** Process
**Status:** Active
**Made By:** Project Team

#### Context
Research project can easily become disorganized with many experiments, configurations, and learnings. Need systematic way to capture and organize knowledge.

#### Options Considered

1. **Minimal Documentation**
   - Pros: Less overhead, more time for experiments
   - Cons: Lost knowledge, hard to reproduce, difficult to demonstrate progress
   - Effort: Low

2. **Standard Academic Documentation**
   - Pros: Familiar format, good for papers
   - Cons: Static, doesn't capture process, hard to maintain
   - Effort: Medium

3. **Comprehensive Living Documentation**
   - Pros: Captures process and outcomes, reproducible, supports TDD, searchable knowledge base
   - Cons: Higher maintenance overhead, requires discipline
   - Effort: Medium to High

#### Decision
**Chosen:** Option 3 - Comprehensive Living Documentation

#### Rationale
- Research projects generate valuable experimental data that must be preserved
- TDD requires documentation (tests, logs, results)
- Future team members need context
- Essential for writing papers later
- Forces clarity in thinking

#### Consequences
**Positive:**
- Complete record of all experiments
- Easy to reproduce results
- Knowledge preserved for papers
- Easy to onboard new people
- Supports data-driven decisions

**Negative:**
- Time overhead for documentation
- Must keep docs in sync with code
- Can become outdated if not maintained

**Dependencies:**
- Team commitment to documentation
- Templates and standards (created)
- Regular review and updates

#### Implementation Notes
Created documentation structure:
- `docs/system_development_plan.md` - Overall plan and methodology
- `docs/experiment_logs.md` - Record of all experiments
- `docs/knowledge_base.md` - Working configs and lessons learned
- `docs/decision_log.md` - This file
- `docs/README.md` - Documentation index
- Folder README.md files for architecture
- File header comments for I/O documentation

Update rule: When code changes, update its header and parent folder README.

#### Re-evaluation Criteria
Review this decision if:
- Documentation takes >30% of development time
- Team finds it burdensome rather than helpful
- Simpler approach proves sufficient

**Revisit Date:** After Phase 2 (to assess overhead)

---

## Decision Summary

| # | Decision | Category | Date | Status |
|---|----------|----------|------|--------|
| 1 | Adopt TDD Approach | Process | 2025-12-23 | Active |
| 2 | MATLAB as Primary Environment | Tool | 2025-12-23 | Active |
| 3 | Five-Phase Development Plan | Process | 2025-12-23 | Active |
| 4 | Comprehensive Documentation Standards | Process | 2025-12-23 | Active |

---

**Last Updated:** 2025-12-23
**Next Review:** After Phase 2 completion
**Maintained By:** Development Team
