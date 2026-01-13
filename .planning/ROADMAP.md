# Roadmap: Search and Rescue Robot

## Overview

Build a complete autonomous search and rescue system on a ROS2/Transbot platform. Starting with bug fixes and ROS2 conventions alignment, we progressively add human tracking, arm control, and mission orchestration until the robot can navigate, detect, pick up, and return humans to a safe zone — first in Gazebo simulation, then on real hardware.

## Domain Expertise

None

## Phases

**Phase Numbering:**
- Integer phases (1, 2, 3): Planned milestone work
- Decimal phases (2.1, 2.2): Urgent insertions (marked with INSERTED)

- [x] **Phase 1: Foundation Fixes** - Fix existing bugs in action server
- [ ] **Phase 2: ROS2 Conventions Audit** - Align codebase with ROS2 best practices
- [ ] **Phase 3: Human Tagging System** - Location-based identity tracking
- [ ] **Phase 4: Safe Zone Configuration** - Parameterize safe zone radius
- [ ] **Phase 5: Arm Grasping Action** - ROS2 action for arm control
- [ ] **Phase 6: Rescue Priority Logic** - Nearest-first human selection
- [ ] **Phase 7: SearchAndRescue Integration** - Wire components into action server
- [ ] **Phase 8: Failure Handling** - Graceful error recovery
- [ ] **Phase 9: Gazebo Simulation** - Development and testing environment
- [ ] **Phase 10: CLI Interface** - Mission trigger script
- [ ] **Phase 11: End-to-End Testing** - Complete rescue loop validation

## Phase Details

### Phase 1: Foundation Fixes
**Goal**: Fix existing bugs identified in the codebase (dict vs set usage, enum comparison issues)
**Depends on**: Nothing (first phase)
**Research**: Unlikely (internal bug fixes)
**Plans**: TBD

Plans:
- [x] 01-01: Fix identified bugs in action server

### Phase 2: ROS2 Conventions Audit
**Goal**: Review and refactor existing code to follow ROS2 style guide, proper node lifecycle management, naming conventions, and message/service design patterns
**Depends on**: Phase 1
**Research**: Likely (ROS2 style guide, design patterns)
**Research topics**: ROS2 style guide, node lifecycle best practices, proper naming conventions, action/service design patterns
**Plans**: TBD

Plans:
- [ ] 02-01: Audit and document current deviations
- [ ] 02-02: Refactor to ROS2 conventions

### Phase 3: Human Tagging System
**Goal**: Implement location-based identity tracking for detected humans (assign persistent IDs based on position)
**Depends on**: Phase 2
**Research**: Unlikely (internal logic using existing TF/detection infrastructure)
**Plans**: TBD

Plans:
- [ ] 03-01: Implement human tagging logic

### Phase 4: Safe Zone Configuration
**Goal**: Make the hardcoded 1.0m safe zone radius configurable via ROS2 parameters
**Depends on**: Phase 2
**Research**: Unlikely (ROS2 parameter patterns established)
**Plans**: TBD

Plans:
- [ ] 04-01: Parameterize safe zone

### Phase 5: Arm Grasping Action
**Goal**: Create ROS2 action for arm control with approach, grasp, lift, carry, and release poses
**Depends on**: Phase 2
**Research**: Likely (servo control integration, action design)
**Research topics**: Transbot_Lib servo API, ROS2 action server design, arm pose sequences, servo timing/coordination
**Plans**: TBD

Plans:
- [ ] 05-01: Design arm action interface
- [ ] 05-02: Implement arm grasping action server

### Phase 6: Rescue Priority Logic
**Goal**: Implement nearest-first human selection algorithm for rescue prioritization
**Depends on**: Phase 3
**Research**: Unlikely (algorithm design, internal patterns)
**Plans**: TBD

Plans:
- [ ] 06-01: Implement priority logic

### Phase 7: SearchAndRescue Integration
**Goal**: Wire human tagging, arm grasping, and priority logic into the existing SearchAndRescue action server
**Depends on**: Phases 4, 5, 6
**Research**: Unlikely (wiring existing components)
**Plans**: TBD

Plans:
- [ ] 07-01: Integrate components into action server

### Phase 8: Failure Handling
**Goal**: Add graceful error recovery — skip and log on grasp failure, handle navigation failures
**Depends on**: Phase 7
**Research**: Unlikely (error handling patterns)
**Plans**: TBD

Plans:
- [ ] 08-01: Implement failure handling

### Phase 9: Gazebo Simulation
**Goal**: Create Gazebo simulation environment with Transbot model and test world
**Depends on**: Phase 2 (can run in parallel with 3-8)
**Research**: Likely (Gazebo Humble setup, robot modeling)
**Research topics**: Gazebo Humble integration, URDF/SDF for Transbot platform, world file creation, sensor simulation
**Plans**: TBD

Plans:
- [ ] 09-01: Create robot model for Gazebo
- [ ] 09-02: Build simulation world

### Phase 10: CLI Interface
**Goal**: Create command-line script for triggering rescue missions
**Depends on**: Phase 7
**Research**: Unlikely (ROS2 action client patterns)
**Plans**: TBD

Plans:
- [ ] 10-01: Implement CLI script

### Phase 11: End-to-End Testing
**Goal**: Validate complete rescue loop — navigate, detect, tag, pick up, return, drop off
**Depends on**: Phases 8, 9, 10
**Research**: Unlikely (testing existing functionality)
**Plans**: TBD

Plans:
- [ ] 11-01: End-to-end validation

## Progress

**Execution Order:**
Phases execute in numeric order: 1 → 2 → 3 → 4 → 5 → 6 → 7 → 8 → 9 → 10 → 11

Note: Phase 9 (Gazebo) can proceed in parallel with Phases 3-8 after Phase 2 completes.

| Phase | Plans Complete | Status | Completed |
|-------|----------------|--------|-----------|
| 1. Foundation Fixes | 1/1 | Complete | 2026-01-13 |
| 2. ROS2 Conventions Audit | 0/2 | Not started | - |
| 3. Human Tagging System | 0/1 | Not started | - |
| 4. Safe Zone Configuration | 0/1 | Not started | - |
| 5. Arm Grasping Action | 0/2 | Not started | - |
| 6. Rescue Priority Logic | 0/1 | Not started | - |
| 7. SearchAndRescue Integration | 0/1 | Not started | - |
| 8. Failure Handling | 0/1 | Not started | - |
| 9. Gazebo Simulation | 0/2 | Not started | - |
| 10. CLI Interface | 0/1 | Not started | - |
| 11. End-to-End Testing | 0/1 | Not started | - |
