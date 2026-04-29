# AMR Final Project

This repository contains the current simulator code for the AMR final competition. The active top-level entrypoint is:

```matlab
result = finalCompetition(Robot, mapMatPath, opts)
```

Use `src/finalCompetition.m` from `SimulatorGUI -> Autonomous -> Start`.

## Current Scope

The current pipeline handles:

- Initial localization from a full-turn RSDepth wall signature.
- Beacon-assisted initialization scoring when tags are visible during the initial turn.
- Online particle-filter localization using RSDepth, odometry, and optional beacon tags.
- Global waypoint ordering over `waypoints + ECwaypoints`.
- A* segment planning on an inflated occupancy grid.
- Stay-away-point avoidance through grid occupancy inflation.
- Clearance-biased A* costs so paths prefer corridor centers instead of wall-hugging shortest paths.
- Local waypoint tracking with conservative speed limits, near-goal slowdown, short local lookahead, and snake-recovery replanning.

True pose is not used for policy, planning, target reach detection, path progress, or recovery decisions in `finalCompetition.m`.

## Main Entrypoint

`src/finalCompetition.m` is the competition entrypoint.

Default map resolution:

```matlab
3credits_practice/map1_3credits.mat
```

If a `.txt` map path is passed and a matching `.mat` file exists, `finalCompetition` resolves to the `.mat` file automatically.

Example:

```matlab
result = finalCompetition(Robot);
```

or:

```matlab
result = finalCompetition(Robot, ...
    '/Users/qiandaoliu/mae4180/final_project/3credits_practice/map1_3credits.mat');
```

## Initialization

Implemented in:

- `src/initialization/runInitialLocalizationSimulator.m`
- `src/initialization/scorePoseHypotheses.m`
- `src/initialization/depthPredict.m`
- `src/initialization/initializeFromWaypoints.m`

Current behavior:

- Robot rotates in place and records a full-turn RSDepth signature.
- Candidate poses are generated from map waypoints and heading hypotheses.
- Candidates are scored against the known-wall depth signature.
- If beacon tags are visible during rotation, beacon likelihood is added to reduce similar-room ambiguity.
- Output includes `bestPose`, `bestWaypointIdx`, and `bestHeadingIdx`.

## Online Localization

Implemented in:

- `src/localization/localizeStepPF.m`
- `src/localization/PF.m`
- `src/localization/integrateOdom.m`
- `src/localization/beaconMeasurementLikelihood.m`
- `src/localization/initParticlesFromPose.m`

Current behavior:

- PF runs continuously after initialization.
- RSDepth provides wall-based measurement updates.
- Beacon tags are fused when visible through `RealSenseTag`.
- `finalCompetition` uses odometry-scaled process noise instead of fixed per-frame process noise. This prevents low-speed motion from being dominated by artificial particle diffusion.
- Control uses a smoothed PF/control pose, not true pose.

Important current defaults:

```matlab
numParticles = 500
measurementNoise = 0.08 * eye(10)
beaconSigma = 0.04
beaconWeightFactor = 8.0
useScaledProcessNoise = true
processNoisePositionBase = 0.006
processNoisePositionPerMeter = 0.08
processNoiseThetaBase = 0.004
processNoiseThetaPerRad = 0.08
```

## Global Planning

Implemented in:

- `src/planning/precomputePairwisePathCosts.m`
- `src/planning/solveGlobalVisitOrder.m`
- `src/planning/buildOptimisticNavMap.m`
- `src/planning/replanIfBlockedByConfirmedWall.m`

Current behavior in `finalCompetition`:

- Candidate starts are `mapStruct.waypoints`.
- Goals are `[mapStruct.waypoints; mapStruct.ECwaypoints]`.
- After initialization, `bestWaypointIdx` selects the actual start node.
- If the selected start is already a goal within tolerance, it is skipped.
- Pairwise A* path costs are precomputed.
- `solveGlobalVisitOrder` uses Held-Karp dynamic programming to compute the shortest remaining visit order.

The current online loop follows the resulting global order one target at a time.

## Segment Planning

Implemented in:

- `src/planning/buildOccupancyGrid.m`
- `src/planning/astarGrid.m`
- `src/planning/planPathAStar.m`

Current behavior:

- Known walls are inflated by `robotInflation`.
- Stay-away points are inflated by `stayAwayInflation` and treated as occupied cells.
- A* uses an additional clearance traversal cost so paths prefer wider clearance and corridor centers.
- Path simplification is occupancy-safe and capped by `maxShortcutLength`, so shortcuts cannot cut through inflated walls or stay-away zones.

Important current defaults:

```matlab
planResolution = 0.10
robotInflation = 0.22
stayAwayInflation = 0.25
preferredClearance = robotInflation + 0.25
clearanceWeight = 2.0
maxShortcutLength = 0.25
nearGoalShortcutLength = 0.12
```

## Local Control

Current behavior:

- Robot tracks the current A* segment using feedback linearization.
- Commands are explicitly capped after feedback linearization.
- Near each target, speed, lookahead, and shortcut length are reduced.
- If the controller detects low progress with high angular command or oscillation, it triggers snake recovery by clearing the current path and replanning with short lookahead/shortcut settings.

Important current defaults:

```matlab
desiredSpeed = 0.12
maxForwardSpeed = 0.14
maxReverseSpeed = 0.04
maxWheelVelocity = 0.25
maxAngularSpeed = 0.6
lookaheadDistance = 0.25
nearGoalLookaheadDistance = 0.15
nearGoalSlowRadius = 0.80
nearGoalMinSpeed = 0.055
snakeAngularThreshold = 0.45
snakeProgressThreshold = 0.003
snakeTriggerCount = 18
snakeRecoverySteps = 20
```

## Debug Visualization

`finalCompetition` opens a visualization window by default.

Displayed elements:

- Black lines: known walls.
- Magenta squares: beacon locations.
- Orange crosses: stay-away points.
- Cyan circles: planned global visit targets.
- Red pentagram: current global target.
- Blue line: current A* path.
- Yellow dot: current local path target.
- Red line/dot: PF/control estimate history.
- Colored particles: PF particles before resampling.

True pose is intentionally not shown or read by `finalCompetition`.

## Legacy Test Entrypoints

These scripts remain useful for isolated testing but are not the active competition entrypoint:

- `src/initialization/runInitialLocalizationSimulator.m`
- `src/localization/runPFWaypointTestSimulator.m`
- `src/planning/runAStarPlannerTestSimulator.m`

## Current Known Issues

- Optional-wall belief integration exists in helper modules but is not yet fully wired into the current stable `finalCompetition` online loop.
- `global dataStore` is still used for simulator compatibility and produces a MATLAB `checkcode` warning.
- PF and path matching are sensitive to aggressive speed increases; current defaults prioritize stability over speed.
- Snake recovery is heuristic and should be tuned from simulator logs if it triggers too often or too late.

## Recommended Next Work

1. Wire optional-wall belief updates into the stable `finalCompetition` loop without changing the current PF/control stability assumptions.
2. Add explicit logging around optional-wall detection and replan decisions.
3. Tune `nearGoalSlowRadius`, `nearGoalLookaheadDistance`, and `snakeTriggerCount` from full-map runs.
4. Add a short simulator regression checklist for map 1 and map 2.
