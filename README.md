# AMR Final Project

Autonomous Mobile Robots final competition workspace.

This repository holds the team code and lightweight project documentation for the AMR final competition. The competition task combines:

- localization without truth pose
- online optional-wall inference
- waypoint and extra-credit waypoint navigation
- stay-away-point avoidance

## Repository Goals

- Keep only source code, lightweight notes, and reusable text assets under version control
- Keep large binary inputs, local planning notes, and private agent instructions out of the remote repository
- Provide a clean starting point for team collaboration

## Current Status

Current implemented pieces:

- initialization by waypoint-depth matching
- online localization with particle filter
- optional beacon-constrained PF update
- simulator-facing initialization script for map 1
- A* planning and replanning on a binary occupancy grid
- recovery logic for bump / invalid pose / no-path conditions

The current simulator-ready entrypoint is:

- `src/initialization/runInitialLocalizationSimulator.m`
- `src/localization/runPFWaypointTestSimulator.m`
- `src/planning/runAStarPlannerTestSimulator.m`

That script rotates the robot in place, collects a 360-degree RSDepth signature, matches it against waypoint and heading hypotheses, and prints the estimated pose in the MATLAB command window.

## Current Project Structure

```text
final_project/
├── src/
│   ├── initialization/
│   ├── localization/
│   └── planning/
├── 3credits_practice/
│   ├── map1_3credits.txt
│   └── map2_3credits.txt
├── README.md
├── .gitignore
└── AGENTS.md        # local only, ignored by git
```

Large local files such as competition PDFs, MATLAB `.mat` files, practice images, generated figures, and private notes are intentionally ignored.

## Initialization Modules

Implemented initialization helpers:

- `src/initialization/depthPredict.m`
- `src/initialization/scorePoseHypotheses.m`
- `src/initialization/initializeFromWaypoints.m`
- `src/initialization/runInitialLocalizationSimulator.m`

Implemented localization helpers:

- `src/localization/integrateOdom.m`
- `src/localization/PF.m`
- `src/localization/localizeStepPF.m`
- `src/localization/beaconMeasurementLikelihood.m`
- `src/localization/initParticlesFromPose.m`
- `src/localization/runPFWaypointTestSimulator.m`

Implemented planning helpers:

- `src/planning/buildOccupancyGrid.m`
- `src/planning/astarGrid.m`
- `src/planning/planPathAStar.m`
- `src/planning/runAStarPlannerTestSimulator.m`

## Running the Simulator Initialization Script

1. Open `SimulatorGUI`
2. Load a practice map in the simulator
3. Set the robot position on a waypoint
4. Click `Autonomous -> Start`
5. Select `src/initialization/runInitialLocalizationSimulator.m`

The current default map file used by the script is:

- `3credits_practice/map1_3credits.mat`

The script prints:

- estimated `x`
- estimated `y`
- estimated `theta`
- best waypoint index
- best heading index

## Running the PF + Waypoint Tracking Test

1. Open `SimulatorGUI`
2. Load a practice map in the simulator
3. Set the robot position near the hardcoded start pose
4. Click `Autonomous -> Start`
5. Select `src/localization/runPFWaypointTestSimulator.m`

This test uses:

- hardcoded start pose seeding for PF
- RSDepth updates
- optional beacon fusion when tags are visible
- two hardcoded target waypoints

## Running the A* Closed-Loop Test

1. Open `SimulatorGUI`
2. Load a practice map in the simulator
3. Set the robot position near the hardcoded start pose
4. Click `Autonomous -> Start`
5. Select `src/planning/runAStarPlannerTestSimulator.m`

This test uses:

- PF localization
- beacon fusion
- A* replanning on an inflated occupancy grid
- path tracking using feedback linearization
- recovery triggered by bump, invalid PF pose, or A* no-path events

Current default PF/planning parameters in the A* test are:

- `numParticles = 500`
- `processNoise = diag([0.004, 0.004, 0.003])`
- `measurementNoise = 0.08 * eye(10)`
- `beaconSigma = 0.05`
- `beaconWeightFactor = 6.0`

## Recommended Next Steps

1. Add the competition code entrypoint, for example `finalCompetition.m`
2. Connect initialization output directly to planning tests instead of hardcoded start poses
3. Add optional-wall belief updates
4. Integrate visited-waypoint bookkeeping and LED signaling
5. Test the full pipeline on both practice maps

## Suggested Next Modules

```text
src/
├── mapping/
├── planning/
├── control/
└── utils/
```

## Git Notes

This folder is configured to work as its own git repository. If needed:

```bash
cd final_project
git init
git branch -M main
git remote add origin https://github.com/Qiandao-Liu/AMR-Final-Project.git
```

Then review tracked files before the first push:

```bash
git status
git add .
git status
```
