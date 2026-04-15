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
- online localization scaffolding with particle filter
- simulator-facing initialization script for map 1

The current simulator-ready entrypoint is:

- `src/initialization/runInitialLocalizationSimulator.m`

That script rotates the robot in place, collects a 360-degree RSDepth signature, matches it against waypoint and heading hypotheses, and prints the estimated pose in the MATLAB command window.

## Current Project Structure

```text
final_project/
├── src/
│   ├── initialization/
│   └── localization/
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

## Recommended Next Steps

1. Add the competition code entrypoint, for example `finalCompetition.m`
2. Add map loading and optional-wall handling
3. Seed PF particles from the initialization result
4. Add waypoint planning and control
5. Test the full pipeline on both practice maps

## Suggested Next Modules

```text
src/
├── control/
├── mapping/
├── planning/
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
