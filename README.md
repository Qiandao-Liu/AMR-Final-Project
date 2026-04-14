# AMR Final Project

Autonomous Mobile Robots final competition workspace.

This repository is intended to hold the team code and lightweight project documentation for the final competition. The competition task combines:

- localization without truth pose
- online optional-wall inference
- waypoint and extra-credit waypoint navigation
- stay-away-point avoidance

## Repository Goals

- Keep only source code, lightweight notes, and reusable text assets under version control
- Keep large binary inputs, local planning notes, and private agent instructions out of the remote repository
- Provide a clean starting point for team collaboration

## Current Project Structure

```text
final_project/
├── 3credits_practice/
│   ├── map1_3credits.txt
│   └── map2_3credits.txt
├── README.md
├── .gitignore
└── AGENTS.md        # local only, ignored by git
```

Large local files such as competition PDFs, MATLAB `.mat` files, practice images, generated figures, and private notes are intentionally ignored.

## Recommended Next Steps

1. Add the competition code entrypoint, for example `finalCompetition.m`
2. Add helper modules for localization, mapping, planning, and control
3. Keep simulator/toolbox dependencies out of this repository
4. Test the full pipeline on the practice maps before integrating on the competition machine

## Suggested Module Layout

```text
src/
├── localization/
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
