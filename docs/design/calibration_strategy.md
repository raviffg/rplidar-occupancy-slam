# Calibration strategy

## Scope
This note records the design considerations for RPLIDAR 2D scan matching and occupancy-grid mapping.

## Current project context
The repository currently implements trimmed point-to-point ICP, encoder fusion, Bresenham raycasting, log-odds occupancy updates. The main code paths are `src/icp_scan_matcher.cpp` and `scripts/occupancy_grid.py`. The principal engineering concerns are scan timestamps, point decimation, outlier rejection, map bounds, odometry drift, reflective surfaces.

## Design questions
- What inputs, units, coordinate conventions, and preconditions does this part of the system require?
- Which state is authoritative, and which values are derived?
- How are invalid, stale, incomplete, or physically implausible inputs represented?
- Which assumptions are supported by source code or measured evidence, and which remain provisional?
- What observable evidence allows a reviewer to reproduce the behavior?

## Recommended design record
Document the selected approach, alternatives considered, rationale, expected failure modes, and any measurable trade-offs. Keep numerical constants linked to source or calibration evidence; do not copy unexplained values into a second source of truth.

## Verification
Pair this design note with the relevant scenarios under `docs/verification/scenarios/`. A design decision is not considered verified merely because the code compiles.

## Status
Working reference. Confirm implementation details against the source before using this note as a hardware, clinical, or production specification.
