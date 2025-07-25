# rplidar-occupancy-slam

2D scan matching and occupancy mapping for an RPLIDAR A1 on a diff-drive base.
No ROS dependency; this is the standalone path used for offline tuning.

## Layout

- `src/icp_scan_matcher.cpp` -- point-to-point ICP, SVD rotation recovery,
  80% trimmed correspondences, hard 0.35 m gate. Odometry guess as the seed.
- `scripts/occupancy_grid.py` -- Bresenham raycasting into a log-odds grid,
  clamped at +/-4. Fuses encoder deltas with the ICP delta.
- `data/rplidar_scan_sample.csv` -- four sweeps, 8 deg decimation.

## Build / run

```
make
make run
make grid        # writes build/map.pgm
```

## Notes

ICP is brute force over decimated clouds (~400 pts). Fine at 5 Hz on the
Jetson; a kd-tree is not worth the dependency yet. Scan-to-map matching is not
implemented, so the map smears once odometry drift passes roughly 0.3 m.
Reflective surfaces and the vest rack produce the usual ghost arcs.


## Expanded project documentation

The `docs/` directory contains project-specific design notes, verification scenario specifications, operations guides, data contracts, configuration templates, safety notes, and project reference templates.

**Important:** added documents are planning and review aids. They do not claim that tests have been executed, that placeholder configurations are supported runtime profiles, or that this project is certified for physical/clinical/production deployment. Confirm interfaces and limits against implementation and measured evidence.
