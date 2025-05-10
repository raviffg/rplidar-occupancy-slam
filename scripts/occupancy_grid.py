"""Log-odds occupancy grid built from RPLIDAR A1 scans + wheel odometry.

We do not have a proper pose graph yet, so the pose is odometry integrated
forward and corrected by the ICP delta when the matcher converges. Good enough
for a single 20 x 15 m room; drift shows up as smeared walls past that.

Grid convention: row 0 is y = 0, resolution 0.05 m, values are log-odds
clamped to [-4, +4]. 0.5 is unknown.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable, List, Sequence, Tuple

import numpy as np

LOG_ODDS_FREE = -0.35
LOG_ODDS_OCC = 0.85
CLAMP_LO = -4.0
CLAMP_HI = 4.0


@dataclass
class OccupancyGrid:
    width: int
    height: int
    resolution: float = 0.05
    origin_x: float = 0.0
    origin_y: float = 0.0
    log_odds: np.ndarray = field(init=False)

    def __post_init__(self) -> None:
        self.log_odds = np.zeros((self.height, self.width), dtype=np.float32)

    def world_to_cell(self, x: float, y: float) -> Tuple[int, int]:
        cx = int((x - self.origin_x) / self.resolution)
        cy = int((y - self.origin_y) / self.resolution)
        return cx, cy

    def cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        return (
            self.origin_x + (cx + 0.5) * self.resolution,
            self.origin_y + (cy + 0.5) * self.resolution,
        )

    def in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height

    def update(self, cx: int, cy: int, delta: float) -> None:
        if not self.in_bounds(cx, cy):
            return
        v = self.log_odds[cy, cx] + delta
        self.log_odds[cy, cx] = min(CLAMP_HI, max(CLAMP_LO, v))

    def probability(self) -> np.ndarray:
        return 1.0 - 1.0 / (1.0 + np.exp(self.log_odds))


def bresenham(x0: int, y0: int, x1: int, y1: int) -> Iterable[Tuple[int, int]]:
    """Integer line, inclusive of both endpoints. The endpoint is the hit cell;
    the caller marks it occupied and everything before it free."""
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    x, y = x0, y0
    while True:
        yield x, y
        if x == x1 and y == y1:
            return
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x += sx
        if e2 < dx:
            err += dx
            y += sy


@dataclass
class Scan:
    """One RPLIDAR sweep already de-rotated into the body frame."""
    angles_deg: np.ndarray
    distances_mm: np.ndarray
    quality: np.ndarray


def integrate_scan(
    grid: OccupancyGrid,
    scan: Scan,
    pose: Tuple[float, float, float],
    max_range_m: float = 8.0,
    min_quality: int = 8,
) -> int:
    """Raycast one sweep from `pose` into the grid. Returns rays applied."""
    px, py, ptheta = pose
    sensor_x = px + 0.09 * math.cos(ptheta)  # A1 sits 9 cm forward of the axle
    sensor_y = py + 0.09 * math.sin(ptheta)

    ox, oy = grid.world_to_cell(sensor_x, sensor_y)
    applied = 0

    for ang_deg, dist_mm, qual in zip(scan.angles_deg, scan.distances_mm, scan.quality):
        if qual < min_quality:
            continue
        r = dist_mm / 1000.0
        # A1 returns 0 for "no return" and drops to 0.0 quality near the
        # minimum range; both are unusable.
        if r <= 0.05 or r > max_range_m:
            continue

        a = math.radians(ang_deg) + ptheta
        ex = sensor_x + r * math.cos(a)
        ey = sensor_y + r * math.sin(a)
        tx, ty = grid.world_to_cell(ex, ey)

        cells = list(bresenham(ox, oy, tx, ty))
        if len(cells) < 2:
            continue
        for cx, cy in cells[:-1]:
            grid.update(cx, cy, LOG_ODDS_FREE)
        grid.update(cells[-1][0], cells[-1][1], LOG_ODDS_OCC)
        applied += 1

    return applied


def fuse_odometry(
    prev_pose: Tuple[float, float, float],
    d_odom: Tuple[float, float, float],
    icp_delta: Tuple[float, float, float] | None,
    alpha: float = 0.65,
) -> Tuple[float, float, float]:
    """Blend odometry with the ICP correction.

    alpha weights ICP. Encoder odometry on this base under-reads yaw by roughly
    12% on carpet, so we trust ICP more when it converged. Callers pass None
    for icp_delta when the matcher bailed out.
    """
    px, py, pth = prev_pose
    if icp_delta is None:
        return (px + d_odom[0], py + d_odom[1], _wrap(pth + d_odom[2]))

    dx = alpha * icp_delta[0] + (1.0 - alpha) * d_odom[0]
    dy = alpha * icp_delta[1] + (1.0 - alpha) * d_odom[1]
    dth = alpha * icp_delta[2] + (1.0 - alpha) * d_odom[2]
    return (px + dx, py + dy, _wrap(pth + dth))


def _wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def load_csv(path: str) -> Tuple[List[float], List[Scan], List[Tuple[float, float, float]]]:
    """Expects timestamp_ms,angle_deg,distance_mm,quality,odom_x,odom_y,odom_theta.

    Rows are grouped by timestamp into sweeps.
    """
    import csv

    stamps: List[float] = []
    scans: List[Scan] = []
    poses: List[Tuple[float, float, float]] = []

    cur_ts = None
    ang: List[float] = []
    dst: List[float] = []
    qua: List[float] = []
    pose = (0.0, 0.0, 0.0)

    def flush() -> None:
        if cur_ts is None or not ang:
            return
        stamps.append(cur_ts)
        scans.append(Scan(np.array(ang), np.array(dst), np.array(qua)))
        poses.append(pose)

    with open(path, newline="", encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            ts = float(row["timestamp_ms"])
            if cur_ts is not None and ts != cur_ts:
                flush()
                ang, dst, qua = [], [], []
            cur_ts = ts
            ang.append(float(row["angle_deg"]))
            dst.append(float(row["distance_mm"]))
            qua.append(float(row["quality"]))
            pose = (float(row["odom_x"]), float(row["odom_y"]), float(row["odom_theta"]))
    flush()
    return stamps, scans, poses


def save_pgm(grid: OccupancyGrid, path: str) -> None:
    prob = grid.probability()
    # 0 = occupied (black), 254 = free, 205 = unknown
    img = np.full(prob.shape, 205, dtype=np.uint8)
    img[prob > 0.65] = 0
    img[prob < 0.35] = 254
    img = np.flipud(img)
    with open(path, "wb") as fh:
        fh.write(f"P5\n{grid.width} {grid.height}\n255\n".encode("ascii"))
        fh.write(img.tobytes())
