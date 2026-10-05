from __future__ import annotations

import math
from typing import List, Tuple

from src.models import CarPose, Cone, Path2D

STEP = 0.25
TARGET_LEN = 6.0
MAX_LEN = 10.0
SIDE_OFFSET = 0.75


class PathPlanning:
    """Cone-gate path planner.

    Strategy: work in the car frame (x = forward along yaw, y = left), pair
    each blue (left) cone with its nearest yellow (right) cone, treat every
    pair's midpoint as a "gate" the path must pass through, then extend the
    path past the last gate. When only one side of the track is visible the
    path runs parallel to that boundary, offset to the side the car is on.
    """

    def __init__(self, car_pose: CarPose, cones: List[Cone]):
        self.car_pose = car_pose
        self.cones = cones

    def generatePath(self) -> Path2D:
        cx, cy, yaw = self.car_pose.x, self.car_pose.y, self.car_pose.yaw
        ca, sa = math.cos(yaw), math.sin(yaw)

        def to_car(c: Cone) -> Tuple[float, float]:
            dx, dy = c.x - cx, c.y - cy
            return dx * ca + dy * sa, -dx * sa + dy * ca

        def to_world(xf: float, yf: float) -> Tuple[float, float]:
            return cx + xf * ca - yf * sa, cy + xf * sa + yf * ca

        ahead = [(c, to_car(c)) for c in self.cones if to_car(c)[0] > -0.5]

        if not ahead:
            waypoints = [(0.0, 0.0), (TARGET_LEN, 0.0)]
        else:
            blues = [p for c, p in ahead if c.color == 1]
            yellows = [p for c, p in ahead if c.color == 0]
            waypoints = [(0.0, 0.0)] + self._gates(blues, yellows)

        base = self._length(waypoints)
        if base < TARGET_LEN:
            if len(waypoints) >= 2:
                dx = waypoints[-1][0] - waypoints[-2][0]
                dy = waypoints[-1][1] - waypoints[-2][1]
            else:
                dx, dy = 1.0, 0.0
            d = math.hypot(dx, dy) or 1.0
            ext = TARGET_LEN - base
            waypoints.append((waypoints[-1][0] + dx / d * ext,
                              waypoints[-1][1] + dy / d * ext))

        path = self._densify(waypoints)
        if self._length(path) > MAX_LEN:
            path = self._truncate(path, MAX_LEN)
        return [to_world(x, y) for x, y in path]

    def _gates(self, blues, yellows) -> List[Tuple[float, float]]:
        if blues and yellows:
            gates = []
            used_y = set()
            for b in sorted(blues, key=lambda p: p[0]):
                candidates = [y for i, y in enumerate(yellows)
                              if i not in used_y]
                if not candidates:
                    break
                nearest = min(candidates,
                              key=lambda y: math.hypot(b[0] - y[0],
                                                       b[1] - y[1]))
                used_y.add(yellows.index(nearest))
                gates.append(((b[0] + nearest[0]) / 2.0,
                              (b[1] + nearest[1]) / 2.0))
            return sorted(gates, key=lambda g: g[0])
        if blues or yellows:
            return self._side_only_gates(blues or yellows)
        return []

    def _side_only_gates(self, side) -> List[Tuple[float, float]]:
        if len(side) == 1:
            ref = side[0]
            lateral = -SIDE_OFFSET if ref[1] > 0 else SIDE_OFFSET
            return [(TARGET_LEN, lateral)]
        (x0, y0), (x1, y1) = self._furthest_pair(side)
        ux, uy = x1 - x0, y1 - y0
        d = math.hypot(ux, uy) or 1.0
        ux, uy = ux / d, uy / d
        t = -x0 * ux - y0 * uy
        px, py = x0 + t * ux, y0 + t * uy
        nx, ny = -uy, ux
        if nx * px + ny * py > 0.0:
            nx, ny = -nx, -ny
        ox, oy = px + SIDE_OFFSET * nx, py + SIDE_OFFSET * ny
        dirx, diry = (ux, uy) if ux >= 0.0 else (-ux, -uy)
        ext = max(0.0, TARGET_LEN - math.hypot(ox, oy))
        return [(ox, oy), (ox + dirx * ext, oy + diry * ext)]

    @staticmethod
    def _furthest_pair(pts):
        best, best_d = None, -1.0
        for i, a in enumerate(pts):
            for b in pts[i + 1:]:
                d = math.hypot(b[0] - a[0], b[1] - a[1])
                if d > best_d:
                    best, best_d = (a, b), d
        return best

    @staticmethod
    def _length(pts) -> float:
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(pts, pts[1:]))

    @staticmethod
    def _densify(wps) -> List[Tuple[float, float]]:
        out = [wps[0]]
        for a, b in zip(wps, wps[1:]):
            d = math.hypot(b[0] - a[0], b[1] - a[1])
            n = max(1, int(math.ceil(d / STEP)))
            for i in range(1, n + 1):
                t = i / n
                out.append((a[0] + (b[0] - a[0]) * t,
                            a[1] + (b[1] - a[1]) * t))
        return out

    @staticmethod
    def _truncate(pts, max_len) -> List[Tuple[float, float]]:
        kept = [pts[0]]
        acc = 0.0
        for a, b in zip(pts, pts[1:]):
            seg = math.hypot(b[0] - a[0], b[1] - a[1])
            if acc + seg >= max_len:
                t = (max_len - acc) / seg
                kept.append((a[0] + (b[0] - a[0]) * t,
                             a[1] + (b[1] - a[1]) * t))
                break
            kept.append(b)
            acc += seg
        return kept
