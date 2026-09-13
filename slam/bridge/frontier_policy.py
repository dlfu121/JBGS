#!/usr/bin/env python3
"""Frontier validity and short-lived visit suppression helpers."""
import math

import numpy as np
from scipy import ndimage


def build_frontier_mask(data):
    """Return free cells adjacent to unknown cells."""
    free = (data >= 0) & (data < 50)
    unknown = (data == -1)
    connectivity = ndimage.generate_binary_structure(2, 2)
    return free & ndimage.binary_dilation(unknown, connectivity)


def goal_has_frontier(data, resolution, origin, goal, radius):
    """Whether a frontier cell remains within ``radius`` of a map-frame goal."""
    mask = build_frontier_mask(data)
    h, w = mask.shape
    ox, oy = origin
    gx = int(math.floor((goal[0] - ox) / resolution))
    gy = int(math.floor((goal[1] - oy) / resolution))
    cells = int(math.ceil(radius / resolution))
    x0, x1 = max(0, gx - cells), min(w, gx + cells + 1)
    y0, y1 = max(0, gy - cells), min(h, gy + cells + 1)
    if x0 >= x1 or y0 >= y1:
        return False

    ys, xs = np.nonzero(mask[y0:y1, x0:x1])
    if len(xs) == 0:
        return False
    wx = ox + (xs + x0 + 0.5) * resolution
    wy = oy + (ys + y0 + 0.5) * resolution
    return bool(np.any((wx - goal[0]) ** 2 + (wy - goal[1]) ** 2
                       <= radius ** 2))


def prune_visits(visits, now, ttl):
    """Drop expired ``(x, y, timestamp)`` visit records."""
    return [(x, y, stamp) for x, y, stamp in visits
            if now - stamp < ttl]


def filter_visited_frontiers(frontiers, visits, radius):
    """Suppress frontier centroids near a recently visited location."""
    return [frontier for frontier in frontiers
            if not any(math.hypot(frontier[0] - x, frontier[1] - y) < radius
                       for x, y, _ in visits)]
