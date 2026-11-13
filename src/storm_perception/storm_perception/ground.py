"""Bounded RANSAC ground fitting. Missing ground evidence is an explicit failure."""
import math

import numpy as np


def segment(points, ground_tolerance=.08, obstacle_height=.12, max_slope=25., radius=10.):
    points = np.asarray(points, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("Expected Nx3 points")
    finite = points[np.isfinite(points).all(axis=1)]
    distance = np.linalg.norm(finite[:, :2], axis=1)
    points = finite[(distance >= .65) & (distance <= radius)]
    candidates = points[(points[:, 2] > -.8) & (points[:, 2] < .5)]
    if len(candidates) < 30:
        raise ValueError("Insufficient local ground observations")
    candidates = candidates[::max(1, len(candidates) // 1500)]
    rng = np.random.default_rng(42)
    best = None
    best_count = 0
    cosine = math.cos(math.radians(max_slope))
    for _ in range(64):
        a, b, c = candidates[rng.choice(len(candidates), 3, replace=False)]
        normal = np.cross(b - a, c - a)
        length = np.linalg.norm(normal)
        if length < 1e-6:
            continue
        normal /= length
        if normal[2] < 0:
            normal *= -1
        offset = -np.dot(normal, a)
        if normal[2] < cosine or abs(offset / normal[2]) > .5:
            continue
        inliers = np.abs(candidates @ normal + offset) < ground_tolerance
        count = int(inliers.sum())
        if count > best_count:
            best_count, best = count, inliers
    if best_count < max(30, len(candidates) * .25):
        raise ValueError("No supported ground plane within slope/height limits")
    inliers = candidates[best]
    if np.linalg.norm(np.ptp(inliers[:, :2], axis=0)) < 1.:
        raise ValueError("Ground observations have insufficient spatial extent")
    center = inliers.mean(axis=0)
    _, _, vectors = np.linalg.svd(inliers - center, full_matrices=False)
    normal = vectors[-1]
    if normal[2] < 0:
        normal *= -1
    offset = -np.dot(normal, center)
    if normal[2] < cosine or abs(offset / normal[2]) > .5:
        raise ValueError("Refined plane exceeds local ground limits")
    heights = (points @ normal + offset) / normal[2]
    ground = points[np.abs(heights) < ground_tolerance]
    obstacles = points[(heights >= obstacle_height) & (heights <= 2.)]
    return ground.astype(np.float32), obstacles.astype(np.float32)
