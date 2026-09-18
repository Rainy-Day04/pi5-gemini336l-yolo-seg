"""Pure geometry helpers, kept ROS-independent for unit testing."""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class Projection:
    x: float
    y: float
    z: float
    u: int
    v: int
    valid_pixels: int


def depth_to_meters(depth: np.ndarray, encoding: str) -> np.ndarray:
    """Convert common ROS depth encodings into float32 metres."""
    encoding = encoding.upper()
    if encoding in ("16UC1", "MONO16"):
        return depth.astype(np.float32) * 0.001
    if encoding == "32FC1":
        return depth.astype(np.float32)
    raise ValueError(f"Unsupported depth encoding: {encoding}")


def project_mask(
    mask: np.ndarray,
    depth_m: np.ndarray,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    min_depth_m: float,
    max_depth_m: float,
    min_valid_pixels: int,
) -> Projection | None:
    """Project robust median mask depth and pixel centroid into camera XYZ."""
    if mask.shape != depth_m.shape:
        raise ValueError("mask and depth image must have the same shape")

    valid = mask.astype(bool) & np.isfinite(depth_m)
    valid &= depth_m >= min_depth_m
    valid &= depth_m <= max_depth_m
    count = int(np.count_nonzero(valid))
    if count < min_valid_pixels or fx <= 0.0 or fy <= 0.0:
        return None

    rows, cols = np.nonzero(valid)
    z = float(np.median(depth_m[valid]))
    # Use the median pixel location to reject thin mask outliers.
    u = int(np.median(cols))
    v = int(np.median(rows))
    x = (float(u) - cx) * z / fx
    y = (float(v) - cy) * z / fy
    return Projection(x=x, y=y, z=z, u=u, v=v, valid_pixels=count)
