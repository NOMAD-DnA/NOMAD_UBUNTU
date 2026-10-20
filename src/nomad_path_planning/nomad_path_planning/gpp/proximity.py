"""GPP-only soft preferences; never modify occupancy or unknown cells."""
import math
import cv2
import numpy as np


def proximity_costs(data, width, height, resolution, radius=2.0, weight=4.0):
    if not all(math.isfinite(v) for v in (resolution, radius, weight)) or resolution <= 0 or radius <= 0 or weight < 0:
        raise ValueError("Invalid GPP proximity resolution/radius/weight")
    cells = np.asarray(data).reshape(height, width)
    # Source 100 denotes confirmed obstacles in the navigation map. Costs
    # 80..99 are already inflated and must not become new distance origins.
    source = cells == 100
    if weight == 0 or not np.any(source):
        return np.zeros(cells.size).tolist()
    distance = cv2.distanceTransform((~source).astype(np.uint8),
                                    cv2.DIST_L2, cv2.DIST_MASK_PRECISE) * resolution
    extra = weight * np.maximum(0., 1. - distance / radius) ** 2
    extra[(cells < 0) | (cells >= 80)] = 0.
    return extra.ravel().tolist()
