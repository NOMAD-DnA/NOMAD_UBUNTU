"""Bounded confidence penalty; neighbors supply evidence, never their costs."""
from dataclasses import asdict, dataclass
import math
import numpy as np
from nomad_perception.traversability_core import CLASS_COSTS

NEIGHBORS = ((-1, -1), (0, -1), (1, -1), (-1, 0),
             (1, 0), (-1, 1), (0, 1), (1, 1))


@dataclass(frozen=True)
class SemanticCostPolicy:
    confidence_beta: float = 0.25
    neighbor_confidence_enabled: bool = True
    confidence_low_threshold: float = 0.5
    confidence_neighbor_min: float = 0.7
    confidence_histogram_tv_max: float = 0.1
    confidence_high_cost_share_tolerance: float = 0.02

    def __post_init__(self):
        if not isinstance(self.neighbor_confidence_enabled, bool):
            raise ValueError('neighbor_confidence_enabled must be bool')
        for name in ('confidence_beta', 'confidence_low_threshold',
                     'confidence_neighbor_min', 'confidence_histogram_tv_max',
                     'confidence_high_cost_share_tolerance'):
            value = getattr(self, name)
            if not math.isfinite(value) or not 0 <= value <= 1:
                raise ValueError(f'{name} must be finite and within [0, 1]')
        if self.confidence_beta >= 1:
            raise ValueError('confidence_beta must be less than 1')
        if not self.confidence_low_threshold < self.confidence_neighbor_min:
            raise ValueError('confidence_low_threshold must be below confidence_neighbor_min')

    def compatible(self, own, neighbor):
        if own is None or neighbor is None:
            return False
        delta = np.abs(own - neighbor)
        if .5 * float(delta.sum()) > self.confidence_histogram_tv_max + 1e-12:
            return False
        high = np.isfinite(CLASS_COSTS) & (CLASS_COSTS >= 90)
        # A newly introduced costly class is a boundary cue, even at low share.
        if np.any((own[high] > 0) != (neighbor[high] > 0)):
            return False
        return not np.any(delta[high] > self.confidence_high_cost_share_tolerance + 1e-12)

    def adjusted(self, raw, confidence):
        if raw is None:
            return None
        if raw >= 100:
            return 100.
        base = max(5., raw)
        return float(np.float32(min(99., base + self.confidence_beta *
                                  (1. - confidence) * (99. - base))))

    def metadata(self):
        return {'name': 'bounded_semantic_confidence_v2', **asdict(self),
                'formula': 'S_adj=S+beta*(1-effective_q_s)*(99-S); floor(S_adj+0.5)',
                'neighbors': 'all eight original high-support, recent, compatible observations; one pass',
                'neighbor_max_age_s': 'raw_window_s',
                'high_cost_class_threshold': 90,
                'confidence_is_probability': False}
