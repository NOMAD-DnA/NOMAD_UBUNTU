"""Disabling timeouts must retain expansion limits and explicit deadlines."""
import math

import pytest

from nomad_path_planning.gpp import base
from nomad_path_planning.gpp.base import Budget, BudgetExceeded, Request
from nomad_path_planning.gpp.registry import NAMES, create


def test_unlimited_time_retains_expansion_limit(monkeypatch):
    budget = Budget(seconds=0., expansions=2)
    monkeypatch.setattr(base.time, 'monotonic', lambda: 1e30)
    budget.tick()
    budget.tick()
    with pytest.raises(BudgetExceeded):
        budget.tick()


def test_positive_timeout_still_expires(monkeypatch):
    monkeypatch.setattr(base.time, 'monotonic', lambda: 10.)
    budget = Budget(seconds=1.)
    monkeypatch.setattr(base.time, 'monotonic', lambda: 12.)
    with pytest.raises(BudgetExceeded):
        budget.tick()


@pytest.mark.parametrize('name', NAMES)
def test_default_strategy_has_no_time_limit(name, monkeypatch):
    # Every clock check advances by two seconds, including reverse guidance.
    ticks = iter(range(0, 10000000, 2))
    monkeypatch.setattr(base.time, 'monotonic', lambda: next(ticks))
    r = Request(dict(width=20, height=12, resolution=.2,
                     origin_x=0., origin_y=0., data=[0]*240),
                (.5, 1.1, 0.), (2.9, 1.1))
    result = create(name).plan(r)
    assert result.path, result.detail
    assert math.dist(result.path[-1][:2], r.goal) < .3


@pytest.mark.parametrize('seconds', [-1., math.nan, math.inf])
def test_invalid_timeout_rejected(seconds):
    with pytest.raises(ValueError):
        create('astar', seconds=seconds)
    with pytest.raises(ValueError):
        Budget(seconds=seconds)
