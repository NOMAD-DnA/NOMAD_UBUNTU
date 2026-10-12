#!/usr/bin/env python3
"""Measurement-level GNSS fault model. ENU meters; no satellite propagation."""
import copy
import math
import random

MODES = {'UNKNOWN': 0, 'NO_FIX': 1, 'SINGLE': 2, 'DGNSS': 3,
         'RTK_FLOAT': 4, 'RTK_FIXED': 5}


def shift_geodetic(lat, lon, alt, enu):
    """Apply ENU translation using WGS84 ECEF (transpose of ECEF->ENU)."""
    a, e2 = 6378137.0, 6.6943799901413165e-3
    p, l = math.radians(lat), math.radians(lon)
    sp, cp, sl, cl = math.sin(p), math.cos(p), math.sin(l), math.cos(l)
    n = a / math.sqrt(1-e2*sp*sp)
    east, north, up = enu
    x = (n+alt)*cp*cl - sl*east - sp*cl*north + cp*cl*up
    y = (n+alt)*cp*sl + cl*east - sp*sl*north + cp*sl*up
    z = (n*(1-e2)+alt)*sp + cp*north + sp*up
    r = math.hypot(x, y)
    if r < 1e-9:
        return math.copysign(90., z), lon, abs(z)-a*math.sqrt(1-e2)
    p = math.atan2(z, r*(1-e2))
    for _ in range(12):
        n = a/math.sqrt(1-e2*math.sin(p)**2)
        p2 = math.atan2(z+e2*n*math.sin(p), r)
        if abs(p2-p) < 1e-14:
            p = p2
            break
        p = p2
    n = a/math.sqrt(1-e2*math.sin(p)**2)
    height = r*math.cos(p)+z*math.sin(p)-n*(1-e2*math.sin(p)**2)
    return math.degrees(p), math.degrees(math.atan2(y, x)), height


def number(v, name, nonnegative=True):
    if isinstance(v, bool):
        raise ValueError(name+' must be numeric')
    v = float(v)
    if not math.isfinite(v) or (nonnegative and v < 0):
        raise ValueError(name+' must be finite'+(' and nonnegative' if nonnegative else ''))
    return v


def vec(v, name, nonnegative=False):
    if not isinstance(v, (list, tuple)) or len(v) != 3:
        raise ValueError(name+' must have three ENU components')
    return tuple(number(x, name, nonnegative) for x in v)


class GNSSFaultModel:
    """Schedules relative to first input stamp; resets RNG on time rewind."""
    def __init__(self, profile):
        if set(profile)-{'seed', 'actual_sigma_enu_m', 'reported_sigma_enu_m', 'quality', 'events'}:
            raise ValueError('Unknown GNSS profile field')
        self.seed = profile.get('seed', 42)
        if not isinstance(self.seed, int) or isinstance(self.seed, bool):
            raise ValueError('seed must be integer')
        self.sigma = vec(profile.get('actual_sigma_enu_m', [0, 0, 0]), 'actual sigma', True)
        reported = profile.get('reported_sigma_enu_m')
        self.reported = None if reported is None else vec(reported, 'reported sigma', True)
        self.quality = self.validate_quality(profile.get('quality', {'solution_type': 'UNKNOWN'}))
        self.events = []
        events = profile.get('events', [])
        if not isinstance(events, list) or len(events) > 1000:
            raise ValueError('events must be a list, <=1000')
        for src in events:
            e = dict(src)
            allowed = {'name', 'type', 'start_s', 'duration_s', 'quality', 'reported_sigma_enu_m'}
            extras = {'noise': {'actual_sigma_enu_m'}, 'spike': {'offset_enu_m'},
                      'offset': {'offset_enu_m'}, 'drift': {'rate_enu_mps'},
                      'delay': {'delay_s'}, 'dropout': set(), 'no_fix': set(),
                      'freeze': set(), 'quality': set()}
            kind = e.get('type')
            if kind not in extras or set(e)-(allowed|extras[kind]):
                raise ValueError('Unknown event type/field: '+str(kind))
            e['start_s'] = number(e.get('start_s', 0), 'start_s')
            e['duration_s'] = number(e.get('duration_s', 1), 'duration_s')
            if e['duration_s'] <= 0:
                raise ValueError('duration_s must be positive')
            for key in extras[kind]-{'delay_s'}:
                e[key] = vec(e[key], key, key == 'actual_sigma_enu_m')
            if kind == 'delay':
                e['delay_s'] = number(e['delay_s'], 'delay_s')
                if e['delay_s'] > 60:
                    raise ValueError('delay_s must be <=60')
            if 'reported_sigma_enu_m' in e:
                e['reported_sigma_enu_m'] = vec(e['reported_sigma_enu_m'], 'reported sigma', True)
            if 'quality' in e:
                e['quality'] = self.validate_quality(e['quality'])
            self.events.append(e)
        self.reset()

    @staticmethod
    def validate_quality(q):
        if not isinstance(q, dict) or set(q)-{'solution_type', 'satellites_used', 'pdop', 'correction_age_s'}:
            raise ValueError('Unknown quality field')
        q = dict(q)
        if 'solution_type' in q and q['solution_type'] not in MODES:
            raise ValueError('Unknown solution_type')
        if 'satellites_used' in q:
            n = q['satellites_used']
            if isinstance(n, bool) or not isinstance(n, int) or not 0 <= n <= 65535:
                raise ValueError('satellites_used must be uint16')
        for key in ('pdop', 'correction_age_s'):
            if key in q:
                q[key] = number(q[key], key)
        return q

    def reset(self):
        self.rng = random.Random(self.seed)
        self.first = self.last = None
        self.fired, self.frozen = set(), {}

    def process(self, packet):
        stamp = packet['stamp_ns']
        if self.last is not None and stamp < self.last:
            self.reset()
        if self.last == stamp:
            return None
        if self.first is None:
            self.first = stamp
        self.last = stamp
        elapsed = (stamp-self.first)*1e-9
        active = [(i, e) for i, e in enumerate(self.events)
                  if e['start_s'] <= elapsed < e['start_s']+e['duration_s']]
        out = copy.deepcopy(packet)
        sigma, reported, q = self.sigma, self.reported, dict(self.quality)
        offset = [0., 0., 0.]
        delay = 0.
        for i, e in active:
            kind = e['type']
            if kind == 'dropout':
                return None  # No position AND no quality message.
            if kind == 'noise':
                sigma = e['actual_sigma_enu_m']
            elif kind == 'offset' or (kind == 'spike' and i not in self.fired):
                offset = [a+b for a, b in zip(offset, e['offset_enu_m'])]
                self.fired.add(i)
            elif kind == 'drift':
                offset = [a+b*(elapsed-e['start_s']) for a, b in zip(offset, e['rate_enu_mps'])]
            elif kind == 'delay':
                delay = max(delay, e['delay_s'])
            q.update(e.get('quality', {}))
            reported = e.get('reported_sigma_enu_m', reported)
            if kind == 'no_fix':
                q['solution_type'] = 'NO_FIX'
        mode = MODES.get(q.get('solution_type', 'UNKNOWN'), 0)
        if out['status'] < 0:
            mode = MODES['NO_FIX']  # Never turn invalid source fixes into a valid solution.
        q['solution_type'] = mode
        out['quality'], out['delay_s'] = q, delay
        if mode == MODES['NO_FIX']:
            out['status'], out['position'] = -1, (float('nan'),)*3
            out['covariance'], out['covariance_type'] = [0.]*9, 0
        else:
            if not all(math.isfinite(v) for v in out['position']):
                return None
            offset = [v+self.rng.gauss(0, s) if s else v for v, s in zip(offset, sigma)]
            if any(offset):
                out['position'] = shift_geodetic(*out['position'], offset)
            if reported is not None:
                out['covariance'] = [0.]*9
                for j, s in zip((0, 4, 8), reported):
                    out['covariance'][j] = s*s  # ENU m^2, NOT angular variance.
                out['covariance_type'] = 2
            if mode:
                out['status'] = 0 if mode == 2 else 2  # standard FIX/SBAS/GBAS enum only
        for i, e in active:
            if e['type'] == 'freeze':
                self.frozen.setdefault(i, copy.deepcopy(out))
                out = copy.deepcopy(self.frozen[i])  # Preserve measurement stamp, not fresh data.
        return out
