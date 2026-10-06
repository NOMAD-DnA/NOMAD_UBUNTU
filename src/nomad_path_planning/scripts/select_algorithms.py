#!/usr/bin/env python3
"""Select a numbered planner pair; stdout contains only ROS launch arguments."""
import os
import sys

PAIRS = (
    ('Hybrid A* + RPP', 'hybrid_astar', 'rpp'),
    ('State Lattice + RPP', 'state_lattice', 'rpp'),
    ('D* Lite + Ackermann rollout', 'dstar_lite', 'rollout'),
    ('A* + Ackermann rollout', 'astar', 'rollout'),
    ('Weighted A* + Ackermann rollout', 'weighted_astar', 'rollout'),
    ('Field D* 변형 + RPP', 'field_dstar', 'rpp'),
)


def choose(selection=None):
    if selection is None:
        if not sys.stdin.isatty():
            return None  # Preserve YAML defaults in automated runs.
        print('\n알고리즘 조합 선택:', file=sys.stderr)
        for number, (label, _, _) in enumerate(PAIRS, 1):
            print(f'  {number}. {label}', file=sys.stderr)
        while True:
            print('번호 입력 [기본 3]: ', end='', file=sys.stderr, flush=True)
            value = input().strip() or '3'
            try:
                return choose(value)
            except ValueError:
                print('1~6 사이의 번호를 입력해 주세요.', file=sys.stderr)
    selection = selection.strip()
    if selection not in tuple(str(n) for n in range(1, len(PAIRS)+1)):
        raise ValueError('알고리즘 조합 번호는 1~6이어야 합니다.')
    return PAIRS[int(selection)-1]


def main():
    pair = choose(os.environ.get('NOMAD_PLANNER_PAIR'))
    if pair is not None:
        label, gpp, lpp = pair
        print(f'선택: {label}', file=sys.stderr)
        print(f'gpp:={gpp}\nlpp:={lpp}')


if __name__ == '__main__':
    try:
        main()
    except (ValueError, EOFError) as error:
        raise SystemExit(str(error) or '알고리즘 선택이 취소되었습니다.')
    except KeyboardInterrupt:
        raise SystemExit(130)
