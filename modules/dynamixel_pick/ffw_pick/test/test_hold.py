"""Hold before motion: release, cancel, stale plan and timeout with a fake clock (no ROS)."""
import math

import pytest

from ffw_pick.hold import (
    HOLD_MAX_AGE, HOLD_TIMEOUT, hold_verdict, joints_moved, wait_for_release)


class Clock:
    def __init__(self):
        self.now = 50.0

    def __call__(self):
        return self.now


def run(release_at=None, cancel_at=None, step=0.5):
    """wait_for_release with a release (and a cancel) at these seconds of holding."""
    clock = Clock()
    start = clock.now

    def check_cancel():
        if cancel_at is not None and clock.now - start >= cancel_at:
            raise KeyboardInterrupt

    def pause():
        clock.now += step
    return wait_for_release(lambda: release_at is not None and clock.now - start >= release_at,
                            check_cancel, clock, pause)


def test_release_while_holding_goes():
    verdict, waited = run(release_at=3.0)
    assert verdict == 'go'
    assert waited == pytest.approx(3.0)


def test_release_after_max_age_observes_again():
    verdict, waited = run(release_at=HOLD_MAX_AGE + 1.0)
    assert verdict == 'replan'
    assert waited == pytest.approx(HOLD_MAX_AGE + 1.0)


def test_no_release_times_out():
    verdict, waited = run()
    assert verdict == 'timeout'
    assert waited == pytest.approx(HOLD_TIMEOUT)


def test_cancel_while_holding_raises():
    with pytest.raises(KeyboardInterrupt):
        run(release_at=20.0, cancel_at=2.0)


def test_verdict_edges():
    assert hold_verdict(0.0, False) == 'wait'
    assert hold_verdict(HOLD_MAX_AGE, True) == 'go'
    assert hold_verdict(HOLD_MAX_AGE + 0.01, True) == 'replan'
    assert hold_verdict(HOLD_TIMEOUT - 0.01, False) == 'wait'
    assert hold_verdict(HOLD_TIMEOUT, False) == 'timeout'
    assert hold_verdict(1.0, True, max_age=0.5) == 'replan'


def test_joints_moved():
    before = {'a': 0.10, 'b': -1.00, 'c': 0.0}
    assert joints_moved(before, {'a': 0.105, 'b': -1.00, 'c': 0.0}, ['a', 'b'], 0.01) == []
    moved = joints_moved(before, {'a': 0.13, 'b': -1.00}, ['a', 'b', 'c'], 0.01)
    assert moved[0] == 'a +0.030'
    assert moved[1].startswith('c ') and math.isnan(float(moved[1].split()[1]))
