"""Hold before the first pick motion until the demo releases it (no ROS here)."""
import math

HOLD_MAX_AGE = 10.0  # s: a plan held longer is observed again after the release
HOLD_TIMEOUT = 60.0  # s without a release: the request fails before any motion


def hold_verdict(waited, released, max_age=HOLD_MAX_AGE, timeout=HOLD_TIMEOUT):
    """'wait', 'go' (released in time), 'replan' (released after max_age) or 'timeout'."""
    if released:
        return 'go' if waited <= max_age else 'replan'
    return 'timeout' if waited >= timeout else 'wait'


def wait_for_release(is_released, check_cancel, clock, pause, max_age=HOLD_MAX_AGE,
                     timeout=HOLD_TIMEOUT):
    """(verdict, s waited) once hold_verdict is not 'wait'; check_cancel raises to stop."""
    started = clock()
    while True:
        check_cancel()
        waited = clock() - started
        verdict = hold_verdict(waited, is_released(), max_age, timeout)
        if verdict != 'wait':
            return verdict, waited
        pause()


def joints_moved(before, now, names, tolerance):
    """Names (with the change) of joints that moved more than tolerance or are missing."""
    out = []
    for name in names:
        change = abs(now.get(name, math.nan) - before.get(name, math.nan))
        if not change <= tolerance:  # NaN (missing) counts as moved
            out.append(f'{name} {change:+.3f}')
    return out


STALE_JOINT_AGE = 1.0  # s: an older joint value counts as unreadable


def fresh_joints(values, times, names, now, max_age=STALE_JOINT_AGE):
    """{name: value} of the names received within max_age s of now (monotonic)."""
    return {n: values[n] for n in names
            if n in values and now - times.get(n, -math.inf) <= max_age}


def path_recheck_needed(before, after, names, tolerance):
    """False only when every name is in both (fresh) snapshots and moved at most tolerance."""
    return any(n not in before or n not in after or not abs(after[n] - before[n]) <= tolerance
               for n in names)
