"""The per-request pick timing line, with a fake clock (no ROS)."""
import pytest

from ffw_pick.pick_moveit import command_wait, PickTiming, recent_commands


class Clock:
    def __init__(self):
        self.now = 100.0

    def __call__(self):
        return self.now


def test_successful_pick_line():
    clock = Clock()
    timing = PickTiming(clock)
    clock.now += 2.1
    timing.mark_init()
    with timing.measure('plan'):
        clock.now += 3.4
    clock.now += 0.5  # pose wait and scene: other
    for phase, seconds in (('search', 1.2), ('open', 0.9), ('approach', 3.0), ('approach', 3.0),
                           ('close', 2.5), ('lift', 3.1)):
        with timing.measure(phase):
            clock.now += seconds
    assert timing.line('ok') == (
        'pick timing: init 2.1, plan 3.4 (1 call), wait 0.0, search 1.2, hold 0.0, open 0.9,'
        ' approach 6.0, close 2.5, lift 3.1, other 0.5, total 19.7 s, result ok')


def test_empty_basket_lists_each_plan_call_and_the_wait():
    clock = Clock()
    timing = PickTiming(clock)
    clock.now += 3.0
    timing.mark_init()
    with timing.measure('plan'):
        clock.now += 2.0
    with timing.measure('wait'):
        clock.now += 2.0
    timing.mark_init()  # Only the first plan request ends init.
    with timing.measure('plan'):
        clock.now += 1.5
    line = timing.line('basket empty, code BASKET_EMPTY')
    assert line.startswith('pick timing: init 3.0, plan 3.5 (2 calls: 2.0 + 1.5), wait 2.0,')
    assert line.endswith('other 0.0, total 8.5 s, result basket empty, code BASKET_EMPTY')


def test_phase_time_is_kept_when_the_block_raises():
    clock = Clock()
    timing = PickTiming(clock)
    timing.mark_init()
    with pytest.raises(RuntimeError):
        with timing.measure('approach'):
            clock.now += 1.5
            raise RuntimeError('execution error')
    assert 'approach 1.5' in timing.line('error')


def test_failure_before_any_plan_counts_all_as_init():
    clock = Clock()
    timing = PickTiming(clock)
    clock.now += 4.0
    assert timing.line('stop').startswith('pick timing: init 4.0, plan 0.0 (0 calls),')


def test_command_check_needs_no_extra_wait_after_a_long_search():
    assert command_wait(watched_from=100.0, now=133.8) == 0.0
    assert command_wait(watched_from=100.0, now=101.0) == 0.0


def test_command_check_waits_only_the_missing_time():
    assert command_wait(watched_from=100.0, now=100.3) == pytest.approx(0.7)
    assert command_wait(watched_from=100.0, now=100.0) == pytest.approx(1.0)


def test_only_commands_of_the_last_second_count():
    times = [100.5, 131.0, 133.2, 133.8]
    assert recent_commands(times, now=133.8) == 2
    assert recent_commands([100.5], now=133.8) == 0
    assert recent_commands([], now=133.8) == 0
