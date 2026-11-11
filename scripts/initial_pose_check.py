"""One arm at the menu-1 initial pose: the rule both wrist observers share (no ROS here).

At initial pose = every arm joint within joint_tolerance of initial, the lift within
lift_tolerance of lift, and no joint moved more than still_tolerance during the last
still_window seconds.
"""
from collections import deque

INITIAL_ARM = [1.05, 0.0, 0.0, -2.27, 0.0, 0.35, 0.0]  # config/demo.yaml observe_robot
INITIAL_LIFT = -0.1
LIFT_JOINT = "lift_joint"


def arm_joints(side):
    return [f"arm_{'l' if side == 'left' else 'r'}_joint{i}" for i in range(1, 8)]


class InitialPoseCheck:
    def __init__(self, side, joint_tolerance=0.02, lift_tolerance=0.01, still_window=0.5,
                 still_tolerance=0.005, max_age=0.5, initial=None, lift=INITIAL_LIFT):
        self.names = arm_joints(side) + [LIFT_JOINT]
        self.target = list(initial or INITIAL_ARM) + [lift]
        self.tolerances = [joint_tolerance] * 7 + [lift_tolerance]
        self.still_window = still_window
        self.still_tolerance = still_tolerance
        self.max_age = max_age
        self.history = deque()  # (seconds, positions)
        if not (len(self.target) == 8 and 0 < joint_tolerance <= 0.1 and 0 < lift_tolerance
                and still_window > 0 and still_tolerance > 0):
            raise ValueError("Invalid initial-pose check parameters")

    def observe(self, seconds, names, positions):
        """Store one joint state; states without every needed joint are ignored."""
        values = dict(zip(names, positions))
        if not all(name in values for name in self.names):
            return
        self.history.append((seconds, [values[name] for name in self.names]))
        while self.history and self.history[0][0] < seconds - 2 * self.still_window:
            self.history.popleft()

    def reason(self, now):
        """None at the initial pose and still, else why not."""
        if not self.history:
            return "NO_JOINT_STATES"
        stamp, latest = self.history[-1]
        if now - stamp > self.max_age:
            return "JOINT_STATES_STALE"
        for name, value, target, tolerance in zip(self.names, latest, self.target,
                                                  self.tolerances):
            if abs(value - target) > tolerance:
                return f"NOT_INITIAL_POSE: {name} {value - target:+.3f}"
        window = [p for t, p in self.history if t >= stamp - self.still_window]
        if self.history[0][0] > stamp - self.still_window:
            return "SETTLING"  # Not enough history yet to call the arm still.
        for i, name in enumerate(self.names):
            values = [p[i] for p in window]
            if max(values) - min(values) > self.still_tolerance:
                return f"MOVING: {name}"
        return None
