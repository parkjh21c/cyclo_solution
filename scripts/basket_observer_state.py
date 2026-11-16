"""Basket observer bookkeeping without ROS: when to log and when to resend the scene boxes."""

import math

import numpy as np


def pose_change(a, b):
    """(distance m, rotation angle rad) between two (position, rotation matrix) pairs."""
    moved = float(np.linalg.norm(np.asarray(b[0]) - np.asarray(a[0])))
    cosine = (np.trace(np.asarray(a[1]).T @ np.asarray(b[1])) - 1) / 2
    return moved, math.acos(float(np.clip(cosine, -1.0, 1.0)))


def reason_kind(text):
    """Status kind without its details, e.g. 'MOVING: arm_r_joint1' -> 'MOVING'."""
    return text.split(":", 1)[0].split(" ", 1)[0].strip()


class BasketObserverState:
    MEASURED = "BASKET"  # Status kind of a successful measurement.

    def __init__(self, min_move, min_turn, jump_move=0.010, jump_turn=math.radians(3.0)):
        self.min_move = min_move  # m
        self.min_turn = min_turn  # rad
        self.jump_move = jump_move  # m: farther from the applied value needs a second look
        self.jump_turn = jump_turn  # rad
        self.accepted = None  # (position, rotation) last published value
        self.candidate = None  # (position, rotation) jumped value waiting for confirmation
        self.at_initial = False
        self.returns = 0  # Bumped on every return to the initial pose.
        self.first_after_return = False
        self.applied = None  # (position, rotation) MoveIt confirmed since the last return
        self.logged = None  # (position, rotation) last measurement logged
        self.kind = None  # Last status kind seen

    def pose(self, at_initial):
        """Record the pose check; True on a return to the initial pose (scene is resent)."""
        returned = at_initial and not self.at_initial
        self.at_initial = at_initial
        if not at_initial:
            self.candidate = None  # A view from another pose cannot confirm it.
        if returned:
            self.returns += 1
            self.first_after_return = True
            self.applied = None  # The pick may have moved the basket_* boxes meanwhile.
        return returned

    def near(self, a, b):
        moved, turned = pose_change(a, b)
        return moved < self.jump_move and turned < self.jump_turn

    def screen(self, position, rotation):
        """(apply?, dropped candidate?) for one measurement; a jump must repeat to be applied."""
        value = (np.array(position, dtype=float), np.array(rotation, dtype=float))
        dropped = False
        if self.accepted is not None and self.near(self.accepted, value):
            dropped = self.candidate is not None  # The jump did not repeat.
        elif self.candidate is None or not self.near(self.candidate, value):
            self.candidate = value  # First value, or a different jump: wait for one more.
            return False, False
        self.candidate = None
        self.accepted = value
        return True, dropped

    def status(self, text):
        """True when this status starts a new kind of reason (log it once)."""
        kind = reason_kind(text)
        changed = kind != self.kind
        self.kind = kind
        return changed

    def measured(self, position, rotation):
        """(log?, mm moved since the last logged value or None) for one successful measurement."""
        self.kind = self.MEASURED
        moved = None
        log = self.first_after_return or self.logged is None
        if self.logged is not None:
            distance, turned = pose_change(self.logged, (position, rotation))
            moved = distance * 1000.0
            log = log or distance >= self.min_move or turned >= self.min_turn
        if log:
            self.first_after_return = False
            self.logged = (np.array(position, dtype=float), np.array(rotation, dtype=float))
        return log, moved

    def needs_scene(self, position, rotation):
        """True with no confirmed box since the last return, or when the basket moved enough."""
        if self.applied is None:
            return True
        moved, turned = pose_change(self.applied, (position, rotation))
        return moved >= self.min_move or turned >= self.min_turn

    def scene_applied(self, token, position, rotation):
        """Keep a confirmed update unless the arm left and returned after it was sent."""
        if token == self.returns:
            self.applied = (np.array(position, dtype=float), np.array(rotation, dtype=float))
