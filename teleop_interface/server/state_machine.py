"""
Thread-safe state machine for the bimanual teleop system.

States
------
IDLE                  — no robots connected
TELEOP_PASSIVE        — robots connected, holding last position (no GELLO input)
TELEOP_ACTIVE         — robots connected, GELLO devices driving the arms
MOTION_PLAN_IDLE      — motion-plan mode, waiting for a goal
MOTION_PLAN_EXECUTING — executing a moveJ or joint trajectory

Valid transitions are defined in VALID_TRANSITIONS.  Any invalid transition
request returns False and leaves the state unchanged.
"""

import threading
from enum import Enum
from typing import Callable, List, Set


class SystemState(Enum):
    IDLE                   = "idle"
    TELEOP_PASSIVE         = "teleop_passive"
    TELEOP_ACTIVE          = "teleop_active"
    MOTION_PLAN_IDLE       = "motion_plan_idle"
    MOTION_PLAN_EXECUTING  = "motion_plan_executing"


VALID_TRANSITIONS: dict[SystemState, Set[SystemState]] = {
    SystemState.IDLE: {
        SystemState.TELEOP_PASSIVE,
        SystemState.MOTION_PLAN_IDLE,
    },
    SystemState.TELEOP_PASSIVE: {
        SystemState.TELEOP_ACTIVE,
        SystemState.MOTION_PLAN_IDLE,
        SystemState.IDLE,
    },
    SystemState.TELEOP_ACTIVE: {
        SystemState.TELEOP_PASSIVE,
        SystemState.MOTION_PLAN_IDLE,
        SystemState.IDLE,
    },
    SystemState.MOTION_PLAN_IDLE: {
        SystemState.TELEOP_PASSIVE,
        SystemState.MOTION_PLAN_EXECUTING,
        SystemState.IDLE,
    },
    SystemState.MOTION_PLAN_EXECUTING: {
        SystemState.MOTION_PLAN_IDLE,
        SystemState.IDLE,  # emergency stop only
    },
}

StateChangeCallback = Callable[[SystemState, SystemState], None]


class StateMachine:
    def __init__(self) -> None:
        self._state = SystemState.IDLE
        self._lock  = threading.Lock()
        self._listeners: List[StateChangeCallback] = []

    # ── State Access ───────────────────────────────────────────────────────

    @property
    def state(self) -> SystemState:
        return self._state

    # ── Listener Registration ──────────────────────────────────────────────

    def add_listener(self, cb: StateChangeCallback) -> None:
        """Register a callback(old_state, new_state) invoked after each transition."""
        self._listeners.append(cb)

    # ── Transitions ────────────────────────────────────────────────────────

    def transition(self, new_state: SystemState, *, force: bool = False) -> bool:
        """
        Attempt to transition to new_state.
        Returns True on success, False if the transition is not allowed.
        Set force=True to bypass the validity check (emergency stop only).
        """
        with self._lock:
            allowed = VALID_TRANSITIONS.get(self._state, set())
            if not force and new_state not in allowed:
                return False
            old_state   = self._state
            self._state = new_state

        for cb in self._listeners:
            try:
                cb(old_state, new_state)
            except Exception as exc:
                print(f"[StateMachine] Listener error: {exc}")

        return True

    # ── Convenience Predicates ─────────────────────────────────────────────

    def is_idle(self) -> bool:
        return self._state is SystemState.IDLE

    def is_teleop(self) -> bool:
        return self._state in (SystemState.TELEOP_PASSIVE, SystemState.TELEOP_ACTIVE)

    def is_active(self) -> bool:
        return self._state is SystemState.TELEOP_ACTIVE

    def is_motion_plan(self) -> bool:
        return self._state in (
            SystemState.MOTION_PLAN_IDLE,
            SystemState.MOTION_PLAN_EXECUTING,
        )
