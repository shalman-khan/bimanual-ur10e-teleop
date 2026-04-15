"""
Unit tests for state_machine.py — zero external dependencies.

Run on the host (any Python ≥3.9, no ROS2 needed):
    python teleop_interface/tests/test_state_machine.py

Run inside Docker with pytest:
    python -m pytest /workspace/teleop_interface/tests/test_state_machine.py -v
"""

import sys
import os
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "server"))

from state_machine import StateMachine, SystemState


class TestInitialState(unittest.TestCase):
    def test_starts_idle(self):
        sm = StateMachine()
        self.assertIs(sm.state, SystemState.IDLE)
        self.assertTrue(sm.is_idle())
        self.assertFalse(sm.is_teleop())
        self.assertFalse(sm.is_motion_plan())
        self.assertFalse(sm.is_active())


class TestValidTransitions(unittest.TestCase):
    def setUp(self):
        self.sm = StateMachine()

    def test_idle_to_teleop_passive(self):
        self.assertTrue(self.sm.transition(SystemState.TELEOP_PASSIVE))
        self.assertIs(self.sm.state, SystemState.TELEOP_PASSIVE)
        self.assertTrue(self.sm.is_teleop())

    def test_idle_to_motion_plan(self):
        self.assertTrue(self.sm.transition(SystemState.MOTION_PLAN_IDLE))
        self.assertTrue(self.sm.is_motion_plan())

    def test_teleop_passive_to_active(self):
        self.sm.transition(SystemState.TELEOP_PASSIVE)
        self.assertTrue(self.sm.transition(SystemState.TELEOP_ACTIVE))
        self.assertTrue(self.sm.is_active())

    def test_teleop_active_to_passive(self):
        self.sm.transition(SystemState.TELEOP_PASSIVE)
        self.sm.transition(SystemState.TELEOP_ACTIVE)
        self.assertTrue(self.sm.transition(SystemState.TELEOP_PASSIVE))
        self.assertIs(self.sm.state, SystemState.TELEOP_PASSIVE)

    def test_teleop_to_motion_plan(self):
        self.sm.transition(SystemState.TELEOP_PASSIVE)
        self.assertTrue(self.sm.transition(SystemState.MOTION_PLAN_IDLE))
        self.assertTrue(self.sm.is_motion_plan())

    def test_teleop_active_to_motion_plan(self):
        self.sm.transition(SystemState.TELEOP_PASSIVE)
        self.sm.transition(SystemState.TELEOP_ACTIVE)
        self.assertTrue(self.sm.transition(SystemState.MOTION_PLAN_IDLE))
        self.assertIs(self.sm.state, SystemState.MOTION_PLAN_IDLE)

    def test_motion_plan_to_executing(self):
        self.sm.transition(SystemState.MOTION_PLAN_IDLE)
        self.assertTrue(self.sm.transition(SystemState.MOTION_PLAN_EXECUTING))

    def test_motion_plan_executing_back_to_idle_state(self):
        self.sm.transition(SystemState.MOTION_PLAN_IDLE)
        self.sm.transition(SystemState.MOTION_PLAN_EXECUTING)
        self.assertTrue(self.sm.transition(SystemState.MOTION_PLAN_IDLE))

    def test_motion_plan_to_teleop(self):
        self.sm.transition(SystemState.MOTION_PLAN_IDLE)
        self.assertTrue(self.sm.transition(SystemState.TELEOP_PASSIVE))
        self.assertIs(self.sm.state, SystemState.TELEOP_PASSIVE)

    def test_any_to_idle(self):
        self.sm.transition(SystemState.TELEOP_PASSIVE)
        self.assertTrue(self.sm.transition(SystemState.IDLE))
        self.assertTrue(self.sm.is_idle())

    def test_full_teleop_cycle(self):
        """IDLE → passive → active → passive → motion_plan → idle"""
        path = [
            SystemState.TELEOP_PASSIVE,
            SystemState.TELEOP_ACTIVE,
            SystemState.TELEOP_PASSIVE,
            SystemState.MOTION_PLAN_IDLE,
            SystemState.IDLE,
        ]
        for target in path:
            ok = self.sm.transition(target)
            self.assertTrue(ok, f"Transition to {target} failed from {self.sm.state}")
        self.assertTrue(self.sm.is_idle())


class TestInvalidTransitions(unittest.TestCase):
    def setUp(self):
        self.sm = StateMachine()

    def test_idle_to_active_rejected(self):
        self.assertFalse(self.sm.transition(SystemState.TELEOP_ACTIVE))
        self.assertIs(self.sm.state, SystemState.IDLE)

    def test_idle_to_executing_rejected(self):
        self.assertFalse(self.sm.transition(SystemState.MOTION_PLAN_EXECUTING))
        self.assertIs(self.sm.state, SystemState.IDLE)

    def test_executing_to_teleop_rejected(self):
        self.sm.transition(SystemState.MOTION_PLAN_IDLE)
        self.sm.transition(SystemState.MOTION_PLAN_EXECUTING)
        self.assertFalse(self.sm.transition(SystemState.TELEOP_PASSIVE))
        self.assertIs(self.sm.state, SystemState.MOTION_PLAN_EXECUTING)

    def test_passive_to_executing_rejected(self):
        self.sm.transition(SystemState.TELEOP_PASSIVE)
        self.assertFalse(self.sm.transition(SystemState.MOTION_PLAN_EXECUTING))
        self.assertIs(self.sm.state, SystemState.TELEOP_PASSIVE)

    def test_state_unchanged_after_rejection(self):
        """State must never change on a rejected transition."""
        self.sm.transition(SystemState.TELEOP_PASSIVE)
        before = self.sm.state
        self.sm.transition(SystemState.MOTION_PLAN_EXECUTING)  # invalid
        self.assertIs(self.sm.state, before)


class TestForceFlag(unittest.TestCase):
    def test_force_bypasses_check(self):
        sm = StateMachine()
        sm.transition(SystemState.MOTION_PLAN_IDLE)
        sm.transition(SystemState.MOTION_PLAN_EXECUTING)
        # Normal: teleop from executing is blocked
        self.assertFalse(sm.transition(SystemState.TELEOP_PASSIVE))
        # Force: emergency stop to IDLE always allowed
        self.assertTrue(sm.transition(SystemState.IDLE, force=True))
        self.assertTrue(sm.is_idle())


class TestListeners(unittest.TestCase):
    def test_listener_called_on_success(self):
        sm  = StateMachine()
        log = []
        sm.add_listener(lambda old, new: log.append((old, new)))
        sm.transition(SystemState.TELEOP_PASSIVE)
        self.assertEqual(log, [(SystemState.IDLE, SystemState.TELEOP_PASSIVE)])

    def test_listener_not_called_on_rejection(self):
        sm  = StateMachine()
        log = []
        sm.add_listener(lambda old, new: log.append((old, new)))
        sm.transition(SystemState.TELEOP_ACTIVE)  # invalid from IDLE
        self.assertEqual(log, [])

    def test_multiple_listeners_all_called(self):
        sm   = StateMachine()
        a, b = [], []
        sm.add_listener(lambda *args: a.append(args))
        sm.add_listener(lambda *args: b.append(args))
        sm.transition(SystemState.TELEOP_PASSIVE)
        self.assertEqual(len(a), 1)
        self.assertEqual(len(b), 1)

    def test_listener_receives_both_states(self):
        sm  = StateMachine()
        log = []
        sm.add_listener(lambda old, new: log.append((old, new)))
        sm.transition(SystemState.TELEOP_PASSIVE)
        sm.transition(SystemState.TELEOP_ACTIVE)
        self.assertEqual(log[0], (SystemState.IDLE,          SystemState.TELEOP_PASSIVE))
        self.assertEqual(log[1], (SystemState.TELEOP_PASSIVE, SystemState.TELEOP_ACTIVE))

    def test_listener_exception_does_not_crash_state_machine(self):
        sm = StateMachine()
        sm.add_listener(lambda old, new: 1/0)   # raises ZeroDivisionError
        # Should not raise, just print
        self.assertTrue(sm.transition(SystemState.TELEOP_PASSIVE))
        self.assertIs(sm.state, SystemState.TELEOP_PASSIVE)


if __name__ == "__main__":
    unittest.main(verbosity=2)
