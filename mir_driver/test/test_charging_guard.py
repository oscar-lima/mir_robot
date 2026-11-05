#!/usr/bin/env python3
"""Offline acceptance matrix of the charging guard that mir_bridge and the mobipick_api bridge use."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'src'))
from mir_driver.charging_guard import CHARGING, CLEAR, UNKNOWN, ChargingGuard, parse_sources  # noqa: E402

READY = {'robotState': 3, 'robotStateString': 'Ready'}
DOCKED = {'robotState': 8, 'robotStateString': 'Docked'}


class Clock(object):
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


class TestChargingGuard(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        self.guard = ChargingGuard(5.0, now=self.clock)
        self.both = ChargingGuard(5.0, required='robot_state,charging_relay', now=self.clock)

    def test_startup_is_unknown_without_grace(self):
        self.assertEqual(
            self.guard.state(), (UNKNOWN, 'navigation refused: charging state unknown, no robot_state from the MiR for 0 s')
        )
        self.clock.t += 2
        self.assertIn('for 2 s', self.guard.refusal())

    def test_fresh_negative_is_clear(self):
        self.guard.on_robot_state(READY)
        self.assertEqual(self.guard.state(), (CLEAR, ''))
        self.assertIsNone(self.guard.refusal())

    def test_docked_is_charging(self):
        self.guard.on_robot_state(DOCKED)
        self.assertEqual(
            self.guard.state(),
            (CHARGING, 'navigation refused: the robot is charging (MiR state Docked); undock or unplug first'),
        )
        self.assertTrue(self.guard.charging())

    def test_stale_required_source_is_unknown(self):
        self.guard.on_robot_state(READY)
        self.clock.t += 7
        self.assertEqual(
            self.guard.state(), (UNKNOWN, 'navigation refused: charging state unknown, no robot_state from the MiR for 7 s')
        )
        self.assertFalse(self.guard.charging())  # unknown is never the charging fact

    def test_both_required_fresh_negatives_clear(self):
        self.both.on_robot_state(READY)
        self.both.on_charging_state({'charging_relay': False})
        self.assertEqual(self.both.state(), (CLEAR, ''))

    def test_relay_positive_with_ready_is_charging(self):
        self.both.on_robot_state(READY)
        self.both.on_charging_state({'charging_relay': True})
        self.assertIn('(charging relay on)', self.both.refusal())

    def test_either_positive_blocks_even_if_not_required(self):
        self.guard.on_robot_state(READY)
        self.guard.on_charging_state({'charging_relay': True})
        self.assertEqual(self.guard.state()[0], CHARGING)

    def test_missing_relay_with_fresh_ready_is_unknown(self):
        self.both.on_robot_state(READY)
        self.assertEqual(
            self.both.refusal(),
            'navigation refused: charging state unknown, no LightCtrl/charging_state from the MiR for 0 s',
        )

    def test_stale_relay_with_fresh_ready_is_unknown(self):
        self.both.on_robot_state(READY)
        self.both.on_charging_state({'charging_relay': False})
        self.clock.t += 6
        self.both.on_robot_state(READY)
        self.assertIn('no LightCtrl/charging_state from the MiR for 6 s', self.both.refusal())

    def test_unpublished_required_topic_named(self):
        self.both.mark_unavailable('charging_relay', 'the MiR does not publish LightCtrl/charging_state')
        self.both.on_robot_state(READY)
        self.assertEqual(
            self.both.refusal(),
            'navigation refused: charging state unknown, the MiR does not publish LightCtrl/charging_state',
        )

    def test_reset_on_connection_loss(self):
        self.guard.on_robot_state(READY)
        self.guard.reset('the connection to the MiR was lost')
        self.assertEqual(
            self.guard.state(), (UNKNOWN, 'navigation refused: charging state unknown, the connection to the MiR was lost')
        )
        self.guard.on_robot_state(READY)
        self.assertEqual(self.guard.state()[0], CLEAR)

    def test_stale_positive_does_not_count_but_blocks_as_unknown(self):
        self.guard.on_robot_state(DOCKED)
        self.clock.t += 6
        self.assertEqual(self.guard.state()[0], UNKNOWN)

    def test_undock_clears(self):
        self.guard.on_robot_state(DOCKED)
        self.guard.on_robot_state(READY)
        self.assertEqual(self.guard.state()[0], CLEAR)

    def test_parse_sources(self):
        self.assertEqual(parse_sources('robot_state, charging_relay'), ('robot_state', 'charging_relay'))
        self.assertEqual(parse_sources(''), ('robot_state',))
        with self.assertRaises(ValueError):
            parse_sources('battery')


if __name__ == '__main__':
    unittest.main()
