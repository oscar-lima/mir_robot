"""Whether the MiR charges, for refusing base motion meanwhile: one guard for mir_bridge and its other users.

Sources (mir_msgs topics of the MiR):

- ``robot_state`` (RobotState): ``robotState`` ROBOT_STATE_DOCKED means it sits in the dock and charges its batteries;
- ``charging_relay`` (``LightCtrl/charging_state``, ChargingState): ``charging_relay`` on.

``state()`` is CHARGING when any source reports charging in a fresh message, else UNKNOWN when a *required* source has
sent nothing yet, is stale (no message for ``timeout`` seconds), is not published by the MiR, or the connection was
lost; else CLEAR. Fail-safe: there is no grace period at the start, and a fresh Ready never clears a missing required
source. Which sources are required is set by the caller from what the real MiR is known to publish.

Receipt times use a monotonic clock (not ROS time, so a slow simulation clock or a time jump cannot make a message
look fresh or stale). The class holds no ROS objects so it can be tested without a ROS master; callers feed it the
message dictionaries.
"""

import time

ROBOT_STATE_DOCKED = 8
CHARGING, CLEAR, UNKNOWN = 'charging', 'clear', 'unknown'
SOURCE_TOPICS = {'robot_state': 'robot_state', 'charging_relay': 'LightCtrl/charging_state'}


def parse_sources(value):
    """'robot_state, charging_relay' or a list -> a tuple of known source names (ValueError for an unknown one)."""
    names = value.replace(',', ' ').split() if isinstance(value, str) else list(value)
    for name in names:
        if name not in SOURCE_TOPICS:
            raise ValueError('unknown charging source %r (known: %s)' % (name, ', '.join(sorted(SOURCE_TOPICS))))
    return tuple(names) or ('robot_state',)


class ChargingGuard(object):
    def __init__(self, timeout=5.0, required=('robot_state',), now=time.monotonic):
        self.timeout = float(timeout)
        self.required = parse_sources(required)
        self._now = now
        self._unavailable = {}
        self.reset()

    def reset(self, why=''):
        """Forget everything (bridge start, connection lost): UNKNOWN until fresh messages arrive again."""
        self._since = self._now()
        self._reset_why = why
        self._stamp = {}  # source -> monotonic receipt time
        self._positive = {}  # source -> reported charging
        self._docked_string = ''

    def is_reset(self):
        """True after reset(why) until the next message."""
        return bool(self._reset_why)

    def mark_unavailable(self, source, why):
        self._unavailable[source] = why

    def on_robot_state(self, msg_dict):
        docked = int(msg_dict.get('robotState', -1)) == ROBOT_STATE_DOCKED
        self._docked_string = str(msg_dict.get('robotStateString', '') or 'Docked') if docked else ''
        self._seen('robot_state', docked)

    def on_charging_state(self, msg_dict):
        self._seen('charging_relay', bool(msg_dict.get('charging_relay', False)))

    def _seen(self, source, positive):
        self._stamp[source] = self._now()
        self._positive[source] = positive
        self._reset_why = ''

    def _fresh(self, source):
        stamp = self._stamp.get(source)
        return stamp is not None and self._now() - stamp <= self.timeout

    def state(self):
        """(CHARGING | CLEAR | UNKNOWN, reason); the reason is '' for CLEAR."""
        reasons = []
        if self._fresh('robot_state') and self._positive.get('robot_state'):
            reasons.append('MiR state %s' % self._docked_string)
        if self._fresh('charging_relay') and self._positive.get('charging_relay'):
            reasons.append('charging relay on')
        if reasons:
            return CHARGING, 'navigation refused: the robot is charging (%s); undock or unplug first' % ', '.join(reasons)
        for source in self.required:
            if self._fresh(source):
                continue
            topic = SOURCE_TOPICS[source]
            if source in self._unavailable:
                why = self._unavailable[source]
            elif self._reset_why:
                why = self._reset_why
            else:
                last = self._stamp.get(source, self._since)
                why = 'no %s from the MiR for %.0f s' % (topic, self._now() - last)
            return UNKNOWN, 'navigation refused: charging state unknown, %s' % why
        return CLEAR, ''

    def charging(self):
        return self.state()[0] == CHARGING

    def refusal(self):
        """None when the base may move, else the human-readable reason why not."""
        state, reason = self.state()
        return None if state == CLEAR else reason
