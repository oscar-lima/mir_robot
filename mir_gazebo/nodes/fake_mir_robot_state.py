#!/usr/bin/env python3
"""Simulated twin of the MiR's charging sources, which mir_bridge forwards on the real robot.

Publishes ``robot_state`` (mir_msgs/RobotState: ROBOT_STATE_READY, or ROBOT_STATE_DOCKED, the real MiR's "in the dock
and charging") and ``LightCtrl/charging_state`` (mir_msgs/ChargingState, ``charging_relay`` on while docked) at
``~rate`` Hz of wall-clock time, so a slow simulation clock does not make them look stale. The simulation has no dock
and no charger, so the docked state is only set by hand, to test how the software reacts to a charging robot:

    rosservice call <namespace>/fake_mir_robot_state/set_docked "data: true"

Parameters (private): ``~docked`` (start state, default false), ``~rate`` (default 1.0 Hz).
"""

import threading

import rospy
from mir_msgs.msg import ChargingState, RobotState
from std_srvs.srv import SetBool, SetBoolResponse


class FakeMirRobotState(object):
    def __init__(self):
        self.docked = bool(rospy.get_param('~docked', False))
        self.period = 1.0 / max(float(rospy.get_param('~rate', 1.0)), 0.1)
        self.state_pub = rospy.Publisher('robot_state', RobotState, queue_size=1, latch=True)
        self.relay_pub = rospy.Publisher('LightCtrl/charging_state', ChargingState, queue_size=1, latch=True)
        self.wake = threading.Event()
        rospy.Service('~set_docked', SetBool, self.set_docked)

    def publish(self):
        state = RobotState()
        if self.docked:
            state.robotState, state.robotStateString = RobotState.ROBOT_STATE_DOCKED, 'Docked'
        else:
            state.robotState, state.robotStateString = RobotState.ROBOT_STATE_READY, 'Ready'
        self.state_pub.publish(state)
        self.relay_pub.publish(ChargingState(charging_relay=self.docked))

    def spin(self):
        while not rospy.is_shutdown():
            self.publish()
            self.wake.wait(self.period)  # wall clock; set_docked publishes at once
            self.wake.clear()

    def set_docked(self, req):
        self.docked = bool(req.data)
        self.wake.set()
        state = 'Docked (charging)' if self.docked else 'Ready'
        rospy.loginfo('[%s] simulated MiR robot_state: %s', rospy.get_name(), state)
        return SetBoolResponse(success=True, message='robot_state ' + state)


if __name__ == '__main__':
    rospy.init_node('fake_mir_robot_state')
    FakeMirRobotState().spin()
