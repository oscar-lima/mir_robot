#!/usr/bin/env python3
# Copyright (c) 2018-2022, Martin Günther (DFKI GmbH) and contributors
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Martin Günther

import rospy

import copy
import sys
from collections.abc import Iterable

from mir_driver import rosbridge
from mir_driver.charging_guard import CLEAR, SOURCE_TOPICS, ChargingGuard, parse_sources
from rospy_message_converter import message_converter

from actionlib import SimpleActionClient
import actionlib_msgs.msg
import diagnostic_msgs.msg
import dynamic_reconfigure.msg
import geometry_msgs.msg
import mir_actions.msg
import mir_msgs.msg
import move_base_msgs.msg
import nav_msgs.msg
import rosgraph_msgs.msg
import sdc21x0.msg
import sensor_msgs.msg
import std_msgs.msg
import tf2_msgs.msg
import visualization_msgs.msg

from collections import OrderedDict

tf_prefix = ''
static_transforms = OrderedDict()


class TopicConfig(object):
    def __init__(self, topic, topic_type, latch=False, dict_filter=None):
        self.topic = topic
        self.topic_type = topic_type
        self.latch = latch
        self.dict_filter = dict_filter


# remap mir_actions/MirMoveBaseAction => move_base_msgs/MoveBaseAction
def _move_base_goal_dict_filter(msg_dict):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['goal']['move_task'] = mir_actions.msg.MirMoveBaseGoal.GLOBAL_MOVE
    filtered_msg_dict['goal']['goal_dist_threshold'] = 0.25
    filtered_msg_dict['goal']['clear_costmaps'] = True
    return filtered_msg_dict


def _move_base_feedback_dict_filter(msg_dict):
    # filter out slots from the dict that are not in our message definition
    # e.g., MiRMoveBaseFeedback has the field "state", which MoveBaseFeedback doesn't
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['feedback'] = {
        key: msg_dict['feedback'][key] for key in move_base_msgs.msg.MoveBaseFeedback.__slots__
    }
    return filtered_msg_dict


def _move_base_result_dict_filter(msg_dict):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    filtered_msg_dict['result'] = {key: msg_dict['result'][key] for key in move_base_msgs.msg.MoveBaseResult.__slots__}
    return filtered_msg_dict


def _cmd_vel_dict_filter(msg_dict):
    """
    Convert Twist to TwistStamped.

    Convert a geometry_msgs/Twist message dict (as sent from the ROS side) to
    a geometry_msgs/TwistStamped message dict (as expected by the MiR on
    software version >=2.7).
    """
    header = std_msgs.msg.Header(frame_id='', stamp=rospy.Time.now())
    filtered_msg_dict = {
        'header': message_converter.convert_ros_message_to_dictionary(header),
        'twist': copy.deepcopy(msg_dict),
    }
    return filtered_msg_dict


def _tf_dict_filter(msg_dict):
    filtered_msg_dict = copy.deepcopy(msg_dict)
    for transform in filtered_msg_dict['transforms']:
        transform['child_frame_id'] = tf_prefix + '/' + transform['child_frame_id'].strip('/')
    return filtered_msg_dict


def _tf_static_dict_filter(msg_dict):
    """
    Cache tf_static messages (simulate latching).

    The tf_static topic needs special handling. Publishers on tf_static are *latched*, which means that the ROS master
    caches the last message that was sent by each publisher on that topic, and will forward it to new subscribers.
    However, since the mir_driver node appears to the ROS master as a single node with a single publisher on tf_static,
    and there are multiple actual publishers hiding behind it on the MiR side, only one of those messages will be
    cached. Therefore, we need to implement the caching ourselves and make sure that we always publish the full set of
    transforms as a single message.
    """
    global static_transforms

    # prepend tf_prefix
    filtered_msg_dict = _tf_dict_filter(msg_dict)

    # The following code was copied + modified from https://github.com/tradr-project/static_transform_mux .

    # Process the incoming transforms, merge them with our cache.
    for transform in filtered_msg_dict['transforms']:
        key = transform['child_frame_id']
        rospy.loginfo(
            "[%s] tf_static: updated transform %s->%s.",
            rospy.get_name(),
            transform['header']['frame_id'],
            transform['child_frame_id'],
        )
        static_transforms[key] = transform

    # Return the cached messages.
    filtered_msg_dict['transforms'] = static_transforms.values()
    rospy.loginfo(
        "[%s] tf_static: sent %i transforms: %s",
        rospy.get_name(),
        len(static_transforms),
        str(static_transforms.keys()),
    )
    return filtered_msg_dict


def _prepend_tf_prefix_dict_filter(msg_dict):
    # filtered_msg_dict = copy.deepcopy(msg_dict)
    if not isinstance(msg_dict, dict):  # can happen during recursion
        return
    for key, value in msg_dict.items():
        if key == 'header':
            try:
                # prepend frame_id
                frame_id = value['frame_id'].strip('/')
                if frame_id != 'map':
                    # prepend tf_prefix, then remove leading '/' (e.g., when tf_prefix is empty)
                    value['frame_id'] = (tf_prefix + '/' + frame_id).strip('/')
                else:
                    value['frame_id'] = frame_id

            except TypeError:
                pass  # value is not a dict
            except KeyError:
                pass  # value doesn't have key 'frame_id'
        elif isinstance(value, dict):
            _prepend_tf_prefix_dict_filter(value)
        elif isinstance(value, Iterable):  # an Iterable other than dict (e.g., a list)
            for item in value:
                _prepend_tf_prefix_dict_filter(item)
    return msg_dict


def _remove_tf_prefix_dict_filter(msg_dict):
    # filtered_msg_dict = copy.deepcopy(msg_dict)
    if not isinstance(msg_dict, dict):  # can happen during recursion
        return
    for key, value in msg_dict.items():
        if key == 'header':
            try:
                # remove frame_id
                s = value['frame_id'].strip('/')
                if s.find(tf_prefix) == 0:
                    value['frame_id'] = (s[len(tf_prefix) :]).strip('/')  # strip off tf_prefix, then strip leading '/'
            except TypeError:
                pass  # value is not a dict
            except KeyError:
                pass  # value doesn't have key 'frame_id'
        elif isinstance(value, dict):
            _remove_tf_prefix_dict_filter(value)
        elif isinstance(value, Iterable):  # an Iterable other than dict (e.g., a list)
            for item in value:
                _remove_tf_prefix_dict_filter(item)
    return msg_dict


# topics we want to publish to ROS (and subscribe to from the MiR)
PUB_TOPICS = [
    # TopicConfig('LightCtrl/bms_data', mir_msgs.msg.BMSData),
    TopicConfig('LightCtrl/charging_state', mir_msgs.msg.ChargingState),
    TopicConfig('LightCtrl/us_list', sensor_msgs.msg.Range),
    # TopicConfig('MC/battery_currents', mir_msgs.msg.BatteryCurrents),
    # TopicConfig('MC/battery_voltage', std_msgs.msg.Float64),
    TopicConfig('MC/currents', sdc21x0.msg.MotorCurrents),
    # TopicConfig('MC/encoders', sdc21x0.msg.StampedEncoders),
    TopicConfig('MissionController/CheckArea/visualization_marker', visualization_msgs.msg.Marker),
    # TopicConfig('MissionController/goal_position_guid', std_msgs.msg.String),
    # TopicConfig('MissionController/prompt_user', mir_msgs.msg.UserPrompt),
    TopicConfig('SickPLC/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    TopicConfig('SickPLC/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('active_mapping_guid', std_msgs.msg.String),
    TopicConfig('amcl_pose', geometry_msgs.msg.PoseWithCovarianceStamped),
    TopicConfig('b_raw_scan', sensor_msgs.msg.LaserScan),
    TopicConfig('b_scan', sensor_msgs.msg.LaserScan),
    TopicConfig('camera_floor/background', sensor_msgs.msg.PointCloud2),
    TopicConfig('camera_floor/depth/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    TopicConfig('camera_floor/depth/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('camera_floor/depth/points', sensor_msgs.msg.PointCloud2),
    TopicConfig('camera_floor/filter/visualization_marker', visualization_msgs.msg.Marker),
    TopicConfig('camera_floor/floor', sensor_msgs.msg.PointCloud2),
    TopicConfig('camera_floor/obstacles', sensor_msgs.msg.PointCloud2),
    TopicConfig('check_area/polygon', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('check_pose_area/polygon', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('data_events/area_events', mir_data_msgs.msg.AreaEventEvent),
    # TopicConfig('data_events/maps', mir_data_msgs.msg.MapEvent),
    # TopicConfig('data_events/positions', mir_data_msgs.msg.PositionEvent),
    # TopicConfig('data_events/registers', mir_data_msgs.msg.PLCRegisterEvent),
    # TopicConfig('data_events/sounds', mir_data_msgs.msg.SoundEvent),
    TopicConfig('diagnostics', diagnostic_msgs.msg.DiagnosticArray),
    TopicConfig('diagnostics_agg', diagnostic_msgs.msg.DiagnosticArray),
    TopicConfig('diagnostics_toplevel_state', diagnostic_msgs.msg.DiagnosticStatus),
    TopicConfig('f_raw_scan', sensor_msgs.msg.LaserScan),
    TopicConfig('f_scan', sensor_msgs.msg.LaserScan),
    TopicConfig('imu_data', sensor_msgs.msg.Imu),
    TopicConfig('laser_back/driver/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    TopicConfig('laser_back/driver/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('laser_front/driver/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    TopicConfig('laser_front/driver/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('localization_score', std_msgs.msg.Float64),
    TopicConfig('/map', nav_msgs.msg.OccupancyGrid, latch=True),
    TopicConfig('/map_metadata', nav_msgs.msg.MapMetaData),
    # TopicConfig('marker_tracking_node/feedback', mir_marker_tracking.msg.MarkerTrackingActionFeedback),
    # TopicConfig(
    #     'marker_tracking_node/laser_line_extract/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription
    # ),
    # TopicConfig('marker_tracking_node/laser_line_extract/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('marker_tracking_node/laser_line_extract/visualization_marker', visualization_msgs.msg.Marker),
    # TopicConfig('marker_tracking_node/result', mir_marker_tracking.msg.MarkerTrackingActionResult),
    # TopicConfig('marker_tracking_node/status', actionlib_msgs.msg.GoalStatusArray),
    # TopicConfig('mirEventTrigger/events', mir_msgs.msg.Events),
    TopicConfig('mir_amcl/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    TopicConfig('mir_amcl/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('mir_amcl/selected_points', sensor_msgs.msg.PointCloud2),
    TopicConfig('mir_log', rosgraph_msgs.msg.Log),
    # TopicConfig('mir_sound/sound_event', mir_msgs.msg.SoundEvent),
    TopicConfig('mir_status_msg', std_msgs.msg.String),
    # TopicConfig('mirspawn/node_events', mirSpawn.msg.LaunchItem),
    TopicConfig('mirwebapp/grid_map_metadata', mir_msgs.msg.LocalMapStat),
    TopicConfig('mirwebapp/laser_map_metadata', mir_msgs.msg.LocalMapStat),
    # TopicConfig('mirwebapp/web_path', mir_msgs.msg.WebPath),
    # really mir_actions/MirMoveBaseActionFeedback:
    TopicConfig(
        'move_base/feedback', move_base_msgs.msg.MoveBaseActionFeedback, dict_filter=_move_base_feedback_dict_filter
    ),
    # really mir_actions/MirMoveBaseActionResult:
    TopicConfig('move_base/result', move_base_msgs.msg.MoveBaseActionResult, dict_filter=_move_base_result_dict_filter),
    TopicConfig('move_base/status', actionlib_msgs.msg.GoalStatusArray),
    # TopicConfig('move_base_node/MIRPlannerROS/cost_cloud', sensor_msgs.msg.PointCloud2),
    # TopicConfig('move_base_node/MIRPlannerROS/global_plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/MIRPlannerROS/len_to_goal', std_msgs.msg.Float64),
    TopicConfig('move_base_node/MIRPlannerROS/local_plan', nav_msgs.msg.Path),
    # TopicConfig('move_base_node/MIRPlannerROS/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('move_base_node/MIRPlannerROS/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('move_base_node/MIRPlannerROS/updated_global_plan', mir_msgs.msg.PlanSegments),
    # TopicConfig('move_base_node/MIRPlannerROS/visualization_marker', visualization_msgs.msg.MarkerArray),
    TopicConfig('move_base_node/SBPLLatticePlanner/plan', nav_msgs.msg.Path),
    # TopicConfig(
    #   'move_base_node/SBPLLatticePlanner/sbpl_lattice_planner_stats', sbpl_lattice_planner.msg.SBPLLatticePlannerStats
    # ),
    # TopicConfig('move_base_node/SBPLLatticePlanner/visualization_marker', visualization_msgs.msg.MarkerArray),
    TopicConfig('move_base_node/current_goal', geometry_msgs.msg.PoseStamped),
    # TopicConfig('move_base_node/global_costmap/inflated_obstacles', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/global_costmap/obstacles', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/global_costmap/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('move_base_node/global_costmap/parameter_updates', dynamic_reconfigure.msg.Config),
    # TopicConfig('move_base_node/global_costmap/robot_footprint', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('move_base_node/global_costmap/unknown_space', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/global_plan', nav_msgs.msg.Path),
    TopicConfig('move_base_node/local_costmap/inflated_obstacles', nav_msgs.msg.GridCells),
    TopicConfig('move_base_node/local_costmap/obstacles', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/local_costmap/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('move_base_node/local_costmap/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('move_base_node/local_costmap/robot_footprint', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('move_base_node/local_costmap/safety_zone', geometry_msgs.msg.PolygonStamped),
    # TopicConfig('move_base_node/local_costmap/unknown_space', nav_msgs.msg.GridCells),
    # TopicConfig('move_base_node/mir_escape_recovery/visualization_marker', visualization_msgs.msg.Marker),
    # TopicConfig('move_base_node/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('move_base_node/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('move_base_node/time_to_coll', std_msgs.msg.Float64),
    TopicConfig('move_base_node/traffic_costmap/inflated_obstacles', nav_msgs.msg.GridCells),
    TopicConfig('move_base_node/traffic_costmap/obstacles', nav_msgs.msg.GridCells),
    TopicConfig('move_base_node/traffic_costmap/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    TopicConfig('move_base_node/traffic_costmap/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('move_base_node/traffic_costmap/robot_footprint', geometry_msgs.msg.PolygonStamped),
    TopicConfig('move_base_node/traffic_costmap/unknown_space', nav_msgs.msg.GridCells),
    TopicConfig('move_base_node/visualization_marker', visualization_msgs.msg.Marker),
    TopicConfig('move_base_simple/visualization_marker', visualization_msgs.msg.Marker),
    TopicConfig('odom', nav_msgs.msg.Odometry),
    TopicConfig('odom_enc', nav_msgs.msg.Odometry),
    # TopicConfig('one_way_map', nav_msgs.msg.OccupancyGrid),
    # TopicConfig('param_update', std_msgs.msg.String),
    # TopicConfig('particlevizmarker', visualization_msgs.msg.MarkerArray),
    # TopicConfig('resource_tracker/needed_resources', mir_msgs.msg.ResourcesState),
    TopicConfig('robot_mode', mir_msgs.msg.RobotMode),
    TopicConfig('robot_pose', geometry_msgs.msg.Pose),
    TopicConfig('robot_state', mir_msgs.msg.RobotState),
    # TopicConfig('robot_status', mir_msgs.msg.RobotStatus),
    TopicConfig('/rosout', rosgraph_msgs.msg.Log),
    TopicConfig('/rosout_agg', rosgraph_msgs.msg.Log),
    TopicConfig('scan', sensor_msgs.msg.LaserScan),
    # TopicConfig('scan_filter/parameter_descriptions', dynamic_reconfigure.msg.ConfigDescription),
    # TopicConfig('scan_filter/parameter_updates', dynamic_reconfigure.msg.Config),
    TopicConfig('scan_filter/visualization_marker', visualization_msgs.msg.Marker),
    # TopicConfig('session_importer_node/info', mirSessionImporter.msg.SessionImportInfo),
    # TopicConfig('set_mc_PID', std_msgs.msg.Float64MultiArray),
    TopicConfig('/tf', tf2_msgs.msg.TFMessage, dict_filter=_tf_dict_filter),
    TopicConfig('/tf_static', tf2_msgs.msg.TFMessage, dict_filter=_tf_static_dict_filter, latch=True),
    # TopicConfig('traffic_map', nav_msgs.msg.OccupancyGrid),
    # TopicConfig('wifi_diagnostics', diagnostic_msgs.msg.DiagnosticArray),
    # TopicConfig('wifi_diagnostics/cur_ap', mir_wifi_msgs.msg.APInfo),
    # TopicConfig('wifi_diagnostics/roam_events', mir_wifi_msgs.msg.WifiRoamEvent),
    # TopicConfig('wifi_diagnostics/wifi_ap_interface_stats', mir_wifi_msgs.msg.WifiInterfaceStats),
    # TopicConfig('wifi_diagnostics/wifi_ap_rssi', mir_wifi_msgs.msg.APRssiStats),
    # TopicConfig('wifi_diagnostics/wifi_ap_time_stats', mir_wifi_msgs.msg.APTimeStats),
    # TopicConfig('wifi_watchdog/ping', mir_wifi_msgs.msg.APPingStats),
]

# topics we want to subscribe to from ROS (and publish to the MiR)
SUB_TOPICS = [
    # really geometry_msgs.msg.TwistStamped:
    TopicConfig('cmd_vel', geometry_msgs.msg.Twist, dict_filter=_cmd_vel_dict_filter),
    TopicConfig('initialpose', geometry_msgs.msg.PoseWithCovarianceStamped),
    TopicConfig('light_cmd', std_msgs.msg.String),
    TopicConfig('mir_cmd', std_msgs.msg.String),
    TopicConfig('move_base/cancel', actionlib_msgs.msg.GoalID),
    # really mir_actions/MirMoveBaseActionGoal:
    TopicConfig('move_base/goal', move_base_msgs.msg.MoveBaseActionGoal, dict_filter=_move_base_goal_dict_filter),
]


class PublisherWrapper(rospy.SubscribeListener):
    def __init__(self, topic_config, robot):
        self.topic_config = topic_config
        self.robot = robot
        self.connected = False
        # Use topic_config.topic directly here. If it does not have a leading slash, it will use the private namespace.
        self.pub = rospy.Publisher(
            name=topic_config.topic,
            data_class=topic_config.topic_type,
            subscriber_listener=self,
            latch=topic_config.latch,
            queue_size=10,
        )
        rospy.loginfo(
            "[%s] publishing topic '%s' [%s]", rospy.get_name(), topic_config.topic, topic_config.topic_type._type
        )
        # latched topics must be subscribed immediately
        if topic_config.latch:
            self.peer_subscribe("", None, None)

    def peer_subscribe(self, topic_name, topic_publish, peer_publish):
        if not self.connected:
            self.connected = True
            rospy.loginfo("[%s] starting to stream messages on topic '%s'", rospy.get_name(), self.topic_config.topic)
            absolute_topic = '/' + self.topic_config.topic.lstrip('/')  # ensure exactly 1 leading slash for MiR comm
            self.robot.subscribe(topic=absolute_topic, callback=self.callback)

    def peer_unsubscribe(self, topic_name, num_peers):
        pass
        # doesn't work: once ubsubscribed, robot doesn't let us subscribe again
        # if self.connected and self.pub.get_num_connections() == 0 and not self.topic_config.latch:
        #     self.connected = False
        #     rospy.loginfo("[%s] stopping to stream messages on topic '%s'", rospy.get_name(), self.topic_config.topic)
        #     absolute_topic = '/' + self.topic_config.topic.lstrip('/')  # ensure exactly 1 leading slash for MiR comm
        #     self.robot.unsubscribe(topic=absolute_topic)

    def callback(self, msg_dict):
        msg_dict = _prepend_tf_prefix_dict_filter(msg_dict)
        if self.topic_config.dict_filter is not None:
            msg_dict = self.topic_config.dict_filter(msg_dict)
        msg = message_converter.convert_dictionary_to_ros_message(self.topic_config.topic_type._type, msg_dict)
        self.pub.publish(msg)


class SubscriberWrapper(object):
    def __init__(self, topic_config, robot, allow=None):
        self.topic_config = topic_config
        self.robot = robot
        # allow(msg) -> bool: False drops the message instead of forwarding it to the MiR
        self.allow = allow
        # Use topic_config.topic directly here. If it does not have a leading slash, it will use the private namespace.
        self.sub = rospy.Subscriber(
            name=topic_config.topic, data_class=topic_config.topic_type, callback=self.callback, queue_size=10
        )
        rospy.loginfo(
            "[%s] subscribing to topic '%s' [%s]", rospy.get_name(), topic_config.topic, topic_config.topic_type._type
        )

    def callback(self, msg):
        if self.allow is not None and not self.allow(msg):
            return
        self.forward(msg)

    def forward(self, msg):
        msg_dict = message_converter.convert_ros_message_to_dictionary(msg)
        msg_dict = _remove_tf_prefix_dict_filter(msg_dict)
        if self.topic_config.dict_filter is not None:
            msg_dict = self.topic_config.dict_filter(msg_dict)
        absolute_topic = '/' + self.topic_config.topic.lstrip('/')  # ensure exactly 1 leading slash for MiR comm
        self.robot.publish(absolute_topic, msg_dict)


class MiRBridge(object):
    def __init__(self):
        try:
            hostname = rospy.get_param('~hostname')
        except KeyError:
            rospy.logfatal('[%s] parameter "hostname" is not set!', rospy.get_name())
            sys.exit(-1)
        port = rospy.get_param('~port', 9090)

        global tf_prefix
        tf_prefix = rospy.get_param('~tf_prefix', '').strip('/')

        rospy.loginfo('[%s] trying to connect to %s:%i...', rospy.get_name(), hostname, port)
        self.robot = rosbridge.RosbridgeSetup(hostname, port)

        r = rospy.Rate(10)
        i = 1
        while not self.robot.is_connected():
            if rospy.is_shutdown():
                sys.exit(0)
            if self.robot.is_errored():
                rospy.logfatal('[%s] connection error to %s:%i, giving up!', rospy.get_name(), hostname, port)
                sys.exit(-1)
            if i % 10 == 0:
                rospy.logwarn('[%s] still waiting for connection to %s:%i...', rospy.get_name(), hostname, port)
            i += 1
            r.sleep()

        rospy.loginfo('[%s] ... connected.', rospy.get_name())

        topics = self.get_topics()
        published_topics = [topic_name for (topic_name, _, has_publishers, _) in topics if has_publishers]
        subscribed_topics = [topic_name for (topic_name, _, _, has_subscribers) in topics if has_subscribers]

        for pub_topic in PUB_TOPICS:
            PublisherWrapper(pub_topic, self.robot)
            absolute_topic = '/' + pub_topic.topic.lstrip('/')  # ensure exactly 1 leading slash for MiR comm
            if absolute_topic not in published_topics:
                rospy.logwarn("[%s] topic '%s' is not published by the MiR!", rospy.get_name(), pub_topic.topic)

        self._wrappers = {}
        self._setup_charging_guard(published_topics)
        allow = {'cmd_vel': self._allow_cmd_vel, 'move_base/goal': self._allow_move_base_goal, 'mir_cmd': self._allow_mir_cmd}

        for sub_topic in SUB_TOPICS:
            self._wrappers[sub_topic.topic] = SubscriberWrapper(sub_topic, self.robot, allow.get(sub_topic.topic))
            absolute_topic = '/' + sub_topic.topic.lstrip('/')  # ensure exactly 1 leading slash for MiR comm
            if absolute_topic not in subscribed_topics:
                rospy.logwarn("[%s] topic '%s' is not yet subscribed to by the MiR!", rospy.get_name(), sub_topic.topic)

        # At least with software version 2.8 there were issues when forwarding a simple goal to the robot
        # This workaround converts it into an action. Check https://github.com/DFKI-NI/mir_robot/issues/60 for details.
        self._move_base_client = SimpleActionClient('move_base', move_base_msgs.msg.MoveBaseAction)
        rospy.Subscriber("move_base_simple/goal", geometry_msgs.msg.PoseStamped, self._move_base_simple_goal_callback)

    def _setup_charging_guard(self, published_topics):
        """Refuse base motion while the MiR charges or its charging state is unknown (~block_nav_while_charging).

        While blocked: move_base goals get a local ABORTED result with the reason and are not forwarded, cmd_vel and
        mir_cmd (opaque MiR commands, treated as motion-capable) are dropped; a cancel and a zero twist always pass.
        On the transition into blocked the bridge sends the MiR a cancel of all goals and one zero twist, so a goal
        forwarded before keeps no running. The flag, ~charging_sources (the required sources, default robot_state) and
        ~charging_state_timeout are read again every second, so they switch at runtime (rosparam set) without a
        restart. Outside this block: whatever moves the MiR without this bridge (its web interface, its joystick).
        """
        self._charging_guard = None
        self._blocked = False
        self._block_nav_while_charging = False
        self._published_topics = set(published_topics)
        self._nav_result_pub = rospy.Publisher('move_base/result', move_base_msgs.msg.MoveBaseActionResult, queue_size=10)
        self._guard_ticks = 0
        self._read_block_flag()
        rospy.Timer(rospy.Duration(0.2), self._check_charging_guard)

    def _read_block_flag(self):
        try:
            enabled = bool(rospy.get_param('~block_nav_while_charging', False))
            required = parse_sources(rospy.get_param('~charging_sources', 'robot_state'))
            timeout = float(rospy.get_param('~charging_state_timeout', 5.0))
        except ValueError as exc:
            rospy.logerr_throttle(10.0, '[%s] charging guard: %s; keeping the last settings' % (rospy.get_name(), exc))
            return
        except Exception:  # master gone: keep the last values
            return
        if enabled and self._charging_guard is None:
            self._charging_guard = ChargingGuard(timeout, required)
            # our own subscriptions on the MiR side, independent of whether any ROS node listens to these topics
            self.robot.subscribe(topic='/robot_state', callback=self._charging_guard.on_robot_state)
            if '/LightCtrl/charging_state' in self._published_topics:
                self.robot.subscribe(topic='/LightCtrl/charging_state', callback=self._charging_guard.on_charging_state)
        if self._charging_guard is not None:
            guard = self._charging_guard
            if (required, timeout) != (guard.required, guard.timeout):
                rospy.loginfo('[%s] charging guard: required sources %s, stale after %.1f s', rospy.get_name(),
                              ', '.join(required), timeout)
            guard.required, guard.timeout = required, timeout
            for source in required:
                if '/' + SOURCE_TOPICS[source] not in self._published_topics:
                    guard.mark_unavailable(source, 'the MiR does not publish %s' % SOURCE_TOPICS[source])
        if enabled != self._block_nav_while_charging:
            rospy.loginfo("[%s] block_nav_while_charging %s", rospy.get_name(), 'on' if enabled else 'off')
        self._block_nav_while_charging = enabled

    def _check_charging_guard(self, _event=None):
        self._guard_ticks += 1
        if self._guard_ticks % 5 == 1:
            self._read_block_flag()
        guard = self._charging_guard
        if guard is not None and not self.robot.is_connected() and not guard.is_reset():
            guard.reset('the connection to the MiR was lost')
        reason = self._nav_refusal()
        blocked = reason is not None
        if blocked and not self._blocked:
            rospy.logwarn('[%s] %s: cancelling all MiR goals and stopping the base', rospy.get_name(), reason)
            self._stop_mir()
        elif not blocked and self._blocked:
            rospy.loginfo('[%s] charging guard clear: base motion allowed again', rospy.get_name())
        self._blocked = blocked

    def _stop_mir(self):
        """Cancel every MiR goal and send one zero twist (both pass the gate)."""
        self._wrappers['move_base/cancel'].forward(actionlib_msgs.msg.GoalID())  # empty id = cancel all goals
        self._wrappers['cmd_vel'].forward(geometry_msgs.msg.Twist())

    def _nav_refusal(self):
        if not self._block_nav_while_charging or self._charging_guard is None:
            return None
        return self._charging_guard.refusal()

    def _allow_move_base_goal(self, msg):
        reason = self._nav_refusal()
        if reason is None:
            return True
        rospy.logwarn("[%s] %s (move_base goal '%s' not forwarded to the MiR)", rospy.get_name(), reason, msg.goal_id.id)
        result = move_base_msgs.msg.MoveBaseActionResult()
        result.header.stamp = rospy.Time.now()
        result.status.goal_id = msg.goal_id
        result.status.status = actionlib_msgs.msg.GoalStatus.ABORTED
        result.status.text = reason
        self._nav_result_pub.publish(result)
        return False

    def _allow_cmd_vel(self, msg):
        twist = getattr(msg, 'twist', msg)  # Twist or TwistStamped
        if not any((twist.linear.x, twist.linear.y, twist.linear.z, twist.angular.x, twist.angular.y, twist.angular.z)):
            return True  # a stop command is always forwarded
        reason = self._nav_refusal()
        if reason is None:
            return True
        rospy.logwarn_throttle(5.0, '[%s] %s (cmd_vel dropped)' % (rospy.get_name(), reason))
        return False

    def _allow_mir_cmd(self, msg):
        reason = self._nav_refusal()
        if reason is None:
            return True
        rospy.logwarn("[%s] %s (mir_cmd '%s' dropped)", rospy.get_name(), reason, msg.data)
        return False

    def get_topics(self):
        srv_response = self.robot.callService('/rosapi/topics', msg={})
        topic_names = sorted(srv_response['topics'])
        topics = []

        for topic_name in topic_names:
            srv_response = self.robot.callService("/rosapi/topic_type", msg={'topic': topic_name})
            topic_type = srv_response['type']

            srv_response = self.robot.callService("/rosapi/publishers", msg={'topic': topic_name})
            has_publishers = True if len(srv_response['publishers']) > 0 else False

            srv_response = self.robot.callService("/rosapi/subscribers", msg={'topic': topic_name})
            has_subscribers = True if len(srv_response['subscribers']) > 0 else False

            topics.append([topic_name, topic_type, has_publishers, has_subscribers])

        print('Publishers:')
        for topic_name, topic_type, has_publishers, has_subscribers in topics:
            if has_publishers:
                print((' * %s [%s]' % (topic_name, topic_type)))

        print('\nSubscribers:')
        for topic_name, topic_type, has_publishers, has_subscribers in topics:
            if has_subscribers:
                print((' * %s [%s]' % (topic_name, topic_type)))

        return topics

    def _move_base_simple_goal_callback(self, msg):
        if not self._move_base_client.wait_for_server(rospy.Duration(2)):
            rospy.logwarn("Could not connect to 'move_base' server after two seconds. Dropping goal.")
            rospy.logwarn("Did you activate 'planner' in the MIR web interface?")
            return
        goal = move_base_msgs.msg.MoveBaseGoal()
        goal.target_pose.header = copy.deepcopy(msg.header)
        goal.target_pose.pose = copy.deepcopy(msg.pose)
        self._move_base_client.send_goal(goal)


def main():
    rospy.init_node('mir_bridge')
    MiRBridge()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
