#!/usr/bin/python3
import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, Float64, Bool
from nav_msgs.msg import Odometry
from smarc_msgs.msg import ThrusterRPM, PercentStamped
from smarc_control_msgs.msg import Topics as ControlTopics
from sam_msgs.msg import Topics as SamTopics
from sam_msgs.msg import ThrusterAngles, ThrusterRPMs

from sam_diving_controller.IDivePub import ActuatorStates

from .ParamUtils import DivingModelParam

try:
    from .IDivePub import IDivePub, MissionStates
except:
    from IDivePub import IDivePub, MissionStates

class SimPub(IDivePub):
    """
    Implements the simple interface we defined in IDiveView for the SAM AUV.
    """
    def __init__(self, node: Node, dive_sub, param) -> None:

        self._node = node

        self._robot_name = self._node.get_parameter('robot_name').get_parameter_value().string_value

        #self._state_pub = node.create_publisher(ControlState, ControlTopics.STATES_CONV, 10)
        self._state_pub = node.create_publisher(Odometry, 'sim/state', 10)

        self._state_msg = None

        self._node = node
        self._dive_sub = dive_sub

    def _loginfo(self, s):
        self._node.get_logger().info(s)

    def _update_state(self) -> None:
        if self._state_msg is None:
            return

        self._state_pub.publish(self._state_msg)

    def set_state(self, msg):
        self._state_msg = msg

    def update(self) -> None:
        """
        Publish all actuator values
        """
        self._update_state()
