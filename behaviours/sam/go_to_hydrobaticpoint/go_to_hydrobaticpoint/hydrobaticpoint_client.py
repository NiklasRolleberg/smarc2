import rclpy
from action_msgs.msg import GoalStatus
from geographic_msgs.msg import GeoPoint
from rclpy.action import CancelResponse
from rclpy.action.client import ClientGoalHandle
from rclpy.node import Node
from smarc_action_base.smarc_action_base import (
    ActionFeedback,
    ActionResult,
    ActionType,
    SMARCActionClient,
    ActionClientState,
)
from smarc_msgs.action import BaseAction
from geometry_msgs.msg import Pose, PoseStamped
from smarc_control_msgs.msg import Topics as ControlTopics
from std_msgs.msg import String

from go_to_hydrobaticpoint.hydrobaticpoint_action import ActionComponent as ActC
from go_to_hydrobaticpoint.hydrobaticpoint_action import HydrobaticPointAction


class HydropointClient(SMARCActionClient):
    def __init__(self, node: Node, action_name: str, action_type: ActionType, **kwargs):
        super().__init__(node, action_name, action_type)
        self.logger = self._node.get_logger()
        self.declare_parameters()
        self._json_ops = HydrobaticPointAction()
        self.logger.set_level(rclpy.logging.LoggingSeverity.INFO)

        self.goal_processed = False
        self._send_latch = False
        self._last_goal_sig = None
        self._last_pose = None  # cache newest mocap goal until server is ready

        # watchdog to unlock latch if we never get a response
        self._watchdog_timer = None
        self._send_timeout_sec = 2.0

        self.logger.info(f"Node {action_name} connected to go_to_hydropoint server")

    def run(self):
        self.logger.info("Subscribing to mocap hydro point topic")
        # QoS tuned for high-rate mocap
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        #self.mocap_goal_sub = self._node.create_subscription(
        #    PoseStamped, ControlTopics.MOCAP_HYDROPOINT, self.mocap_hydro_cb, qos
        #)
        self.mocap_goal_sub = self._node.create_subscription(PoseStamped, 
                                                             '/mqtt/hula/pose',
                                                             self.mocap_hydro_cb, 1)

        # Periodically check server availability and send cached goal if waiting
        self._server_check_timer = self._node.create_timer(0.2, self._try_send_cached_if_ready)

    # ---------- internals ----------
    def declare_parameters(self):
        """Location to declare parameters."""
        pass
    
    def _server_ready(self) -> bool:
        # Non-blocking readiness probe. Your SMARCActionClient should expose _client like rclpy ActionClient.
        try:
            return self._client.wait_for_server(timeout_sec=0.0)
        except Exception:
            return False

    def _sig_pose(self, p: PoseStamped) -> str:
        import hashlib
        pos = (p.pose.position.x, p.pose.position.y, p.pose.position.z)
        ori = (p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w)
        frame = p.header.frame_id
        return hashlib.sha1(f"{pos}|{ori}|{frame}".encode("utf-8")).hexdigest()

    def _start_watchdog(self):
        if self._watchdog_timer:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None

        def _timeout_cb():
            if not self.goal_processed:
                self.logger.warn("Goal response timeout; allowing resend.")
                self._send_latch = False
            if self._watchdog_timer:
                self._watchdog_timer.cancel()
        self._watchdog_timer = self._node.create_timer(self._send_timeout_sec, _timeout_cb)

    def _cancel_watchdog(self):
        if self._watchdog_timer:
            self._watchdog_timer.cancel()
            self._watchdog_timer = None

    def _try_send_cached_if_ready(self):
        if self.goal_processed or self._send_latch:
            return
        if self._last_pose is None:
            return
        if not self._server_ready():
            return
        # Server is up; send once now
        self._do_send(self._last_pose)

    def _do_send(self, mocap_goal: PoseStamped):
        # Latch AFTER confirming server readiness to avoid “sent into the void”
        self._send_latch = True

        # DEBUG ONLY!!
        #mocap_goal.pose.position.x = 3.0

        self.logger.info(f"[HydropointClient] Sending goal (frame={mocap_goal.header.frame_id})")
        goal_msg = BaseAction.Goal()


        goal_msg.goal = self._json_ops.encode(mocap_goal)

        # IMPORTANT: ensure your SMARCActionClient wires callbacks through send_goal
        # Under the hood it should call rclpy ActionClient.send_goal_async with:
        #   feedback_callback=self.feedback_callback
        #   and add_done_callback to invoke goal_response_callback
        self.send_goal(goal_msg)

        self._start_watchdog()

    # ---------- callbacks ----------
    def mocap_hydro_cb(self, mocap_goal: PoseStamped):
        if self.goal_processed:
            return

        self._last_pose = mocap_goal  # always keep the freshest goal
        sig = self._sig_pose(mocap_goal)
        if sig == self._last_goal_sig:
            return  # identical data; ignore
        self._last_goal_sig = sig

        if self._send_latch:
            return  # already attempting a send

        # Only send immediately if server is available; otherwise we'll send from _try_send_cached_if_ready
        if self._server_ready():
            self._do_send(mocap_goal)
        else:
            self.logger.info("Action server not ready yet; caching goal and waiting...")

    def goal_response_callback(self, goal_handle: ClientGoalHandle):
        # This MUST be invoked by your wrapper; add logging to verify it fires.
        if not goal_handle.accepted:
            self.logger.info("Goal was NOT accepted by server.")
            self._send_latch = False     # allow resend on next mocap update / server-ready tick
            self._cancel_watchdog()
            return

        self.logger.info("Goal was accepted by server.")
        self.goal_processed = True
        self._send_latch = False
        self._cancel_watchdog()

        # optional: stop listening to mocap after acceptance
        if hasattr(self, "mocap_goal_sub"):
            self._node.destroy_subscription(self.mocap_goal_sub)

    def feedback_callback(self, feedback_msg: ActionFeedback):
        self.logger.debug(f"Received feedback {feedback_msg.feedback}")
        self.dist_rem = self._json_ops.decode(feedback_msg.feedback, ActC.FEEDBACK)

    def result_callback(self, result: ActionResult, status: GoalStatus):
        self.logger.info(f"Waypoint reached boolean: {result}")
        if result.success:
            return self.get_goal_success()
        else:
            return self.get_goal_error()
            

#class HydropointClient(SMARCActionClient):
#    """Client for sending Geopoint message requests to vehicles.
#
#    Attributes:
#        logger: shorthand for `node.get_logger()`
#    """
#
#    def __init__(
#        self,
#        node: Node,
#        action_name: str,
#        action_type: ActionType,
#        **kwargs,
#    ):
#        super().__init__(node, action_name, action_type)
#        self.logger = self._node.get_logger()
#        self.declare_parameters()
#        self._json_ops = HydrobaticPointAction()
#        self.logger.set_level(rclpy.logging.LoggingSeverity.INFO)
#        self.goal_processed = False
#
#        # Wait for server
#        # while not self._client.wait_for_server(timeout_sec=1.) and rclpy.ok():
#        #     self.logger.info(f"Node {action_name} waiting for go_to_hydropoint server")
#
#        self.logger.info(f"Node {action_name} connected to go_to_hydropoint server")
#
#
#    def run(self):
#        self.logger.info("Subscribing to mocap hydro point topic")
#        #self.mocap_goal_sub = self._node.create_subscription(PoseStamped, 
#        #                                                     ControlTopics.MOCAP_HYDROPOINT,
#        #                                                     self.mocap_hydro_cb, 1)
#        self.mocap_goal_sub = self._node.create_subscription(PoseStamped, 
#                                                             '/mqtt/hula/pose',
#                                                             self.mqtt_hydro_cb, 1)
#
#
#    def mqtt_hydro_cb(self, mqtt_goal: String):
#
#        if not self.goal_processed:
#
#            if self.state != ActionClientState.SENT:
#
#                if self.state == ActionClientState.ACCEPTED or self.state == ActionClientState.RUNNING:
#                    self.goal_processed = True
#                    self._node.destroy_subscription(self.mocap_goal_sub)
#                    return
#
#                self.logger.info(f"Sending goal {mqtt_goal}")
#                goal_msg = BaseAction.Goal()
#                goal_msg.goal = self._json_ops.encode(mqtt_goal)
#                self.send_goal(goal_msg)
#
#
#    def mocap_hydro_cb(self, mocap_goal: PoseStamped):
#
#        if not self.goal_processed:
#
#            if self.state != ActionClientState.SENT:
#
#                if self.state == ActionClientState.ACCEPTED or self.state == ActionClientState.RUNNING:
#                    self.goal_processed = True
#                    self._node.destroy_subscription(self.mocap_goal_sub)
#                    return
#
#                self.logger.info(f"Sending goal {mocap_goal}")
#                goal_msg = BaseAction.Goal()
#                goal_msg.goal = self._json_ops.encode(mocap_goal)
#                self.send_goal(goal_msg)
#
#    def declare_parameters(self):
#        """Location to declare parameters."""
#        pass
#
#    def goal_response_callback(self, goal_handle: ClientGoalHandle):
#        """Result when a goal is sent to the server."""
#        if not goal_handle.accepted:
#            self.logger.info("Goal was not accepted")
#            return
#        else:
#            self.logger.info("Goal was accepted")
#
#    def feedback_callback(self, feedback_msg: ActionFeedback):
#        """Result when a goal is sent to the server."""
#        self.logger.debug(f"Received feedback {feedback_msg.feedback}")
#        self.dist_rem = self._json_ops.decode(
#            feedback_msg.feedback,
#            ActC.FEEDBACK,
#        )
#
#    def result_callback(self, result: ActionResult, status: GoalStatus):
#        """Result when a goal is sent to the server."""
#        self.logger.info(f"Waypoint reached boolean: {result}")
#        if result.success:
#            return self.get_goal_success()
#        else:
#            return self.get_goal_error()
#
#    def cancel_callback(self, response):
#        """Cancellation callback.
#
#        Args:
#            response: receives CancelGoal action msg
#        """
#        if len(response.goals_canceling) > 0:
#            self.logger.info("Successfully canceled goal.")
#        else:
#            self.logger.info("Unsuccessfully canceled goal.")
#
#    def cancel_geopoint(self):
#        """Interacts with action server to cancel action.
#
#        Returns:
#            Boolean where true signifies a goal was successfully canceled. False if not true.
#
#        """
#        self.cancel_goal(self.cancel_callback)


def main(args=None):
    rclpy.init(args=args)
    node_name = "mocap_hydropoint_client"
    node = Node(node_name)
    action_type = ActionType(BaseAction)
    setpoint = HydropointClient(node, "go_to_hydropoint", action_type)
    setpoint.run()
    # setpoint._test_geopoint()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
