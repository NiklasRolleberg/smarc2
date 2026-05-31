import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from smarc_action_base.gentler_action_server import GentlerActionServer
from smarc_action_base.gentler_action_client import GentlerActionClient
from smarc_action_base.smarc_action_base import ActionClientState, ActionType
from smarc_msgs.action import BaseAction
from std_msgs.msg import String
from evolo_msgs.msg import Topics as evoloTopics
import json

class EvoloDeployAt():

    def __init__(self,
                 node: Node,
                 action_name: str):
        self._node = node

        # Initialize any necessary state for your specific action
        # These have nothing to do with the action server itself
        self.unit_to_deploy = None
        self.waypoint = None

        # Initialize the action server with the node and action name
        # Give it all the necessary callbacks
        self._as = GentlerActionServer(
            node,
            action_name,
            self._on_goal_received,
            self._on_cancel_received,
            self._prepare_loop,
            self._loop_inner,
            self._give_feedback,
            loop_frequency=2
        )

    def _on_goal_received(self, goal_request: dict) -> bool:
        self._node.get_logger().info(f"Received goal request: {goal_request}")
        # Here you would typically validate the goal request
        # Return True to accept the goal, False to reject it

        self._node.get_logger().info(f"Keys: {goal_request.keys()}")

        try: 
            self.unit_to_deploy = goal_request['unit']
            self.waypoint = goal_request['waypoint']

            self._node.get_logger().info(f"Waypoint: {type(self.waypoint)}, {self.waypoint}")
            
            lat = float(self.waypoint['latitude'])
            lon = float(self.waypoint['longitude'])

            self._node.get_logger().info(f"Waypoint:  {self.waypoint}")
            self._node.get_logger().info(f"unit:  {self.waypoint}")
        except Exception as e:
            self._node.get_logger().error(f"Error parsing goal: {e}")
            return False
        return False

    
    def _on_cancel_received(self) -> bool:
        self._node.get_logger().info("Received cancel request")
        # Here you would typically handle the cancel request
        # Return True to accept the cancel, False to reject it
        return True

    def _prepare_loop(self) -> None:
        self._node.get_logger().info("Preparing loop for action execution")
        # Here you would typically set up any necessary state or resources
        # This is run once before the loop starts, after you accept the goal

    def _loop_inner(self) -> bool | None:
        # Return true right away. Hopefully one publication of "realease" is enough.
        # Otherwire keep publishing here for a few seconds
        return True #Success

    def _give_feedback(self) -> str:
        feedback = "Deploy feedback"
        self._node.get_logger().info(feedback)
        # Here you would typically generate feedback for the action
        # This is run after each _loop_inner call
        return feedback

def main():
    rclpy.init()
    node = Node("evolo_deploy_action_server")
    
    action_server = EvoloDeployAt(node, "deploy_at")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down evolo deploy at acation server")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()