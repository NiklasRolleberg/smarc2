import rclpy
import math
import json

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Duration, Time
from smarc_msgs.action import BaseAction
from geometry_msgs.msg import PointStamped
from smarc_utilities import georef_utils
import tf_transformations
from rclpy.action import ActionClient as RosActionClient


class test_client(Node):

    def __init__(self):
        super().__init__('test_client')
        self._action_client = ActionClient(self, BaseAction, 'deploy_at')


    def send_goal(self):
        self.get_logger().info('Wait for Action Server…')
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('No server')
            return
        
        goal_msg = BaseAction.Goal()
        payload = {
            'unit': 1,
            'waypoint': {'latitude': 58.8389422670, 'longitude': 17.6534623045, 'tolerance': 3.0}
        }
            

        goal_msg.goal.data = json.dumps(payload)
        if 'polygons' in payload:
            self._send_polygons_to_geofence(payload['polygons'])

        self.get_logger().info('Send mission…')
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Mission rejected')
            return
        self.get_logger().info('Mission accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        future.result().result
        self.get_logger().info('Final Result')
        rclpy.shutdown()

    # ─────────────────────────────────────────────────────────────────────────
    # Feedback — dispatch all server data to RViz
    # ─────────────────────────────────────────────────────────────────────────
    def feedback_callback(self, feedback_msg):
        try:
            data = json.loads(feedback_msg.feedback.feedback.data)
        except Exception as e:
            self.get_logger().error(f'Feedback error: {e}')

def main(args=None):
    rclpy.init(args=args)
    client = test_client()
    client.send_goal()
    rclpy.spin(client)


if __name__ == '__main__':
    main()