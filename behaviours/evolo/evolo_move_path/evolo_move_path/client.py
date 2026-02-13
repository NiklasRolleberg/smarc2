import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from smarc_msgs.action import BaseAction
import json

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

class EvoloMovePathClient(Node):
    def __init__(self):
        super().__init__('evolo_move_path_client')
        self._action_client = ActionClient(self, BaseAction, 'move_path')
        self.marker_pub = self.create_publisher(MarkerArray, '/waypoints_viz', 10)

    def publish_waypoints(self, waypoint_list):
        marker_array = MarkerArray()
        for i, pt in enumerate(waypoint_list):
            marker = Marker()
            marker.header.frame_id = "evolo/odom" 
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id = i
            marker.pose.position.x = pt[0]
            marker.pose.position.y = pt[1]
            marker.scale.x = 0.2 
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0 
            marker.color.r = 1.0 
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

    def send_goal(self):
        self.get_logger().info("Wait for Action Server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("No server")
            return

        goal_msg = BaseAction.Goal()
        
        from std_msgs.msg import String
        
        payload = {
            'speed': 'high',
            'waypoints': [
                {'latitude': 58.8397422670, 'longitude': 17.6534623045, 'tolerance': 5.0},
                {'latitude': 58.8400922670, 'longitude': 17.6540122932, 'tolerance': 5.0},
                {'latitude': 58.8403922670, 'longitude': 17.6533123075, 'tolerance': 5.0},
                {'latitude': 58.8398922670, 'longitude': 17.6528123177, 'tolerance': 5.0},
                {'latitude': 58.8397922670, 'longitude': 17.6543122871, 'tolerance': 5.0}
            ],
            'obstacles': [
                # {'latitude': 58.8399222670, 'longitude': 17.6537422987, 'radius': 8.0},
                # {'latitude': 58.8402422670, 'longitude': 17.6536623004, 'radius': 10.0},
                # {'latitude': 58.8399422670, 'longitude': 17.6530123137, 'radius': 6.0}
            ]
        }
        
        goal_msg.goal = String()
        goal_msg.goal.data = json.dumps(payload)

        self.get_logger().info("Send mission with obstacles...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Mission rejected')
            return
        self.get_logger().info('Mission accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Feedback : {feedback_msg.feedback}")

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Final Result")
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    client = EvoloMovePathClient()
    client.send_goal()
    rclpy.spin(client)

if __name__ == '__main__':
    main()






