import rclpy
import math
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Duration, Time
from smarc_msgs.action import BaseAction
import json
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from geographic_msgs.msg import GeoPoint
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from smarc_utilities import georef_utils
import tf_transformations
import numpy as np


class EvoloMovePathClient(Node):

    def __init__(self):
        super().__init__('evolo_move_path_client')
        self._action_client = ActionClient(self, BaseAction, 'move_path')

        # TF buffer pour latlon_to_local_frame  ← AJOUT
        self._tf_buffer   = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=True)

        # Publishers de visualisation
        self.marker_pub      = self.create_publisher(MarkerArray, 'waypoints_viz',    10)
        self.path_pub        = self.create_publisher(Path,        'visual_path',      10)
        self.viz_pub         = self.create_publisher(MarkerArray, 'visualisation',    10)
        self.dubins_path_pub = self.create_publisher(Path,        'dubins_path',      10)
        self.attractor_pub   = self.create_publisher(Marker,      'attractor_marker', 10)
        self.boundary_pub    = self.create_publisher(MarkerArray, 'path_boundaries',  10) 


        # Subscription odométrie
        self.odom_sub = self.create_subscription(Odometry, 'evolo/smarc/odom', self._odom_callback, 10)

        self.poses_history  = []
        self.frame_id       = 'evolo/odom'
        self.target_list    = []

        self.robot_path_msg = Path()
        self.robot_path_msg.header.frame_id = self.frame_id

    # ─────────────────────────────────────────────────────────────────────────
    # Conversion lat/lon → repère local (identique au serveur)
    # ─────────────────────────────────────────────────────────────────────────
    def latlon_to_local_frame(self, point_list):
        geopoint           = GeoPoint()
        geopoint.latitude  = point_list[0]
        geopoint.longitude = point_list[1]
        geopoint.altitude  = 0.0
        utm_pt = georef_utils.convert_latlon_to_utm(geopoint)
        ps = PoseStamped()
        ps.header        = utm_pt.header
        ps.pose.position = utm_pt.point
        yaw = math.radians(point_list[2]) if len(point_list) > 2 else 0.0
        q   = tf_transformations.quaternion_from_euler(0, 0, yaw)
        ps.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        try:
            t = self._tf_buffer.lookup_transform(
                target_frame=self.frame_id,
                source_frame=ps.header.frame_id,
                time=Time(seconds=0),
                timeout=Duration(seconds=1),
            )
            return do_transform_pose_stamped(ps, t)
        except Exception as e:
            self.get_logger().error(f"TF failed: {e}")
            return None

    # ─────────────────────────────────────────────────────────────────────────
    # Odométrie → historique robot
    # ─────────────────────────────────────────────────────────────────────────
    def _odom_callback(self, msg: Odometry):
        ps = PoseStamped()
        ps.header.frame_id = self.frame_id   # ← forcer le même frame que le Path
        ps.header.stamp    = msg.header.stamp
        ps.pose            = msg.pose.pose

        # Si l'odom est déjà dans evolo/odom, c'est bon directement.
        # Sinon, transformer :
        if msg.header.frame_id != self.frame_id:
            try:
                t = self._tf_buffer.lookup_transform(
                    target_frame=self.frame_id,
                    source_frame=msg.header.frame_id,
                    time=Time(seconds=0),
                    timeout=Duration(seconds=1),
                )
                raw = PoseStamped()
                raw.header = msg.header
                raw.pose   = msg.pose.pose
                ps = do_transform_pose_stamped(raw, t)
            except Exception as e:
                self.get_logger().error(f"Odom TF failed: {e}")
                return

        self.robot_path_msg.poses.append(ps)
        self.robot_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.robot_path_msg)


    """
    # ─────────────────────────────────────────────────────────────────────────

    def publish_boundaries(self):
        ma = MarkerArray()

        left_bound = [
            [58.839967, 17.653462], [58.839967, 17.656493], [58.840957, 17.656493],
            [58.840957, 17.660781], [58.839967, 17.660781], [58.839967, 17.663812]
        ]

        right_bound = [
            [58.839517, 17.653462], [58.839517, 17.657331], [58.840507, 17.657331],
            [58.840507, 17.659943], [58.839517, 17.659943], [58.839517, 17.663812]
        ] 

        for i, bound in enumerate([left_bound, right_bound]):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "boundaries"
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD

            marker.scale.x = 0.7 
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0

            for wp in bound:
                ps = self.latlon_to_local_frame([wp[0], wp[1]])
                if ps:
                    p = Point()
                    p.x = ps.pose.position.x
                    p.y = ps.pose.position.y
                    p.z = 0.0
                    marker.points.append(p)

            ma.markers.append(marker)
        
        self.boundary_pub.publish(ma)
    """

    # ─────────────────────────────────────────────────────────────────────────
    def publish_waypoints(self, waypoint_list):
        marker_array = MarkerArray()
        for i, pt in enumerate(waypoint_list):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.type   = Marker.SPHERE
            marker.action = Marker.ADD
            marker.id     = i
            marker.pose.position.x = pt[0]
            marker.pose.position.y = pt[1]
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

    def _publish_attractor(self, ax, ay):
        m = Marker()
        m.header.frame_id = self.frame_id
        m.header.stamp    = self.get_clock().now().to_msg()
        m.ns = "attractor"; m.id = 0
        m.type = Marker.SPHERE; m.action = Marker.ADD
        m.pose.position.x = ax
        m.pose.position.y = ay
        m.pose.position.z = 1.0
        m.scale.x = 3.0; m.scale.y = 3.0; m.scale.z = 3.0
        m.color.r = 1.0; m.color.g = 1.0; m.color.b = 0.0; m.color.a = 0.9
        self.attractor_pub.publish(m)

    # ─────────────────────────────────────────────────────────────────────────
    def send_goal(self):
        self.get_logger().info("Wait for Action Server...")
        if not self._action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("No server")
            return
        while not self._tf_buffer.can_transform(self.frame_id, 'utm', Time(seconds=0)):
            self.get_logger().info("Waiting for TF...")
            rclpy.spin_once(self, timeout_sec=0.5)
        
        # self.publish_boundaries()
        
        goal_msg = BaseAction.Goal()

        payload = {
            'speed': 'high',

            'waypoints': [
                {'latitude': 58.8397422670, 'longitude': 17.6534623045, 'tolerance': 3.0},
                {'latitude': 58.8400922670, 'longitude': 17.6540122932, 'tolerance': 3.0},
                {'latitude': 58.8403922670, 'longitude': 17.6533123075, 'tolerance': 3.0},
                {'latitude': 58.8398922670, 'longitude': 17.6528123177, 'tolerance': 3.0},
                {'latitude': 58.8397922670, 'longitude': 17.6543122871, 'tolerance': 3.0},
            ]


        }

        goal_msg.goal.data = json.dumps(payload)

        self.get_logger().info("Send mission...")
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
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

    # ─────────────────────────────────────────────────────────────────────────
    def feedback_callback(self, feedback_msg):
        try:
            data = json.loads(feedback_msg.feedback.feedback.data)

            if 'wps' in data:
                self.target_list = data['wps']
                self.publish_waypoints_markers()

            if 'ax' in data and 'ay' in data:
                self._publish_attractor(data['ax'], data['ay'])

            if 'full_path' in data:
                path_msg = self._convert_list_to_path(data['full_path'])
                self.dubins_path_pub.publish(path_msg)

        except Exception as e:
            self.get_logger().error(f"Erreur feedback: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    def publish_waypoints_markers(self):
        if not self.target_list:
            return

        ma = MarkerArray()
        for i, wp in enumerate(self.target_list):
            m = Marker()
            m.header.frame_id = self.frame_id
            m.header.stamp    = self.get_clock().now().to_msg()
            m.ns = "waypoints"; m.id = i
            m.type = Marker.SPHERE; m.action = Marker.ADD
            m.pose.position.x = float(wp['x'])
            m.pose.position.y = float(wp['y'])
            m.pose.position.z = 0.5
            m.scale.x = float(wp['tol']) * 2
            m.scale.y = float(wp['tol']) * 2
            m.scale.z = 1.0
            m.color.g = 1.0; m.color.a = 0.3
            ma.markers.append(m)

            t = Marker()
            t.header = m.header
            t.ns = "waypoint_labels"; t.id = i + 1000
            t.type = Marker.TEXT_VIEW_FACING; t.action = Marker.ADD
            t.pose.position.x = m.pose.position.x
            t.pose.position.y = m.pose.position.y
            t.pose.position.z = 2.0
            t.scale.z = 1.5
            t.color.r = 1.0; t.color.g = 1.0; t.color.b = 1.0; t.color.a = 1.0
            t.text = f"WP{i + 1}"
            ma.markers.append(t)

        self.viz_pub.publish(ma)

    def _convert_list_to_path(self, points):
        """Transforme une liste [[x, y, yaw], ...] en nav_msgs/Path."""
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id
        path_msg.header.stamp    = self.get_clock().now().to_msg()
        for pt in points:
            ps = PoseStamped()
            ps.header          = path_msg.header
            ps.pose.position.x = pt[0]
            ps.pose.position.y = pt[1]
            path_msg.poses.append(ps)
        return path_msg

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info("Final Result")
        rclpy.shutdown()


# ─────────────────────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    client = EvoloMovePathClient()
    client.send_goal()
    rclpy.spin(client)


if __name__ == '__main__':
    main()