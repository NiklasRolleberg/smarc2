import rclpy

from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from smarc_action_base.gentler_action_server import GentlerActionServer
from geodesy import utm
from geographic_msgs.msg import GeoPoint
from tf2_geometry_msgs import do_transform_pose_stamped
from tf_transformations import euler_from_quaternion
from rclpy.time import Duration, Time
from nav_msgs.srv import SetMap
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import MapMetaData
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Twist, TwistStamped
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Empty
from std_msgs.msg import String
from evolo_msgs.msg import Topics as evoloTopics
from smarc_msgs.msg import Topics as smarcTopics
from smarc_control_msgs.msg import Topics as controlTopics
from tf2_ros import Buffer, TransformException, TransformListener
import numpy as np
import time
import math
import json
from geometry_msgs.msg import Quaternion
from smarc_utilities import georef_utils

import tf_transformations

from enum import Enum



class Attractor:
    def __init__(self, x, w):
        self.x = np.array(x, dtype=float)
        self.w = w




class EvoloMovePath():

    class WP:
        def __init__(self, p : PoseStamped, tol : float, speed : str):
            self.p = p
            self.tol = tol
            self.speed = speed

    def __init__(self,
                 node: Node,
                 action_name: str):
        self._node = node

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
            loop_frequency=1
        )

        # Initialize any necessary state for your specific action
        # These have nothing to do with the action server itself

        # Tf listener
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(
            self._tf_buffer, self._node, spin_thread=True
        )
        
        # State variables. gets updated from topic callbacks
        self.robot_position = PoseStamped() #robot positon [geometry_msgs/msg/Pose]
        self.robot_position_time = None #robot position time to be compared with current time
        
        self.distance_to_target = None

        self.target_index = None
        self.target_list = None #self.WP

        self.poses_history = [] # for the path


        #Target frame
        #self.frame_id = 'map_gt'
        self.frame_id = 'evolo/odom'

        #Settings etc
        self.timeout = 1800.0

        self.current_yaw = 0.0
        
        #Time of action start to check for timeout
        self.action_started_time = None
        
        #Callback groups
        self.publisher_callback_group = ReentrantCallbackGroup()
        self.subscriber_callback_group = ReentrantCallbackGroup()

        # Publishers
        self.evolo_pub = self._node.create_publisher(Float32, controlTopics.CONTROL_YAW_TOPIC,10, callback_group=self.publisher_callback_group)
        self.speed_pub = self._node.create_publisher(TwistStamped, 'evolo/ctrl/twist_setpoint', 10, callback_group=self.publisher_callback_group)
        self.path_pub = self._node.create_publisher(Path, 'visual_path', 10, callback_group=self.publisher_callback_group)
        self.marker_pub = self._node.create_publisher(Marker, 'force_markers', 10, callback_group=self.publisher_callback_group)
        self.viz_markers_pub = self._node.create_publisher(MarkerArray, 'visualisation', 10, callback_group=self.publisher_callback_group) 
       
        # Subscribers
        self.robot_sub = self._node.create_subscription(Odometry, 'evolo/smarc/odom', self.robot_odom_callback,10, callback_group=self.subscriber_callback_group)

        self._node.get_logger().info("Action server started")

    


    def _on_goal_received(self, goal_request: dict) -> bool:
        self._node.get_logger().info(f"Received goal request: {goal_request}")

        speed = goal_request['speed']
        waypoints = goal_request['waypoints']
        self.timeout = 600

        if len(waypoints) == 0: 
            self._node.get_logger().info(f"Waypoint list was empty")    
            return False

        # Reset
        self.target_index = 0
        self.target_list = []
        # Waypoints
        if 'waypoints' in goal_request:
            for wp in waypoints:
                self._node.get_logger().info(f"WP: {wp}")
                wp_params = wp

                self._node.get_logger().info(f"wp params: {wp_params}")

                lat = float(wp_params['latitude'])
                lon = float(wp_params['longitude'])
                self._node.get_logger().info(f"lat lon sent to function: {lat}, {lon}")
                target_position = self.latlon_to_local_frame([lat,lon])
                target_speed = speed
                target_tol = float(wp_params['tolerance'])
                self.target_list.append(self.WP(p=target_position, speed = target_speed, tol = target_tol))

        # Publish markers for visualization
        self.publish_waypoints_markers() 

        return True
    
    def _on_cancel_received(self) -> bool:
        self._node.get_logger().info("Received cancel request")
        return True
    
    def _prepare_loop(self) -> None:
        self._node.get_logger().info("Preparing loop for action execution")
        self.action_started_time = int(self._node.get_clock().now().nanoseconds * 1e-9)
    
    def compute_force(self, q, s):
        dq = np.zeros(2)

        # Attractors
        for a in s:
            diff = a.x - q 
            d = np.linalg.norm(diff)
            if d > 0.01:
                dq += a.w * (diff / d)

        return dq

    def publish_waypoints_markers(self):
        marker_array = MarkerArray()
        
        # WAYPOINTS
        for i, wp in enumerate(self.target_list):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = self._node.get_clock().now().to_msg()
            marker.ns = "waypoints"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = wp.p.pose.position.x
            marker.pose.position.y = wp.p.pose.position.y
            marker.pose.position.z = 0.5
            
            marker.scale.x = wp.tol * 2 
            marker.scale.y = wp.tol * 2
            marker.scale.z = 1.0
            
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.3
            
            marker_array.markers.append(marker)
            
            text_marker = Marker()
            text_marker.header.frame_id = self.frame_id
            text_marker.header.stamp = self._node.get_clock().now().to_msg()
            text_marker.ns = "waypoint_labels"
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            
            text_marker.pose.position.x = wp.p.pose.position.x
            text_marker.pose.position.y = wp.p.pose.position.y
            text_marker.pose.position.z = 2.0
            
            text_marker.scale.z = 2.0 
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            
            text_marker.text = f"WP{i+1}"
            
            marker_array.markers.append(text_marker)
        
        
        self._node.get_logger().info(f"Publishing {len(marker_array.markers)} markers")
        self.viz_markers_pub.publish(marker_array)

    def publish_current_target_marker(self):
        if self.target_index >= len(self.target_list):
            return
        
        marker_array = MarkerArray()
        current_wp = self.target_list[self.target_index]
        
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = "current_target"
        marker.id = 9999
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = current_wp.p.pose.position.x
        marker.pose.position.y = current_wp.p.pose.position.y
        marker.pose.position.z = 1.5
        
        marker.scale.x = 2.0
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker_array.markers.append(marker)
        self.viz_markers_pub.publish(marker_array)

    def publish_force_marker(self, robot_pos, force):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self._node.get_clock().now().to_msg()
        marker.ns = "force_vector"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        start = Point()
        start.x = robot_pos.x
        start.y = robot_pos.y
        start.z = 0.5
        
        scale_factor = 5.0  
        end = Point()
        end.x = robot_pos.x + force[0] * scale_factor
        end.y = robot_pos.y + force[1] * scale_factor
        end.z = 0.5
        
        marker.points = [start, end]
        
        marker.scale.x = 0.3 
        marker.scale.y = 0.5  
        marker.scale.z = 0.5  
        
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        self.marker_pub.publish(marker)




    def _loop_inner(self) -> bool | None:
        time_now = int(self._node.get_clock().now().nanoseconds * 1e-9)
        runtime = (time_now - self.action_started_time)

        if runtime > self.timeout:
            return False

        if self.robot_position_time is None or (time_now - self.robot_position_time) > 10:
            self._node.get_logger().error("ERROR: no robot position")
            return False

        if self.target_index >= len(self.target_list):
            return True

        # Position data
        current_wp = self.target_list[self.target_index]
        target_pos = current_wp.p.pose.position
        robot_pos = self.robot_position.pose.position

        self.distance_to_target = self.calculate_distance(self.robot_position, current_wp.p)
        
        # Waypoint reached
        if self.distance_to_target < current_wp.tol:
            self.target_index += 1
            self._node.get_logger().info(f"Waypoint {self.target_index} reached")
            
            # ✅ Réinitialiser la mémoire du contrôle PD
            if hasattr(self, 'previous_angle_error'):
                self.previous_angle_error = 0.0
                self.previous_time = None
            
            return None if self.target_index < len(self.target_list) else True

        self.publish_current_target_marker()

        # ============================================================
        # CHAMP DE POTENTIEL AVEC ANTICIPATION (LOOK-AHEAD)
        # ============================================================
        
        q = np.array([robot_pos.x, robot_pos.y])
        
        # ATTRACTION VERS LE WAYPOINT ACTUEL
        attractors = [Attractor(
            x=[target_pos.x, target_pos.y],
            w=20.0,
        )]
        
        # ANTICIPATION : Ajouter une force vers le prochain waypoint si proche
        LOOK_AHEAD_DISTANCE = 15.0  # Distance à partir de laquelle on anticipe (ajustable)
        NEXT_WP_WEIGHT_FACTOR = 0.4  # Poids relatif du prochain waypoint (0-1)
        
        if self.distance_to_target < LOOK_AHEAD_DISTANCE and self.target_index + 1 < len(self.target_list):
            next_wp = self.target_list[self.target_index + 1]
            next_pos = next_wp.p.pose.position
            
            # Calculer le poids progressif basé sur la distance au waypoint actuel
            # Plus on est proche, plus le prochain waypoint a d'influence
            blend_factor = 1.0 - (self.distance_to_target / LOOK_AHEAD_DISTANCE)
            next_weight = 20.0 * NEXT_WP_WEIGHT_FACTOR * blend_factor
            
            attractors.append(Attractor(
                x=[next_pos.x, next_pos.y],
                w=next_weight,
            ))
            
            self._node.get_logger().info(
                f"Look-ahead active: DTT={self.distance_to_target:.1f}m, "
                f"next_weight={next_weight:.1f}, blend={blend_factor:.2f}"
            )
        
        # Calculer la force totale (attraction)
        force = self.compute_force(q, attractors)
        force_magnitude = np.linalg.norm(force)
        
        # Direction désirée = direction de la force
        if force_magnitude > 0.01:
            desired_angle = math.atan2(force[1], force[0])
        else:
            # Fallback : direction directe vers la cible
            dx = target_pos.x - robot_pos.x
            dy = target_pos.y - robot_pos.y
            desired_angle = math.atan2(dy, dx)
        
        # ============================================================
        # CALCUL DE L'ERREUR ANGULAIRE
        # ============================================================
        
        if not hasattr(self, 'current_yaw'):
            self.current_yaw = 0.0

        angle_error = math.atan2(math.sin(desired_angle - self.current_yaw), 
                                math.cos(desired_angle - self.current_yaw))
        
        abs_err_deg = abs(math.degrees(angle_error))

        # ============================================================
        # CONTRÔLE PD (Proportionnel + Dérivé) POUR ÉVITER OSCILLATIONS
        # ============================================================
        
        # Initialiser les variables de mémoire si nécessaire
        if not hasattr(self, 'previous_angle_error'):
            self.previous_angle_error = 0.0
            self.previous_time = None
        
        # Calcul de la dérivée de l'erreur
        if self.previous_time is not None:
            dt = time_now - self.previous_time
            if dt > 0:
                angle_error_derivative = (angle_error - self.previous_angle_error) / dt
            else:
                angle_error_derivative = 0.0
        else:
            angle_error_derivative = 0.0
        
        # Mémoriser pour la prochaine itération
        self.previous_angle_error = angle_error
        self.previous_time = time_now

        # ============================================================
        # COMMANDES DE VITESSE
        # ============================================================
        
        cmd_speed = TwistStamped()

        V_MIN = 8.0
        V_MAX = 14.0
        OMEGA_MAX = 16.0
        
        # Dans _loop_inner, section commandes de vitesse

        # Paramètres cohérents
        V_MIN = 8.0
        V_MAX = 14.0
        OMEGA_MAX = 16.0
        OMEGA_MIN = 8.0

        # Normaliser l'erreur angulaire
        abs_err_deg = abs(math.degrees(angle_error))

        if abs_err_deg > 45:
            # Grande erreur : rotation prioritaire
            omega = OMEGA_MAX if angle_error > 0 else -OMEGA_MAX
            v = V_MIN
            
        elif abs_err_deg > 10:
            # Erreur moyenne : transition douce
            # Interpolation non-linéaire (cosinus) pour des transitions plus douces
            ratio = (abs_err_deg - 10) / 35.0  # Entre 0 et 1
            smooth_ratio = (1 - math.cos(ratio * math.pi)) / 2  # Courbe en S
            
            omega = (OMEGA_MIN + smooth_ratio * (OMEGA_MAX - OMEGA_MIN)) * (1 if angle_error > 0 else -1)
            v = V_MAX - smooth_ratio * (V_MAX - V_MIN)
            
        else:
            # Petite erreur : contrôle PD complet
            Kp = 1.5 
            Kd = 0.3   
            
            # FORMULE PD COMPLÈTE
            omega = Kp * angle_error + Kd * angle_error_derivative
            omega = max(-OMEGA_MIN, min(OMEGA_MIN, omega))  # Saturation
            v = V_MAX

        # Appliquer le facteur de sécurité
        cmd_speed.twist.linear.x = v

        # Réduire omega proportionnellement si on ralentit beaucoup
        cmd_speed.twist.angular.z = omega

        self.speed_pub.publish(cmd_speed)
        self.publish_force_marker(robot_pos, force)
        
        # Logs détaillés
        self._node.get_logger().info(
            f"DTT: {self.distance_to_target:.1f}m | "
            f"Force: [{force[0]:.1f}, {force[1]:.1f}] (mag: {force_magnitude:.1f}) | "
            f"Err: {abs_err_deg:.1f}° | dErr: {math.degrees(angle_error_derivative):.1f}°/s | "
            f"Cmd: v={cmd_speed.twist.linear.x:.2f}, ω={cmd_speed.twist.angular.z:.2f}"
        )

        ty_msg = Float32()
        ty_msg.data = desired_angle
        self.evolo_pub.publish(ty_msg)

        return None





    def _give_feedback(self) -> str:
        time_now = int(self._node.get_clock().now().nanoseconds * 1e-9)
        runtime = time_now - self.action_started_time
        
        feedback = f"Action runtime: {runtime}. DTT: {self.distance_to_target}"
        self._node.get_logger().info(feedback)
        return feedback
   
    def calculate_distance(self, pose1:PoseStamped, pose2:PoseStamped) -> float:
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def latlon_to_local_frame(self, point_list: list) -> PoseStamped:

        geopoint = GeoPoint()
        geopoint.latitude = point_list[0]
        geopoint.longitude = point_list[1]
        geopoint.altitude = 0.0

        utm_point_stamped = georef_utils.convert_latlon_to_utm(geopoint)

        pose_stamp = PoseStamped()
        pose_stamp.header = utm_point_stamped.header
        pose_stamp.pose.position = utm_point_stamped.point

        yaw = math.radians(point_list[2]) if len(point_list) > 2 else 0.0
        quaternion_values = tf_transformations.quaternion_from_euler(0, 0, yaw)
        pose_stamp.pose.orientation = Quaternion(
            x=quaternion_values[0],
            y=quaternion_values[1],
            z=quaternion_values[2],
            w=quaternion_values[3]
        )

        try:
            t = self._tf_buffer.lookup_transform(
                target_frame=self.frame_id,
                source_frame=pose_stamp.header.frame_id,
                time=Time(seconds=0),
                timeout=Duration(seconds=1),
            )
            transformed_pose = do_transform_pose_stamped(pose_stamp, t)
            return transformed_pose
        except Exception as e:
            self._node.get_logger().error(f"Failed to transform pose: {e}")
            return None
            
    def robot_odom_callback(self, msg: Odometry):
        if msg.header.frame_id == self.frame_id:
            self.robot_position = PoseStamped()
            self.robot_position.header = msg.header
            self.robot_position.pose = msg.pose.pose
        else:
            pose_in_odom_frame = PoseStamped()
            pose_in_odom_frame.header = msg.header
            pose_in_odom_frame.pose = msg.pose.pose
            
            try:
                t = self._tf_buffer.lookup_transform(
                    target_frame=self.frame_id,
                    source_frame=msg.header.frame_id,
                    time=Time(seconds=0),
                    timeout=Duration(seconds=1),
                )
                self.robot_position = do_transform_pose_stamped(pose_in_odom_frame, t)
                
            except Exception as e:
                self._node.get_logger().error(f"Could not transform robot position: {e}")
                return
        
        self.robot_position_time = int(self._node.get_clock().now().nanoseconds * 1e-9)
        
        orientation_q = self.robot_position.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (_, _, self.current_yaw) = euler_from_quaternion(orientation_list)
        
        if not hasattr(self, '_odom_log_counter'):
            self._odom_log_counter = 0
        
        self._odom_log_counter += 1
        if self._odom_log_counter % 50 == 0:
            self._node.get_logger().info(
                f"Robot local position: ({self.robot_position.pose.position.x:.2f}, "
                f"{self.robot_position.pose.position.y:.2f}), yaw: {math.degrees(self.current_yaw):.1f}°"
            )
        
        # Path visualization
        path_msg = Path()
        path_msg.header.frame_id = self.frame_id 
        path_msg.header.stamp = self._node.get_clock().now().to_msg()
        self.poses_history.append(self.robot_position)
        path_msg.poses = self.poses_history
        self.path_pub.publish(path_msg)


def main():
    rclpy.init()
    node = Node("evolo_move_path_action_server")
    
    action_client = EvoloMovePath(node, "move_path")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down evolo move path acation server")
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()