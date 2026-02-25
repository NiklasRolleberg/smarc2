import rclpy
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo
import tf2_ros
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import PolygonStamped, Point32

from dji_msgs.msg import Topics
from dji_msgs.msg import Links


class ProjectionNode(Node):
    def __init__(self, name = 'projection_node'):
        super().__init__(name)
        
        self.declare_parameters(
            namespace="",
            parameters=[
                ("z_water", 0.0),
                ("topics.camera_info", "gimbal_camera/camera/cam_info"),
                ("topics.predicted_position.sam_head_obb", Topics.ESTIMATED_AUV_HEAD_OBB_TOPIC),
                ("topics.rviz.camera_rays", "rviz/projection_rays"),
                ("topics.rviz.projected_auv_head", "rviz/projected_auv_head"),
                ("topics.rviz.projected_auv_obb", "rviz/projected_auv_obb"),
                ("frames.map", Links.MAP),                      
                ("frames.camera", Links.GIMBAL_CAMERA_LINK),      
                ("frames.estimated_auv", Links.ESTIMATED_AUV)])

        # Parameters
        self.z_water = float(self.get_parameter("z_water").value)  # water plane height in map frame
        # Topics
        self.topic_camera_info = self.get_parameter("topics.camera_info").value
        self.topic_sam_head_obb = self.get_parameter("topics.predicted_position.sam_head_obb").value
        self.topic_rays = self.get_parameter("topics.rviz.camera_rays").value
        self.topic_head = self.get_parameter("topics.rviz.projected_auv_head").value
        self.topic_obb = self.get_parameter("topics.rviz.projected_auv_obb").value

        # Frames
        map_frame = self.get_parameter("frames.map").value
        cam_frame = self.get_parameter("frames.camera").value
        estimated_auv_frame = self.get_parameter("frames.estimated_auv").value

        namepsace = self.get_namespace().strip("/")
        self.map_frame = f'{namepsace}/{map_frame}'
        self.cam_frame = f'{namepsace}/{cam_frame}'
        self.estimated_auv_frame = f'{namepsace}/{estimated_auv_frame}'

        # Cam parameters
        self.cam_info = False
        self.K_inv = None
        self.width = None
        self.height = None
        self.R_im_cam = np.array([[ 0, -1,  0], [-1,  0,  0], [ 0,  0, -1]])  # Optical to cam link frame
        
        # tf
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Publishers
        self.pub_head = self.create_publisher(PoseStamped, self.topic_head, 10)
        self.ray_pub  = self.create_publisher(Marker, self.topic_rays, 10)
        self.pub_obb  = self.create_publisher(PolygonStamped, self.topic_obb, 10)

        # Subscribers
        self.sub_cam = self.create_subscription(CameraInfo, self.topic_camera_info, self.cam_info_cb, 10)
        self.sub_obb = self.create_subscription(PolygonStamped, self.topic_sam_head_obb, self.project, 10)

    def cam_info_cb(self, msg: CameraInfo):

        # Callback for cam info, only needed once.
        if self.cam_info:  # cam info already received
            return
        self.width = msg.width
        self.height = msg.height
        K = np.array(msg.k).reshape((3, 3))  # cam intrinsic matrix
        self.K_inv = np.linalg.inv(K)
        self.cam_info = True
        self.get_logger().info(f"CameraInfo received: {self.width}x{self.height}, K={K}")
    
    def project(self, msg: PolygonStamped):
        # Projects head and obb points from image to 3d points on water plane
        if not self.cam_info:
            self.get_logger().warning("No CameraInfo received yet")
            return
        
        stamp = msg.header.stamp
        points_im = np.array([(p.x, p.y) for p in msg.polygon.points])

        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.cam_frame, stamp, timeout=rclpy.duration.Duration(seconds=0.5))
        except Exception as e:
            self.get_logger().info(f"TF lookup failed ({self.cam_frame} -> {self.map_frame}): {e}")
            return
        
        t = transform.transform.translation
        cam_pos_map = np.array([t.x, t.y, t.z])  # camera position in map frame

        q = transform.transform.rotation
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()

        # pixelcoords from normalized image coords
        u = (points_im[:, 0] + 1.0) * 0.5 * self.width
        v = (1.0 - (points_im[:, 1] + 1.0) * 0.5) * self.height # y up -> v down (optical frame)

        ray_im = self.K_inv @ np.vstack((u, v, np.ones_like(u))) # projection from pixel to 3D ray in image frame (pc = K_inv @ xs)
        ray_cam = self.R_im_cam @ ray_im # Convert from image frame to cam link frame

        ray_map = rot @ ray_cam # rotate ray to map frame
        dz = ray_map[2, :]
        if np.any(dz > -1e-2):  # if the z component of the norm ray is positive or close to zero, it means the ray is parallel to or pointing away from the water plane
            self.get_logger().info(f"Projection doesn't intersect with water surface")
            return
        
        t_intersect = (self.z_water - cam_pos_map[2]) / dz # translation along ray to intersect with water
        intersection_points_map = cam_pos_map[:, None] + ray_map * t_intersect  # intersection points in map frame
        intersection_points_cam = ray_cam * t_intersect  # intersection points in cam frame
        
        #self.publish_all_points(stamp, intersection_points_map)
        yaw = self.determine_orientation(intersection_points_map)

        self.broadcast_estimated_auv_tf(stamp, intersection_points_cam[:, 0], yaw - np.pi/2)
        self.publish_head_and_obb_markers(stamp, intersection_points_map.T, yaw)
        self.publish_ray_marker(stamp, intersection_points_cam[:, 0])
    
    def determine_orientation(self, points_3d):
        # Determine orientation of AUV based on projected head and OBB points
        head = points_3d[:, 0]
        obb_points = points_3d[:, 1:]
        
        # orientation from center of obb to head
        center = np.mean(obb_points, axis=1)
        orientation_vec = head - center
        yaw = np.arctan2(orientation_vec[1], orientation_vec[0])
        return yaw
    
    def broadcast_estimated_auv_tf(self, stamp, position, yaw):
        # broadcast tf of projected auv position and orientation
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.cam_frame  
        tf_msg.child_frame_id = self.estimated_auv_frame
        tf_msg.transform.translation.x = position[0]
        tf_msg.transform.translation.y = position[1]
        tf_msg.transform.translation.z = position[2]
        q = R.from_euler('z', yaw).as_quat()
        tf_msg.transform.rotation.x = q[0]
        tf_msg.transform.rotation.y = q[1]
        tf_msg.transform.rotation.z = q[2]
        tf_msg.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(tf_msg)

    def publish_ray_marker(self, stamp, intersection_point):
        # publish projected line (ray) for rviz
        marker = Marker()
        marker.header.frame_id = self.cam_frame
        marker.header.stamp = stamp
        marker.ns = "camera_rays"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = 0.02  
        marker.color.a = 1.0
        marker.color.r = 1.0  
        p0 = Point(x=0.0, y=0.0, z=0.0)
        p1 = Point(x=intersection_point[0], y=intersection_point[1], z=intersection_point[2])
        marker.points.append(p0)
        marker.points.append(p1)
        self.ray_pub.publish(marker)

    def publish_head_and_obb_markers(self, stamp, points, yaw):
        # Publish head pose and obb corners
        head = PoseStamped()
        head.header.stamp = stamp
        head.header.frame_id = self.map_frame
        head.pose.position.x = points[0][0]
        head.pose.position.y = points[0][1]
        head.pose.position.z = points[0][2]
        q = R.from_euler('z', yaw).as_quat()
        head.pose.orientation.x = q[0]
        head.pose.orientation.y = q[1]
        head.pose.orientation.z = q[2]
        head.pose.orientation.w = q[3]
        self.pub_head.publish(head)

        obb = PolygonStamped()
        obb.header.frame_id = self.map_frame
        obb.header.stamp = stamp
        for px, py, pz in points[1:]:
            point32 = Point32()
            point32.x = px
            point32.y = py
            point32.z = pz
            obb.polygon.points.append(point32)
        self.pub_obb.publish(obb)
    
    """def publish_all_points(self, stamp, points): 
        # Publish head pose and obb corners (may be used for filter)
        obb_marker = PolygonStamped()
        obb_marker.header.frame_id = self.map_frame
        obb_marker.header.stamp = stamp
        for px, py, pz in points:
            point32 = Point32()
            point32.x = px
            point32.y = py
            point32.z = pz
            obb_marker.polygon.points.append(point32)
        self.pub_obb.publish(obb_marker)"""


def main():
    rclpy.init()
    node = ProjectionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
