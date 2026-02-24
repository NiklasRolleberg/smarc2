#!/usr/bin/python3

import rclpy

from rclpy.node import Node
from rclpy.time import Time, Duration
from rclpy.executors import MultiThreadedExecutor
from builtin_interfaces.msg import Time as TimeMsg

from geographic_msgs.msg import GeoPoint, GeoPath, GeoPointStamped
from geographic_msgs.srv import GetGeoPath

from smarc_msgs.msg import Topics as SmarcTopics
from smarc_action_base.gentler_action_server import GentlerActionServer



class GeofenceNode():
    def __init__(self, node: Node):
        self._node = node

        self._gps_subscriber = self._node.create_subscription(GeoPoint, SmarcTopics.POS_LATLON_TOPIC, self.pos_latlon_cb, 10)
        self._geofence_ok_publisher = self._node.create_publisher(TimeMsg, SmarcTopics.GEOFENCE_OK_TOPIC, 10)

        self._start_as = GentlerActionServer(
            node,
            "start_geofence",
            self._on_goal_received_start,
            self._on_cancel_received_start,
            lambda: None,
            lambda: True,
            lambda: "Geofence node running",
            loop_frequency = 5
        )

        self._stop_as = GentlerActionServer(
            node,
            "stop_geofence",
            lambda _: self._on_cancel_received_start(), # "start the stop action" == "cancel the start action"
            lambda: True,
            lambda: None,
            lambda: True,
            lambda: "Geofence node stopped",
            loop_frequency = 5
        )

        self._get_fence_srv = node.create_service(GetGeoPath, SmarcTopics.GET_GEOFENCE_SERVICE_TOPIC, self._get_fence_srv_cb)


        self._robot_position : GeoPoint | None = None
        self._robot_position_time : Time | None = None
        self._geofence_vertices : list[GeoPoint] | None = None
        self._ceiling_altitude : float | None = None
        self._floor_altitude : float | None = None # aka depth for underwater vehicles
        self._fence_start_time : TimeMsg | None = None

        self._node.declare_parameter('check_period', 0.5)
        self._check_period = self._node.get_parameter('check_period').get_parameter_value().double_value
        self._pub_timer = self._node.create_timer(self._check_period, self.publish_geofence_ok)

        self._node.declare_parameter('max_position_age', 1.0)
        self._max_position_age = self._node.get_parameter('max_position_age').get_parameter_value().double_value
        self._max_position_duration = Duration(seconds=int(self._max_position_age), nanoseconds=int((self._max_position_age - int(self._max_position_age)) * 1e9))


    def pos_latlon_cb(self, msg: GeoPoint):
        self._robot_position_time = self._node.get_clock().now()
        self._robot_position = msg


    def publish_geofence_ok(self):
        if self._robot_position is None or self._robot_position_time is None: 
            self._node.get_logger().info("No robot position received yet...")
            return
        
        if self._geofence_vertices is None or len(self._geofence_vertices) < 3: 
            self._node.get_logger().info("No valid geofence defined yet...")
            return

        if self._robot_position_time + self._max_position_duration < self._node.get_clock().now():
            self._node.get_logger().info(f"Robot position is older than {self._max_position_age}s, not publishing geofence_ok")
            return

        if self._point_in_fence(self._robot_position):
            geofence_ok_msg = self._robot_position_time.to_msg()
            self._node.get_logger().info(f"Robot is INSIDE t={self._robot_position_time.seconds_nanoseconds()[0]}s")
        else:
            self._node.get_logger().warn(f"Robot is OUTSIDE t={self._robot_position_time.seconds_nanoseconds()[0]}s")
            geofence_ok_msg = TimeMsg(sec=-1, nanosec=0)
        
        self._geofence_ok_publisher.publish(geofence_ok_msg)



    def _on_goal_received_start(self, goal_request: dict) -> bool:
        try:
            wps = goal_request['waypoints']
            if len(wps) < 3:
                self._node.get_logger().error("Geofence action server requires at least 3 waypoints to define a geofence")
                return False
            
            self._geofence_vertices = [(GeoPoint(latitude=wp['latitude'], longitude=wp['longitude'])) for wp in wps]
            self._ceiling_altitude = goal_request['ceiling_altitude']
            self._floor_altitude = goal_request['floor_altitude']
            self._fence_start_time = self._node.get_clock().now().to_msg()
            self._node.get_logger().info(f"Geofence defined with {len(self._geofence_vertices)} vertices, ceiling altitude {self._ceiling_altitude}, floor altitude {self._floor_altitude}")
            return True

        except Exception as e:
            self._node.get_logger().error(f"Error parsing geofence waypoints: {e}")
            return False
        
    def _on_cancel_received_start(self):
        self._geofence_vertices = None
        self._ceiling_altitude = None
        self._floor_altitude = None
        self._fence_start_time = None
        self._node.get_logger().info("Geofence cleared")
        return True
    
    def _point_in_fence(self, point: GeoPoint) -> bool:
        if self._geofence_vertices is None: return False
        if len(self._geofence_vertices) < 3: return False
        if self._ceiling_altitude is not None:
            if point.altitude > self._ceiling_altitude: return False
        if self._floor_altitude is not None:
            if point.altitude < self._floor_altitude: return False

        return is_point_inside_polygon(point, self._geofence_vertices)
    

    def _get_fence_srv_cb(self, request: GetGeoPath.Request, response: GetGeoPath.Response):
        if self._geofence_vertices is None:
            response.success = False
            response.status = "No geofence defined"
            self._node.get_logger().info("Get geofence request received but no geofence defined, returning False")
            return response
        
        # return the fence either way
        response.plan = GeoPath()
        response.plan.header.stamp = self._fence_start_time if self._fence_start_time is not None else Time(seconds=0, nanoseconds=0).to_msg()
        response.plan.header.frame_id = "latlon"
        response.plan.poses = [GeoPointStamped(geo_point=vertex) for vertex in self._geofence_vertices]

        valid_start = not (request.start.altitude == 0.0 and request.start.latitude == 0.0 and request.start.longitude == 0.0)
        valid_end = not (request.goal.altitude == 0.0 and request.goal.latitude == 0.0 and request.goal.longitude == 0.0)

        if valid_start and not valid_end:
            response.success = self._point_in_fence(request.start)
            response.status = "Start point is valid" if response.success else "Start point is outside geofence"
            self._node.get_logger().info(response.status)
            return response
        
        if valid_end and not valid_start:
            response.success = self._point_in_fence(request.goal)
            response.status = "Goal point is valid" if response.success else "Goal point is outside geofence"
            self._node.get_logger().info(response.status)
            return response
        
        if valid_end and valid_start:
            response.success = self._point_in_fence(request.start) and self._point_in_fence(request.goal)
            response.status = "Start and goal points are valid" if response.success else "Start and/or goal point is outside geofence"
            self._node.get_logger().info(response.status)
            return response
        
        # no points given, return just the fence
        response.success = True
        response.status = "Geofence retrieved"
        self._node.get_logger().info(response.status)
        return response


# lifted from "utils/geofence_checker/geofence_checker/geofence_checker_node.py"
# this node should supercede that one
def is_point_inside_polygon(point: GeoPoint, vertices: list[GeoPoint]) -> bool:
    n = len(vertices)
    inside = False

    lat_1, lon_1 = vertices[0].latitude, vertices[0].longitude
    for i in range(n + 1):
        lat_2, lon_2 = vertices[i % n].latitude, vertices[i % n].longitude
        if point.latitude > min(lat_1, lat_2):
            if point.latitude <= max(lat_1, lat_2):
                if point.longitude <= max(lon_1, lon_2):
                    if lat_1 != lat_2:
                        lon_inters = (point.latitude - lat_1) * (lon_2 - lon_1) / (lat_2 - lat_1) + lon_1
                        print(lon_inters)
                    if lon_1 == lon_2 or point.longitude <= lon_inters:
                        inside = not inside
        lat_1, lon_1 = lat_2, lon_2

    return inside




def main():
    rclpy.init()
    node = Node("geofence_node")
    geofence_node = GeofenceNode(node)
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down")
        node.destroy_node()


def test_action():
    import json
    from smarc_action_base.bt_action_client_action import A_ActionClient
    from py_trees.trees import BehaviourTree

    goal = {
        "waypoints": [
            {"latitude": 59.0, "longitude": 18.0, "altitude": 0},
            {"latitude": 59.0, "longitude": 18.1, "altitude": 0},
            {"latitude": 59.1, "longitude": 18.1, "altitude": 0},
            {"latitude": 59.1, "longitude": 18.0, "altitude": 0},
        ],
        "ceiling_altitude": 20.0,
        "floor_altitude": 8.0,
    }
    goal = json.dumps(goal)

    rclpy.init()
    node = Node("geofence_test_client")
    action_client = A_ActionClient(node, "start_geofence")
    action_client.setup()
    action_client.set_goal(goal)

    tree = BehaviourTree(action_client)
    node.create_timer(10, tree.tick)

    latlonpub = node.create_publisher(GeoPoint, SmarcTopics.POS_LATLON_TOPIC, 10)
    alt = 0.1
    lat = 59.05
    def publish_test_position():
        nonlocal alt, lat, node
        if alt < 19:
            alt += 1.0
        else:
            lat += 0.01
        msg = GeoPoint(latitude=lat, longitude=18.05, altitude=alt)
        node.get_logger().info(f"Publishing test position: {msg}")
        latlonpub.publish(msg)

    node.create_timer(0.5, publish_test_position)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    rclpy.spin(node, executor=executor)
    rclpy.shutdown()

    
    
    




if __name__ == "__main__":
    main()