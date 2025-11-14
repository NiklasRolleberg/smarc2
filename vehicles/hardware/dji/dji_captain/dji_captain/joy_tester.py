#!/usr/bin/python3

import rclpy, sys, math, time
import numpy as np
from enum import Enum
from typing import Optional

from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time, Duration
from rclpy.timer import Timer
from tf2_ros import Buffer, TransformListener


from std_msgs.msg import Float32, Int8, String
from std_srvs.srv import Trigger
from sensor_msgs.msg import NavSatFix, Joy, BatteryState, JoyFeedback
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped, Pose, PoseStamped, TransformStamped, QuaternionStamped, PointStamped, Vector3Stamped, Quaternion
from geographic_msgs.msg import GeoPoint
from tf2_msgs.msg import TFMessage

from psdk_interfaces.msg import PositionFused, ControlMode, EscData, EscStatusIndividual
from smarc_msgs.msg import Topics as SmarcTopics
from dji_msgs.msg import Links as DjiLinks
from dji_msgs.msg import Topics as DjiTopics


from smarc_utilities.georef_utils import convert_latlon_to_utm, convert_utm_to_latlon
from tf_transformations import euler_from_quaternion, quaternion_from_euler, quaternion_matrix
from tf2_geometry_msgs import do_transform_pose_stamped


class JoyTester():
    def __init__(self, node: Node, pattern:str):
        self.node = node

        ns = "/M350/"
        WRAPPER_NS = ns + "wrapper/psdk_ros2/"
        FLUvel_JOY          = WRAPPER_NS + "flight_control_setpoint_FLUvelocity_yawrate"
        ENUvel_JOY          = WRAPPER_NS + "flight_control_setpoint_ENUvelocity_yawrate"
        ENUpos_JOY          = WRAPPER_NS + "flight_control_setpoint_ENUposition_yaw"
        TAKE_CONTROL_SRV    = WRAPPER_NS + "obtain_ctrl_authority"
        RELEASE_CONTROL_SRV = WRAPPER_NS + "release_ctrl_authority"

        self.log(f"FLUvel_JOY topic: {FLUvel_JOY}")    
        self.FLU_vel_joy_pub = node.create_publisher(Joy, FLUvel_JOY, qos_profile=10)
        self.ENU_vel_joy_pub = node.create_publisher(Joy, ENUvel_JOY, qos_profile=10)
        self.ENU_pos_joy_pub = node.create_publisher(Joy, ENUpos_JOY, qos_profile=10)

        self.take_control_srv = node.create_client(Trigger, TAKE_CONTROL_SRV)


        try:
            parts = pattern.split(',')
            self.mode = parts[0].lower()
            move_duration = float(parts[1].split('=')[1])
            move_speed = float(parts[2].split('=')[1])
            pause_duration = float(parts[3].split('=')[1])
            repeat_count = int(parts[4].split('=')[1])
        except (IndexError, ValueError):
            self.log("Invalid input format. Please follow the specified pattern.")
            self.log("Enter motion pattern like: [fluvel/enuvel/enupos],d=1.0,s=0.5,p=1.0,r=3 for fluvel, duration 1s, speed 0.5m/s, pause 1s, repeat 3 times.")
            sys.exit(1)

        if self.mode not in ['fluvel', 'enuvel', 'enupos']:
            self.log("Invalid mode. Choose from [fluvel, enuvel, enupos].")
            sys.exit(1)

        if self.mode == 'fluvel':
            pub = self.FLU_vel_joy_pub
        elif self.mode == 'enuvel':
            pub = self.ENU_vel_joy_pub
        elif self.mode == 'enupos':
            pub = self.ENU_pos_joy_pub

        self.got_control = False
        self._take_control()

        while not self.got_control:
            self.log("Waiting to obtain control...")
            rclpy.spin_once(self.node, timeout_sec=0.1)

        for i in range(repeat_count):
            self.log(f"Iteration {i+1} of {repeat_count}")
            self.log("Moving forward")
            self.send_joy(pub, 0.0, move_speed, 0.0, move_duration)
            self.log("Pausing")
            time.sleep(pause_duration)
            self.log("Moving backward")
            self.send_joy(pub, 0.0, -move_speed, 0.0, move_duration)
            self.log("Pausing")
            time.sleep(pause_duration)
        
        self.log("Done")
        sys.exit(0)



    def log(self, msg: str):
        self.node.get_logger().info(msg)

    def _take_control(self):
        def on_result(f):
            self.log(f"Take control service called, success: {f.result().success}, message: {f.result().message}")
            self.got_control = f.result().success

        self.log("Taking control.")
        if not self.take_control_srv.wait_for_service(timeout_sec=5.0):
            self.log("Take control service not available...")
            return
        future = self.take_control_srv.call_async(Trigger.Request())
        future.add_done_callback(on_result)


    def send_joy(self, pub, vx: float, vy: float, vz: float, duration: float):
        joy_msg = Joy()
        joy_msg.axes = [float(vx), float(vy), float(vz), 0.0]
        joy_msg.header.stamp = self.node.get_clock().now().to_msg()

        timer = self.node.create_timer(0.1, lambda: pub.publish(joy_msg)) 

        end_time = time.time() + duration
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        timer.cancel()
            

def main(args=None):
    rclpy.init(args=sys.argv)

    node = rclpy.create_node('joy_tester_node')

    joy_tester = JoyTester(node, sys.argv[1] if len(sys.argv) > 1 else "")
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

    rclpy.shutdown()


if __name__ == '__main__':
    main()