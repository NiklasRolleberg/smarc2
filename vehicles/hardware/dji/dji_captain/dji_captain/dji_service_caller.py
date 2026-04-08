#!/usr/bin/python3

import rclpy, sys
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from dji_msgs.msg import Topics as DjiTopics
from dji_msgs.msg import PsdkTopics as PSDKTopics


class ServiceCaller():
    def __init__(self, node: Node):
        self._node = node

        self._node.declare_parameter("robot_name", "M350")
        self.ROBOT_NAME : str = self._node.get_parameter("robot_name").get_parameter_value().string_value        
        
        # services to take and give-up control + take-off and land
        # call service: obtain/release_ctrl_authority
        self._take_control_srv = node.create_client(Trigger, PSDKTopics.TAKE_CONTROL_SRV)
        self._release_control_srv = node.create_client(Trigger, PSDKTopics.RELEASE_CONTROL_SRV)
        self._takeoff_srv = node.create_client(Trigger, PSDKTopics.TAKEOFF_SRV)
        self._land_srv = node.create_client(Trigger, PSDKTopics.LAND_SRV)


        if not self._take_control_srv.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error("Take control service not available...")
            sys.exit(1)
        if not self._release_control_srv.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error("Release control service not available...")
            sys.exit(1)
        if not self._takeoff_srv.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error("Take off service not available...")
            sys.exit(1)
        if not self._land_srv.wait_for_service(timeout_sec=5.0):
            self._node.get_logger().error("Land service not available...")
            sys.exit(1)

        while rclpy.ok():
            commands = "Commands:\n"
            commands += "  1: Take control\n"
            commands += "  2: Release control\n"
            commands += "  3: Take off\n"
            commands += "  4: Land\n"
            commands += "  0: EXIT\n"
            commands += "Enter command: "

            try:
                user_input = input(commands)
                if user_input == "1":
                    future = self._take_control_srv.call_async(Trigger.Request())
                    future.add_done_callback(lambda future: self._node.get_logger().info("Take control response: " + str(future.result().success)))
                elif user_input == "2":
                    future = self._release_control_srv.call_async(Trigger.Request())
                    future.add_done_callback(lambda future: self._node.get_logger().info("Release control response: " + str(future.result().success)))
                elif user_input == "3":
                    future = self._takeoff_srv.call_async(Trigger.Request())
                    future.add_done_callback(lambda future: self._node.get_logger().info("Take off response: " + str(future.result().success)))
                elif user_input == "4":
                    future = self._land_srv.call_async(Trigger.Request())
                    future.add_done_callback(lambda future: self._node.get_logger().info("Land response: " + str(future.result().success)))
                elif user_input == "0":
                    self._node.get_logger().info("Exiting...")
                    sys.exit(0)
                else:
                    self._node.get_logger().warn("Invalid command.")
                    continue

            except Exception as e:
                self._node.get_logger().error("Error: " + str(e))
                sys.exit(1)

        
            
    

def main():
    rclpy.init(args=sys.argv)
    node = Node("ServiceCaller")
    ServiceCaller(node)

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()


if __name__ == "__main__":
    main()
