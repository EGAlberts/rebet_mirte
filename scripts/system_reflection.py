#!/usr/bin/env python3

from rebet_msgs.srv import SetAttributesInBlackboard
from rebet_msgs.msg import SystemAttributeValue, SystemAttribute
from sensor_msgs.msg import BatteryState
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Range, BatteryState
from diagnostic_msgs.msg import KeyValue, DiagnosticArray
from std_msgs.msg import Float32
from rclpy.clock import Clock


class SystemReflection(Node):

    def __init__(self):

        super().__init__("system_reflection")

        self.cli = self.create_client(
            SetAttributesInBlackboard,
            "/set_attributes_in_blackboard",
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.laserscan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.ls_scan_cb,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.req = SetAttributesInBlackboard.Request()

        self.laser_msg = None

        self.msg_timestamp = None

        self.get_logger().info("System Reflection node created")

        self.clock = Clock()

        self.time_monitor_timer = self.create_timer(2, self.process_reflection,
                                                    callback_group=MutuallyExclusiveCallbackGroup())

    def ls_scan_cb(self, msg):
        self.laser_msg = msg
        self.get_logger().info("Receiving LaserScans", throttle_duration_sec=20,
                               throttle_time_source_type=self.clock)


    def _append_sys_attribute(self, name, msg):
        if msg is None:
            return

        new_sys_att = SystemAttribute()
        new_sys_att.name = name
        if isinstance(msg, Odometry):
            value_type = 1
            value_field = "odom_value"
        elif isinstance(msg, KeyValue):
            value_type = 2
            value_field = "diag_value"
        elif isinstance(msg, LaserScan):
            value_type = 3
            value_field = "laser_value"
        elif isinstance(msg, Float32):
            value_type = 4
            value_field = "float_value"
        elif isinstance(msg, Range):
            value_type = 5
            value_field = "range_value"
        elif isinstance(msg, BatteryState):
            value_type = 6
            value_field = "battery_value"
        else:
            # Add more types as needed
            return

        att_value = SystemAttributeValue()
        att_value.header.stamp = self.msg_timestamp
        att_value.type = value_type
        setattr(att_value, value_field, msg)
        new_sys_att.value = att_value
        self.req.sys_attributes.append(new_sys_att)

    def process_reflection(self):
        self.get_logger().info("Processing reflections")

        self.msg_timestamp = self.get_clock().now().to_msg()

        self._append_sys_attribute("laser_scan", self.laser_msg)

        if len(self.req.sys_attributes) > 0:
            self.get_logger().info("Trying to call set att...")

            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("service not available, waiting again...")
            response = self.cli.call(self.req)
            self.get_logger().info(
                "Result of call to set attribute in blackboard " + str(response.success)
            )
            self.req.sys_attributes = []
        else:
            self.get_logger().info("No attributes to set in blackboard")

        self.get_logger().info("Finished trying to set attributes in blackboard")

def main():
    rclpy.init()

    sys_reflec_node = SystemReflection()

    mt_executor = MultiThreadedExecutor()
    mt_executor.add_node(sys_reflec_node)

    mt_executor.spin()

    sys_reflec_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
