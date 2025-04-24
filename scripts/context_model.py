#!/usr/bin/env python3

from rebet_msgs.srv import GetContextVar
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import queue
import threading
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
from statistics import mean
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters
from rcl_interfaces.msg import ParameterValue
from rclpy.executors import ExternalShutdownException
import sys

class ContextModel(Node):

    def __init__(self):
        self.context_dict = {
            "nearest_object": self.calc_nearest_object,
            "movement_power": self.calc_movement_power,
            "current_velocity": self.calc_current_velocity,
            "maximum_velocity": self.get_max_velocity,
        }
        super().__init__("context_model")

        self.get_logger().info("Initializing the context_model node...")

        exclusive_group = MutuallyExclusiveCallbackGroup()

        self.srv = self.create_service(
            GetContextVar, '/get_context_var',
            self.get_context_var_callback,
            callback_group=MutuallyExclusiveCallbackGroup())

        self.laserscan_subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.ls_scan_cb,
            1,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        self.odometry_subscription = self.create_subscription(
            Odometry,
            "/odom",
            self.tb_odom_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.cli = self.create_client(GetParameters, '/velocity_smoother/get_parameters',callback_group=MutuallyExclusiveCallbackGroup())


        self.odom_msgs = queue.Queue()
        self.laser_msg = None
        self.odom_queue_lock = threading.Lock()

        self.get_logger().info("Initialization finished the following context variables are available:")
        for context_variable_name in self.context_dict.keys():
            self.get_logger().info(f" - {context_variable_name}")


    def calc_linear_velocity(self, odom_msg):

        # if linear x is super small, replace with 0.0
        if odom_msg.twist.twist.linear.x < 0.001:
            odom_msg.twist.twist.linear.x = 0.0
        if odom_msg.twist.twist.linear.y < 0.001:
            odom_msg.twist.twist.linear.y = 0.0
        return math.hypot(abs(odom_msg.twist.twist.linear.x),
                          abs(odom_msg.twist.twist.linear.y))

    def access_context(self, context_variable_name):
        return self.context_dict.get(context_variable_name, lambda: None)()

    def send_request_async(self, client, request):
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        future = client.call_async(request)

        while rclpy.spin_until_future_complete(self, future):
            self.get_logger().info("Waiting for future to complete")

        if future.done():
            return future.result()
        else:
            self.get_logger().error("Service call failed or did not complete.")
            return None

    def get_max_velocity(self):
        self.get_logger().info("Getting max velocity")
        self.req = GetParameters.Request()
        self.req.names = ['max_velocity']

        res = self.send_request_async(self.cli, self.req)


        return res.values[0]

    def calc_nearest_object(self):
        laser_min = self.laser_msg.range_min
        laser_max = self.laser_msg.range_max

        # Initialize nearest_object to a value greater than laser_max
        nearest_object = laser_max * 2
        laser_min = 0.28 # mirte sees itself with its laser, so we need to make sure that the min is not too small.
        # Iterate through the laser ranges to find the nearest object
        for laser_dist in self.laser_msg.ranges:
            if laser_min < laser_dist < laser_max:
                nearest_object = min(nearest_object, laser_dist)

        self.get_logger().info(f"Nearest object distance: {nearest_object}")

        self.get_logger().info(f"Laser min: {laser_min}")
        self.get_logger().info(f"Laser max: {laser_max}")

        # Normalize the nearest object distance
        self._fitted_nearest = (nearest_object - laser_min) / (laser_max - laser_min)

        # Clamp the metric between 0.0 and 1.0
        return max(0.0, min(1.0, self._fitted_nearest))

    def calc_current_velocity(self):
        with self.odom_queue_lock:
            if self.odom_msgs.empty():
                return 0.0

            # Get the most recent odometry message
            odom_msg = self.odom_msgs.queue[-1]
            return self.calc_linear_velocity(odom_msg)

    def calc_movement_power(self):
        with self.odom_queue_lock:
            if self.odom_msgs.empty():
                return 0.0

            # power over the last second of movement.
            speed = mean([self.calc_linear_velocity(odom_msg)
                         for odom_msg in self.odom_msgs.queue])
            return 6.25 * pow(speed, 2) + 9.79 * speed + 3.66

    def ls_scan_cb(self, msg): self.laser_msg = msg

    def tb_odom_cb(self, msg):
        with self.odom_queue_lock:
            self.odom_msgs.put_nowait(msg)

            # Get the current time from the message's timestamp
            current_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

            # Remove messages older than 1 second
            while not self.odom_msgs.empty():
                oldest_msg = self.odom_msgs.queue[0]  # Peek at the oldest message
                oldest_time = oldest_msg.header.stamp.sec + oldest_msg.header.stamp.nanosec * 1e-9

                if current_time - oldest_time > 1.0:
                    self.odom_msgs.get_nowait()  # Remove the oldest message
                else:
                    break  # Stop removing messages if they are within the 1-second window

    def get_context_var_callback(self, request, response):

        self.get_logger().info(
            "Incoming request: %s" % request.variable_name
        )

        context_value = self.access_context(request.variable_name)

        if context_value is not None:
            self.get_logger().info(
                "Context value: %s" % context_value
            )
            # check if the context_value is a ParameterValue object already or not
            if (isinstance(context_value, ParameterValue)):
                response.variable_value = context_value
            else:
                response.variable_value = Parameter(name="", value=context_value).get_parameter_value()
        else:
            self.get_logger().error(
                "Context variable not found: %s" % request.variable_name
            )
        return response


def main():
    rclpy.init()

    node = ContextModel()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
