#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
import sys
from btcpp_ros2_interfaces.action import ExecuteTree
from btcpp_ros2_interfaces.msg import NodeStatus
import pandas as pd
import json
import time

got_response = False


class TreeActionClient(Node):

    def __init__(self):
        super().__init__("tree_action_client")
        self._action_client = ActionClient(self, ExecuteTree, "behavior_server")

        self.declare_parameter("autostart", False)

        self.results = pd.DataFrame()

    def send_goal(self, tree_name):
        if self.get_parameter("autostart").value or not input("Press enter to start"):
            goal_msg = ExecuteTree.Goal()

            goal_msg.target_tree = tree_name

            self._action_client.wait_for_server()

            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg, feedback_callback=self.feedback_callback
            )

            self._send_goal_future.add_done_callback(self.goal_response_callback)

            return self._send_goal_future

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")
            return

        self.get_logger().info("Goal accepted :)")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        global got_response

        result = future.result().result

        self.get_logger().info("END MESSAGE " + str(result.return_message))
        if result.node_status.status == NodeStatus.SUCCESS:
            self.get_logger().info("Result: Done ticking BT, ended on Success")
        elif result.node_status.status == NodeStatus.FAILURE:
            self.get_logger().info("Result: Done ticking BT, ended on Failure")
        else:
            self.get_logger().info(
                "Result: Done ticking BT, ended with unexpected NodeStatus"
            )

        self.results.to_csv(str(int(time.time())) + "_results.csv")

        got_response = True

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        blackboard_json = json.loads(feedback.message)

        self.results = pd.concat([self.results, pd.DataFrame([blackboard_json])])


def main():
    if len(sys.argv) < 2:
        print("Missing bt_name argument!")
        sys.exit(1)

    rclpy.init()

    action_client = TreeActionClient()

    action_client.send_goal(str(sys.argv[1]))

    while not got_response:
        rclpy.spin_once(action_client)

    action_client.destroy_node()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
