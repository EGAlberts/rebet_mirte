#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include <ament_index_cpp/get_package_share_directory.hpp>

using NavigateToPose = nav2_msgs::action::NavigateToPose;
using PoseStamped = geometry_msgs::msg::PoseStamped;
using Pose = geometry_msgs::msg::Pose;

namespace BT
{

template<>
[[nodiscard]] Pose convertFromString<Pose>(StringView str);

template<>
[[nodiscard]] std::string toStr<Pose>(const Pose & direction);

template<>
Pose convertFromString<Pose>(StringView str)
{
  std::vector<double> pose_parts = convertFromString<std::vector<double>>(str);
  if (pose_parts.size() != 7) {
    throw RuntimeError(
            std::string("Cannot convert this to Pose: ") +
            static_cast<std::string>(str));
  }
  Pose pose;
  pose.position.x = pose_parts[0];
  pose.position.y = pose_parts[1];
  pose.position.z = pose_parts[2];
  pose.orientation.x = pose_parts[3];
  pose.orientation.y = pose_parts[4];
  pose.orientation.z = pose_parts[5];
  pose.orientation.w = pose_parts[6];
  return pose;
}

template<>
std::string toStr<Pose>(const Pose & pose)
{
  std::stringstream ss;
  ss << pose.position.x << ";" << pose.position.y << ";" << pose.position.z << ";"
     << pose.orientation.x << ";" << pose.orientation.y << ";" << pose.orientation.z
     << ";" << pose.orientation.w;
  return ss.str();
}

std::ostream & operator<<(std::ostream & os, const Pose & pose)
{
  os << toStr(pose);
  return os;
}


class NavigateToPoseAction : public RosActionNode<NavigateToPose>
{
public:
  //Name for the pose input port
  static constexpr const char * POSE = "pose";

  NavigateToPoseAction(
    const std::string & name,
    const NodeConfig & conf,
    const RosNodeParams & params)
  : RosActionNode<NavigateToPose>(name, conf, params)
  {
    std::cout << "Someone made me (an NavigateToPoseAction Action Nodee)" << std::endl;

    // RCLCPP_INFO(logger(), node_->get_name());

  }

  static PortsList providedPorts()
  {
    PortsList base_ports = RosActionNode::providedPorts();
    PortsList child_ports = {
      InputPort<Pose>(POSE),
      InputPort<std::string>("planner"),
    };
    child_ports.merge(base_ports);
    return child_ports;
  }

  bool setGoal(RosActionNode::Goal & goal) override
  {
    RCLCPP_INFO(logger(), "rego");
    RCLCPP_INFO(logger(), registrationName().c_str());
    setOutput("name_of_task", registrationName());
    // #goal definition
    // geometry_msgs/PoseStamped pose
    // string behavior_tree
    std::stringstream ss;

    ss << "setGoal in pose";

    goal.behavior_tree = "";
    std::string planner;
    getInput(POSE, pose_to_navigate_to);
    getInput("planner", planner);

    std::string bt_name = ament_index_cpp::get_package_share_directory("rebet_mirte") + "/trees/nav2/" + planner;
    PoseStamped stamped_pose;
    stamped_pose.header.frame_id = "map";

    RCLCPP_INFO(logger(), "now() before assignment: %.9f", now().seconds());
    stamped_pose.header.stamp = now();
    stamped_pose.pose = pose_to_navigate_to;

    goal.pose = stamped_pose;
    goal.behavior_tree = bt_name;

    RCLCPP_INFO(logger(), ss.str().c_str());

    return true;
  }

  // Callback executed when the reply is received.
  // Based on the reply you may decide to return SUCCESS or FAILURE.
  NodeStatus onResultReceived(const WrappedResult & wr) override
  {
    // #result definition
    // std_msgs/Empty result
    std::stringstream ss;
    ss << "NavigateToPose Result received";
    //Unfortunately, NavigateToPose in Nav2 right now provides no actual result indicating you reached the pose or not..

    RCLCPP_INFO(logger(), ss.str().c_str());

    RCLCPP_INFO(logger(), "SUCCESS IN RESULT RCV GOTOPOSE");

    return NodeStatus::SUCCESS;
  }

  virtual NodeStatus onFailure(ActionNodeErrorCode error) override
  {
    RCLCPP_INFO(logger(), "Here we are");
    RCLCPP_ERROR(logger(), "Navigate to Pose Error: %d", error);
    return NodeStatus::FAILURE;
  }

  NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback)
  {
    return NodeStatus::RUNNING;
  }

private:
  Pose pose_to_navigate_to;
};

BT_REGISTER_ROS_NODES(factory, params)
{
  RosNodeParams aug_params;
  aug_params.nh = params.nh;
  aug_params.server_timeout = std::chrono::milliseconds(40000);   //Nav2 can take a while to respond, especialy in a container.
  //TODO: options.use_global_arguments(false) need to fix this for plguins somehow. also applies to client name
  factory.registerNodeType<NavigateToPoseAction>("roamTo", aug_params);
}

}
