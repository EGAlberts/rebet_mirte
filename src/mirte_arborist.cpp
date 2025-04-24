#include "rebet/adapt_node.hpp"

#include "rebet/arborist.hpp"
#include "rebet/json_serialization.hpp"

#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/json_export.h"
#include "nav_msgs/msg/odometry.hpp"
#include <nlohmann/json.hpp>


class MirteArborist : public Arborist
{
public:
  int time_since_last = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  int total_elapsed = 0;
  int time_limit = 300;
  bool _publish_feedback = false;

  MirteArborist(const rclcpp::NodeOptions & options)
  : Arborist(options) {}

  void registerNodesIntoFactory(BT::BehaviorTreeFactory & factory) override
  {
    //I suppose here you register all the possible custom nodes, and the determination as to whether they are actually used lies in the xml tree provided.
    factory.registerNodeType<AdaptOnConditionAny>("AdaptOnConditionAny");
  }

  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus status) override
  {
    _publish_feedback = false;

    auto curr_time_pointer = std::chrono::system_clock::now();


    int current_time = std::chrono::duration_cast<std::chrono::seconds>(
      curr_time_pointer.time_since_epoch()).count();
    int elapsed_seconds = current_time - time_since_last;
    // Every second I record an entry
    if (elapsed_seconds >= 1) {
      RCLCPP_INFO(
        node()->get_logger(), "%d seconds have passed, current_status %s", total_elapsed,
        toStr(status).c_str());
    }


    if (total_elapsed >= time_limit) {
      return BT::NodeStatus::SUCCESS;
    }

    return std::nullopt;
  }

  std::optional<std::string> onLoopFeedback() override
  {
    if (_publish_feedback) {return ExportBlackboardToJSON(*globalBlackboard()).dump();}
    return std::nullopt;
  }

  virtual std::optional<std::string> onTreeExecutionCompleted(
    BT::NodeStatus status,
    bool was_cancelled)
  {
    RCLCPP_INFO(node()->get_logger(), "Was cancelled?: %s", was_cancelled ? "true" : "false");
    if (status == BT::NodeStatus::SUCCESS) {
      return "Ended well";
    }
    return "Ended Poorly";
  }
};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto action_server = std::make_shared<MirteArborist>(options);

  // TODO: This workaround is for a bug in MultiThreadedExecutor where it can deadlock when spinning without a timeout.
  // Deadlock is caused when Publishers or Subscribers are dynamically removed as the node is spinning.
  rclcpp::executors::MultiThreadedExecutor exec(rclcpp::ExecutorOptions(), 0, false,
    std::chrono::milliseconds(250));
  exec.add_node(action_server->node());
  exec.spin();
  exec.remove_node(action_server->node());

  rclcpp::shutdown();

}
