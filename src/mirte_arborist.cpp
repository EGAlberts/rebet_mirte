#include "rebet/adapt_node.hpp"

#include "rebet/arborist.hpp"
#include "rebet/json_serialization.hpp"
#include "rebet_school/quality_requirements.hpp"
#include "rebet_school/adapt_navtopose.hpp"
#include "std_msgs/msg/string.hpp"
#include "behaviortree_cpp/json_export.h"
#include "nav_msgs/msg/odometry.hpp"
#include <nlohmann/json.hpp>
#include <behaviortree_cpp/loggers/bt_cout_logger.h>
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include "behaviortree_cpp/xml_parsing.h"



class MirteArborist : public Arborist
{
public:
  int time_since_last = std::chrono::duration_cast<std::chrono::seconds>(
    std::chrono::system_clock::now().time_since_epoch()).count();
  int total_elapsed = 0;
  int time_limit = 300;
  bool _publish_feedback = false;
  rclcpp::Clock::SharedPtr clock_;

  MirteArborist(const rclcpp::NodeOptions & options)
  : Arborist(options) {
    node()->declare_parameter("log_blackboard", false);
    node()->declare_parameter("factory_xml", false);
  }

  void onTreeCreated(BT::Tree& tree) override
  {
    logger_cout_ = std::make_shared<BT::StdCoutLogger>(tree);
    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    bool factory_xml = false;
    node()->get_parameter("factory_xml", factory_xml);
    if(factory_xml) {
      const std::string xml_models = BT::writeTreeNodesModelXML(factory());
      std::cout << "Registered nodes into factory: " << xml_models << std::endl;
    }
  }

  void registerNodesIntoFactory(BT::BehaviorTreeFactory & factory) override
  {
    factory.registerNodeType<NoObjectsNearbyQR>("NoObjectsNearby");
    factory.registerNodeType<CPULimitQR>("CPULimit");
    factory.registerNodeType<SimpleAdaptMaxVelocity>("SimpleAdaptMaxVelocity");
    factory.registerNodeType<ComplexAdaptMaxVelocity>("ComplexAdaptMaxVelocity");
    factory.registerNodeType<InTheWayQR>("InTheWay");
    factory.registerNodeType<AdaptPlanner>("AdaptPlanner");
  }

  std::optional<BT::NodeStatus> onLoopAfterTick(BT::NodeStatus /*status*/) override
  {
    _publish_feedback = true;

    auto curr_time_pointer = std::chrono::system_clock::now();


    int current_time = std::chrono::duration_cast<std::chrono::seconds>(
      curr_time_pointer.time_since_epoch()).count();
    int elapsed_seconds = current_time - time_since_last;
    // Every second I record an entry
    // if (elapsed_seconds >= 1) {
    //   RCLCPP_INFO(
    //     node()->get_logger(), "%d seconds have passed, current_status %s", total_elapsed,
    //     toStr(status).c_str());
    // }


    if (total_elapsed >= time_limit) {
      return BT::NodeStatus::SUCCESS;
    }

    return std::nullopt;
  }

  std::optional<std::string> onLoopFeedback() override
  {
    bool to_log = false;
    node()->get_parameter("log_blackboard", to_log);

    auto json_obj = ExportBlackboardToJSON(*globalBlackboard());
    summarize_laserscan_json(json_obj);
    auto pretty_json = json_obj.dump(2);

    if (to_log) {
      RCLCPP_INFO_THROTTLE(
        node()->get_logger(),
        *clock_,
        5000,
        "Blackboard contents:\n%s",
        pretty_json.c_str()
      );
    }
    // std::cout << ExportBlackboardToJSON(*globalBlackboard()).dump() << std::endl;
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

  private:
    std::shared_ptr<BT::StdCoutLogger> logger_cout_;
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
