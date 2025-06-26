#pragma once

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rebet_school/adaptations.hpp"
#include "rebet/rebet_utilities.hpp"

class SimpleAdaptMaxVelocity : public AdaptNavToPose
{
  public:

    SimpleAdaptMaxVelocity(const std::string& name, const NodeConfig& config) : AdaptNavToPose(name, config)
    {
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptNavToPose::providedPorts();

      PortsList child_ports =  {
        InputPort<std::string>(NEARBY_IN,"status of nearby QR, whether violated or not"),
      };
      child_ports.merge(base_ports);

      return child_ports;
    }

    // When this method returns true, adaptations will be performed.
    bool evaluate_condition() override
    {
        if(!AdaptNavToPose::evaluate_condition()) {
            return false;
        }

        std::string nearby_status;
        if(!getInput(NEARBY_IN, nearby_status)) {
            return false;
        }
        
        if(nearby_status == "VIOLATED"){
            // This returns true if it is possible to lower the max velocity.
            return decrease_velocity();
        }
       
        // If the nearby QRs status is not violated, we can increase the velocity.
        return increase_velocity();
    }

    static constexpr const char* NEARBY_IN = "in_nearby_qr";
};

class ComplexAdaptMaxVelocity : public AdaptNavToPose
{
  public:

    ComplexAdaptMaxVelocity(const std::string& name, const NodeConfig& config) : AdaptNavToPose(name, config)
    {
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptNavToPose::providedPorts();

      PortsList child_ports =  {
        InputPort<std::string>(NEARBY_IN,"status of nearby QR, whether violated or not"),
        InputPort<std::string>(INTHEWAY_IN, "status of in the way QR, whether violated or not"),
        InputPort<double>(PROXIMITY_IN, "how close as a percentage of the lidar range is the nearest object"),
      };
      child_ports.merge(base_ports);

      return child_ports;
    }

    // When this method returns true, adaptations will be performed.
    bool evaluate_condition() override
    {
      RCLCPP_INFO(node_->get_logger(), "ComplexAdaptMaxVelocity evaluate_condition called");
      if(!AdaptNavToPose::evaluate_condition()) {
          return false;
      }

      std::string nearby_status;
      if(!getInput(NEARBY_IN, nearby_status)) {
          return false;
      }


        // Insert more complex logic here which considers multiple quality requirements.
        
    }

    static constexpr const char* NEARBY_IN = "in_nearby_qr";
    static constexpr const char* INTHEWAY_IN = "in_intheway_qr";
    static constexpr const char* PROXIMITY_IN = "in_proximity";

};

class AdaptPlanner : public AdaptNavToPose
{
  public:

    AdaptPlanner(const std::string& name, const NodeConfig& config) : AdaptNavToPose(name, config)
    {
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptNavToPose::providedPorts();

      PortsList child_ports =  {
        InputPort<std::string>(CPU_IN,"status of CPU QR, whether violated or not"),
      };
      child_ports.merge(base_ports);

      return child_ports;
    }

    // When this method returns true, adaptations will be performed.
    bool evaluate_condition() override
    {
        if(!AdaptNavToPose::evaluate_condition()) {
            return false;
        }


        //These are your adaptation options.
        //return change_path_planner(PathPlanner::SMAC);
        //return change_path_planner(PathPlanner::NavFn);

        //SMAC uses more CPU than NavFn.
    }

    static constexpr const char* CPU_IN = "in_cpu_limit_qr";
};