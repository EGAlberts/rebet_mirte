#pragma once

#include "rclcpp/rclcpp.hpp"

#include "builtin_interfaces/msg/time.hpp"
#include <algorithm>
#include <chrono>
#include <ctime> 
#include "rebet/system_attribute_value.hpp"
#include "rcl_interfaces/msg/parameter_value.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rebet/adapt_node.hpp"
#include "rebet/rebet_utilities.hpp"

enum class PathPlanner {NavFn, SMAC};

class AdaptNavToPose : public AdaptPeriodicallyOnRunning<double>
{
  public:

    AdaptNavToPose(const std::string& name, const NodeConfig& config) : AdaptPeriodicallyOnRunning<double>(name, config, AdaptationTarget::RosParameter, AdaptationType::Internal)
    {
      //Since we are only interested in modifying the x-axis (backwards/forwards) velocity, we wrap the adaptation_options given into the required triple of x y theta velocitys.
        std::vector<double> param_values;
        std::string param_name;
        std::string node_name;
        getInput(ADAP_OPT, param_values);
        getInput(ADAP_SUB, param_name);
        getInput(ADAP_LOC, node_name);
    }

    void velocity_adaptation_msg(double max_velocity_value)
    {
      _current_max_velocity = max_velocity_value;

      std::vector<double> velocity_vector = {max_velocity_value,_default_y_velocity,_default_theta_velocity};

      std::string param_name;
      std::string node_name;

      getInput(ADAP_SUB, param_name);
      getInput(ADAP_LOC, node_name);

      aal_msgs::msg::Adaptation adap;
      rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue(velocity_vector));
      adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::RosParameter);
      adap.parameter_adaptation = adap_param.to_parameter_msg();
      adap.node_name = node_name;

      _internal_adaptations.push_back(adap);

    }

    void planner_adaptation_msg(std::string planner_path)
    {
      std::string param_name;
      std::string node_name;

      getInput(ADAP_SUB, param_name);
      getInput(ADAP_LOC, node_name);

      aal_msgs::msg::Adaptation adap;
      rclcpp::Parameter adap_param = rclcpp::Parameter(param_name,rclcpp::ParameterValue(planner_path));
      adap.adaptation_target = static_cast<int8_t>(AdaptationTarget::RosParameter);
      adap.parameter_adaptation = adap_param.to_parameter_msg();
      adap.node_name = node_name;

      _internal_adaptations.push_back(adap);
    }

    bool change_path_planner(PathPlanner planner_name)
    {
      if(planner_name == _current_path_planner)
      {
        return false;
      }

      _current_path_planner = planner_name;

      switch(planner_name)
      {
        case PathPlanner::NavFn:
          planner_adaptation_msg(NAV_FN);
          break;
        case PathPlanner::SMAC:
          planner_adaptation_msg(SMAC);
          break;
        default:
          return false;
      }

      return true;
    }

    bool decrease_max_velocity()
    {
      _internal_adaptations = {};

      double new_max_velocity = _current_max_velocity - VELOCITY_INCREMENT;

      if((new_max_velocity + EPSILON) < MIN_VELOCITY)
      {
        return false;
      }

      velocity_adaptation_msg(new_max_velocity);

      return true;
    }

    bool increase_max_velocity()
    {
      _internal_adaptations = {};

      double new_max_velocity = _current_max_velocity + VELOCITY_INCREMENT;

      if(new_max_velocity > MAX_MAX_VELOCITY)
      {
        return false;
      }

      velocity_adaptation_msg(new_max_velocity);

      return true;
    }

    static PortsList providedPorts()
    {
      PortsList base_ports = AdaptPeriodicallyOnRunning::providedPorts();

      PortsList child_ports =  {   
              };
      child_ports.merge(base_ports);

      return child_ports;
    }

  private:
    double _default_y_velocity = 0.0; //The robot in question can not move along its y axis independently.
    double _default_theta_velocity = 2.5; //We are not interested in modifying the rotation velocity here, but could be extended to do so.
    const double VELOCITY_INCREMENT = 0.30;
    const double MIN_VELOCITY = 0.10;
    const double MAX_MAX_VELOCITY = 1.0;
    const double EPSILON = 0.01; //Floating-point tolerance..
    double _current_max_velocity = MAX_MAX_VELOCITY;
    static constexpr const char* NAV_FN = "navfn_navigate_to_pose.xml";
    static constexpr const char* SMAC = "smac_navigate_to_pose.xml";
    PathPlanner _current_path_planner = PathPlanner::NavFn;
};