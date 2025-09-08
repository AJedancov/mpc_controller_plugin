#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"


#ifndef PARAMETERS_MANAGER_HPP_
#define PARAMETERS_MANAGER_HPP_


struct Parameters{ 
  double dt = 0.0;
  double max_lin_vel = 0.0;
  double min_lin_vel = 0.0;
  double max_ang_vel = 0.0;
  double min_ang_vel = 0.0;
  int prediction_horizon = 0;
  int nx = 0;
  int nu = 0;
  int ny = 0;
  bool use_input_increment = true;
  std::vector<double> state_weights_diag;
  std::vector<double> control_weights_diag;
};


class ParameterManager{
public:

  ParameterManager(){}

  void configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent,
    const std::string& name)
  {
    node_ = parent.lock();
    plugin_name_ = name;

    params_callback_handle_ = node_->add_post_set_parameters_callback(
      std::bind(&ParameterManager::updateParametersCallback, this, std::placeholders::_1)
    );

    node_->declare_parameter(plugin_name_ + ".dt", rclcpp::ParameterValue(0.05));
    node_->declare_parameter(plugin_name_ + ".max_lin_vel", rclcpp::ParameterValue(0.5));
    node_->declare_parameter(plugin_name_ + ".min_lin_vel", rclcpp::ParameterValue(-0.5));
    node_->declare_parameter(plugin_name_ + ".max_ang_vel", rclcpp::ParameterValue(1.0));
    node_->declare_parameter(plugin_name_ + ".min_ang_vel", rclcpp::ParameterValue(-0.5));
    node_->declare_parameter(plugin_name_ + ".prediction_horizon", rclcpp::ParameterValue(5));
    node_->declare_parameter(plugin_name_ + ".nx", rclcpp::ParameterValue(3));
    node_->declare_parameter(plugin_name_ + ".nu", rclcpp::ParameterValue(2));
    node_->declare_parameter(plugin_name_ + ".ny", rclcpp::ParameterValue(3));
    node_->declare_parameter(plugin_name_ + ".use_input_increment", rclcpp::ParameterValue(false));
    node_->declare_parameter(plugin_name_ + ".state_weights_diag", rclcpp::ParameterValue(state_weights_diag));
    node_->declare_parameter(plugin_name_ + ".control_weights_diag", rclcpp::ParameterValue(control_weights_diag));

    node_->get_parameter(plugin_name_ + ".dt", params_.dt);
    node_->get_parameter(plugin_name_ + ".max_lin_vel", params_.max_lin_vel);
    node_->get_parameter(plugin_name_ + ".min_lin_vel", params_.min_lin_vel);
    node_->get_parameter(plugin_name_ + ".max_ang_vel", params_.max_ang_vel);
    node_->get_parameter(plugin_name_ + ".min_ang_vel", params_.min_ang_vel);
    node_->get_parameter(plugin_name_ + ".prediction_horizon", params_.prediction_horizon);
    node_->get_parameter(plugin_name_ + ".nx", params_.nx);
    node_->get_parameter(plugin_name_ + ".nu", params_.nu);
    node_->get_parameter(plugin_name_ + ".ny", params_.ny);
    node_->get_parameter(plugin_name_ + ".use_input_increment", params_.use_input_increment);
    node_->get_parameter(plugin_name_ + ".state_weights_diag", params_.state_weights_diag);
    node_->get_parameter(plugin_name_ + ".control_weights_diag", params_.control_weights_diag);
  }

  Parameters* get_parameters(){
    return &params_;
  }

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string plugin_name_;
  Parameters params_;
  std::vector<double> state_weights_diag{10, 10, 10};
  std::vector<double> control_weights_diag{0.1, 0.1};
  
  rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr params_callback_handle_;

  rcl_interfaces::msg::SetParametersResult updateParametersCallback(const std::vector<rclcpp::Parameter> &params){
    (void) params;

    // # TODO: optimize parameter update
    node_->get_parameter(plugin_name_ + ".dt", params_.dt);
    node_->get_parameter(plugin_name_ + ".max_lin_vel", params_.max_lin_vel);
    node_->get_parameter(plugin_name_ + ".min_lin_vel", params_.min_lin_vel);
    node_->get_parameter(plugin_name_ + ".max_ang_vel", params_.max_ang_vel);
    node_->get_parameter(plugin_name_ + ".min_ang_vel", params_.min_ang_vel);
    node_->get_parameter(plugin_name_ + ".prediction_horizon", params_.prediction_horizon);
    node_->get_parameter(plugin_name_ + ".nx", params_.nx);
    node_->get_parameter(plugin_name_ + ".nu", params_.nu);
    node_->get_parameter(plugin_name_ + ".ny", params_.ny);
    node_->get_parameter(plugin_name_ + ".use_input_increment", params_.use_input_increment);
    node_->get_parameter(plugin_name_ + ".state_weights_diag", params_.state_weights_diag);
    node_->get_parameter(plugin_name_ + ".control_weights_diag", params_.control_weights_diag);

    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    return result;
  }
};


#endif  // PARAMETERS_MANAGER_HPP_