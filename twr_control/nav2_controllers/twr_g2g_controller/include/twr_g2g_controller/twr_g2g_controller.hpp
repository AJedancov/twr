#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"

#ifndef TWR_CONTROLLER_HPP_
#define TWR_CONTROLLER_HPP_

namespace twr_g2g_controller
{

struct Parameters{
  double max_lin_vel;
  double max_ang_vel;
  double Kp_ang_vel; // Proportional gain
  bool debug_info;
  std::string local_frame;
};

class G2GController: public nav2_core::Controller
{
public:
  G2GController() = default;
  ~G2GController() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
  ) override;


  void cleanup() override;
  void activate() override;
  void deactivate() override;
  void setSpeedLimit(const double & speed_limit, const bool & percentage) override;
  void setPlan(const nav_msgs::msg::Path & path) override;

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & robot_pose,
    const geometry_msgs::msg::Twist & robot_velocity,
    nav2_core::GoalChecker * goal_checker
  ) override;


protected:
  rclcpp_lifecycle::LifecycleNode::WeakPtr node_;
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Logger logger_ {rclcpp::get_logger("G2GController")};
  std::string plugin_name_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;

  nav_msgs::msg::Path global_plan_;

  // Node parameters
  Parameters params_;

private:

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr params_callback_handle_;
  rcl_interfaces::msg::SetParametersResult paramsCallback(const std::vector<rclcpp::Parameter> &params){
  
    rcl_interfaces::msg::SetParametersResult result;

    for(auto &param : params){
      if (param.get_name() == plugin_name_ + ".max_lin_vel"){
        params_.max_lin_vel = param.as_double();
      }else if (param.get_name() == plugin_name_ + ".max_ang_vel"){
        params_.max_ang_vel = param.as_double();
      }else if (param.get_name() == plugin_name_ + ".Kp_ang_vel"){
        params_.Kp_ang_vel = param.as_double();
      }else if (param.get_name() == plugin_name_ + ".debug_info"){
        params_.debug_info = param.as_bool();
      }else if (param.get_name() == plugin_name_ + ".local_frame"){
        params_.local_frame = param.as_string();
      }
    }

    result.successful = true;
    return result;
  }
};

} // namespace twr_controller


#endif  // TWR_CONTROLLER_HPP_