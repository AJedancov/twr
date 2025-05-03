#include "rclcpp/rclcpp.hpp"
#include "nav2_core/controller.hpp"
#include "pluginlib/class_loader.hpp"
#include "pluginlib/class_list_macros.hpp"

#ifndef TWR_CONTROLLER_HPP_
#define TWR_CONTROLLER_HPP_

namespace twr_g2g_controller
{
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
  double desired_linear_vel_;
  double max_lin_vel;
  double max_ang_vel;

  double Kp_ang_vel; // Proportional gain
  bool debug_info;
};

} // namespace twr_controller


#endif  // TWR_CONTROLLER_HPP_