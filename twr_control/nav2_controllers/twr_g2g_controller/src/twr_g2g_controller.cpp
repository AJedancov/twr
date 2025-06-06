#include "twr_g2g_controller/twr_g2g_controller.hpp"

namespace twr_g2g_controller
{


void G2GController::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, 
  const std::shared_ptr<tf2_ros::Buffer> tf_buffer,
  const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  auto node = parent.lock();
  node_ = parent;
  tf_buffer_ = tf_buffer;
  plugin_name_ = name;
  (void) costmap_ros;
  
  clock_ = node->get_clock();

  params_callback_handle_ = node->add_on_set_parameters_callback(
    std::bind(&G2GController::paramsCallback, this, std::placeholders::_1)
  );

  node->declare_parameter(plugin_name_ + ".max_lin_vel", rclcpp::ParameterValue(0.5));
  node->declare_parameter(plugin_name_ + ".max_ang_vel", rclcpp::ParameterValue(1.5));
  node->declare_parameter(plugin_name_ + ".Kp_ang_vel", rclcpp::ParameterValue(1.0));
  node->declare_parameter(plugin_name_ + ".debug_info", rclcpp::ParameterValue(false));
  node->declare_parameter(plugin_name_ + ".local_frame", rclcpp::ParameterValue(std::string("odom")));

  node->get_parameter(plugin_name_ + ".max_lin_vel", params_.max_lin_vel);
  node->get_parameter(plugin_name_ + ".max_ang_vel", params_.max_ang_vel);
  node->get_parameter(plugin_name_ + ".Kp_ang_vel", params_.Kp_ang_vel);
  node->get_parameter(plugin_name_ + ".debug_info", params_.debug_info);
  node->get_parameter(plugin_name_ + ".local_frame", params_.local_frame);

}

void G2GController::cleanup(){}

void G2GController::activate(){}

void G2GController::deactivate(){}

void G2GController::setSpeedLimit(const double &speed_limit, const bool &percentage){
  (void) speed_limit;
  (void) percentage;
}

void G2GController::setPlan(const nav_msgs::msg::Path& path){
  global_plan_ = path;
}

geometry_msgs::msg::TwistStamped G2GController::computeVelocityCommands(
  const geometry_msgs::msg::PoseStamped& robot_pose,
  const geometry_msgs::msg::Twist& robot_velocity,
  nav2_core::GoalChecker* goal_checker)
{
  (void) robot_velocity;
  (void) goal_checker;

  // Find the last position in the resulting path
  // auto last_pose_it = std::prev(global_plan_.poses.end());
  // geometry_msgs::msg::PoseStamped goal_pose = *last_pose_it;
  geometry_msgs::msg::PoseStamped goal_pose_in_map_frame = global_plan_.poses.back();
  goal_pose_in_map_frame.header.frame_id = global_plan_.header.frame_id; // global_plan is initially in the map frame
  goal_pose_in_map_frame.header.stamp = clock_->now();
  goal_pose_in_map_frame.header.stamp.sec -= .01;

  // Get goal position in odom frame
  geometry_msgs::msg::PoseStamped goal_pose_in_odom_frame;
  try{
    goal_pose_in_odom_frame = tf_buffer_->transform(goal_pose_in_map_frame, params_.local_frame);
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(logger_, "Exception during transform: %s", ex.what());
  }

  double dx = goal_pose_in_odom_frame.pose.position.x - robot_pose.pose.position.x;
  double dy = goal_pose_in_odom_frame.pose.position.y - robot_pose.pose.position.y;
  
  // double distance_to_goal = std::hypot(dx, dy);
  double angle_to_goal = std::atan2(dy, dx);

  // Get Yaw from Quaternion 
  double yaw = tf2::getYaw(robot_pose.pose.orientation);

  double angle_error = angle_to_goal - yaw;
  // Angle normalization
  if(angle_error > M_PI) angle_error -= 2 * M_PI;
  if(angle_error < -M_PI) angle_error += 2 * M_PI;
  
  if(params_.debug_info){
    RCLCPP_INFO(logger_, 
      "\n  Goal position:  [%lf, %lf, %lf]\n  Robot position: [%lf, %lf, %lf]\n  Position error: [%lf, %lf, %lf]", 
      goal_pose_in_odom_frame.pose.position.x, 
      goal_pose_in_odom_frame.pose.position.y,
      angle_to_goal,
      robot_pose.pose.position.x,
      robot_pose.pose.position.y,
      yaw,
      dx,
      dy,
      angle_error
    );
  }

  double linear_vel, angular_vel;
  angular_vel = params_.Kp_ang_vel * angle_error;
  linear_vel = params_.max_lin_vel / std::sqrt((std::fabs(angular_vel) + 1));

  // Apply speed limits
  linear_vel = std::clamp(linear_vel, -params_.max_lin_vel, params_.max_lin_vel);
  angular_vel = std::clamp(angular_vel, -params_.max_ang_vel, params_.max_ang_vel);

  // Send calculated speed to nav2 controller_server
  geometry_msgs::msg::TwistStamped cmd_vel;
  cmd_vel.header.frame_id = robot_pose.header.frame_id;
  cmd_vel.header.stamp = clock_->now();
  cmd_vel.twist.linear.x = linear_vel;
  cmd_vel.twist.angular.z = angular_vel;

  return cmd_vel;
}


} // namespace twr_g2g_controller

// Register this controller as a nav2_core plugin
PLUGINLIB_EXPORT_CLASS(twr_g2g_controller::G2GController, nav2_core::Controller)