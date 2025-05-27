#include "spbot_planners/rrt_star_planner.hpp"
#include <pluginlib/class_list_macros.hpp>

namespace spbot_planners
{

void RRTStarPlanner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name,
  std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
)
{
  node_        = parent.lock();
  name_        = std::move(name);
  tf_          = std::move(tf);
  costmap_ros_ = std::move(costmap_ros);

  RCLCPP_INFO(node_->get_logger(), "[%s] Configuring", name_.c_str());

  // Declare & read parameters
  node_->declare_parameter("max_iterations", max_iterations_);
  node_->declare_parameter("step_size",       step_size_);
  node_->declare_parameter("search_radius",   search_radius_);

  node_->get_parameter("max_iterations", max_iterations_);
  node_->get_parameter("step_size",       step_size_);
  node_->get_parameter("search_radius",   search_radius_);
}

void RRTStarPlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] Activated", name_.c_str());
}

void RRTStarPlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] Deactivated", name_.c_str());
}

void RRTStarPlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] Cleaning up", name_.c_str());
}

nav_msgs::msg::Path RRTStarPlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal
)
{
  nav_msgs::msg::Path path;
  path.header = start.header;

  // Stub: straight‐line fallback
  path.poses.push_back(start);
  path.poses.push_back(goal);

  return path;
}

}  // namespace spbot_planners

PLUGINLIB_EXPORT_CLASS(
  spbot_planners::RRTStarPlanner,
  nav2_core::GlobalPlanner
)
