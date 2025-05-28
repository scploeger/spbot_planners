#include "spbot_planners/straight_line_planner.hpp"
#include <pluginlib/class_list_macros.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 

namespace spbot_planners
{

void StraightLinePlanner::configure(
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

void StraightLinePlanner::activate()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] Activated", name_.c_str());
}

void StraightLinePlanner::deactivate()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] Deactivated", name_.c_str());
}

void StraightLinePlanner::cleanup()
{
  RCLCPP_INFO(node_->get_logger(), "[%s] Cleaning up", name_.c_str());
}

nav_msgs::msg::Path StraightLinePlanner::createPlan(
  const geometry_msgs::msg::PoseStamped & start, // two poses, start and goal
  const geometry_msgs::msg::PoseStamped & goal
)
{
  nav_msgs::msg::Path path;
  path.header = start.header;

    // parameters
    double step = 0.05;                  // 5 cm between poses
    auto dx     = goal.pose.position.x - start.pose.position.x;
    auto dy     = goal.pose.position.y - start.pose.position.y;
    double length = std::hypot(dx, dy);
    int    n_steps = std::max<int>(1, std::round(length / step));

    double yaw = std::atan2(dy, dx);     // heading along the segment

    // orientation quaternion for every intermediate pose
    tf2::Quaternion q_tf;
    q_tf.setRPY(0.0, 0.0, yaw);               // roll, pitch, yaw
    geometry_msgs::msg::Quaternion q = tf2::toMsg(q_tf);

    for (int i = 0; i <= n_steps; ++i) {
        double t = static_cast<double>(i) / n_steps;

        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position.x = start.pose.position.x + t * dx;
        pose.pose.position.y = start.pose.position.y + t * dy;
        pose.pose.position.z = 0.0;
        pose.pose.orientation = q;
        path.poses.push_back(pose);
    }
    // last pose keeps the goal's final orientation if you wish:
    path.poses.back().pose.orientation = goal.pose.orientation;
    return path;
}

}  // namespace spbot_planners

PLUGINLIB_EXPORT_CLASS(
  spbot_planners::StraightLinePlanner,
  nav2_core::GlobalPlanner
)
