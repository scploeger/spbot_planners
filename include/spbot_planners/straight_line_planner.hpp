#ifndef SPBOT_PLANNERS__STRAIGHT_LINE_PLANNER_HPP_
#define SPBOT_PLANNERS__STRAIGHT_LINE_PLANNER_HPP_

#include <memory>
#include <string>

#include "nav2_core/global_planner.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"

namespace spbot_planners
{

class StraightLinePlanner : public nav2_core::GlobalPlanner
{
public:
  StraightLinePlanner() = default;
  ~StraightLinePlanner() override = default;

  // Must match exactly the base‐class signature:
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros
  ) override;

  void activate() override;
  void deactivate() override;
  void cleanup() override;

  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal
  ) override;

private:
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

  // Planner params
  int    max_iterations_{1000};
  double step_size_{0.5};
  double search_radius_{1.0};
};

}  // namespace spbot_planners

#endif  // SPBOT_PLANNERS__STRAIGHT_LINE_PLANNER_HPP_
