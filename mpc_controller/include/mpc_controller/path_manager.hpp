#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <Eigen/Core>



#ifndef PATH_MANAGER_HPP_
#define PATH_MANAGER_HPP_

// namespace path_manager
// {

class PathManager{
public:
  PathManager();

  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent);

  void setGlobalPath(const nav_msgs::msg::Path& global_path);
  
  Eigen::VectorXd computeReferencePath(
    const geometry_msgs::msg::PoseStamped& robot_pose,
    int prediction_horizon,
    int nx,
    double max_lin_vel,
    double dt);

private:
  nav_msgs::msg::Path global_path_;
  Eigen::VectorXd reference_path_;
  
  rclcpp::Clock::SharedPtr clock_;
  
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PointStamped>> projection_point_publisher_;
  std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path>> lerp_ref_path_publisher_;

};

// } // namespace path_manager
#endif  //PATH_MANAGER_HPP_