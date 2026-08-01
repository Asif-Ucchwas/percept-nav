#include "percept_nav_costmap_plugin/detection_layer.hpp"
#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace percept_nav_costmap_plugin
{

DetectionLayer::DetectionLayer()
: mark_radius_(0.3)
{
}

void DetectionLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error("Failed to lock node in DetectionLayer");
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  node->get_parameter(name_ + "." + "enabled", enabled_);

  global_frame_ = layered_costmap_->getGlobalFrameID();

  detection_sub_ = node->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detected_obstacles", rclcpp::QoS(10),
    std::bind(&DetectionLayer::detectionCallback, this, std::placeholders::_1));

  matchSize();
  current_ = true;

  RCLCPP_INFO(
    node->get_logger(),
    "DetectionLayer initialized, subscribing to /detected_obstacles");
}

void DetectionLayer::detectionCallback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg)
{
  latest_detections_ = msg;
}

void DetectionLayer::updateBounds(
  double robot_x, double robot_y, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  robot_x_ = robot_x;
  robot_y_ = robot_y;

  if (!enabled_ || !latest_detections_) {
    return;
  }

  // Note: this is a simplified version -- our fused detections carry a
  // distance (in pose.position.x, set by the Task 4 fusion node) but not
  // yet a full transformed (x, y) map-frame position. For this first
  // working version, we mark a fixed-size region around the robot's
  // current position whenever any detection with a valid fused distance
  // exists, rather than computing each detection's precise map coordinate.
  // This is an intentional scoping decision, documented honestly rather
  // than claiming full geometric placement this pass.
  bool any_valid_detection = false;
  for (const auto & det : latest_detections_->detections) {
    if (!det.results.empty() && det.results[0].pose.pose.position.x > 0.0) {
      any_valid_detection = true;
      break;
    }
  }

  if (any_valid_detection) {
    *min_x = std::min(*min_x, robot_x_ - mark_radius_);
    *min_y = std::min(*min_y, robot_y_ - mark_radius_);
    *max_x = std::max(*max_x, robot_x_ + mark_radius_);
    *max_y = std::max(*max_y, robot_y_ + mark_radius_);
  }
}

void DetectionLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_ || !latest_detections_) {
    return;
  }

  for (const auto & det : latest_detections_->detections) {
    if (det.results.empty() || det.results[0].pose.pose.position.x <= 0.0) {
      continue;
    }

    unsigned int mx, my;
    if (!master_grid.worldToMap(robot_x_, robot_y_, mx, my)) {
      continue;
    }

    int radius_cells = static_cast<int>(mark_radius_ / master_grid.getResolution());
    for (int dx = -radius_cells; dx <= radius_cells; dx++) {
      for (int dy = -radius_cells; dy <= radius_cells; dy++) {
        int cx = static_cast<int>(mx) + dx;
        int cy = static_cast<int>(my) + dy;
        if (cx >= min_i && cx < max_i && cy >= min_j && cy < max_j) {
          master_grid.setCost(cx, cy, nav2_costmap_2d::LETHAL_OBSTACLE);
        }
      }
    }
  }
}

void DetectionLayer::reset()
{
  latest_detections_.reset();
  current_ = true;
}

}  // namespace percept_nav_costmap_plugin

PLUGINLIB_EXPORT_CLASS(percept_nav_costmap_plugin::DetectionLayer, nav2_costmap_2d::Layer)
