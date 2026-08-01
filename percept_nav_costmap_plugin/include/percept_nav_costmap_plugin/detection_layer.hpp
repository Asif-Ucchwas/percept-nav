#ifndef PERCEPT_NAV_COSTMAP_PLUGIN__DETECTION_LAYER_HPP_
#define PERCEPT_NAV_COSTMAP_PLUGIN__DETECTION_LAYER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/costmap_layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"

namespace percept_nav_costmap_plugin
{

class DetectionLayer : public nav2_costmap_2d::CostmapLayer
{
public:
  DetectionLayer();

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;
  void reset() override;
  bool isClearable() override { return true; }

private:
  void detectionCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);

  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  vision_msgs::msg::Detection2DArray::SharedPtr latest_detections_;
  double mark_radius_;
  std::string global_frame_;
  double robot_x_;
  double robot_y_;
};

}  // namespace percept_nav_costmap_plugin

#endif  // PERCEPT_NAV_COSTMAP_PLUGIN__DETECTION_LAYER_HPP_
