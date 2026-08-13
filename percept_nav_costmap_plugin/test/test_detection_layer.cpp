#include <gtest/gtest.h>
#include <memory>
#include <chrono>
#include <limits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "tf2_ros/buffer.h"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "vision_msgs/msg/detection2_d.hpp"
#include "vision_msgs/msg/object_hypothesis_with_pose.hpp"
#include "percept_nav_costmap_plugin/detection_layer.hpp"

using namespace std::chrono_literals;

class DetectionLayerTest : public ::testing::Test
{
protected:
  void SetUp() override
  {
    node_ = std::make_shared<rclcpp_lifecycle::LifecycleNode>("test_detection_layer_node");
    node_->declare_parameter("test_layer.enabled", rclcpp::ParameterValue(true));

    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());

    layered_costmap_ = std::make_shared<nav2_costmap_2d::LayeredCostmap>(
      "map", false, false);
    layered_costmap_->resizeMap(20, 20, 1.0, -10.0, -10.0);

    layer_ = std::make_shared<percept_nav_costmap_plugin::DetectionLayer>();
    layer_->initialize(
      layered_costmap_.get(), "test_layer", tf_buffer_.get(),
      node_, nullptr);

    pub_ = node_->create_publisher<vision_msgs::msg::Detection2DArray>(
      "/detected_obstacles", rclcpp::QoS(10));
    pub_->on_activate();
  }

  void publishAndSpin(double detection_x)
  {
    vision_msgs::msg::Detection2DArray msg;
    vision_msgs::msg::Detection2D det;
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.pose.pose.position.x = detection_x;
    det.results.push_back(hyp);
    msg.detections.push_back(det);
    pub_->publish(msg);

    rclcpp::spin_some(node_->get_node_base_interface());
    std::this_thread::sleep_for(50ms);
    rclcpp::spin_some(node_->get_node_base_interface());
  }

  std::shared_ptr<rclcpp_lifecycle::LifecycleNode> node_;
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<nav2_costmap_2d::LayeredCostmap> layered_costmap_;
  std::shared_ptr<percept_nav_costmap_plugin::DetectionLayer> layer_;
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection2DArray>::SharedPtr pub_;
};

TEST_F(DetectionLayerTest, UpdateBoundsNoOpWithoutDetections)
{
  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);
  EXPECT_DOUBLE_EQ(min_x, 0.0);
  EXPECT_DOUBLE_EQ(min_y, 0.0);
  EXPECT_DOUBLE_EQ(max_x, 0.0);
  EXPECT_DOUBLE_EQ(max_y, 0.0);
}

TEST_F(DetectionLayerTest, UpdateBoundsExpandsOnValidDetection)
{
  publishAndSpin(1.5);  // valid: position.x > 0.0

  // Match LayeredCostmap's real convention: bounds start at +/-infinity
  // so the first layer's expansion always wins the min/max comparison.
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  layer_->updateBounds(2.0, 3.0, 0.0, &min_x, &min_y, &max_x, &max_y);

  // mark_radius_ is 0.3 (default), robot at (2.0, 3.0)
  EXPECT_DOUBLE_EQ(min_x, 2.0 - 0.3);
  EXPECT_DOUBLE_EQ(min_y, 3.0 - 0.3);
  EXPECT_DOUBLE_EQ(max_x, 2.0 + 0.3);
  EXPECT_DOUBLE_EQ(max_y, 3.0 + 0.3);
}

TEST_F(DetectionLayerTest, UpdateBoundsNoOpOnInvalidDetection)
{
  publishAndSpin(-1.0);  // invalid: position.x <= 0.0

  double min_x = 5.0, min_y = 5.0, max_x = 5.0, max_y = 5.0;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);

  // bounds must stay untouched since no valid detection exists
  EXPECT_DOUBLE_EQ(min_x, 5.0);
  EXPECT_DOUBLE_EQ(min_y, 5.0);
  EXPECT_DOUBLE_EQ(max_x, 5.0);
  EXPECT_DOUBLE_EQ(max_y, 5.0);
}

TEST_F(DetectionLayerTest, UpdateCostsMarksLethalOnValidDetection)
{
  publishAndSpin(1.5);

  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);

  nav2_costmap_2d::Costmap2D * master = layered_costmap_->getCostmap();
  layer_->updateCosts(*master, 0, 0, 20, 20);

  unsigned int mx, my;
  ASSERT_TRUE(master->worldToMap(0.0, 0.0, mx, my));
  EXPECT_EQ(master->getCost(mx, my), nav2_costmap_2d::LETHAL_OBSTACLE);
}

TEST_F(DetectionLayerTest, ResetClearsDetections)
{
  publishAndSpin(1.5);
  layer_->reset();

  double min_x = 0.0, min_y = 0.0, max_x = 0.0, max_y = 0.0;
  layer_->updateBounds(0.0, 0.0, 0.0, &min_x, &min_y, &max_x, &max_y);

  // after reset, latest_detections_ is cleared -> updateBounds is a no-op again
  EXPECT_DOUBLE_EQ(min_x, 0.0);
  EXPECT_DOUBLE_EQ(min_y, 0.0);
  EXPECT_DOUBLE_EQ(max_x, 0.0);
  EXPECT_DOUBLE_EQ(max_y, 0.0);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
