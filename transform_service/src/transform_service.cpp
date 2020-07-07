#include <tf2_ros/transform_listener.h>
#include <memory>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "rclcpp/rclcpp.hpp"
#include "transform_service_srv/srv/transform_pose.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

class TransformServer : public rclcpp::Node {
 public:
  TransformServer()
      : Node("transform_service"),
        buffer_(std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)),
        tfl_(this->buffer_, this, false) {
    this->create_service<transform_service_srv::srv::TransformPose>(
        "transform_service",
        std::bind(&TransformServer::transform_pose_callback, this, _1, _2));
    buffer_.setUsingDedicatedThread(true);
    RCLCPP_INFO(get_logger(), "Ready to transform poses.");
  }

 private:
  void transform_pose_callback(
      const std::shared_ptr<transform_service_srv::srv::TransformPose::Request>
          request,
      std::shared_ptr<transform_service_srv::srv::TransformPose::Response>
          response) {
    try {
      buffer_.transform(request->source_pose, response->target_pose,
                        request->target_frame);
      response->success = true;
    } catch (tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      response->success = false;
    }
  }

  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener tfl_;
  rclcpp::Service<transform_service_srv::srv::TransformPose>::SharedPtr
      service_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<TransformServer>();
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
