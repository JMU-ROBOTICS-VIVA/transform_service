#include "rclcpp/rclcpp.hpp"
#include "transform_service_srv/srv/transform_pose.hpp"
//#include "geometry_msgs/msg/pose_stamped.hpp"

#include <memory>

void transform_pose_callback(const std::shared_ptr<transform_service_srv::srv::TransformPose::Request> request,
          std::shared_ptr<transform_service_srv::srv::TransformPose::Response>      response)
{
  response->target_pose.pose.position.x = request->source_pose.pose.position.x + .1;
  response->target_pose.pose.position.y = request->source_pose.pose.position.y + .2;
  response->target_pose.pose.position.z = request->source_pose.pose.position.z + .3;

  response->success = true;

}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("transform_server");

  rclcpp::Service<transform_service_srv::srv::TransformPose>::SharedPtr service =
    node->create_service<transform_service_srv::srv::TransformPose>("transform_server", &transform_pose_callback);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to transform a pose.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
