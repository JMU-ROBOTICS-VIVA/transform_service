#include "rclcpp/rclcpp.hpp"
//#include "transform_service_srv/srv/transform_pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <tf2_ros/transform_listener.h>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <builtin_interfaces/msg/time.hpp>
#include "rclcpp/time_source.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#include <memory>

// void transform_pose_callback(const std::shared_ptr<transform_service_srv::srv::TransformPose::Request> request,
//           std::shared_ptr<transform_service_srv::srv::TransformPose::Response>      response)
// {
//   response->target_pose.pose.position.x = request->source_pose.pose.position.x + .1;
//   response->target_pose.pose.position.y = request->source_pose.pose.position.y + .2;
//   response->target_pose.pose.position.z = request->source_pose.pose.position.z + .3;

//   response->success = true;

// }

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("transform_server");


  rclcpp::Clock::SharedPtr clock = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
  rclcpp::WallRate loop_rate(30);

  tf2_ros::Buffer buffer(clock);
  buffer.setUsingDedicatedThread(true);
  tf2_ros::TransformListener tfl(buffer, node, false);
  
  rclcpp::Time rclcpp_time = clock->now();
  tf2::TimePoint tf2_time(std::chrono::nanoseconds(rclcpp_time.nanoseconds()));

  // rclcpp::Service<transform_service_srv::srv::TransformPose>::SharedPtr service =
  //   node->create_service<transform_service_srv::srv::TransformPose>("transform_server", &transform_pose_callback);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to transform a pose.");


  auto from_pose = geometry_msgs::msg::PoseStamped();
  auto to_pose = geometry_msgs::msg::PoseStamped();
  
  
  from_pose.pose.position.x = 1;
  from_pose.pose.position.y = 2;
  from_pose.pose.position.z = 3;
  from_pose.header.frame_id = "wheel_left_link";


  
  int count = 0;
  
  while (rclcpp::ok()){

    rclcpp::spin_some(node);
    loop_rate.sleep();
    
    count++;
    geometry_msgs::msg::TransformStamped  transformStamped;
    try{
      buffer.transform(from_pose, to_pose, "base_link");
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%f %f %f", to_pose.pose.position.x,
		  to_pose.pose.position.y,to_pose.pose.position.z );
      
      //transformStamped = buffer.lookupTransform("turtle2", "turtle1",
      //						tf2_time);
    }
    catch (tf2::TransformException & ex) {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "%s%d", ex.what(), count);
      //rclcpp::sleep_for(&1000);
      continue;
    }
   
  }


  

  rclcpp::spin(node);
  rclcpp::shutdown();
}
