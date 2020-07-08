"""
This is a simple node designed to demonstrate the use of the transform_service
for transforming stamped poses from one frame to another. It repeatedly
transforms a pose object one meter ahead of the robot (in the 'base_link' frame)
into the 'odom' frame, then publishes the resulting pose for the purposes of
visualization.

Author: Nathan Sprague

"""
import rclpy
import rclpy.node
from geometry_msgs.msg import PoseStamped
from transform_service_srv.srv import TransformPose


class TransformTester(rclpy.node.Node):

    def __init__(self):
        super().__init__('transform_tester')

        # Create the client link and loop until the server is available.
        self.cli = self.create_client(TransformPose, 'transform_service')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

        # Set up the request object (most fields will be re-used each time.)
        self.req = TransformPose.Request()
        self.req.source_pose.pose.position.x = 1.0
        self.req.source_pose.pose.orientation.w = 1.0
        self.req.source_pose.header.frame_id = "base_link"
        self.req.target_frame = "odom"

        # Create the publisher
        self.publisher_ = self.create_publisher(PoseStamped, 'offset_pose', 10)
        timer_period = 0.01  # seconds

        # Set up a timed callback that will repeatedly make service requests.
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # This will store values returned from call_async
        self.future = None

    def timer_callback(self):

        # There is no outstanding service request, so we will make one:
        if self.future is None:
            self.get_logger().info('Making service request...')
            self.req.source_pose.header.stamp = self.get_clock().now().to_msg()
            self.future = self.cli.call_async(self.req)

        elif self.future.done():  # Current server request has completed.
            try:
                response = self.future.result()
            except Exception as e:
                self.get_logger().info('Service call failed %r' % (e,))
            else:
                if response.success:
                    self.publisher_.publish(response.target_pose)
                    self.get_logger().info('Publishing: "%s"' %
                                           str(response.target_pose))
                else:
                    self.get_logger().info(
                        'Service call succeeded, but transform was not available.')

            self.future = None
        else:
            self.get_logger().info('Waiting for service call to finish...')


def main(args=None):
    rclpy.init(args=args)

    transform_tester = TransformTester()

    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(pose_publisher)
    # executor.spin()

    rclpy.spin(transform_tester)

    transform_tester.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
