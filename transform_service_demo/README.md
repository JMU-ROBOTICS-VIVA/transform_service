# Transform Service Demo

To run:

`ros2 launch transform_service_demo transform_service_demo_launch.py`

The launch file will start the turtlebot3 simulator along with three
additional nodes:

* transform_service - This is the C++ service provider that uses tf to
  perform pose transforms on request.
* transform_service_demo - This a Python node that illustrates how
  `transform_service` can be used to perfom transforms.  It just
  repeatedly transforms a `PoseStemped from the `base_link` to the
  `odom` frame.  It publishes the resulting pose so that it can be
  visualized in RViz2.
* RViz2 - The green arrow illustrates that transformed pose.  It should
  always be one meter in front of the robot.

You should be able to drive the robot around by executing:

`ros2 run turtlebot3_teleop teleop_keyboard`

in a separate terminal.

The result should look like this:

![Demo](demo/demo.gif)