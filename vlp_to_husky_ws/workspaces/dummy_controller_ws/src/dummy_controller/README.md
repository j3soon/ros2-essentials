# Examples of minimal publisher/subscriber using rclcpp/rclpy

Build the package:

```sh
# under template_ws
colcon build
```

Run one of the publisher in the first terminal:

```sh
# C++ publisher
ros2 run minimal_pkg talker
# Python publisher
ros2 run minimal_pkg publisher.py
```

and then run one of the subscriber in the second terminal:

```sh
# C++ listener
ros2 run minimal_pkg listener
# Python subscriber
ros2 run minimal_pkg subscriber.py
```
