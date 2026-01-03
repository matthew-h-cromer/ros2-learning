# ROS2 Learning Project

ROS2 Kilted workspace for learning. Target: local + Raspberry Pi.

## Commands
- Build: `colcon build --symlink-install`
- Source: `source install/setup.bash`
- New package: `ros2 pkg create --build-type ament_python <name>`
- Run node: `ros2 run <package> <executable>`

## Packages
- `py_pubsub` - Basic publisher/subscriber example

## Style
- Python: snake_case, type hints encouraged
- Use rclpy logging: `self.get_logger().info(...)`
