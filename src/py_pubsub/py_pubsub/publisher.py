import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    """A node that publishes string messages on a timer."""

    def __init__(self):
        # Initialize the node with a name
        super().__init__('minimal_publisher')
        
        # Create a publisher
        # Args: message type, topic name, queue size
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        
        # Create a timer that fires every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        """Called every time the timer fires."""
        msg = String()
        msg.data = f'Hello World: {self.count}'
        
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        
        self.count += 1


def main(args=None):
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)
    
    # Create and spin the node
    node = MinimalPublisher()
    
    try:
        rclpy.spin(node)  # Keep the node running
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()