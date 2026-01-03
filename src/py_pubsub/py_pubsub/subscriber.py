import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    """A node that subscribes to string messages."""

    def __init__(self):
        super().__init__('minimal_subscriber')
        
        # Create a subscription
        # Args: message type, topic name, callback function, queue size
        self.subscription = self.create_subscription(
            String,
            'chatter',  # Must match publisher's topic!
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        """Called whenever a message is received."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    
    node = MinimalSubscriber()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()