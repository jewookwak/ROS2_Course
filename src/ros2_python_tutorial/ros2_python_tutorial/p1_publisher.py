import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.fast_publisher_ = self.create_publisher(String, 'fast_topic', 10)
        self.slow_publisher_ = self.create_publisher(String, 'slow_topic', 10)
        fast_timer_period = 0.5  # seconds
        slow_timer_period = 1.0  # seconds
        self.fast_timer_ = self.create_timer(fast_timer_period, self.fast_timer_callback)
        self.slow_timer_ = self.create_timer(slow_timer_period, self.slow_timer_callback)
        self.fast_count_ = 0
        self.slow_count_ = 0

    def fast_timer_callback(self):
        msg = String()
        msg.data = 'Fast timer %d' % self.fast_count_
        self.fast_publisher_.publish(msg)
        self.get_logger().info('[Fast] Publishing: "%s"' % msg.data)
        self.fast_count_ += 1

    def slow_timer_callback(self):
        msg = String()
        msg.data = 'Slow timer %d' % self.slow_count_
        self.slow_publisher_.publish(msg)
        self.get_logger().info('[Slow] Publishing: "%s"' % msg.data)
        self.slow_count_ += 1

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()