import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class SlowAgent(Node):

    def __init__(self):
        super().__init__('slow_agent')
        self.publisher_ = self.create_publisher(String, 'slow_topic', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.subscription = self.create_subscription(
            String,
            'fast_topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Slow timer %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('[Slow] Publishing: "%s"' % msg.data)
        self.i += 1

    def listener_callback(self, msg):
        self.get_logger().info('[Slow] I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    slow_agent = SlowAgent()

    rclpy.spin(slow_agent)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    slow_agent.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()