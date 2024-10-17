import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class LedToggleClient(Node):
    def __init__(self):
        super().__init__('led_toggle_client')
        self.client = self.create_client(Empty, 'toggle_led')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.send_request()

    def send_request(self):
        req = Empty.Request()
        future = self.client.call_async(req)
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        self.get_logger().info('Service call completed.')
        raise SystemExit

def main(args=None):
    rclpy.init(args=args)
    led_toggle_client = LedToggleClient()
    rclpy.spin(led_toggle_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
