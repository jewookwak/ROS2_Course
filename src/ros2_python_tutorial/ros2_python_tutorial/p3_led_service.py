import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class LedToggleService(Node):
    def __init__(self):
        super().__init__('led_toggle_service')
        self.srv = self.create_service(Empty, 'toggle_led', self.toggle_led_callback)
        self.led_on = False
        self.get_logger().info('LED Toggle Service Ready')

    def toggle_led_callback(self, request, response):
        self.led_on = not self.led_on
        status = 'ON' if self.led_on else 'OFF'
        self.get_logger().info(f'LED is now {status}')
        return response

def main(args=None):
    rclpy.init(args=args)
    led_toggle_service = LedToggleService()
    rclpy.spin(led_toggle_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()