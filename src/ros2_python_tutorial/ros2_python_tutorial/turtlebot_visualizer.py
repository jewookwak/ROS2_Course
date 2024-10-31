import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker, MarkerArray
from turtlesim.msg import Pose
import math

class State:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

class TurtlebotVisualizer(Node):
    def __init__(self):
        super().__init__('turtlebot_visualizer')
        self.publisher_ = self.create_publisher(MarkerArray, 'turtle1/marker', 10)
        self.subscription_ = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.topic_callback,
            10
        )
        self.timer_ = self.create_timer(0.05, self.timer_callback)
        self.turtlebot_state = State()

    def timer_callback(self):
        marker_array = MarkerArray()

        # Set marker parameters
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'marker_namespace'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = self.turtlebot_state.x
        marker.pose.position.y = self.turtlebot_state.y
        marker.pose.position.z = 0.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(self.turtlebot_state.theta * 0.5)
        marker.pose.orientation.w = math.cos(self.turtlebot_state.theta * 0.5)
        marker.scale.x = 0.3
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        marker.color.a = 1.0  # Don't forget to set the alpha!
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        marker_array.markers.append(marker)
        self.publisher_.publish(marker_array)

    def topic_callback(self, msg):
        self.turtlebot_state.x = msg.x
        self.turtlebot_state.y = msg.y
        self.turtlebot_state.theta = msg.theta

def main(args=None):
    rclpy.init(args=args)
    turtlebot_visualizer = TurtlebotVisualizer()
    rclpy.spin(turtlebot_visualizer)
    turtlebot_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
