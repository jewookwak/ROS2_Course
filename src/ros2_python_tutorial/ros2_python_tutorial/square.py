import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, target, current):
        error = target - current
        error = math.fabs(error)
        self.integral += error
        derivative = error - self.previous_error
        self.previous_error = error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        return output

class TurtlebotController(Node):
    def __init__(self):
        super().__init__('turtlebot_controller')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription_ = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10)
        self.timer_ = self.create_timer(0.1, self.timer_callback)
        self.init_x = 5.544445
        self.init_y = 5.544445
        self.target_positions = [(self.init_x + 2, self.init_y), (self.init_x + 2, self.init_y + 2), (self.init_x, self.init_y + 2), (self.init_x, self.init_y)]
        self.current_target_index = 0

        # Initialize PID controllers for distance and angle
        self.distance_pid = PID(1, 0.0, 0.0)  # Set the gains as needed
        self.angle_pid = PID(1, 0.0, 0.0)     # Set the gains as needed

        self.turtlebot_state = Pose()
        self.is_rotating = False

    def pose_callback(self, msg):
        self.turtlebot_state = msg
        # self.get_logger().info(f"Position0: ({self.turtlebot_state.x:.2f}, {self.turtlebot_state.y:.2f}), Angle0: {self.turtlebot_state.theta:.2f}")

    def timer_callback(self):
        target_x, target_y = self.target_positions[self.current_target_index]
        dx = target_x - self.turtlebot_state.x
        dy = target_y - self.turtlebot_state.y
        
        # angle_to_target = math.atan2(dy, dx)
        if self.current_target_index == 0:
            angle_to_target = 0
            distance_to_target = dx
        elif self.current_target_index == 1:
            angle_to_target = math.pi/2
            distance_to_target = dy
        elif self.current_target_index == 2:
            angle_to_target = math.pi
            distance_to_target = dx
        else:
            angle_to_target = -math.pi/2
            distance_to_target = dy
        self.get_logger().info(f"Position: ({self.turtlebot_state.x:.2f}, {self.turtlebot_state.y:.2f}), Angle: {angle_to_target:.2f}")
        
        cmd = Twist()

        # 90-degree rotation complete, move to next position
        error_margin = 0.01
        self.get_logger().info(f"current_target_idex: {self.current_target_index}")
        if math.fabs(distance_to_target) < error_margin:
            self.get_logger().info("Arrived to target point")
            self.is_rotating = True
            self.current_target_index = (self.current_target_index + 1) % len(self.target_positions)
        elif self.is_rotating:
            angle_error = angle_to_target - self.turtlebot_state.theta
            self.get_logger().info(f"self.turtlebot_state.theta: {self.turtlebot_state.theta:.4f}")
            cmd.linear.x = 0.0
            cmd.angular.z = self.angle_pid.compute(angle_to_target, self.turtlebot_state.theta)

            # Allow some tolerance for angle error
            if abs(angle_error) < error_margin:
                self.get_logger().info("Turning finished")
                self.is_rotating = False
        else:
            # PID control to reach target
            self.get_logger().info("Reaching target")
            cmd.linear.x = self.distance_pid.compute(math.fabs(distance_to_target), 0)
            angle_error = angle_to_target - self.turtlebot_state.theta
            # cmd.angular.z = self.angle_pid.compute(angle_error, 0)
            cmd.angular.z = 0.0

        self.get_logger().info(f"Position: ({self.turtlebot_state.x:.2f}, {self.turtlebot_state.y:.2f}), Distance to Target: {distance_to_target:.2f}")
        self.publisher_.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    turtlebot_controller = TurtlebotController()
    rclpy.spin(turtlebot_controller)
    turtlebot_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
