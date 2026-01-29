import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from velocity_profiler_interfaces.action import Move2D
import math
import time
from tf_transformations import euler_from_quaternion


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))


class VelocityProfilerAction(Node):
    def __init__(self):
        super().__init__("velocity_profiler_action")

        # ROS parameters
        self.declare_parameter("max_linear_velocity", 0.4)
        self.declare_parameter("max_angular_velocity", 2.0)

        self.max_v = self.get_parameter("max_linear_velocity").value
        self.max_w = self.get_parameter("max_angular_velocity").value

        # Publishers and subscriptions
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Odometry, "/odom", self.odom_callback, 10)

        # Action server
        self.action_server = ActionServer(
            self,
            Move2D,
            "move_2d",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

        # Robot state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

    # ------------------- CALLBACKS ------------------- #
    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def goal_callback(self, goal_request) -> GoalResponse:
        self.get_logger().info("Goal accepted")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        self.get_logger().warn("Goal canceled")
        return CancelResponse.ACCEPT

    # ------------------- HELPER FUNCTIONS ------------------- #
    @staticmethod
    def trapezoidal_velocity(t: float, tf: float, vmax: float) -> float:
        """Compute trapezoidal velocity profile."""
        t1, t2 = tf / 3, 2 * tf / 3
        if t < t1:
            return vmax * t / t1
        elif t < t2:
            return vmax
        elif t < tf:
            return vmax * (tf - t) / (tf - t2)
        return 0.0

    def perform_rotation(self, goal_handle, dtheta: float) -> bool:
        """Rotate the robot to align with target orientation."""
        if abs(dtheta) < 1e-3:
            return True  # No rotation needed

        tf = 1.5 * abs(dtheta) / self.max_w
        t0 = time.time()
        feedback = Move2D.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.cmd_pub.publish(Twist())
                return False

            t = time.time() - t0
            w = math.copysign(self.trapezoidal_velocity(t, tf, self.max_w), dtheta)

            cmd = Twist()
            cmd.angular.z = w
            self.cmd_pub.publish(cmd)

            feedback.phase = "ROTATE"
            feedback.remaining_angle = abs(dtheta) * max(0.0, 1 - t / tf)
            feedback.remaining_distance = 0.0
            goal_handle.publish_feedback(feedback)

            if t >= tf:
                break

            time.sleep(0.02)

        return True

    def perform_translation(self, goal_handle, distance: float) -> bool:
        """Move the robot forward to reach target position."""
        if distance < 1e-3:
            return True  # No translation needed

        tf = 1.5 * distance / self.max_v
        t0 = time.time()
        feedback = Move2D.Feedback()

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.cmd_pub.publish(Twist())
                return False

            t = time.time() - t0
            v = self.trapezoidal_velocity(t, tf, self.max_v)

            cmd = Twist()
            cmd.linear.x = v
            self.cmd_pub.publish(cmd)

            feedback.phase = "ADVANCE"
            feedback.remaining_distance = distance * max(0.0, 1 - t / tf)
            feedback.remaining_angle = 0.0
            goal_handle.publish_feedback(feedback)

            if t >= tf:
                break

            time.sleep(0.02)

        return True

    # ------------------- EXECUTE CALLBACK ------------------- #
    def execute_callback(self, goal_handle):
        """Main action execution: rotate then translate."""
        dx = goal_handle.request.x - self.x
        dy = goal_handle.request.y - self.y
        target_yaw = math.atan2(dy, dx)
        dtheta = wrap_angle(target_yaw - self.yaw)

        if not self.perform_rotation(goal_handle, dtheta):
            return Move2D.Result(success=False, message="Rotation canceled")

        dx = goal_handle.request.x - self.x
        dy = goal_handle.request.y - self.y
        distance = math.hypot(dx, dy)

        if not self.perform_translation(goal_handle, distance):
            return Move2D.Result(success=False, message="Translation canceled")

        # Stop robot
        self.cmd_pub.publish(Twist())
        goal_handle.succeed()
        return Move2D.Result(success=True, message="Goal reached")


def main():
    rclpy.init()
    node = VelocityProfilerAction()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
