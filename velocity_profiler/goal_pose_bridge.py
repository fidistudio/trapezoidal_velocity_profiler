import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from velocity_profiler_interfaces.action import Move2D
from tf_transformations import euler_from_quaternion


class GoalPoseBridge(Node):
    """Bridge node that converts PoseStamped messages into Move2D action goals."""

    def __init__(self):
        super().__init__("goal_pose_bridge")

        # Subscriber for PoseStamped goals
        self.create_subscription(PoseStamped, "/goal_pose", self.goal_callback, 10)

        # Action client to Move2D
        self.action_client = ActionClient(self, Move2D, "move_2d")

        # Current goal handle
        self.current_goal_handle = None

    # ------------------- CALLBACKS ------------------- #
    def goal_callback(self, msg: PoseStamped):
        """Handle incoming PoseStamped goal."""
        if not self.action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Move2D action server not available")
            return

        # Cancel previous goal if exists
        if self.current_goal_handle is not None:
            self.get_logger().info("Canceling previous goal")
            self.action_client.cancel_goal_async(self.current_goal_handle)

        # Prepare new goal
        goal = Move2D.Goal()
        goal.x = msg.pose.position.x
        goal.y = msg.pose.position.y

        q = msg.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        goal.theta = yaw

        self.get_logger().info(
            f"New goal received: x={goal.x:.2f}, y={goal.y:.2f}, theta={goal.theta:.2f}"
        )

        # Send goal
        send_future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback
        )
        send_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Handle the response from the action server."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal was rejected by the action server")
            return

        self.current_goal_handle = goal_handle
        self.get_logger().info("Goal accepted by the action server")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        fb = feedback_msg.feedback
        self.get_logger().info(
            f"[{fb.phase}] remaining_distance={fb.remaining_distance:.2f}, "
            f"remaining_angle={fb.remaining_angle:.2f}"
        )

    def result_callback(self, future):
        """Handle the final result from the action server."""
        result = future.result().result
        self.get_logger().info(f"Action finished: {result.message}")
        self.current_goal_handle = None


def main():
    rclpy.init()
    node = GoalPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
