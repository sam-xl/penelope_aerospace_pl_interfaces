import rclpy
from geometry_msgs.msg import PoseArray
from penelope_aerospace_pl_msgs.action import InfraredThermographyInspect
from rclpy.action import ActionClient
from rclpy.node import Node


class ExampleActionClient(Node):
    def __init__(self):
        super().__init__("example_action_client")
        self._action_client = ActionClient(self, InfraredThermographyInspect, "example_action")

    def send_goal(self):
        # Wait till the action server is online
        if not self._action_client.wait_for_server(2):
            self.get_logger().error("Timeout: Server is not running")
            rclpy.shutdown()  # No reason to keep the example node up
            return

        ## Create goal message #
        goal_msg = InfraredThermographyInspect.Goal()

        measurement_poses = PoseArray()
        # .. fill in pose_array
        goal_msg.measurement_poses = measurement_poses

        goal_msg.config_file = "/path/or/uri/to/config/file"

        ## Send the goal ##
        # Goal is send async and a done callback is added
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        # Check if goal is accepted
        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected by ExampleActionServer")
            rclpy.shutdown()  # No reason to keep the example node up
            return

        # Wait for result async
        self.get_logger().info("Goal accepted by ExampleActionServer")
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f"Result: {result}")
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Received feedback: {feedback}")


def main(args=None):
    # Initialize the ROS2 action client node
    rclpy.init(args=args)
    action_client = ExampleActionClient()

    # Send the action goal
    action_client.send_goal()

    # Spin to wait for feedback and results
    rclpy.spin(action_client)


if __name__ == "__main__":
    main()
