import time

import rclpy
from penelope_aerospace_pl_msgs.action import InfraredThermographyInspect
from penelope_aerospace_pl_msgs.msg import InfraredThermographyState, ResultCodes
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Image


class ExampleActionServer(Node):
    def __init__(self):
        super().__init__("example_action_server")
        self._action_server = ActionServer(self, InfraredThermographyInspect, "example_action", self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing example goal...")

        ## Read goal ##
        pose_array = goal_handle.request.measurement_poses
        filename = goal_handle.request.config_file

        # ...

        ## Provide regular feedback ##
        for i in range(0, 100):
            self.get_logger().info(f"Feedback itteration: {i}")

            # ...

            # Create a feedback message
            feedback_msg = InfraredThermographyInspect.Feedback()

            # Fill in the feedback message
            feedback_msg.measurement_status = list(range(0 + i, 5 + i))

            # Set process state and module state
            #   note: 2 ways to set the "child" message are shown, both can be used
            process_state = InfraredThermographyState()
            process_state.data = 1
            feedback_msg.process_state = process_state

            feedback_msg.module_state.data = 2

            # Send feedback
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)  # Just to slow the feedback down

        # Indicate the action succeeded (this does not indicate succes!)
        goal_handle.succeed()

        ## Provide result ##
        result = InfraredThermographyInspect.Result()
        images = [Image() for _ in range(0, 5)]  # Images are empty for the example

        result.images = images
        result.measurement_status = list(range(0, 5))

        # Set result code and message
        result.result_code = ResultCodes.RC_SUCCES
        result.message = ResultCodes.SUCCES

        return result


def main(args=None):
    # Initialize the ROS2 action server node
    rclpy.init(args=args)
    action_server = ExampleActionServer()

    # Spin so the server does not shutdown untill requested (e.g. Ctrl-C or another shutdown event)
    rclpy.spin(action_server)


if __name__ == "__main__":
    main()
