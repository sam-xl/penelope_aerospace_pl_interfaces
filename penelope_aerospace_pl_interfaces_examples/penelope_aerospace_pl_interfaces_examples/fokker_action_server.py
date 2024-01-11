import time
import rclpy

from penelope_aerospace_pl_msgs.action import CobotOp
from penelope_aerospace_pl_msgs.msg import AssemblyAction
from penelope_aerospace_pl_msgs.msg import AssemblyActionState
from penelope_aerospace_pl_msgs.msg import AssemblyDrill
from penelope_aerospace_pl_msgs.msg import AssemblyEe
from penelope_aerospace_pl_msgs.msg import AssemblyEeDockingPos
from penelope_aerospace_pl_msgs.msg import AssemblyEeState
from penelope_aerospace_pl_msgs.msg import AssemblyFast
from penelope_aerospace_pl_msgs.msg import AssemblyFastState
from penelope_aerospace_pl_msgs.msg import AssemblyHoleLocation
from penelope_aerospace_pl_msgs.msg import AssemblyHoleLocationContainer
from penelope_aerospace_pl_msgs.msg import AssemblyMaterialLayer
from penelope_aerospace_pl_msgs.msg import AssemblyTempFast
from penelope_aerospace_pl_msgs.msg import AssemblyWaypoint
from penelope_aerospace_pl_msgs.msg import ModuleState
from penelope_aerospace_pl_msgs.msg import ResultCodes
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Image


class FokkerActionServer(Node):
    def __init__(self):
        super().__init__("fokker_action_server")
        self._action_server = ActionServer(self, CobotOp, "Cobot Operation", self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing FokkerActionServer...")

        # Accessing the request data
        holes_specification = goal_handle.request.holes_specification
        filename = goal_handle.request.config_file

        # ...

        ## Provide regular feedback ##
        for i in range(0, 100):
            self.get_logger().info(f"Feedback itteration: {i}")

            action_file = CobotOp()
            # ...

            # Create a feedback message
            feedback_msg = CobotOp.Feedback()

            # Fill in the feedback message


            # Send feedback
            goal_handle.publish_feedback(feedback_msg)

            time.sleep(0.1)  # Just to slow the feedback down

        # Indicate the action succeeded (this does not indicate succes!)
        goal_handle.succeed()

        ## Provide result ##
        result = CobotOp.Result()

        # Set result code and message
        result.result_code = ResultCodes.RC_SUCCES
        result.message = ResultCodes.SUCCES

        return result


def main(args=None):
    # Initialize the ROS2 action server node
    rclpy.init(args=args)
    action_server = FokkerActionServer()

    # Spin so the server does not shutdown untill requested (e.g. Ctrl-C or another shutdown event)
    rclpy.spin(action_server)


if __name__ == "__main__":
    main()
