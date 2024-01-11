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

        # Accessing the request data and send to cobot
        # This instantiates and populates the classes in the Cobot controller
        self._send_storage_to_cobot(goal_handle.request.storage)           # list of storage location containers
        self._send_products_to_cobot(goal_handle.request.products)         # list of product containers with holes
        self._send_waypoints_to_cobot(goal_handle.request.waypoints)       # list of defined waypoints
        self._send_actions_to_cobot(goal_handle.request.actions)           # list of defined actions
        self._send_drill_tasks_to_cobot(goal_handle.request.drill_tasks)   # list of holes to be drilled
        self._send_fasteners_to_cobot(goal_handle.request.fasteners)       # list of available fasteners
        self._send_tempfs_to_cobot(goal_handle.request.tempfs)             # list of available temporary fasteners
        self._send_docking_pos_to_cobot(goal_handle.request.docking_pos)   # list of available docking positions for End Effectors
        self._send_ee_to_cobot(goal_handle.request.ee)                     # list of available End Effectors

        # TODO check if everything was received well

        # start sending the uid of the actions to the cobot 
        # to execute these actions
        action_number = 0
        for action in goal_handle.request.actions:
            action_number = action_number + 1
            self.get_logger().info(f"Start action: {action_number} " + action.uid)
            self._send_execution_action_uid_to_cobot(action.uid)

            # Provide regular feedback during each execution
            iteration_number = 0
            while iteration_number >= 0:
                iteration_number = iteration_number + 1
                self.get_logger().info(f"Feedback iteration: {action_number}.{iteration_number}")

                action_file = CobotOp()
                # ...

                # Create a feedback message
                feedback_msg = CobotOp.Feedback()

                # Fill in the feedback message
                feedback_msg.actions_out = self._create_actions_from_cobot_output()  
                feedback_msg.drill_tasks_out = self._create_drill_tasks_from_cobot_output()
                feedback_msg.tempfs_out = self._create_tempfs_from_cobot_output()
                feedback_msg.drill_tasks_out = self._create_drill_tasks_from_cobot_output()
                feedback_msg.ee_out = self._create_ee_from_cobot_output()
                feedback_msg.result_code = 1 # TODO determie sensible result_code 
                feedback_msg.message = ""

                # Send feedback
                goal_handle.publish_feedback(feedback_msg)

                time.sleep(0.1)  # Just to slow the feedback down

                if feedback_msg.actions_out(action_number-1).state == AssemblyActionState.SUCCESS:
                    # Will stop the while loop and continue to the next action
                    iteration_number = -10

        # Indicate the action succeeded (this does not indicate succes!)
        goal_handle.succeed()

        ## Provide result ##
        result = CobotOp.Result()

        # Set result code and message
        result.result_code = ResultCodes.RC_SUCCES
        result.message = ResultCodes.SUCCES

        return result
    
    # Function to send list of storage location containers to the cobot controller
    def _send_storage_to_cobot(storage_in):
        return 1
                   
    # Function to send list of product containers with holes to the cobot controller
    def _send_products_to_cobot(products_in):
        return 1

    # Function to send list of defined waypoints to the cobot controller
    def _send_waypoints_to_cobot(waypoints_in):
        return 1 

    # Function to send list of defined actions to the cobot controller
    def _send_actions_to_cobot(actions_in): 
        return 1 

    # Function to send list of holes to be drilled to the cobot controller
    def _send_drill_tasks_to_cobot(drill_tasks_in): 
        return 1

    # Function to send list of available fasteners to the cobot controller
    def _send_fasteners_to_cobot(fasteners_in):  
        return 1 

    # Function to send list of available temporary fasteners to the cobot controller
    def _send_tempfs_to_cobot(tempfs_in): 
        return 1

    # Function to send list of available docking positions for End Effectors to the cobot controller
    def _send_docking_pos_to_cobot(docking_pos_in): 
        return 1
      
    # Function to send list of available End Effectors to the cobot controller
    def _send_ee_to_cobot(ee_in):  
        return 1   

    # Function to send execution commands to the cobot controller
    # Excution commands are given by sending the uid of the action
    # to be executed.
    def _send_execution_action_uid_to_cobot(uid_in):  
        return 1    

    # Function to create list of defined actions to the cobot controller
    def _create_actions_from_cobot_output(): 
        return 1 

    # Function to create list of holes to be drilled to the cobot controller
    def _create_drill_tasks_from_cobot_output(): 
        return 1

    # Function to create list of available fasteners to the cobot controller
    def _create_fasteners_from_cobot_output():  
        return 1 

    # Function to create list of available temporary fasteners to the cobot controller
    def _create_tempfs_from_cobot_output(): 
        return 1
      
    # Function to create list of available End Effectors to the cobot controller
    def _create_ee_from_cobot_output():  
        return 1            


def main(args=None):
    # Initialize the ROS2 action server node
    rclpy.init(args=args)
    action_server = FokkerActionServer()

    # Spin so the server does not shutdown untill requested (e.g. Ctrl-C or another shutdown event)
    rclpy.spin(action_server)


if __name__ == "__main__":
    main()
