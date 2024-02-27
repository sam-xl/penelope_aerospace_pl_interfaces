import time
import rclpy

import socket
import select
import cobot_TCP_server
import queue
import get_str_function
import create_action_obj

from penelope_aerospace_pl_msgs.action import CobotOp
from penelope_aerospace_pl_msgs.msg import AssemblyAction
from penelope_aerospace_pl_msgs.msg import AssemblyActionState

from penelope_aerospace_pl_msgs.msg import ModuleState
from penelope_aerospace_pl_msgs.msg import ResultCodes
from rclpy.action import ActionServer
from rclpy.node import Node

class FokkerActionServer(Node):
    def __init__(self):
        super().__init__("fokker_action_server")
        self._action_server = ActionServer(self, CobotOp, "Cobot Operation", self.execute_callback)

        message_queue = queue.Queue()
        cobot_server = cobot_TCP_server.CobotTCPServer(self.handle_cobot_message, message_queue)
        cobot_server.start()    # Or run()?

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing FokkerActionServer...")

        # Accessing the request data and send to cobot
        # This instantiates and populates the classes in the Cobot controller
        self.send_goal_handle_to_cobot(goal_handle)

        # start sending the uid of the actions to the cobot 
        # to execute these actions
        action_number = 0
        for uid in goal_handle.request.execute:
            action_number = action_number + 1
            self.get_logger().info(f"Sending action number {action_number} with uid: " + uid + " to Cobot.")
            self.send_execution_action_uid_to_cobot(uid)

        self.get_logger().info(f"Finished passing action file to cobot.")

        # Indicate the action succeeded (this does not indicate succes!)
        goal_handle.succeed()
    
    # pick up incoming messages from the Cobot and send the result response.
    def handle_cobot_message(self, cobot_msg):
        bytes_msg_length = 4

        cobot_str = cobot_msg[bytes_msg_length:]
        
        feedback_msg = self._create_feedback_message_from_cobot_output(cobot_str)

        # Send as feedback in all cases
        self._action_server._goal_handles[-1].publish_feedback(feedback_msg)

        result_msg = self._create_result_message_from_cobot_output(cobot_str)
        # send result if ready
        if result_msg.result_code == ResultCodes.RC_SUCCES:
        
            self._action_server._send_result_response(result_msg)


    # Function to send execution commands to the cobot controller
    # Excution commands are given by sending the uid of the action
    # to be executed.
    def send_execution_action_uid_to_cobot(self, uid_in):  
        str = get_str_function.EXECUTE_TAG

        # uid of the action to be executed
        str = str + get_str_function.UID_TAG + uid_in + get_str_function.CLOSE_TAG

        str = str + get_str_function.CLOSE_TAG

        return self.send_msg_to_cobot(str)  
    
    # Function to send goal_handle contents to the cobot controller
    # Sends everything to the Cobot except for the uids to execute
    def send_goal_handle_to_cobot(self, goal_handle_in):
        func_output = [0] * 8

        # list of storage location containers
        func_output[0] = self.send_msg_to_cobot(get_str_function.storage_str_to_cobot(goal_handle_in.request.storage))          
        if func_output[0] < 0:
            self.get_logger().info("Failed to send storage to cobot.")

        # list of product containers with holes
        func_output[1] = self.send_msg_to_cobot(get_str_function.products_str_to_cobot(goal_handle_in.request.products))        
        if func_output[1] < 0:
            self.get_logger().info("Failed to send products to cobot.")

        # list of defined waypoints
        func_output[2] = self.send_msg_to_cobot(get_str_function.waypoints_str_to_cobot(goal_handle_in.request.waypoints))     
        if func_output[2] < 0:
            self.get_logger().info("Failed to send waypoints to cobot.")

        # list of holes to be drilled
        func_output[4] = self.send_msg_to_cobot(get_str_function.drill_tasks_str_to_cobot(goal_handle_in.request.drill_tasks)) 
        if func_output[4] < 0:
            self.get_logger().info("Failed to send drill_tasks to cobot.")

        # list of available fasteners
        func_output[5] = self.send_msg_to_cobot(get_str_function.fasteners_str_to_cobot(goal_handle_in.request.fasteners))    
        if func_output[5] < 0:
            self.get_logger().info("Failed to send fasteners to cobot.")

        # list of available temporary fasteners
        func_output[6] = self.send_msg_to_cobot(get_str_function.tempfs_str_to_cobot(goal_handle_in.request.tempfs))          
        if func_output[6] < 0:
            self.get_logger().info("Failed to send tempfs to cobot.")

        # list of available docking positions for End Effectors
        func_output[7] = self.send_msg_to_cobot(get_str_function.docking_pos_str_to_cobot(goal_handle_in.request.docking_pos))
        if func_output[7] < 0:
            self.get_logger().info("Failed to send docking_pos to cobot.")

        # list of available End Effectors
        func_output[8] = self.send_msg_to_cobot(get_str_function.ee_str_to_cobot(goal_handle_in.request.ee))              
        if func_output[8] < 0:
            self.get_logger().info("Failed to send ee to cobot.")

        # list of defined actions
        # actions must be last because they require all other stuff to be there
        func_output[3] = self.send_msg_to_cobot(get_str_function.actions_str_to_cobot(goal_handle_in.request.actions))         
        if func_output[3] < 0:
            self.get_logger().info("Failed to send actions to cobot.")

        return min(func_output)     

    # function to send a message to the cobot
    # the message will be received by the cobot controller
    # the message will be used to instantiate/popuate classes
    # in the cobot conroller
    def send_msg_to_cobot(self, str_in):
        self.message_queue.put(str_in)

        # TODO return 1 only if success
        return 1

    # Function to populate feedback message based on information from the cobot controller
    def _create_feedback_message_from_cobot_output(self, c_str):
        
        # Create a feedback message
        feedback_msg = CobotOp.Feedback()

        a_str = create_action_obj._find_substring(c_str, get_str_function.ACTIONS_TAG)
        if a_str is not None:
            feedback_msg.actions = create_action_obj._create_actions_from_cobot_output(a_str)  

        d_str = create_action_obj._find_substring(c_str, get_str_function.DRILL_TASKS_TAG)
        if d_str is not None:
            feedback_msg.drill_tasks = create_action_obj._create_drill_tasks_from_cobot_output(d_str)

        t_str = create_action_obj._find_substring(c_str, get_str_function.TEMPFS_TAG)
        if t_str is not None:
            feedback_msg.tempfs = create_action_obj._create_tempfs_from_cobot_output(t_str)

        f_str = create_action_obj._find_substring(c_str, get_str_function.FASTENERS_TAG)
        if f_str is not None:
            feedback_msg.drill_tasks = create_action_obj._create_fasteners_from_cobot_output(f_str)

        ee_str = create_action_obj._find_substring(c_str, get_str_function.END_EFFECTORS_TAG)
        if ee_str is not None:
            feedback_msg.ee = create_action_obj._create_ee_from_cobot_output(ee_str)
        
        # calculate the percentage complete
        complete = 0
        total_actions = 0

        for action in feedback_msg.actions:
            total_actions += 1
            if action.state == AssemblyActionState.SUCCESS:
                complete += 1

        if total_actions > 0:
            feedback_msg.percent_complete = complete / total_actions
        else:
            feedback_msg.percent_complete = 0.0

        if total_actions > 0 and complete == 0:
            feedback_msg.module_state = ModuleState.ACCEPTED
        if complete > 0 and complete < total_actions:
            feedback_msg.module_state = ModuleState.EXECUTING
        
        #TODO implement logic for states below
        #feedback_msg.module_state = ModuleState.PAUSED
        #feedback_msg.module_state = ModuleState.CANCELING
        
        #TODO get any relevant messages from the cobot
        feedback_msg.message = "Not implemented yet"   

        return feedback_msg
    
    # Function to populate feedback message based on information from the cobot controller
    def _create_result_message_from_cobot_output(self, c_str):
    
        # Create a feedback message
        result_msg = CobotOp.Result()

        a_str = create_action_obj._find_substring(c_str, get_str_function.ACTIONS_TAG)
        if a_str is not None:
            result_msg.actions_out = create_action_obj._create_actions_from_cobot_output(a_str)  

        d_str = create_action_obj._find_substring(c_str, get_str_function.DRILL_TASKS_TAG)
        if d_str is not None:
            result_msg.drill_tasks_out = create_action_obj._create_drill_tasks_from_cobot_output(d_str)

        t_str = create_action_obj._find_substring(c_str, get_str_function.TEMPFS_TAG)
        if t_str is not None:
            result_msg.tempfs_out = create_action_obj._create_tempfs_from_cobot_output(t_str)

        f_str = create_action_obj._find_substring(c_str, get_str_function.FASTENERS_TAG)
        if f_str is not None:
            result_msg.drill_tasks_out = create_action_obj._create_fasteners_from_cobot_output(f_str)

        ee_str = create_action_obj._find_substring(c_str, get_str_function.END_EFFECTORS_TAG)
        if ee_str is not None:
            result_msg.ee_out = create_action_obj._create_ee_from_cobot_output(ee_str)
        
        # see how many actions are complete
        complete = 0
        total_actions = 0

        for action in result_msg.actions:
            total_actions += 1
            if action.state == AssemblyActionState.SUCCESS:
                complete += 1

        if total_actions == complete:
            result_msg.result_code = ResultCodes.RC_SUCCES 
        else:
            result_msg.result_code = ResultCodes.RC_FAILED

        result_msg.message = "Not implemented yet"  # TODO get any relevant messages from the cobot

        return result_msg


def main(args=None):
    # Initialize the ROS2 action server node
    rclpy.init(args=args)
    action_server = FokkerActionServer()

    # Spin so the server does not shutdown untill requested (e.g. Ctrl-C or another shutdown event)
    rclpy.spin(action_server)


if __name__ == "__main__":
    main()
