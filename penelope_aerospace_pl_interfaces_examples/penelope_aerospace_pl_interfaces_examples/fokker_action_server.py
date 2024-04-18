import rclpy

import cobot_TCP_server
import queue
import get_str_function
import create_action_obj

from penelope_aerospace_pl_msgs.action import CobotOp
from penelope_aerospace_pl_msgs.msg import AssemblyActionState

from penelope_aerospace_pl_msgs.msg import ModuleState
from penelope_aerospace_pl_msgs.msg import ResultCodes
from rclpy.action import ActionServer
from rclpy.node import Node
from time import sleep


class FokkerActionServer(Node):
    def __init__(self):
        super().__init__("fokker_action_server")
        self._action_server = ActionServer(self, CobotOp, "Cobot Operation", self.execute_callback)

        self.result_msg

        message_queue = queue.Queue()
        cobot_server = cobot_TCP_server.CobotTCPServer(self.handle_cobot_message, message_queue)
        cobot_server.start()    # Or run()?

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing FokkerActionServer...")
        self.is_ready = False

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
            #TODO blocking?
            #TODO check whether it went ok
            
        self.get_logger().info(f"Finished passing action file to cobot.")

        while not self.is_ready:
            sleep(0.1)

        # Indicate the action succeeded (this does not indicate succes!)
        goal_handle.succeed()

        return self.result_msg
    
    # pick up incoming messages from the Cobot and send the result response.
    def handle_cobot_message(self, cobot_msg):
        bytes_msg_length = 4

        cobot_str = cobot_msg[bytes_msg_length:]
        
        feedback_msg = self._create_feedback_message_from_cobot_output(cobot_str)

        # Send as feedback in all cases
        self._action_server._goal_handles[-1].publish_feedback(feedback_msg)

        self.result_msg = feedback_msg
        
        # trigger sending result if ready
        if self.result_msg.result_code == ResultCodes.RC_SUCCES:
            self.is_ready = True

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
        # storage location container
        res = self.send_msg_to_cobot(get_str_function.permf_storage_str_to_cobot(goal_handle_in.request.permf_storage))          
        if res < 0:
            self.get_logger().error("Failed to send permf storage to cobot.")
            return -1
        
        # storage location container
        res = self.send_msg_to_cobot(get_str_function.tempf_storage_str_to_cobot(goal_handle_in.request.tempf_storage))          
        if res < 0:
            self.get_logger().error("Failed to send tempf storage to cobot.")
            return -1

        # product container with holes
        res = self.send_msg_to_cobot(get_str_function.product_str_to_cobot(goal_handle_in.request.product))        
        if res < 0:
            self.get_logger().error("Failed to send product to cobot.")
            return -1

        # list of defined waypoints
        res = self.send_msg_to_cobot(get_str_function.waypoints_str_to_cobot(goal_handle_in.request.waypoints))     
        if res < 0:
            self.get_logger().error("Failed to send waypoints to cobot.")
            return -1

        # list of holes to be drilled
        res = self.send_msg_to_cobot(get_str_function.drill_tasks_str_to_cobot(goal_handle_in.request.drill_tasks)) 
        if res < 0:
            self.get_logger().error("Failed to send drill_tasks to cobot.")
            return -1

        # list of available fasteners
        res = self.send_msg_to_cobot(get_str_function.fasteners_str_to_cobot(goal_handle_in.request.fasteners))    
        if res < 0:
            self.get_logger().error("Failed to send fasteners to cobot.")
            return -1

        # list of available temporary fasteners
        res = self.send_msg_to_cobot(get_str_function.tempfs_str_to_cobot(goal_handle_in.request.tempfs))          
        if res < 0:
            self.get_logger().error("Failed to send tempfs to cobot.")
            return -1

        # list of available docking positions for End Effectors
        res = self.send_msg_to_cobot(get_str_function.docking_pos_str_to_cobot(goal_handle_in.request.docking_pos))
        if res < 0:
            self.get_logger().error("Failed to send docking_pos to cobot.")
            return -1

        # list of available End Effectors
        res = self.send_msg_to_cobot(get_str_function.ee_str_to_cobot(goal_handle_in.request.ee))              
        if res < 0:
            self.get_logger().error("Failed to send ee to cobot.")
            return -1

        # list of defined actions
        # actions must be last because they require all other stuff to be there
        res = self.send_msg_to_cobot(get_str_function.actions_str_to_cobot(goal_handle_in.request.actions))         
        if res < 0:
            self.get_logger().error("Failed to send actions to cobot.")
            return -1

        return 1     

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
