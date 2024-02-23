import time
import rclpy
import re

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
from geometry_msgs.msg import Pose

OPEN_TAG = "<"
CLOSE_TAG = ">"
PROVIDE_STATUS_MSG = OPEN_TAG + "PROVIDE_STATUS" + CLOSE_TAG
UID_TAG = "uid" + OPEN_TAG
STORAGE_TAG = "storage" + OPEN_TAG
PRODUCTS_TAG = "products" + OPEN_TAG
PRODUCT_TAG = "product" + OPEN_TAG
LOCATIONS_TAG = "locations" + OPEN_TAG
LOC_UID_TAG = "loc_uid" + OPEN_TAG
MAX_OBST_HEIGHT_TAG = "max_obstacle_height" + OPEN_TAG
APPR_POS_UID_TAG = "approach_pos_uid" + OPEN_TAG
HOLE_LOCATION_CONT_TAG = "hole_location_container" + OPEN_TAG
HOLE_LOCATION_TAG = "hole_location" + OPEN_TAG
POSE_TAG = "pose" + OPEN_TAG
POSE_PX_TAG = "pose_p_x" + OPEN_TAG
POSE_PY_TAG = "pose_p_y" + OPEN_TAG
POSE_PZ_TAG = "pose_p_z" + OPEN_TAG
POSE_OX_TAG = "pose_o_x" + OPEN_TAG
POSE_OY_TAG = "pose_o_y" + OPEN_TAG
POSE_OZ_TAG = "pose_o_z" + OPEN_TAG
POSE_OW_TAG = "pose_o_w" + OPEN_TAG
DIAM_TAG = "diam" + OPEN_TAG
STACK_T_TAG = "stack_thickness_tag" + OPEN_TAG
DRILL_JIG_DIST_TAG = "drill_jig_dist" + OPEN_TAG
DRILLED_TAG = "drilled" + OPEN_TAG
LAYERS_TAG = "layers" + OPEN_TAG
MAT_LAYER_TAG = "material_layer" + OPEN_TAG
LAYER_THICKNESS_TAG = "layer_thickness" + OPEN_TAG  
SHAFT_HEIGHT_TAG = "shaft_height" + OPEN_TAG
MIN_STACK_TAG = "min_stack_thickness" + OPEN_TAG
MAX_STACK_TAG = "max_stack_thickness" + OPEN_TAG
TCP_TIP_DIST_TAG = "tcp_tip_dist" + OPEN_TAG
TCP_TOP_DIST_TAG = "tcp_top_dist" + OPEN_TAG                        
DRILL_SPEED_TAG = "drill_speed" + OPEN_TAG                           
DRILL_FEED_TAG = "drill_feed" + OPEN_TAG                            
LOWER_TORQUE_LIMIT_TAG = "lower_torque_limit" + OPEN_TAG             
UPPER_TORQUE_LIMIT_TAG = "upper_torque_limit" + OPEN_TAG            
TORQUE_THRESHOLD_TAG = "torque_threshold" + OPEN_TAG                      
MAX_TEMPF_CLAMP_FORCE_TAG = "max_tempf_clamp_force" + OPEN_TAG
WAYPOINT_TAG = "waypoint" + OPEN_TAG
WAYPOINTS_TAG = "waypoints" + OPEN_TAG
ORIENTATION_TAG = "orientation" + OPEN_TAG
BLEND_RADIUS_TAG = "blend_radius" + OPEN_TAG
ACTIONS_TAG = "actions" + OPEN_TAG
ACTION_TAG = "action" + OPEN_TAG
OBJ_UID_TAG = "obj_uid" + OPEN_TAG
ACTION_STATE_TAG = "action_state" + OPEN_TAG
PASSING_UIDS_TAG = "passing_uids" + OPEN_TAG
PASSING_UID_TAG = "passing_uid" + OPEN_TAG
SPEED_TAG = "speed" + OPEN_TAG
DRILL_TASKS_TAG = "drill_tasks" + OPEN_TAG
DRILL_TASK_TAG = "drill_task" + OPEN_TAG
FASTENERS_TAG = "fasteners" + OPEN_TAG
FASTENER_TAG = "fastener" + OPEN_TAG
FASTENER_STATE_TAG = "fastener_state" + OPEN_TAG
TEMPFS_TAG = "tempfs" + OPEN_TAG
TEMPF_TAG = "tempf" + OPEN_TAG
DOCKING_POSS_TAG = "docking_positions" + OPEN_TAG
DOCKING_POS_TAG = "docking_position" + OPEN_TAG
END_EFFECTORS_TAG = "end_effectors" + OPEN_TAG
END_EFFECTOR_TAG = "end_effector" + OPEN_TAG
END_EFFECTOR_STATE_TAG = "end_effector_state" + OPEN_TAG
END_EFFECTOR_UID_TAG = "end_effector_uid" + OPEN_TAG
EXECUTE_TAG = "execute" + OPEN_TAG


class FokkerActionServer(Node):
    def __init__(self):
        super().__init__("fokker_action_server")
        self._action_server = ActionServer(self, CobotOp, "Cobot Operation", self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing FokkerActionServer...")

        # Accessing the request data and send to cobot
        # This instantiates and populates the classes in the Cobot controller
        self._send_goal_handle_to_cobot(goal_handle)

        # start sending the uid of the actions to the cobot 
        # to execute these actions
        action_number = 0
        for uid in goal_handle.request.execute:
            action_number = action_number + 1
            self.get_logger().info(f"Start action: {action_number} " + uid)
            self._send_execution_action_uid_to_cobot(uid)

            # Provide regular feedback during each execution
            iteration_number = 0
            while iteration_number >= 0:
                iteration_number = iteration_number + 1
                self.get_logger().info(f"Feedback iteration: {action_number}.{iteration_number}")

                # Fill in the feedback message
                feedback_msg = self._create_feedback_message_from_cobot_output()

                # Send feedback
                goal_handle.publish_feedback(feedback_msg)

                time.sleep(0.5)  # Just to slow the feedback down

                if feedback_msg.actions(action_number-1).state == AssemblyActionState.SUCCESS:
                    # Will stop the while loop and continue to the next action
                    iteration_number = -10

        self.get_logger().info(f"Finished actions")

        # Indicate the action succeeded (this does not indicate succes!)
        goal_handle.succeed()

        # Provide result
        return self._create_result_message_from_cobot_output()
    
    # Function to send execution commands to the cobot controller
    # Excution commands are given by sending the uid of the action
    # to be executed.
    def _send_execution_action_uid_to_cobot(self, uid_in):  
        str = EXECUTE_TAG

        # uid of the action to be executed
        str = str + UID_TAG + uid_in + CLOSE_TAG

        str = str + CLOSE_TAG

        return self._send_msg_to_cobot(str)  
    
    # Function to send goal_handle contents to the cobot controller
    def _send_goal_handle_to_cobot(self, goal_handle_in):
        func_output = [0] * 8

        # list of storage location containers
        func_output[0] = self._send_storage_to_cobot(goal_handle_in.request.storage)           
        if func_output[0] < 0:
            self.get_logger().info("Failed to send storage to cobot.")

        # list of product containers with holes
        func_output[1] = self._send_products_to_cobot(goal_handle_in.request.products)         
        if func_output[1] < 0:
            self.get_logger().info("Failed to send products to cobot.")

        # list of defined waypoints
        func_output[2] = self._send_waypoints_to_cobot(goal_handle_in.request.waypoints)       
        if func_output[2] < 0:
            self.get_logger().info("Failed to send waypoints to cobot.")

        # list of holes to be drilled
        func_output[4] = self._send_drill_tasks_to_cobot(goal_handle_in.request.drill_tasks)   
        if func_output[4] < 0:
            self.get_logger().info("Failed to send drill_tasks to cobot.")

        # list of available fasteners
        func_output[5] = self._send_fasteners_to_cobot(goal_handle_in.request.fasteners)       
        if func_output[5] < 0:
            self.get_logger().info("Failed to send fasteners to cobot.")

        # list of available temporary fasteners
        func_output[6] = self._send_tempfs_to_cobot(goal_handle_in.request.tempfs)             
        if func_output[6] < 0:
            self.get_logger().info("Failed to send tempfs to cobot.")

        # list of available docking positions for End Effectors
        func_output[7] = self._send_docking_pos_to_cobot(goal_handle_in.request.docking_pos)   
        if func_output[7] < 0:
            self.get_logger().info("Failed to send docking_pos to cobot.")

        # list of available End Effectors
        func_output[8] = self._send_ee_to_cobot(goal_handle_in.request.ee)                     
        if func_output[8] < 0:
            self.get_logger().info("Failed to send ee to cobot.")

        # list of defined actions
        # actions must be last because they require all other stuff to be there
        func_output[3] = self._send_actions_to_cobot(goal_handle_in.request.actions)           
        if func_output[3] < 0:
            self.get_logger().info("Failed to send actions to cobot.")

        return min(func_output)
        
    # Function to send list of storage location containers to the cobot controller
    # AssemblyHoleLocationContainer
    def _send_storage_to_cobot(self, storage_in):
        str = STORAGE_TAG + self._get_hole_location_container_to_cobot_str(storage_in) + CLOSE_TAG
        return self._send_msg_to_cobot(str)

    # get message string for AssemblyHoleLocationContainer
    def _get_hole_location_container_to_cobot_str(self, cont_in):
        str = HOLE_LOCATION_CONT_TAG 
        
        # uid of the container
        str = str + UID_TAG + cont_in.uid + CLOSE_TAG

        # list of hole locations
        str = str + LOCATIONS_TAG
        for ahl in cont_in.locations:
            str = str + self._get_hole_location_to_cobot_str(ahl)
        str = str + CLOSE_TAG

        # max_obstacle_height
        str = str + MAX_OBST_HEIGHT_TAG + str(cont_in.max_obstacle_height) + CLOSE_TAG

        # approach_pos_uid
        str = str + APPR_POS_UID_TAG + cont_in.approach_pos_uid + CLOSE_TAG 

        return str + CLOSE_TAG
    
    # get message string for AssemblyHoleLocation
    def _get_hole_location_to_cobot_str(self, loc_in):
        str = HOLE_LOCATION_TAG 
        # uid of the hole position: string uid
        str = str + UID_TAG + loc_in.uid + CLOSE_TAG                         

        # Nominal location of the hole: geometry_msgs/Pose nom_pos
        str = str + self._get_pose_str(loc_in.nom_pos)             

        # Diameter of the hole: float32 diam
        str = str + DIAM_TAG + str(loc_in.diam) + CLOSE_TAG                

        # Total thickness at the hole
        # Must be equal to the sum of layer thicknesses: float32 stack_t
        str = str + STACK_T_TAG + str(loc_in.stack_t) + CLOSE_TAG        

        # distance of the drill jig hole entry above the surface: float32 drill_jig_dist                  
        str = str + DRILL_JIG_DIST_TAG + str(loc_in.drill_jig_dist) + CLOSE_TAG

        # Whether or not the hole has been drilled: bool drilled
        str = str + DRILLED_TAG + str(loc_in.drilled) + CLOSE_TAG             

        # Material definition for each layer: AssemblyMaterialLayer[] layers
        str = str + LAYERS_TAG
        for layer in loc_in.layers:
            str = str + self._get_material_layer_to_cobot_str(layer)
        str = str + CLOSE_TAG

        return str + CLOSE_TAG
    
    # get message string for geometry_msgs/Pose
    def _get_pose_str(self, pose_in):
        str = POSE_TAG 

        # Get position
        str = str + POSE_PX_TAG + str(pose_in.position.x) + CLOSE_TAG
        str = str + POSE_PY_TAG + str(pose_in.position.y) + CLOSE_TAG
        str = str + POSE_PZ_TAG + str(pose_in.position.z) + CLOSE_TAG

        # Get orientation
        str = str + POSE_OX_TAG + str(pose_in.orientation.x) + CLOSE_TAG
        str = str + POSE_OY_TAG + str(pose_in.orientation.y) + CLOSE_TAG
        str = str + POSE_OZ_TAG + str(pose_in.orientation.z) + CLOSE_TAG
        str = str + POSE_OW_TAG + str(pose_in.orientation.w) + CLOSE_TAG

        return str + CLOSE_TAG
    
    # get message string for AssemblyMaterialLayer
    def _get_material_layer_to_cobot_str(self, layer_in):
        str = MAT_LAYER_TAG 

        # uid of the layer: string uid   
        str = str + UID_TAG + layer_in.uid + CLOSE_TAG

        # Thickness of the layer:float32 thickness 
        str = str + LAYER_THICKNESS_TAG + str(layer_in.thickness) + CLOSE_TAG

        # Drill specifications during the drilling in this layer
        # Speed of the drill:float32 speed     
        str = str + DRILL_SPEED_TAG + str(layer_in.speed) + CLOSE_TAG  

        # Feed rate of the drill:float32 feed  
        str = str + DRILL_FEED_TAG + str(layer_in.feed) + CLOSE_TAG 

        # Lower limit(s) of the torque bandwidth(s):float32 lower_torque_limits  
        str = str + LOWER_TORQUE_LIMIT_TAG + str(layer_in.lower_torque_limits) + CLOSE_TAG

        # Upper limit(s) of the torque bandwidth(s):float32 upper_torque_limits  
        str = str + UPPER_TORQUE_LIMIT_TAG + str(layer_in.upper_torque_limits) + CLOSE_TAG  

        # Torque level/threshold to switch to the next layer in the stack: float32 threshold 
        str = str + TORQUE_THRESHOLD_TAG + str(layer_in.threshold) + CLOSE_TAG 

        # Temporary fastening specifications for this layer
        # Maximum temporary fastener clamp force:float32 max_clamp_force                 
        str = str + MAX_TEMPF_CLAMP_FORCE_TAG + str(layer_in.max_clamp_force) + CLOSE_TAG

        return str + CLOSE_TAG
                   
    # Function to send list of product containers with holes to the cobot controller
    def _send_products_to_cobot(self, products_in):
        str = PRODUCTS_TAG

        for product in products_in:
            str = PRODUCT_TAG + self._get_hole_location_container_to_cobot_str(product) + CLOSE_TAG

        str = str + CLOSE_TAG

        return self._send_msg_to_cobot(str)

    # Function to send list of defined waypoints to the cobot controller
    def _send_waypoints_to_cobot(self, waypoints_in):
        str = WAYPOINTS_TAG

        for waypoint in waypoints_in:
            str = str + self._get_waypoint_to_cobot_str(waypoint)

        str = str + CLOSE_TAG

        return self._send_msg_to_cobot(str)
    
    # get message string for AssemblyWaypoint
    def _get_waypoint_to_cobot_str(self, waypoint_in):
        str = WAYPOINT_TAG

        # uid: uid of the hole Waypoint
        str = str + UID_TAG + waypoint_in.uid + CLOSE_TAG

        # pos: location of the waypoint
        str = str + self._get_pose_str(waypoint_in.pos)

        # bool: orient: whether or not the orientation is mandatory
        str = str + ORIENTATION_TAG + str(waypoint_in.orient) + CLOSE_TAG

        # float: blend_radius: Whether the cobot movement can cut a corner when
        # passing this waypoint.
        str = str + BLEND_RADIUS_TAG + str(waypoint_in.blend_radius) + CLOSE_TAG

        return str + CLOSE_TAG

    # Function to send list of defined actions to the cobot controller
    def _send_actions_to_cobot(self, actions_in): 
        str = ACTIONS_TAG

        for action in actions_in:
            str = str + self._get_action_to_cobot_str(action)

        str = str + CLOSE_TAG

        return self._send_msg_to_cobot(str) 
    
    # get message string for AssemblyAction
    def _get_action_to_cobot_str(self, action_in):
        str = ACTIONS_TAG

        # string uid: uid of the hole Action
        str = str + UID_TAG + action_in.uid + CLOSE_TAG

        # string obj_uid: uid of the action object to execute
        str = str + OBJ_UID_TAG + action_in.obj_uid + CLOSE_TAG

        # string loc_uid: uid of the target location of the object
        str = str + LOC_UID_TAG + action_in.loc_uid + CLOSE_TAG
                                        
        # AssemblyActionState state: Status of the action (uint8)
        str = str + ACTION_STATE_TAG + str(action_in.state) + CLOSE_TAG

        # string[] passing: list of waypoints uids to pass
        # e.g. to pass between pick up and place of a temporary fastener
        # string loc_uid: uid of the target location of the object
        str = str + PASSING_UIDS_TAG 
        for passing in action_in.passing:
            str = str + PASSING_UID_TAG + action_in.passing + CLOSE_TAG
        str = str + CLOSE_TAG

        # uint8 speed: the speed as percentage of the maximum speed
        str = str + SPEED_TAG + str(action_in.speed) + CLOSE_TAG

        return str + CLOSE_TAG

    # Function to send list of holes to be drilled to the cobot controller
    def _send_drill_tasks_to_cobot(self, drill_tasks_in): 
        str = DRILL_TASKS_TAG

        for drill_task in drill_tasks_in:
            str = str + self._get_drill_task_to_cobot_str(drill_task)

        str = str + CLOSE_TAG

        return self._send_msg_to_cobot(str) 
    
    # get message string for AssemblyDrillTask
    def _get_drill_task_to_cobot_str(self, drill_task_in):
        str = DRILL_TASK_TAG

        #string ee_uid                           # uid of the end effector needed drill this hole
        str = str + UID_TAG + drill_task_in.uid + CLOSE_TAG

        #float32 diam                            # diameter of the drill
                                                 # for reference only because real diameter defined by ee_uid
        str = str + DIAM_TAG + str(drill_task_in.diam) + CLOSE_TAG

        #geometry_msgs/Pose jig_pos              # Location of the drill jig drill guiding hole
                                                 # Only available after drilling
        str = str + self._get_pose_str(drill_task_in.jig_pos)

        #AssemblyMaterialLayer[] layers          # Material layers as encountered during drilling
                                                 # Data filled during and_or after drilling
        str = str + LAYERS_TAG
        for layer in drill_task_in.layers:
            str = str + self._get_material_layer_to_cobot_str(layer)
        str = str + CLOSE_TAG

        return str + CLOSE_TAG 

    # Function to send list of available fasteners to the cobot controller
    def _send_fasteners_to_cobot(self, fasteners_in):  
        str = FASTENERS_TAG

        for fastener in fasteners_in:
            str = str + self._get_fastener_to_cobot_str(fastener)

        str = str + CLOSE_TAG

        return self._send_msg_to_cobot(str) 
    
    # get message string for AssemblyFastener
    def _get_fastener_to_cobot_str(self, fastener_in):
        str = FASTENER_TAG

        #string uid                              # uid of the fastener
        str = str + UID_TAG + fastener_in.uid + CLOSE_TAG

        #string loc_uid                          # uid of the location of the fastener in one of the containers
        str = str + LOC_UID_TAG + fastener_in.loc_uid + CLOSE_TAG

        #string ee_uid                           # uid of the end effector needed to install this fastener
        str = str + END_EFFECTOR_UID_TAG + fastener_in.ee_uid + CLOSE_TAG

        #AssemblyFastState state                 # The state of the fastener
        str = str + FASTENER_STATE_TAG + str(fastener_in.state) + CLOSE_TAG

        #geometry_msgs/Pose inst_pos             # Installed location of the fastener
                                                 # Only available after installation
        str = str + self._get_pose_str(fastener_in.inst_pos)

        #float32 diam                            # diameter of the fastener
        str = str + DIAM_TAG + str(fastener_in.diam) + CLOSE_TAG

        #float32 shaft_height                    # the height of the shaft that is sticking out when in storage location
        str = str + SHAFT_HEIGHT_TAG + str(fastener_in.shaft_height) + CLOSE_TAG

        #float32 min_stack                       # the minimum stack
        str = str + MIN_STACK_TAG + str(fastener_in.min_stack) + CLOSE_TAG

        #float32 max_stack                       # the maximum stack
        str = str + MAX_STACK_TAG + str(fastener_in.max_stack) + CLOSE_TAG

        #float32 tcp_tip_distace                 # distance between hole entry point and tip if inserted in a hole
        str = str + TCP_TIP_DIST_TAG + str(fastener_in.tcp_tip_distace) + CLOSE_TAG

        #float32 tcp_top_distace                 # distance between hole entry point and top if inserted in a hole
                                                 # the top is where the tcp is when engaging the tempf
        str = str + TCP_TOP_DIST_TAG + str(fastener_in.tcp_tip_distace) + CLOSE_TAG

        return str + CLOSE_TAG 

    # Function to send list of available temporary fasteners to the cobot controller
    def _send_tempfs_to_cobot(self, tempfs_in): 
        str = TEMPFS_TAG

        for tempf in tempfs_in:
            str = str + self._get_tempf_to_cobot_str(tempf)

        str = str + CLOSE_TAG

        return self._send_msg_to_cobot(str) 
    
    # get message string for AssemblyTempf
    def _get_tempf_to_cobot_str(self, tempf_in):
        str = TEMPF_TAG

        #string uid                              # uid of the temporary fastener
        str = str + UID_TAG + tempf_in.uid + CLOSE_TAG

        #string loc_uid                          # uid of the location of the temporary fastener
        str = str + LOC_UID_TAG + tempf_in.loc_uid + CLOSE_TAG

        #string ee_uid                           # uid of the end effector needed to manipulate this temporary fastener
        str = str + END_EFFECTOR_UID_TAG + tempf_in.ee_uid + CLOSE_TAG

        #AssemblyFastState state                 # The state of the temporary fastener
        str = str + FASTENER_STATE_TAG + str(tempf_in.state) + CLOSE_TAG

        #geometry_msgs/Pose inst_pos             # Installed location of the temporary fastener
                                                 # Only available after installation
        str = str + self._get_pose_str(tempf_in.inst_pos)

        #float32 diam                            # diameter of the temporary fastener
        str = str + DIAM_TAG + str(tempf_in.diam) + CLOSE_TAG

        #float32 shaft_height                    # the height of the shaft that is sticking out when installed
        str = str + SHAFT_HEIGHT_TAG + str(tempf_in.shaft_height) + CLOSE_TAG

        #float32 min_stack                       # the minimum stack
        str = str + MIN_STACK_TAG + str(tempf_in.min_stack) + CLOSE_TAG

        #float32 max_stack                       # the maximum stack
        str = str + MAX_STACK_TAG + str(tempf_in.max_stack) + CLOSE_TAG

        #float32 tcp_tip_distace                 # distance between hole entry point and tip if inserted in a hole 
        str = str + TCP_TIP_DIST_TAG + str(tempf_in.tcp_tip_distace) + CLOSE_TAG

        #float32 tcp_top_distace                 # distance between hole entry point and top if inserted in a hole
                                                 # the top is where the tcp is when engaging the tempf 
        str = str + TCP_TOP_DIST_TAG + str(tempf_in.tcp_tip_distace) + CLOSE_TAG        

        return str + CLOSE_TAG 

    # Function to send list of available docking positions for End Effectors to the cobot controller
    def _send_docking_pos_to_cobot(self, docking_pos_in): 
        str = DOCKING_POSS_TAG

        for docking_pos in docking_pos_in:
            str = str + self._get_docking_pos_to_cobot_str(docking_pos)

        str = str + CLOSE_TAG

        return self._send_msg_to_cobot(str) 
    
    # get message string for AssemblyEeDockingPos
    def _get_docking_pos_to_cobot_str(self, docking_pos_in):
        str = DOCKING_POS_TAG

        #string uid                  # uid of the Docking Position
        str = str + UID_TAG + docking_pos_in.uid + CLOSE_TAG

        #geometry_msgs/Pose pos      # location of the Docking Position
        str = str + self._get_pose_str(docking_pos_in.pos)

        return str + CLOSE_TAG 
      
    # Function to send list of available End Effectors to the cobot controller
    def _send_ee_to_cobot(self, ee_in):  
        str = END_EFFECTORS_TAG

        for ee in ee_in:
            str = str + self._get_end_effector_to_cobot_str(ee)

        str = str + CLOSE_TAG

        return self._send_msg_to_cobot(str) 

    # get message string for End Effector
    def _get_end_effector_to_cobot_str(self, ee_in):
        str = END_EFFECTOR_TAG

        #string uid                      # uid of the End Effector
        str = str + UID_TAG + ee_in.uid + CLOSE_TAG

        #string loc_uid                  # uid of the location of the End Effector in one of
                                         # possible docking positions
        str = str + LOC_UID_TAG + ee_in.loc_uid + CLOSE_TAG

        str = str + DOCKING_POSS_TAG
        #string[] poss_dock_pos_uids     # list of possible docking position uid's
        for poss_docking_pos_uid in ee_in.poss_dock_pos_uids:
            str = str + DOCKING_POS_TAG + poss_docking_pos_uid + CLOSE_TAG
        str = str + CLOSE_TAG

        #AssemblyEeState state           # The state of the End Effector
        #uint8 STORED = 1, uint8 ON_COBOT = 2, uint8 STORED_NEED_SERVICE = 3
        str = str + END_EFFECTOR_STATE_TAG + str(ee_in.state) + CLOSE_TAG

        return str + CLOSE_TAG  

    # function to send a message to the cobot
    # the message will be received by the cobot controller
    # the message will be used to instantiate/popuate classes
    # in the cobot conroller
    def _send_msg_to_cobot(self, str_in):
        #TODO send to cobot
        return 1 
    
    # function to get messages send by the cobot
    def _get_str_from_cobot(self):
        #TODO clear the messages
        
        # trigger for cobot to send string with status
        self._send_msg_to_cobot(PROVIDE_STATUS_MSG)   

        #TODO wait until cobot finishes writing

        #TODO get string message from cobot
        return "not implemented yet" 

    # Function to populate feedback message based on information from the cobot controller
    def _create_feedback_message_from_cobot_output(self):
        
        # Create a feedback message
        feedback_msg = CobotOp.Feedback()

        c_str = self._get_str_from_cobot()

        a_str = self._find_substring(c_str, ACTIONS_TAG)
        if a_str is not None:
            feedback_msg.actions = self._create_actions_from_cobot_output(a_str)  

        d_str = self._find_substring(c_str, DRILL_TASKS_TAG)
        if d_str is not None:
            feedback_msg.drill_tasks = self._create_drill_tasks_from_cobot_output(d_str)

        t_str = self._find_substring(c_str, TEMPFS_TAG)
        if t_str is not None:
            feedback_msg.tempfs = self._create_tempfs_from_cobot_output(t_str)

        f_str = self._find_substring(c_str, FASTENERS_TAG)
        if f_str is not None:
            feedback_msg.drill_tasks = self._create_fasteners_from_cobot_output(f_str)

        ee_str = self._find_substring(c_str, END_EFFECTORS_TAG)
        if ee_str is not None:
            feedback_msg.ee = self._create_ee_from_cobot_output(ee_str)
        
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
    def _create_result_message_from_cobot_output(self):
        
        # get the string from the cobot
        c_str = self._get_str_from_cobot()
    
        # Create a feedback message
        result_msg = CobotOp.Result()

        a_str = self._find_substring(c_str, ACTIONS_TAG)
        if a_str is not None:
            result_msg.actions_out = self._create_actions_from_cobot_output(a_str)  

        d_str = self._find_substring(c_str, DRILL_TASKS_TAG)
        if d_str is not None:
            result_msg.drill_tasks_out = self._create_drill_tasks_from_cobot_output(d_str)

        t_str = self._find_substring(c_str, TEMPFS_TAG)
        if t_str is not None:
            result_msg.tempfs_out = self._create_tempfs_from_cobot_output(t_str)

        f_str = self._find_substring(c_str, FASTENERS_TAG)
        if f_str is not None:
            result_msg.drill_tasks_out = self._create_fasteners_from_cobot_output(f_str)

        ee_str = self._find_substring(c_str, END_EFFECTORS_TAG)
        if ee_str is not None:
            result_msg.ee_out = self._create_ee_from_cobot_output(ee_str)
        
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

    # function to get a substring from an original_string
    # will return the string after the first search_string 
    # the substring is that part of the original_string after a search_string
    def _find_substring(self, original_string, search_string):

        # Find the position of the search string
        position = original_string.find(search_string)

        if position != -1:
            # Extract the part of the string after the search string
            return original_string[position + len(search_string):]
        else:
            return None

    # function to get the content of a leaf object between open_tag and close_tag
    # Example usage:
    #input_string = "tag1<tag2<content1>more<tag3<content3>>>tag4<content4>"
    #tag1_content = extract_leaf_content(input_string, "tag2<", ">")
    def extract_leaf_content(self, input_string, open_tag, close_tag):
        pattern = re.compile(r'{}(.*?)(?={})'.format(open_tag, close_tag), re.DOTALL)
        match = pattern.search(input_string)

        if match:
            return match.group(1)
        else:
            return None
    
    # Function to create list of defined actions based on information from the cobot controller
    def _create_actions_from_cobot_output(self, c_str): 
        # get the string after the first ACTION_TAG
        a_str = self._find_substring(c_str, ACTION_TAG)

        lst = []  # [AssemblyAction]

        while a_str is not None:
            lst.append(self._get_action_from_str(a_str))
            
            # check if there is another string after an ACTION_TAG
            a_str = self._find_substring(a_str, ACTION_TAG)

        return lst

    # Function to create list of holes to be drilled based on information from the cobot controller
    def _create_drill_tasks_from_cobot_output(self, c_str): 
        # get the string after the first DRILL_TASK_TAG
        d_str = self._find_substring(c_str, DRILL_TASK_TAG)
        lst = []  # [AssemblyDrill]

        while d_str is not None:
            lst.append(self._get_drill_task_from_str(d_str))
            
            # check if there is another string after an DRILL_TASK_TAG
            d_str = self._find_substring(d_str, DRILL_TASK_TAG)

        return lst

    # Function to create list of available fasteners based on information from the cobot controller
    def _create_fasteners_from_cobot_output(self, c_str): 
        # get the string after the first FASTENER_TAG
        f_str = self._find_substring(c_str, FASTENER_TAG)
        lst = []  # [AssemblyFast]

        while f_str is not None:
            lst.append(self._get_fastener_from_str(f_str))
            
            # check if there is another string after an FASTENER_TAG
            f_str = self._find_substring(f_str, FASTENER_TAG)

        return lst

    # Function to create list of available temporary fasteners based on information from the cobot controller
    def _create_tempfs_from_cobot_output(self, c_str): 
        # get the string after the first TEMPF_TAG
        t_str = self._find_substring(c_str, TEMPF_TAG)
        lst = []  # [AssemblyTempFast]

        while t_str is not None:
            lst.append(self._get_tempf_from_str(t_str))
            
            # check if there is another string after an TEMPF_TAG
            t_str = self._find_substring(t_str, TEMPF_TAG)

        return lst
      
    # Function to create list of available End Effectors based on information from the cobot controller
    def _create_ee_from_cobot_output(self, c_str): 
        # get the string after the first END_EFFECTOR_TAG
        ee_str = self._find_substring(c_str, END_EFFECTOR_TAG)
        lst = []  # [AssemblyEe]

        while ee_str is not None:
            lst.append(self._get_ee_from_str(ee_str))
            
            # check if there is another string after an END_EFFECTOR_TAG
            ee_str = self._find_substring(ee_str, END_EFFECTOR_TAG)

        return lst            

    # Function to create AssemblyAction object from a string
    def _get_action_from_str(self, c_str): 
        obj = AssemblyAction()

        # get the 'uid': 'string',
        obj.uid = self.extract_leaf_content(c_str, UID_TAG, CLOSE_TAG)

        # get the 'obj_uid': 'string',
        obj.obj_uid = self.extract_leaf_content(c_str, OBJ_UID_TAG, CLOSE_TAG)

        # get the 'loc_uid': 'string',
        obj.loc_uid = self.extract_leaf_content(c_str, LOC_UID_TAG, CLOSE_TAG)

        # get the 'state': 'penelope_aerospace_pl_msgs/AssemblyActionState',
        obj.state = self.extract_leaf_content(c_str, ACTION_STATE_TAG, CLOSE_TAG)

        # get the 'passing': 'sequence<string>',
        # get the string after the first PASSING_UIDS_TAG
        p_str = self._find_substring(c_str, PASSING_UIDS_TAG)
        lst = [] 

        while p_str is not None:
            lst.append(self.extract_leaf_content(p_str, PASSING_UID_TAG, CLOSE_TAG))
            
            # check if there is another string after an PASSING_UID_TAG
            p_str = self._find_substring(p_str, PASSING_UID_TAG)

        obj.passing = lst

        # get the 'speed': 'uint8',
        obj.speed = int(self.extract_leaf_content(c_str, SPEED_TAG, CLOSE_TAG))

        return obj
    
    # Function to create AssemblyDrill object from a string
    def _get_drill_task_from_str(self, c_str): 
        obj = AssemblyDrill()

        # get the 'uid': 'string',
        obj.uid = self.extract_leaf_content(c_str, UID_TAG, CLOSE_TAG)

        # get the 'loc_uid': 'string',
        obj.loc_uid = self.extract_leaf_content(c_str, LOC_UID_TAG, CLOSE_TAG)

        # get the 'ee_uid': 'string',
        obj.ee_uid = self.extract_leaf_content(c_str, END_EFFECTOR_UID_TAG, CLOSE_TAG)

        # get the 'diam': 'float',
        obj.diam = float()

        # get the 'jig_pos': 'geometry_msgs/Pose',
        obj.pose = self._get_pose_from_str(self._find_substring(c_str, POSE_TAG))

        # get the 'layers': 'sequence<penelope_aerospace_pl_msgs/AssemblyMaterialLayer>',
        # get the string after the first LAYERS_TAG
        l_str = self._find_substring(c_str, LAYERS_TAG)
        lst = [] 

        while l_str is not None:
            lst.append(self._get_material_layer_from_str(self._find_substring(l_str, MAT_LAYER_TAG)))
            
            # check if there is another string after an MAT_LAYER_TAG
            l_str = self._find_substring(l_str, MAT_LAYER_TAG)

        obj.layers = lst

        return obj
    
    # Function to create Pose object from a string
    def _get_pose_from_str(self, c_str):
        # Create a new Pose object
        pose = Pose()

        # Set the position (x, y, z)
        pose.position.x = float(self.extract_leaf_content(c_str, POSE_PX_TAG, CLOSE_TAG))
        pose.position.y = float(self.extract_leaf_content(c_str, POSE_PY_TAG, CLOSE_TAG))
        pose.position.z = float(self.extract_leaf_content(c_str, POSE_PZ_TAG, CLOSE_TAG))

        # Set the orientation (quaternion: x, y, z, w)
        pose.orientation.x = float(self.extract_leaf_content(c_str, POSE_OX_TAG, CLOSE_TAG))
        pose.orientation.y = float(self.extract_leaf_content(c_str, POSE_OY_TAG, CLOSE_TAG))
        pose.orientation.z = float(self.extract_leaf_content(c_str, POSE_OZ_TAG, CLOSE_TAG))
        pose.orientation.w = float(self.extract_leaf_content(c_str, POSE_OW_TAG, CLOSE_TAG))

        return pose

    # Function to create AssemblyMaterialLayer object from a string
    def _get_material_layer_from_str(self, c_str):
        obj = AssemblyMaterialLayer()

        # get the 'uid': 'string',
        obj.uid = self.extract_leaf_content(c_str, UID_TAG, CLOSE_TAG)

        # get the 'thickness': 'float',
        obj.thickness = float(self.extract_leaf_content(c_str, LAYER_THICKNESS_TAG, CLOSE_TAG))

        # get the 'speed': 'float',
        obj.speed = float(self.extract_leaf_content(c_str, DRILL_SPEED_TAG, CLOSE_TAG))

        # get the 'feed': 'float',
        obj.feed = float(self.extract_leaf_content(c_str, DRILL_FEED_TAG, CLOSE_TAG))

        # get the 'lower_torque_limits': 'float',
        obj.lower_torque_limits = float(self.extract_leaf_content(c_str, LOWER_TORQUE_LIMIT_TAG, CLOSE_TAG))

        # get the 'upper_torque_limits': 'float',
        obj.upper_torque_limits = float(self.extract_leaf_content(c_str, UPPER_TORQUE_LIMIT_TAG, CLOSE_TAG))

        # get the 'threshold': 'float',
        obj.threshold = float(self.extract_leaf_content(c_str, TORQUE_THRESHOLD_TAG, CLOSE_TAG))

        # get the 'max_clamp_force': 'float',
        obj.max_clamp_force = float(self.extract_leaf_content(c_str, MAX_TEMPF_CLAMP_FORCE_TAG, CLOSE_TAG))
    
        return obj
    
    # Function to create AssemblyFast object from a string
    def _get_fastener_from_str(self, c_str): 
        obj = AssemblyFast()

        # get the 'uid': 'string',
        obj.uid = self.extract_leaf_content(c_str, UID_TAG, CLOSE_TAG)

        # get the 'loc_uid': 'string',
        obj.loc_uid = self.extract_leaf_content(c_str, LOC_UID_TAG, CLOSE_TAG)

        # get the 'ee_uid': 'string',
        obj.ee_uid = self.extract_leaf_content(c_str, END_EFFECTOR_UID_TAG, CLOSE_TAG)

        # get the 'state': 'penelope_aerospace_pl_msgs/AssemblyFastState',
        obj.state = int(self.extract_leaf_content(c_str, FASTENER_STATE_TAG, CLOSE_TAG))

        # get the 'inst_pos': 'geometry_msgs/Pose',
        obj.inst_pos = self._get_pose_from_str(self._find_substring(c_str, POSE_TAG))

        # get the 'diam': 'float',DIAM_TAG
        obj.diam = float(self.extract_leaf_content(c_str, DIAM_TAG, CLOSE_TAG))

        # get the 'shaft_height': 'float',SHAFT_HEIGHT_TAG
        obj.shaft_height = float(self.extract_leaf_content(c_str, SHAFT_HEIGHT_TAG, CLOSE_TAG))

        # get the 'min_stack': 'float',MIN_STACK_TAG
        obj.min_stack = float(self.extract_leaf_content(c_str, MIN_STACK_TAG, CLOSE_TAG))

        # get the 'max_stack': 'float',MAX_STACK_TAG
        obj.max_stack = float(self.extract_leaf_content(c_str, MAX_STACK_TAG, CLOSE_TAG))

        # get the 'tcp_tip_distace': 'float',TCP_TIP_DIST_TAG
        obj.tcp_tip_distace = float(self.extract_leaf_content(c_str, TCP_TIP_DIST_TAG, CLOSE_TAG))

        # get the 'tcp_top_distace': 'float',TCP_TOP_DIST_TAG
        obj.tcp_top_distace = float(self.extract_leaf_content(c_str, TCP_TOP_DIST_TAG, CLOSE_TAG))

        return obj
    
    # Function to create AssemblyTempFast object from a string
    def _get_tempf_from_str(self, c_str): 
        obj = AssemblyTempFast()

        # get the 'uid': 'string',
        obj.uid = self.extract_leaf_content(c_str, UID_TAG, CLOSE_TAG)

        # get the 'loc_uid': 'string',
        obj.loc_uid = self.extract_leaf_content(c_str, LOC_UID_TAG, CLOSE_TAG)

        # get the 'ee_uid': 'string',
        obj.ee_uid = self.extract_leaf_content(c_str, END_EFFECTOR_UID_TAG, CLOSE_TAG)

        # get the 'state': 'penelope_aerospace_pl_msgs/AssemblyFastState',
        obj.state = int(self.extract_leaf_content(c_str, FASTENER_STATE_TAG, CLOSE_TAG))

        # get the 'inst_pos': 'geometry_msgs/Pose',
        obj.inst_pos = self._get_pose_from_str(self._find_substring(c_str, POSE_TAG))

        # get the 'diam': 'float',DIAM_TAG
        obj.diam = float(self.extract_leaf_content(c_str, DIAM_TAG, CLOSE_TAG))

        # get the 'shaft_height': 'float',SHAFT_HEIGHT_TAG
        obj.shaft_height = float(self.extract_leaf_content(c_str, SHAFT_HEIGHT_TAG, CLOSE_TAG))

        # get the 'min_stack': 'float',MIN_STACK_TAG
        obj.min_stack = float(self.extract_leaf_content(c_str, MIN_STACK_TAG, CLOSE_TAG))

        # get the 'max_stack': 'float',MAX_STACK_TAG
        obj.max_stack = float(self.extract_leaf_content(c_str, MAX_STACK_TAG, CLOSE_TAG))

        # get the 'tcp_tip_distace': 'float',TCP_TIP_DIST_TAG
        obj.tcp_tip_distace = float(self.extract_leaf_content(c_str, TCP_TIP_DIST_TAG, CLOSE_TAG))

        # get the 'tcp_top_distace': 'float',TCP_TOP_DIST_TAG
        obj.tcp_top_distace = float(self.extract_leaf_content(c_str, TCP_TOP_DIST_TAG, CLOSE_TAG))

        return obj
    
    # Function to create AssemblyEe object from a string
    def _get_ee_from_str(self, c_str): 
        obj = AssemblyEe()

        # get the 'uid': 'string',
        obj.uid = self.extract_leaf_content(c_str, UID_TAG, CLOSE_TAG)

        # get the 'loc_uid': 'string',
        obj.loc_uid = self.extract_leaf_content(c_str, LOC_UID_TAG, CLOSE_TAG)

        # get the 'poss_dock_pos_uids': 'sequence<string>',
        # get the string after the first DOCKING_POSS_TAG
        p_str = self._find_substring(c_str, DOCKING_POSS_TAG)
        lst = [] 

        while p_str is not None:
            lst.append(self.extract_leaf_content(p_str, DOCKING_POS_TAG, CLOSE_TAG))
            
            # check if there is another string after an DOCKING_POS_TAG
            p_str = self._find_substring(p_str, DOCKING_POS_TAG)

        obj.passing = lst

        # get the 'state': 'penelope_aerospace_pl_msgs/AssemblyEeState',END_EFFECTOR_STATE_TAG
        obj.state = int(self.extract_leaf_content(p_str, END_EFFECTOR_STATE_TAG, CLOSE_TAG))

        return obj


def main(args=None):
    # Initialize the ROS2 action server node
    rclpy.init(args=args)
    action_server = FokkerActionServer()

    # Spin so the server does not shutdown untill requested (e.g. Ctrl-C or another shutdown event)
    rclpy.spin(action_server)


if __name__ == "__main__":
    main()
