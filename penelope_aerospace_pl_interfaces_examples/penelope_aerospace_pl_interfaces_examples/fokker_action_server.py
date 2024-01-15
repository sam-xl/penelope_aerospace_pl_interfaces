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

CLOSE_TAG = ")"
UID_TAG = "uid("
STORAGE_TAG = "storage("
PRODUCT_TAG = "product("
LOCATIONS_TAG = "locations("
MAX_OBST_HEIGHT_TAG = "max_obstacle_height("
APPR_POS_UID_TAG = "approach_pos_uid("
HOLE_LOCATION_CONT_TAG = "hole_location_container("
HOLE_LOCATION_TAG = "hole_location("
POSE_TAG = "pose("
POSE_PX_TAG = "pose_p_x("
POSE_PY_TAG = "pose_p_y("
POSE_PZ_TAG = "pose_p_z("
POSE_OX_TAG = "pose_o_x("
POSE_OY_TAG = "pose_o_y("
POSE_OZ_TAG = "pose_o_z("
POSE_OW_TAG = "pose_o_w("
DIAM_TAG = "diam("
STACK_T_TAG = "stack_thickness_tag("
DRILL_JIG_DIST_TAG = "drill_jig_dist("
DRILLED_TAG = "drilled("
LAYERS_TAG = "layers("
MAT_LAYER_TAG = "material_layer("
LAYER_THICKNESS_TAG = "layer_thickness("                               
DRILL_SPEED_TAG = "drill_speed("                           
DRILL_FEED_TAG = "drill_feed("                            
LOWER_TORQUE_LIMIT_TAG = "lower_torque_limit("             
UPPER_TORQUE_LIMIT_TAG = "upper_torque_limit("            
TORQUE_THRESHOLD_TAG = "torque_threshold("                      
MAX_TEMPF_CLAMP_FORCE_TAG = "max_tempf_clamp_force("
WAYPOINT_TAG = "waypoint("
WAYPOINTS_TAG = "waypoints("
ORIENTATION_TAG = "orientation("
BLEND_RADIUS_TAG = "blend_radius("
ACTIONS_TAG = "actions("
ACTION_TAG = "action("
OBJ_UID_TAG = "obj_uid("
LOC_UID_TAG = "loc_uid("
ACTION_STATE_TAG = "action_state("
PASSING_UIDS_TAG = "passing_uids("
PASSING_UID_TAG = "passing_uid("
SPEED_TAG = "speed("

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
                self._populate_feedback_message_from_cobot_output(feedback_msg)

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

        # list of defined actions
        func_output[3] = self._send_actions_to_cobot(goal_handle_in.request.actions)           
        if func_output[3] < 0:
            self.get_logger().info("Failed to send actions to cobot.")

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

        return min(func_output)
        
    # Function to send list of storage location containers to the cobot controller
    # AssemblyHoleLocationContainer
    def _send_storage_to_cobot(self, storage_in):
        str = STORAGE_TAG + self._get_hole_location_container_to_cobot_str(storage_in) + CLOSE_TAG
        self._send_msg_to_cobot(str)
        return 1

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
        str = str + MAX_OBST_HEIGHT_TAG + cont_in.max_obstacle_height + CLOSE_TAG

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
        str = str + DIAM_TAG + loc_in.diam + CLOSE_TAG                

        # Total thickness at the hole
        # Must be equal to the sum of layer thicknesses: float32 stack_t
        str = str + STACK_T_TAG + loc_in.stack_t + CLOSE_TAG        

        # distance of the drill jig hole entry above the surface: float32 drill_jig_dist                  
        str = str + DRILL_JIG_DIST_TAG + loc_in.drill_jig_dist + CLOSE_TAG

        # Whether or not the hole has been drilled: bool drilled
        str = str + DRILLED_TAG + loc_in.drilled + CLOSE_TAG             

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
        str = str + POSE_PX_TAG + pose_in.position.x + CLOSE_TAG
        str = str + POSE_PY_TAG + pose_in.position.y + CLOSE_TAG
        str = str + POSE_PZ_TAG + pose_in.position.z + CLOSE_TAG

        # Get orientation
        str = str + POSE_OX_TAG + pose_in.orientation.x + CLOSE_TAG
        str = str + POSE_OY_TAG + pose_in.orientation.y + CLOSE_TAG
        str = str + POSE_OZ_TAG + pose_in.orientation.z + CLOSE_TAG
        str = str + POSE_OW_TAG + pose_in.orientation.w + CLOSE_TAG

        return str + CLOSE_TAG
    
    # get message string for AssemblyMaterialLayer
    def _get_material_layer_to_cobot_str(self, layer_in):
        str = MAT_LAYER_TAG 

        # uid of the layer: string uid   
        str = str + UID_TAG + layer_in.uid + CLOSE_TAG

        # Thickness of the layer:float32 thickness 
        str = str + LAYER_THICKNESS_TAG + layer_in.thickness + CLOSE_TAG

        # Drill specifications during the drilling in this layer
        # Speed of the drill:float32 speed     
        str = str + DRILL_SPEED_TAG+ layer_in.speed + CLOSE_TAG  

        # Feed rate of the drill:float32 feed  
        str = str + DRILL_FEED_TAG + layer_in.feed + CLOSE_TAG 

        # Lower limit(s) of the torque bandwidth(s):float32 lower_torque_limits  
        str = str + LOWER_TORQUE_LIMIT_TAG + layer_in.lower_torque_limits + CLOSE_TAG

        # Upper limit(s) of the torque bandwidth(s):float32 upper_torque_limits  
        str = str + UPPER_TORQUE_LIMIT_TAG + layer_in.upper_torque_limits + CLOSE_TAG  

        # Torque level/threshold to switch to the next layer in the stack: float32 threshold 
        str = str + TORQUE_THRESHOLD_TAG + layer_in.threshold + CLOSE_TAG 

        # Temporary fastening specifications for this layer
        # Maximum temporary fastener clamp force:float32 max_clamp_force                 
        str = str + MAX_TEMPF_CLAMP_FORCE_TAG + layer_in.max_clamp_force + CLOSE_TAG

        return str + CLOSE_TAG
                   
    # Function to send list of product containers with holes to the cobot controller
    def _send_products_to_cobot(self, products_in):
        str = PRODUCT_TAG + self._get_hole_location_container_to_cobot_str(products_in) + CLOSE_TAG
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
        str = str + ORIENTATION_TAG + waypoint_in.orient + CLOSE_TAG

        # float: blend_radius: Whether the cobot movement can cut a corner when
        # passing this waypoint.
        str = str + BLEND_RADIUS_TAG + waypoint_in.blend_radius + CLOSE_TAG

        return str + CLOSE_TAG

    # Function to send list of defined actions to the cobot controller
    def _send_actions_to_cobot(self, actions_in): 
        str = ACTIONS_TAG

        for action in actions_in:
            str = str + self._get_action_in_to_cobot_str(action)

        str = str + CLOSE_TAG

        return self._send_msg_to_cobot(str) 
    
    # get message string for AssemblyAction
    def _get_action_in_to_cobot_str(self, action_in):
        str = ACTIONS_TAG

        # string uid: uid of the hole Action
        str = str + UID_TAG + action_in.uid + CLOSE_TAG

        # string obj_uid: uid of the action object to execute
        str = str + OBJ_UID_TAG + action_in.obj_uid + CLOSE_TAG

        # string loc_uid: uid of the target location of the object
        str = str + LOC_UID_TAG + action_in.loc_uid + CLOSE_TAG
                                        
        # AssemblyActionState state: Status of the action
        str = str + ACTION_STATE_TAG + action_in.state + CLOSE_TAG

        # string[] passing: list of waypoints uids to pass
        # e.g. to pass between pick up and place of a temporary fastener
        # string loc_uid: uid of the target location of the object
        str = str + PASSING_UIDS_TAG 
        for passing in action_in.passing:
            str = str + PASSING_UID_TAG + action_in.passing + CLOSE_TAG
        str = str + CLOSE_TAG

        # uint8 speed: the speed as percentage of the maximum speed
        str = str + SPEED_TAG + action_in.speed + CLOSE_TAG

        return str + CLOSE_TAG

    # Function to send list of holes to be drilled to the cobot controller
    def _send_drill_tasks_to_cobot(self, drill_tasks_in): 
        return 1

    # Function to send list of available fasteners to the cobot controller
    def _send_fasteners_to_cobot(self, fasteners_in):  
        return 1 

    # Function to send list of available temporary fasteners to the cobot controller
    def _send_tempfs_to_cobot(self, tempfs_in): 
        return 1

    # Function to send list of available docking positions for End Effectors to the cobot controller
    def _send_docking_pos_to_cobot(self, docking_pos_in): 
        return 1
      
    # Function to send list of available End Effectors to the cobot controller
    def _send_ee_to_cobot(self, ee_in):  
        return 1   

    # Function to send execution commands to the cobot controller
    # Excution commands are given by sending the uid of the action
    # to be executed.
    def _send_execution_action_uid_to_cobot(self, uid_in):  
        return 1 

    # function to send a message to the cobot
    # the message will be received by the cobot controller
    # the message will be used to instantiate/popuate classes
    # in the cobot conroller
    def _send_msg_to_cobot(str_in):
        #TODO send to cobot
        return 1 

    # Function to populate feedback message based on information from the cobot controller
    def _populate_feedback_message_from_cobot_output(self, feedback_msg):
        
        feedback_msg.actions_out = self._create_actions_from_cobot_output()  
        feedback_msg.drill_tasks_out = self._create_drill_tasks_from_cobot_output()
        feedback_msg.tempfs_out = self._create_tempfs_from_cobot_output()
        feedback_msg.drill_tasks_out = self._create_fasteners_from_cobot_output()
        feedback_msg.ee_out = self._create_ee_from_cobot_output()
        
        feedback_msg.result_code = 1 # TODO determie sensible result_code 
        
        feedback_msg.message = ""  # TODO make message 

        return 1

    # Function to create list of defined actions based on information from the cobot controller
    def _create_actions_from_cobot_output(self): 
        lst = []  # [AssemblyAction]
        return lst

    # Function to create list of holes to be drilled based on information from the cobot controller
    def _create_drill_tasks_from_cobot_output(self): 
        lst = []  # [AssemblyDrill]
        return lst

    # Function to create list of available fasteners based on information from the cobot controller
    def _create_fasteners_from_cobot_output(self):  
        lst = []  # [AssemblyFast]
        return lst

    # Function to create list of available temporary fasteners based on information from the cobot controller
    def _create_tempfs_from_cobot_output(self): 
        lst = []  # [AssemblyTempFast]
        return lst
      
    # Function to create list of available End Effectors based on information from the cobot controller
    def _create_ee_from_cobot_output(self):  
        lst = []  # [AssemblyEe]
        return lst            


def main(args=None):
    # Initialize the ROS2 action server node
    rclpy.init(args=args)
    action_server = FokkerActionServer()

    # Spin so the server does not shutdown untill requested (e.g. Ctrl-C or another shutdown event)
    rclpy.spin(action_server)


if __name__ == "__main__":
    main()
