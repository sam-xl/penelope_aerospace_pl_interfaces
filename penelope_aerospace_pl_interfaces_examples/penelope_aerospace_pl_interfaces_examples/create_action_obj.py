import get_str_function  
import re  

from penelope_aerospace_pl_msgs.msg import AssemblyAction
from penelope_aerospace_pl_msgs.msg import AssemblyDrill
from penelope_aerospace_pl_msgs.msg import AssemblyEe
from penelope_aerospace_pl_msgs.msg import AssemblyFast
from penelope_aerospace_pl_msgs.msg import AssemblyMaterialLayer
from penelope_aerospace_pl_msgs.msg import AssemblyTempFast
from geometry_msgs.msg import Pose

# function to get a substring from an original_string
# will return the string after the first search_string 
# the substring is that part of the original_string after a search_string
def _find_substring(original_string, search_string):

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
def extract_leaf_content(input_string, open_tag, close_tag):
    pattern = re.compile(r'{}(.*?)(?={})'.format(open_tag, close_tag), re.DOTALL)
    match = pattern.search(input_string)

    if match:
        return match.group(1)
    else:
        return None


# Function to create list of defined actions based on information from the cobot controller
def _create_actions_from_cobot_output(c_str): 
    # get the string after the first ACTION_TAG
    a_str = _find_substring(c_str, get_str_function.ACTION_TAG)

    lst = []  # [AssemblyAction]

    while a_str is not None:
        lst.append(_get_action_from_str(a_str))
        
        # check if there is another string after an ACTION_TAG
        a_str = _find_substring(a_str, get_str_function.ACTION_TAG)

    return lst

# Function to create list of holes to be drilled based on information from the cobot controller
def _create_drill_tasks_from_cobot_output(c_str): 
    # get the string after the first DRILL_TASK_TAG
    d_str = _find_substring(c_str, get_str_function.DRILL_TASK_TAG)
    lst = []  # [AssemblyDrill]

    while d_str is not None:
        lst.append(_get_drill_task_from_str(d_str))
        
        # check if there is another string after an DRILL_TASK_TAG
        d_str = _find_substring(d_str, get_str_function.DRILL_TASK_TAG)

    return lst

# Function to create list of available fasteners based on information from the cobot controller
def _create_fasteners_from_cobot_output(c_str): 
    # get the string after the first FASTENER_TAG
    f_str = _find_substring(c_str, get_str_function.FASTENER_TAG)
    lst = []  # [AssemblyFast]

    while f_str is not None:
        lst.append(_get_fastener_from_str(f_str))
        
        # check if there is another string after an FASTENER_TAG
        f_str = _find_substring(f_str, get_str_function.FASTENER_TAG)

    return lst

# Function to create list of available temporary fasteners based on information from the cobot controller
def _create_tempfs_from_cobot_output(c_str): 
    # get the string after the first TEMPF_TAG
    t_str = _find_substring(c_str, get_str_function.TEMPF_TAG)
    lst = []  # [AssemblyTempFast]

    while t_str is not None:
        lst.append(_get_tempf_from_str(t_str))
        
        # check if there is another string after an TEMPF_TAG
        t_str = _find_substring(t_str, get_str_function.TEMPF_TAG)

    return lst
    
# Function to create list of available End Effectors based on information from the cobot controller
def _create_ee_from_cobot_output(c_str): 
    # get the string after the first END_EFFECTOR_TAG
    ee_str = _find_substring(c_str, get_str_function.END_EFFECTOR_TAG)
    lst = []  # [AssemblyEe]

    while ee_str is not None:
        lst.append(_get_ee_from_str(ee_str))
        
        # check if there is another string after an END_EFFECTOR_TAG
        ee_str = _find_substring(ee_str, get_str_function.END_EFFECTOR_TAG)

    return lst            

# Function to create AssemblyAction object from a string
def _get_action_from_str(c_str): 
    obj = AssemblyAction()

    # get the 'uid': 'string',
    obj.uid = extract_leaf_content(c_str, get_str_function.UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'obj_uid': 'string',
    obj.obj_uid = extract_leaf_content(c_str, get_str_function.OBJ_UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'loc_uid': 'string',
    obj.loc_uid = extract_leaf_content(c_str, get_str_function.LOC_UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'state': 'penelope_aerospace_pl_msgs/AssemblyActionState',
    obj.state = extract_leaf_content(c_str, get_str_function.ACTION_STATE_TAG, get_str_function.CLOSE_TAG)

    # get the 'passing': 'sequence<string>',
    # get the string after the first PASSING_UIDS_TAG
    p_str = _find_substring(c_str, get_str_function.PASSING_UIDS_TAG)
    lst = [] 

    while p_str is not None:
        lst.append(extract_leaf_content(p_str, get_str_function.PASSING_UID_TAG, get_str_function.CLOSE_TAG))
        
        # check if there is another string after an PASSING_UID_TAG
        p_str = _find_substring(p_str, get_str_function.PASSING_UID_TAG)

    obj.passing = lst

    # get the 'speed': 'uint8',
    obj.speed = int(extract_leaf_content(c_str, get_str_function.SPEED_TAG, get_str_function.CLOSE_TAG))

    return obj

# Function to create AssemblyDrill object from a string
def _get_drill_task_from_str(c_str): 
    obj = AssemblyDrill()

    # get the 'uid': 'string',
    obj.uid = extract_leaf_content(c_str, get_str_function.UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'loc_uid': 'string',
    obj.loc_uid = extract_leaf_content(c_str, get_str_function.LOC_UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'ee_uid': 'string',
    obj.ee_uid = extract_leaf_content(c_str, get_str_function.END_EFFECTOR_UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'diam': 'float',
    obj.diam = float()

    # get the 'jig_pos': 'geometry_msgs/Pose',
    obj.pose = _get_pose_from_str(_find_substring(c_str, get_str_function.POSE_TAG))

    # get the 'layers': 'sequence<penelope_aerospace_pl_msgs/AssemblyMaterialLayer>',
    # get the string after the first LAYERS_TAG
    l_str = _find_substring(c_str, get_str_function.LAYERS_TAG)
    lst = [] 

    while l_str is not None:
        lst.append(_get_material_layer_from_str(_find_substring(l_str, get_str_function.MAT_LAYER_TAG)))
        
        # check if there is another string after an MAT_LAYER_TAG
        l_str = _find_substring(l_str, get_str_function.MAT_LAYER_TAG)

    obj.layers = lst

    return obj

# Function to create Pose object from a string
def _get_pose_from_str(c_str):
    # Create a new Pose object
    pose = Pose()

    # Set the position (x, y, z)
    pose.position.x = float(extract_leaf_content(c_str, get_str_function.POSE_PX_TAG, get_str_function.CLOSE_TAG))
    pose.position.y = float(extract_leaf_content(c_str, get_str_function.POSE_PY_TAG, get_str_function.CLOSE_TAG))
    pose.position.z = float(extract_leaf_content(c_str, get_str_function.POSE_PZ_TAG, get_str_function.CLOSE_TAG))

    # Set the orientation (quaternion: x, y, z, w)
    pose.orientation.x = float(extract_leaf_content(c_str, get_str_function.POSE_OX_TAG, get_str_function.CLOSE_TAG))
    pose.orientation.y = float(extract_leaf_content(c_str, get_str_function.POSE_OY_TAG, get_str_function.CLOSE_TAG))
    pose.orientation.z = float(extract_leaf_content(c_str, get_str_function.POSE_OZ_TAG, get_str_function.CLOSE_TAG))
    pose.orientation.w = float(extract_leaf_content(c_str, get_str_function.POSE_OW_TAG, get_str_function.CLOSE_TAG))

    return pose

# Function to create AssemblyMaterialLayer object from a string
def _get_material_layer_from_str(c_str):
    obj = AssemblyMaterialLayer()

    # get the 'uid': 'string',
    obj.uid = extract_leaf_content(c_str, get_str_function.UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'thickness': 'float',
    obj.thickness = float(extract_leaf_content(c_str, get_str_function.LAYER_THICKNESS_TAG, get_str_function.CLOSE_TAG))

    # get the 'speed': 'float',
    obj.speed = float(extract_leaf_content(c_str, get_str_function.DRILL_SPEED_TAG, get_str_function.CLOSE_TAG))

    # get the 'feed': 'float',
    obj.feed = float(extract_leaf_content(c_str, get_str_function.DRILL_FEED_TAG, get_str_function.CLOSE_TAG))

    # get the 'lower_torque_limits': 'float',
    obj.lower_torque_limits = float(extract_leaf_content(c_str, get_str_function.LOWER_TORQUE_LIMIT_TAG, get_str_function.CLOSE_TAG))

    # get the 'upper_torque_limits': 'float',
    obj.upper_torque_limits = float(extract_leaf_content(c_str, get_str_function.UPPER_TORQUE_LIMIT_TAG, get_str_function.CLOSE_TAG))

    # get the 'threshold': 'float',
    obj.threshold = float(extract_leaf_content(c_str, get_str_function.TORQUE_THRESHOLD_TAG, get_str_function.CLOSE_TAG))

    # get the 'max_clamp_force': 'float',
    obj.max_clamp_force = float(extract_leaf_content(c_str, get_str_function.MAX_TEMPF_CLAMP_FORCE_TAG, get_str_function.CLOSE_TAG))

    return obj

# Function to create AssemblyFast object from a string
def _get_fastener_from_str(c_str): 
    obj = AssemblyFast()

    # get the 'uid': 'string',
    obj.uid = extract_leaf_content(c_str, get_str_function.UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'loc_uid': 'string',
    obj.loc_uid = extract_leaf_content(c_str, get_str_function.LOC_UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'ee_uid': 'string',
    obj.ee_uid = extract_leaf_content(c_str, get_str_function.END_EFFECTOR_UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'state': 'penelope_aerospace_pl_msgs/AssemblyFastState',
    obj.state = int(extract_leaf_content(c_str, get_str_function.FASTENER_STATE_TAG, get_str_function.CLOSE_TAG))

    # get the 'inst_pos': 'geometry_msgs/Pose',
    obj.inst_pos = _get_pose_from_str(_find_substring(c_str, get_str_function.POSE_TAG))

    # get the 'diam': 'float',DIAM_TAG
    obj.diam = float(extract_leaf_content(c_str, get_str_function.DIAM_TAG, get_str_function.CLOSE_TAG))

    # get the 'shaft_height': 'float',SHAFT_HEIGHT_TAG
    obj.shaft_height = float(extract_leaf_content(c_str, get_str_function.SHAFT_HEIGHT_TAG, get_str_function.CLOSE_TAG))

    # get the 'min_stack': 'float',MIN_STACK_TAG
    obj.min_stack = float(extract_leaf_content(c_str, get_str_function.MIN_STACK_TAG, get_str_function.CLOSE_TAG))

    # get the 'max_stack': 'float',MAX_STACK_TAG
    obj.max_stack = float(extract_leaf_content(c_str, get_str_function.MAX_STACK_TAG, get_str_function.CLOSE_TAG))

    # get the 'tcp_tip_distace': 'float',TCP_TIP_DIST_TAG
    obj.tcp_tip_distace = float(extract_leaf_content(c_str, get_str_function.TCP_TIP_DIST_TAG, get_str_function.CLOSE_TAG))

    # get the 'tcp_top_distace': 'float',TCP_TOP_DIST_TAG
    obj.tcp_top_distace = float(extract_leaf_content(c_str, get_str_function.TCP_TOP_DIST_TAG, get_str_function.CLOSE_TAG))

    return obj

# Function to create AssemblyTempFast object from a string
def _get_tempf_from_str(c_str): 
    obj = AssemblyTempFast()

    # get the 'uid': 'string',
    obj.uid = extract_leaf_content(c_str, get_str_function.UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'loc_uid': 'string',
    obj.loc_uid = extract_leaf_content(c_str, get_str_function.LOC_UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'ee_uid': 'string',
    obj.ee_uid = extract_leaf_content(c_str, get_str_function.END_EFFECTOR_UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'state': 'penelope_aerospace_pl_msgs/AssemblyFastState',
    obj.state = int(extract_leaf_content(c_str, get_str_function.FASTENER_STATE_TAG, get_str_function.CLOSE_TAG))

    # get the 'inst_pos': 'geometry_msgs/Pose',
    obj.inst_pos = _get_pose_from_str(_find_substring(c_str, get_str_function.POSE_TAG))

    # get the 'diam': 'float',DIAM_TAG
    obj.diam = float(extract_leaf_content(c_str, get_str_function.DIAM_TAG, get_str_function.CLOSE_TAG))

    # get the 'shaft_height': 'float',SHAFT_HEIGHT_TAG
    obj.shaft_height = float(extract_leaf_content(c_str, get_str_function.SHAFT_HEIGHT_TAG, get_str_function.CLOSE_TAG))

    # get the 'min_stack': 'float',MIN_STACK_TAG
    obj.min_stack = float(extract_leaf_content(c_str, get_str_function.MIN_STACK_TAG, get_str_function.CLOSE_TAG))

    # get the 'max_stack': 'float',MAX_STACK_TAG
    obj.max_stack = float(extract_leaf_content(c_str, get_str_function.MAX_STACK_TAG, get_str_function.CLOSE_TAG))

    # get the 'tcp_tip_distace': 'float',TCP_TIP_DIST_TAG
    obj.tcp_tip_distace = float(extract_leaf_content(c_str, get_str_function.TCP_TIP_DIST_TAG, get_str_function.CLOSE_TAG))

    # get the 'tcp_top_distace': 'float',TCP_TOP_DIST_TAG
    obj.tcp_top_distace = float(extract_leaf_content(c_str, get_str_function.TCP_TOP_DIST_TAG, get_str_function.CLOSE_TAG))

    return obj

# Function to create AssemblyEe object from a string
def _get_ee_from_str(c_str): 
    obj = AssemblyEe()

    # get the 'uid': 'string',
    obj.uid = extract_leaf_content(c_str, get_str_function.UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'loc_uid': 'string',
    obj.loc_uid = extract_leaf_content(c_str, get_str_function.LOC_UID_TAG, get_str_function.CLOSE_TAG)

    # get the 'poss_dock_pos_uids': 'sequence<string>',
    # get the string after the first DOCKING_POSS_TAG
    p_str = _find_substring(c_str, get_str_function.DOCKING_POSS_TAG)
    lst = [] 

    while p_str is not None:
        lst.append(extract_leaf_content(p_str, get_str_function.DOCKING_POS_TAG, get_str_function.CLOSE_TAG))
        
        # check if there is another string after an DOCKING_POS_TAG
        p_str = _find_substring(p_str, get_str_function.DOCKING_POS_TAG)

    obj.passing = lst

    # get the 'state': 'penelope_aerospace_pl_msgs/AssemblyEeState',END_EFFECTOR_STATE_TAG
    obj.state = int(extract_leaf_content(p_str, get_str_function.END_EFFECTOR_STATE_TAG, get_str_function.CLOSE_TAG))

    return obj