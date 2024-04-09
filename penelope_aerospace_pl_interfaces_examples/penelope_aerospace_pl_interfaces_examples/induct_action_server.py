import time

import rclpy
#from penelope_aerospace_pl_msgs.action import InfraredThermographyInspect
#from penelope_aerospace_pl_msgs.msg import InfraredThermographyState, ResultCodes
from penelope_aerospace_pl_msgs.action import InductionWeld
from penelope_aerospace_pl_msgs import InductionWeldingState, ResultCodes
from orchestrator_mainv0 import connect_and_process
from rclpy.action import ActionServer
from rclpy.node import Node
from sensor_msgs.msg import Image


class ExampleActionServer(Node):
    def __init__(self):
        super().__init__("example_action_server")
        self._action_server = ActionServer(self, InductionWeld, "example_action", self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info("Executing example goal...")

        ## Read goal ##
        #pose_array = goal_handle.request.measurement_poses
        #filename = goal_handle.request.config_file
        stringer_id = goal_handle.request.stringer_id
        weld_trajectory = goal_handle.request.weld_trajectory
        action_required = goal_handle.request.action_required

        ### add here script to connect to the plc and the correct exit clauses
        ### rewrite PLC connection to get the correct data from the correct places
        #connect_and_process()

        ###explanation of the different states
        """   
        ### state safeoperatormode
        # state specifially to note that the operator can safely work on the tooling
        # means that the gantry, induction heating equipment and the pneumatic systems may not do anything. This excluded the pneumatic holding of the tool open
        ## conditions for safeoperatormode
        # Gantry may not move
        # Tool is up and in locked up-position 
        # Generator heating is off


        ### state readytoweld
        # everything is ready to weld 
        ## conditions for readytoweld
        # tool is down and locked down-position
        # parts are loaded
        # TCs are in correct range --> all at least below 35C


        ### state preweld
        # The "weld" part of the program has been started
        # This phase is used both to pickup the end-effector aswell as move to the start of the weld
        ### conditions for preweld
        # Start signal has been given
        # tool is down and locked down-position
        # parts are loaded
        # TCs are in correct range --> all at least below 35C  
        # Detection if end-effector loading has been finished     


        ### state welding
        # This is the actual welding where the generator is on and coil is in the weld tool
        ## conditions for welding 
        # End-effector loading has been finished
        # Start signal has been given
        # tool is down and locked down-position
        # parts are loaded
        # TCs are in correct range --> all at least below 35C


        ### state postweld
        # Coil moving out of tool and moves to decouple position
        # Pressure stays on weld
        ## conditions for postweld
        # Welding finished
        # Decoupling not yet performed


        ### state weldaborted
        # weld aborted, either by E-stop or by command from PL
        ## conditions for weldaborted
        # e-stop pressed while either in pre-weld or welding state
        # command from PL to stop welding either pre-weld or welding state


        ### state pressurecool
        # The module will keep the pressure on while the parts are cooling down to below 90C
        ## conditions for pressure cool
        # TC temperatures are above 90C
        # Postweld state has been finished


        ### state waiting
        # tool + machine are in an idle state, next steps could be "ready to weld" or "safe operator mode"
        # state is used as a default unless conditions for other states are met
        ## conditions for waiting:
        # state does not have specific conditions to be met to be able to enter
        """

        if E_stop_pressed or action_required == 'abort_weld': #weldaborted
            state = 4
        elif action_required == 'waiting_for_command' & GeneratorHeating == 0 & gantrymoving == 0 & Tool_up_locked: #safeoperatormode
            state = 5
        elif action_required == 'waiting_for_command' & parts_are_loaded & tool_down_locked & TCs_below_35C: #readytoweld
            state = 6
        elif action_required == 'end_effector_couple' & tool_down_locked & parts_are_loaded & end_effector_loading_phinished == 0 & TCs_below_35C: #preweld
            state = 1
        elif end_effector_loading_phinished & TCs_below_35C & action_required == 'welding' & tool_down_locked & parts_are_loaded & welding_finished == 0: #weld
            state = 2
        elif welding_finished & tool_down_locked & end_effector_connected: #postweld (action required determines when decouple movement started)
            state = 7
        elif action_required == 'waiting_for_command' & welding_finished & tool_down_locked & end_effector_connected == 0 & TC_above_90: #pressurecool
            state = 3
        else:
            state = 8 #waiting

        ## Provide regular feedback ##

        while state == 1: #preweld
            # move to pickup position
            # confirm when part has been picked up
            # move to welding-ready position
            # set 'parts_are_loaded'
        
        while state == 2: #welding
            # get required data which stringer need to be welded and which trajectory needs to be taken
            # communicate to PLC welding started
            # perform welding routing
            # get current data from weld and give as feedback(tc, pressure, gen data)
            # save datalog file in appropriate place
        
        while state == 3: #pressurecool
            # pressure remains on
            # tool must remain locked
            # gantry "released" from induct duty
        
        while state == 4: #weldaborted
            # tool must remain locked 
            # stop heating
            # gantry stop moving
        
        while state == 5: #safeoperatormode
            # tool must not close unless e-stop acknowledged, key release and button press all within 30 seconds
            # heating off
            # gantry may not move (also not for other processes)
        
        while state == 6: #readytoweld
            # tool must reamin closed & locked
            # read out TC values --> to ensure
            # welding not pressurised
            # gantry may perform other tasks
        
        while state == 7: #postweld
            # wait for signal to hang up end-effector
            # move to end-effector hang up point when signal is given
            # wait until end-effector is disconnected until gantry can be moved to home position
        
        while state == 8: #waiting
            # gantry can do any movement
            # heating is off
            # weld pressure is off




        

  



        while True:
        #for i in range(0, 100):
            #self.get_logger().info(f"induct itteration: {i}")

            # ...

            # Create a feedback message
            feedback_msg = InductionWeld.Feedback()

            # Fill in the feedback message
            #feedback_msg.measurement_status = list(range(0 + i, 5 + i))
            feedback_msg.module_state = state

            # Set process state and module state
            #   note: 2 ways to set the "child" message are shown, both can be used
            process_state = InductionWeldingState()
            process_state.data = 1
            feedback_msg.process_state = process_state

            feedback_msg.module_state.data = 2
            self.get_logger().InductionWeld.info(feedback_msg)


            # Send feedback
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.1)  # Just to slow the feedback down
        
        


        # Indicate the action succeeded (this does not indicate succes!)
        goal_handle.succeed()



        ## Provide result ##
        result = InductionWeld.Result()
        images = [Image() for _ in range(0, 5)]  # Images are empty for the example

        result.images = images
        result.measurement_status = list(range(0, 5))

        # Set result code and message ---> question for Dave/Eugenio. 
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
