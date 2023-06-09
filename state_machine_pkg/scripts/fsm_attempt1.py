#!/usr/bin/env python
#General Imports
import roslib; #roslib.load_manifest('smach_tutorials')
#roslib.load_manifest('fsm')
import rospy
import time
import random
#Importing SMACH package 
import smach
import roslib; roslib.load_manifest('smach_ros')
import rospy

import threading
import smach
from smach import CBState
import numpy as np
from numpy import linalg as la

from state_machine_pkg import introspection
# import introspection
# import Kinematics as kin

# from fsm_package.msg import berry_pos
from pi_comm_pkg.msg import servo_camera
from pi_comm_pkg.msg import theta_4_arm
from task_space_pkg.msg import rigid_arm_position_desired
from std_msgs.msg import Bool
from state_machine_pkg.srv import berry_detect

__all__ = ['set_preempt_handler']

# kinematics = kin.Kinematics() # Create kinematics class

global berry_detect_output
berry_detect_output = None
global berry_detect_world
berry_detect_world = None

global pandt
pandt = None


global counter
counter = 0

global berry_detect_list
berry_detect_list = []

global picked_list
picked_list = [[0.0,0.0,0.0]]
global cur_arm_position
cur_arm_position = None
# def berry_detect_callback(msg):
#     print("HERE,HERE,HERE,HERE")
#     #print(msg.points[0].x)
#     global berry_detect_output
#     berry_detect_output = [msg.points[0].x, msg.points[0].y, msg.points[0].z]
#     # print(berry_detect_output)
#     # print(msg.points[0])

# def berry_detect_callback(msg):


"""
Subscribers
"""
# berry_detect_sub = rospy.Subscriber('/berry_detector', berry_pos, berry_detect_callback)

"""
Publishers
"""
servo_camera_pub = rospy.Publisher('pi_comm/servo_camera', servo_camera, queue_size=10)
berry_detect_pub = rospy.Publisher('/berry_detector_call', Bool, queue_size=10)
rigid_arm_position_desired_pub = rospy.Publisher('task_space/rigid_arm_position_desired', rigid_arm_position_desired, queue_size=10)
servo_theta_4_pub = rospy.Publisher('pi_comm/theta_4_arm', theta_4_arm, queue_size=10)


# Signal handler
def set_preempt_handler(sc):
    """Sets a ROS pre-shutdown handler to preempt a given SMACH container when
    ROS receives a shutdown request.
    
    This can be attached to multiple containers, but only needs to be used on
    the top-level containers.

    @type sc: L{smach.Container}
    @param sc: Container to preempt on ROS shutdown.
    """
    ### Define handler
    def handler(sc):
        sc.request_preempt()

        while sc.is_running():
            rospy.loginfo("Received shutdown request... sent preempt... waiting for state machine to terminate.")
            rospy.sleep(1.0)

    ### Add handler
    rospy.core.add_client_shutdown_hook(lambda: handler(sc))

#For dealing with msgs
from std_msgs.msg import String, Float64, Bool
from geometry_msgs.msg import Pose, Point, Quaternion

#from fsm_states import *


# define state START
class START(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['choose_berry'])


    def execute(self, userdata):
##        rospy.loginfo('Executing state START')

        return 'choose_berry'

# define state PAN_TILT
class PAN_TILT(smach.State):
    def __init__(self, iteration):
        smach.State.__init__(self, outcomes=['send_to_servo_controller'])
        # #self.iteration = iteration + 1
        self._pan_limit_low = -20
        self._pan_limit_high = 10
        self._tilt_limit_low = 0 # Low potition, negative value angles up
        self._tilt_limit_high = -50 # High position, negative value angles up
        self._pan_step = 10
        self._tilt_step = -10 # Negative to move up
        # Current Values
        self.pan_value_deg = 0
        self.tilt_value_deg = 0
        self.direction = 1 # Pan direction
        self.end_flag = False
        # #print("Iteration: {}".format(self.iteration))


    def execute(self, userdata):
        rospy.loginfo('Executing state PAN_TILT')
        global servo_camera_pub  
        global pandt
            

        pandt = [self.pan_value_deg, self.tilt_value_deg]
        print("Pan and Tilt Values: {}".format(pandt))
        msg = servo_camera()
        msg.camera_pan_rad =  np.deg2rad(self.pan_value_deg)
        msg.camera_tilt_rad = np.deg2rad(self.tilt_value_deg)
        # msg.camera_pan_rad =  np.deg2rad(0) #np.deg2rad(self.pan_value_deg)
        # msg.camera_tilt_rad = np.deg2rad(20)#np.deg2rad(self.tilt_value_deg)
        servo_camera_pub.publish(msg)
        # #if self.iteration <= 1 :
        # #pan_value  = -80
        # #tilt_value = 0
        # #else:
        	# #pan_value  = (self.iteration - 1.0)*10
        	# #tilt_value = -(self.iteration - 1.0)*10
        	# #if self.pan_value < 90:
				# #self.pan_value += self.direction*10
        if self.pan_value_deg >= self._pan_limit_high and self.end_flag==False:
            self.pan_value_deg = self._pan_limit_high
            self.direction = -1 # Pan direction
            self.tilt_value_deg += self._tilt_step
            self.end_flag = True
        elif self.pan_value_deg <= self._pan_limit_low and self.end_flag==False:
            self.pan_value_deg = self._pan_limit_low
            self.direction = 1 # Pan direction
            self.tilt_value_deg += self._tilt_step
            self.end_flag = True
        else:
            self.pan_value_deg += self.direction*self._pan_step
            self.end_flag = False
			
        if self.tilt_value_deg > self._tilt_limit_low: # It is pointed too low, positive values aim downward
            self.tilt_value_deg = self._tilt_limit_low
        if self.tilt_value_deg < self._tilt_limit_high: # It is pointed too high, negative values aim upward
            self.tilt_value_deg = self._tilt_limit_high


				 
        # #self.iteration = self.iteration + 1
        # pandt = [self.pan_value_deg, self.tilt_value_deg]
        # msg = servo_camera()
        # msg.camera_pan_rad =  np.deg2rad(self.pan_value_deg)
        # msg.camera_tilt_rad = np.deg2rad(self.tilt_value_deg)
        # # msg.camera_pan_rad =  np.deg2rad(0) #np.deg2rad(self.pan_value_deg)
        # # msg.camera_tilt_rad = np.deg2rad(20)#np.deg2rad(self.tilt_value_deg)
        # servo_camera_pub.publish(msg)
        # kinematics.cur_pan_value_deg = self.pan_value_deg
        # kinematics.cur_tilt_value_deg = self.tilt_value_deg
        return 'send_to_servo_controller'


# define state DETECTION_ALGORITHM
class DETECTION_ALGORITHM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'])
    
        self.received = False

    def execute(self, userdata):
        # global berry_detect_output
        global pandt
        global berry_detect_list


        rospy.loginfo('Executing state DETECTION_ALGORITHM')
        rospy.wait_for_service('state_machine_pkg/berry_detect')
        berry_detect_srv = rospy.ServiceProxy('state_machine_pkg/berry_detect', berry_detect)
        point_response = berry_detect_srv([pandt[0],pandt[1]])
        # print(point_response)
        # print("Point Response {}".format(point_response.points[0]))

        if len(point_response.points) == 0: #[point_response.points[0].x, point_response.points[0].y, point_response.points[0].z] == [0, 0, 0]:
            transition_output = 'not_detected'
            rospy.loginfo('DETECTION_ALGORITHM not detected')
            berry_detect_list = []
            
        else:
            transition_output = 'detected'
            rospy.loginfo('DETECTION_ALGORITHM detected')
            # berry_detect_output = [point_response.points[0].x, point_response.points[0].y, point_response.points[0].z]
            for i in point_response.points:
                # print(i)
                new_point = [i.x, i.y, i.z]

                for n in picked_list:
                    print("B")
                    print(la.norm(np.asarray(new_point)-np.asarray(n)))
                    if la.norm(np.asarray(new_point)-np.asarray(n))>0.1:


                        berry_detect_list.append(new_point)


            if len(berry_detect_list) == 0:
                transition_output = 'not_detected'
                return transition_output
            # berry_detect_list_temp = copy.copy(berry_detect_list)
            print("C")
            print(berry_detect_list)
            del_list = []
            for i in range(len(berry_detect_list)):
                for j in range(len(berry_detect_list)):
                    if j>i:
                        if la.norm(np.asarray(berry_detect_list[i])-np.asarray(berry_detect_list[j]))<0.1:
                            del_list.append(j)

            clear_del_list = list(set(del_list))
            clear_del_list.sort(reverse=True)
            print("del_list: {}".format(clear_del_list))
            for item in clear_del_list:
                berry_detect_list.pop(item)

            print("A")
            print(berry_detect_list)

        return transition_output




        # # berry_detect_pub.publish(1) # Call for an image
        # start = time.time()
        # transition_output = 'detected'
        # while berry_detect_output == None:
        #     # rospy.loginfo('Waiting for detection')
        #     # #print(start - time.time())
        #     if time.time() - start > 10:
        #         print("Moving")
        #         transition_output = 'not_detected'
        #         rospy.loginfo('DETECTION_ALGORITHM not detected')
        #         break
        # if  transition_output == 'detected':
        #     rospy.loginfo('DETECTION_ALGORITHM detected')
        # # berry_detect_output = None
        # return transition_output

# define state TRANSFORM_COORDINATES
class TRANSFORM_COORDINATES(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_to_next_state'])
        

    def execute(self, userdata):
        # global berry_detect_output
        # global berry_detect_world
        # global pandt
        global berry_detect_list
        ##        rospy.loginfo('Executing state TRANSFORM_COORDINATES')
        # berry_detect_world = kinematics.get_camera2world(berry_detect_output[0], berry_detect_output[1], berry_detect_output[2])
        # berry_detect_world = kinematics._camera2arm_base(berry_detect_output, pandt[0],pandt[1])
        # berry_detect_world = berry_detect_output # kinematics._camera2arm_base(berry_detect_output, 0, 20)
        """
        This is just passing the data through for now
        """
        # print("Pan and  {}".format(pandt))
        # print("Berry Detect List: ".format(berry_detect_list[0]))

        return 'go_to_next_state'


#-----------------------------------------------------------------------------------------------------------

# define state REACHABLE_WITH_RIGID_OR_SOFT_ARM
class REACHABLE_WITH_RIGID_OR_SOFT_ARM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['yes', 'no'])


    def execute(self, userdata):
##      rospy.loginfo('Executing state REACHABLE_WITH_RIGID_OR_SOFT_ARM')

        random_check = 0 #random.choice([0, 1])
        if random_check == 0 :
            transition_output = 'yes'
        else:
            transition_output = 'no'

        return transition_output

# define state OBSTACLES
class OBSTACLES(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['yes', 'no'])


    def execute(self, userdata):
##      rospy.loginfo('Executing state OBSTACLES')

        random_check = 1 #random.choice([0, 1])
        if random_check == 0 :
            transition_output = 'yes'
        else:
            transition_output = 'no'

        return transition_output

# define state REACHABLE_WITH_RIGID_ARM
class REACHABLE_WITH_RIGID_ARM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['yes', 'no'])


    def execute(self, userdata):
##      rospy.loginfo('Executing state REACHABLE_WITH_RIGID_ARM')

        random_check = 0 #random.choice([0, 1])
        if random_check == 0 :
            transition_output = 'yes'
        else:
            transition_output = 'no'

        return transition_output

# define state MOTION_PLANNING_ALGO
class MOTION_PLANNING_ALGO(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['send_to_servo_Controller'])


    def execute(self, userdata):
##      rospy.loginfo('Executing state MOTION_PLANNING_ALGO')

        return 'send_to_servo_Controller'

# define state SERVO_CONROLLER_ARM
class SERVO_CONROLLER_ARM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_check_rigid_arm'])
        global rigid_arm_position_desired_pub
        # global berry_detect_world
        global berry_detect_list
        global picked_list


    def execute(self, userdata):
##      rospy.loginfo('Executing state SERVO_CONROLLER_ARM')
        current_target = berry_detect_list.pop(0)
        print("Current Target: {}".format(current_target))
        msg = rigid_arm_position_desired()
        # msg.rigid_arm_position_desired = current_target
        msg.rigid_arm_position_desired = [current_target[0] - 0.07, current_target[1], current_target[2]]
        rigid_arm_position_desired_pub.publish(msg)
        rospy.sleep(2)
        msg.rigid_arm_position_desired = current_target
        rigid_arm_position_desired_pub.publish(msg)
        global cur_arm_position
        cur_arm_position = current_target
        picked_list.append(current_target)

        return 'error_check_rigid_arm'

# define state ERROR_CHECK_RIGID_ARM
class ERROR_CHECK_RIGID_ARM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['yes', 'no', 'reach'])


    def execute(self, userdata):
##      rospy.loginfo('Executing state ERROR_CHECK_RIGID_ARM')
        # global counter
        # counter += 1
        # random_check = 0 #random.choice([0, 1])
        # if random_check == 0 :
        #     transition_output = 'yes'
        # else:
        #     transition_output = 'no'


        transition_output = 'reach'

        return transition_output

# define state MOTION_PLANNING_PERIPHERY
class MOTION_PLANNING_PERIPHERY(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['send_to_servo_Controller'])


    def execute(self, userdata):
##      rospy.loginfo('Executing state MOTION_PLANNING_PERIPHERY')

        return 'send_to_servo_Controller'

# define state SERVO_CONROLLER_ARM_2
class SERVO_CONROLLER_ARM_2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['reachable_using_soft_arm'])


    def execute(self, userdata):
##      rospy.loginfo('Executing state SERVO_CONROLLER_ARM_2')

        return 'reachable_using_soft_arm'


# define state REACHABLE_WITH_SOFT_ARM
class REACHABLE_WITH_SOFT_ARM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['yes', 'no'])


    def execute(self, userdata):
##      rospy.loginfo('Executing state REACHABLE_WITH_SOFT_ARM')

        random_check = 0 #random.choice([0, 1])
        if random_check == 0 :
            transition_output = 'yes'
        else:
            transition_output = 'no'

        return transition_output

# define state SOFT_ARM_CONTROL
class SOFT_ARM_CONTROL(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['error_check_soft_arm'])


    def execute(self, userdata):
##      rospy.loginfo('Executing state SOFT_ARM_CONTROL')

        return 'error_check_soft_arm'

# define state ERROR_CHECK_SOFT_ARM
class ERROR_CHECK_SOFT_ARM(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['yes', 'no'])


    def execute(self, userdata):
##      rospy.loginfo('Executing state ERROR_CHECK_SOFT_ARM')

        random_check = 1 #random.choice([0, 1])
        if random_check == 0 :
            transition_output = 'yes'
        else:
            transition_output = 'no'

        return transition_output


# # define state ERROR_CHECK_SOFT_ARM
class REACH_BERRY(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop', 'go_back', 'go_back_scan'])


    def execute(self, userdata):
##        rospy.loginfo('Executing state ERROR_CHECK_SOFT_ARM')
        global counter
        global berry_detect_list
        counter += 1
        rospy.sleep(5)
        global cur_arm_position
        global rigid_arm_position_desired_pub

        msg = rigid_arm_position_desired()
        msg.rigid_arm_position_desired = [cur_arm_position[0] - 0.07, cur_arm_position[1], cur_arm_position[2]]
        
        rigid_arm_position_desired_pub.publish(msg)



        rospy.sleep(2)
        msg.rigid_arm_position_desired = [0.3, 0.0, 0.6]
        
        rigid_arm_position_desired_pub.publish(msg)
        rospy.sleep(5)

        print("Counter: {}".format(counter))
        # if counter < 3:
        #     transition_output = 'go_back'
        # else:
        #     transition_output = 'stop'


        if len(berry_detect_list) == 0:
            transition_output = 'go_back_scan'
        elif counter < 6:
            transition_output = 'go_back'
        else:
            transition_output = 'stop'
             # random_check = 1 #random.choice([0, 1])
             # if random_check == 0 :
             #     transition_output = 'stop'
             # else:
             #     transition_output = 'go_back'

        return transition_output

#class iteration_save( ):
#    def __init__(self, iteration):
 #       self.iteration = iteration

def main():
    rospy.init_node('TOP_FSM')
    while rospy.Time.now().to_sec()==0:
        rospy.sleep(0.1)
    rospy.sleep(1)
    print("MOVING ARM") 
    msg = rigid_arm_position_desired()
    msg.rigid_arm_position_desired = [0.3, 0.0, 0.6]
    global rigid_arm_position_desired_pub
    rigid_arm_position_desired_pub.publish(msg)
    global cur_arm_position
    cur_arm_position = [0.3, 0.0, 0.6]
    global servo_theta_4_pub
    msg = theta_4_arm()
    msg.theta_4_rad = np.deg2rad(0)
    servo_theta_4_pub.publish(msg)
    rospy.sleep(5)
    # Create a SMACH state machine
    # The FSM has one outcome of "Done" when mission exceedes alloted time or otherwise defined in the transitions

   
    #Safety Parameters (there is no const in python, careful when accesing these from within the sub_fsms)
    # stable              = True
   

    #Define Static Parameters 
    tolerance = 1                    #[cm] proximity measure for arm reaching berry
    
    iter_save = 0
    sm_top  = smach.StateMachine(outcomes=['STOP'])
    # Open the container
    with sm_top:
        # Add states to the container
        smach.StateMachine.add('START', START(), transitions={'choose_berry':'BERRY_DETECT_SM'})


#----------------------------------------------------------------------------------------
    # Create the BERRY DETECTION state machine
        sm_berry_detection = smach.StateMachine(outcomes=['STOP1'])
        #Add the BERRY DETECTION sub state machine                                                            
        
        
        with sm_berry_detection:

            # Create and add the PAN_TILT SMACH state
            smach.StateMachine.add('PAN_TILT', PAN_TILT(iter_save), {'send_to_servo_controller':'DETECTION_ALGORITHM'})

	        # Create and add the DETECTION_ALGORITHM SMACH state
            smach.StateMachine.add('DETECTION_ALGORITHM',DETECTION_ALGORITHM(), {'detected':'TRANSFORM_COORDINATES', 'not_detected':'PAN_TILT'})
            # Create and add the TRANSFORM_COORDINATES SMACH state
            smach.StateMachine.add('TRANSFORM_COORDINATES', TRANSFORM_COORDINATES(), {'go_to_next_state':'STOP1'})

        smach.StateMachine.add('BERRY_DETECT_SM',  sm_berry_detection, {'STOP1':'REACH_BERRY_SM'})

        sm_reach_berry = smach.StateMachine(outcomes=['STOP2', 'go_back_scan'])

        with sm_reach_berry:
            # Create and add the REACHABLE_WITH_RIGID_OR_SOFT_ARM SMACH state
            smach.StateMachine.add('REACHABLE_WITH_RIGID_OR_SOFT_ARM', REACHABLE_WITH_RIGID_OR_SOFT_ARM(), {'yes':'OBSTACLES', 'no':'go_back_scan'})

            # Create and add the OBSTACLES SMACH state
            smach.StateMachine.add('OBSTACLES', OBSTACLES(), {'yes':'MOTION_PLANNING_PERIPHERY', 'no':'REACHABLE_WITH_RIGID_ARM'})

            # Create and add the REACHABLE_WITH_RIGID_ARM SMACH state
            smach.StateMachine.add('REACHABLE_WITH_RIGID_ARM', REACHABLE_WITH_RIGID_ARM(), {'yes':'MOTION_PLANNING_ALGO', 'no':'MOTION_PLANNING_PERIPHERY'})

            # Create and add the MOTION_PLANNING_ALGO SMACH state
            smach.StateMachine.add('MOTION_PLANNING_ALGO', MOTION_PLANNING_ALGO(), {'send_to_servo_Controller':'SERVO_CONROLLER_ARM'})

            # Create and add the SERVO_CONROLLER_ARM SMACH state
            smach.StateMachine.add('SERVO_CONROLLER_ARM', SERVO_CONROLLER_ARM(), {'error_check_rigid_arm':'ERROR_CHECK_RIGID_ARM'})

            # Create and add the ERROR_CHECK_RIGID_ARM SMACH state
            smach.StateMachine.add('ERROR_CHECK_RIGID_ARM', ERROR_CHECK_RIGID_ARM(), {'yes':'STOP2', 'no':'MOTION_PLANNING_ALGO', 'reach':'REACH_BERRY'})

            # Create and add the MOTION_PLANNING_PERIPHERY SMACH state
            smach.StateMachine.add('MOTION_PLANNING_PERIPHERY', MOTION_PLANNING_PERIPHERY(), {'send_to_servo_Controller':'SERVO_CONROLLER_ARM_2'})

            # Create and add the SERVO_CONROLLER_ARM_2 SMACH state
            smach.StateMachine.add('SERVO_CONROLLER_ARM_2', SERVO_CONROLLER_ARM_2(), {'reachable_using_soft_arm':'REACHABLE_WITH_SOFT_ARM'})

            # Create and add the REACHABLE_WITH_SOFT_ARM SMACH state
            smach.StateMachine.add('REACHABLE_WITH_SOFT_ARM', REACHABLE_WITH_SOFT_ARM(), {'yes':'SOFT_ARM_CONTROL', 'no':'MOTION_PLANNING_PERIPHERY'})

            # Create and add the SOFT_ARM_CONTROL SMACH state
            smach.StateMachine.add('SOFT_ARM_CONTROL', SOFT_ARM_CONTROL(), {'error_check_soft_arm':'ERROR_CHECK_SOFT_ARM'}) 

             # Create and add the ERROR_CHECK_SOFT_ARM SMACH state
            smach.StateMachine.add('ERROR_CHECK_SOFT_ARM', ERROR_CHECK_SOFT_ARM(), {'yes':'STOP2', 'no':'SOFT_ARM_CONTROL'})

            smach.StateMachine.add('REACH_BERRY', REACH_BERRY(), {'stop':'STOP2', 'go_back_scan':'go_back_scan', 'go_back':'SERVO_CONROLLER_ARM'})

        smach.StateMachine.add('REACH_BERRY_SM',  sm_reach_berry, {'STOP2':'STOP', 'go_back_scan':'BERRY_DETECT_SM'})




#----------------------------------------------------------------------------------------   

                              

    # Create and start the introspection server for visualization of the FSM
    sis = introspection.IntrospectionServer('FSM', sm_top, '/SM_TOP')
    sis.start()
    #Starting Process
    #Wait till some data is accumulated ()
    rospy.sleep(1.)
    # Execute SMACH plan
    outcome = sm_top.execute()
    # Wait for ctrl-c to stop the application
    rospy.spin()
    sis.stop()

if __name__ == '__main__':
    main()

