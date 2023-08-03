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
import math
from numpy import linalg as la

import vs

from state_machine_pkg import introspection
# import introspection
# import Kinematics as kin

# from fsm_package.msg import berry_pos
from pi_comm_pkg.msg import servo_camera
from pi_comm_pkg.msg import servo_arm
from pi_comm_pkg.msg import theta_4_arm
from pi_comm_pkg.msg import arm_state
from pi_comm_pkg.msg import extrusion
from pi_comm_pkg.msg import soft_arm
from task_space_pkg.msg import rigid_arm_position_desired
from vs import berry_vs, cal_error
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
# extrusion
global exd
exd = 0
global delta
delta = []



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
servo_arm_pub = rospy.Publisher('pi_comm/servo_arm', servo_arm, queue_size=10)
stepper_control_pub = rospy.Publisher('pi_comm/set_extrusion', extrusion, queue_size=10)
soft_arm_pub = rospy.Publisher('pi_comm/soft_arm', soft_arm, queue_size=10)


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
        self._tilt_step = -5 # Negative to move up
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



# define state SERVO_CONROLLER_ARM
class SERVO_CONTROLLER_ARM(smach.State):
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
        # move to somewhere in front of the target
        msg.rigid_arm_position_desired = [current_target[0]-0.1, current_target[1], current_target[2]]
        rigid_arm_position_desired_pub.publish(msg)
        rospy.sleep(2)
        global cur_arm_position
        cur_arm_position = current_target
        picked_list.append(current_target)

        return 'error_check_rigid_arm'

# check delta of image position
class CHECK_VISUAL_SERVO(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered', 'not_centered','not_detected'])


    def execute(self, userdata):
        global delta

        delta = berry_vs()

        if np.abs(delta[0]) < 30 and np.abs(delta[1]) < 30 and np.abs(delta[2]) < 1.5:
            transition_output = 'centered'
        else:
            transition_output = 'not_centered'
        if np.abs(delta[2]>45):
            transition_output = 'not_detected'
        return transition_output


# jacobian
class MOVE_GRIPPER(smach.State):
    def __init__(self):
        self._cur_rigid_arm_config = [math.pi/2, math.pi/2, math.pi/2]
        smach.State.__init__(self, outcomes=['check'])
        rospy.Subscriber("pi_comm/arm_state", arm_state, self._state_update_callback)
    def _state_update_callback(self, msg):
        # find the current angle of each joint
		self._cur_rigid_arm_config = [msg.joints_rad[0], msg.joints_rad[1], msg.joints_rad[2], msg.theta_4_rad]
    def execute(self, userdata):
        # global berry_detect_output
        global delta
        global exd
        global rigid_arm_position_desired_pub
        # current_angles
        current_deg = self._cur_rigid_arm_config
        # theta1, 2 ,3
        t1 = current_deg[0]
        t2 = current_deg[1]
        t3 = current_deg[2]
        # Convert image pixel error to base coordinate error
        errorx = (delta[2]*(delta[1]-4.896)/262.2)/100
        errory = (delta[2]*(delta[0]+13.73)/-127)/100
        print('error', errorx, errory)
        if delta[2]>1.5:
            # set error z relative to the distance
            errorz = 0.01+0.0005*delta[2]
        else:
            errorz = 0
        # new position
        xyz_new = cal_error(t1,t2,t3, errorx, errory, errorz)
        # task space
        msg = rigid_arm_position_desired()
        msg.rigid_arm_position_desired = xyz_new
        rigid_arm_position_desired_pub.publish(msg)
        return 'check'


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
        # msg.rigid_arm_position_desired = [cur_arm_position[0] - 0.10, cur_arm_position[1], cur_arm_position[2]]
        #
        # rigid_arm_position_desired_pub.publish(msg)


        rospy.sleep(2)
        # go back to start position
        msg.rigid_arm_position_desired = [0.3, 0.0, 0.6]
        rigid_arm_position_desired_pub.publish(msg)
        rospy.sleep(5)

        print("Counter: {}".format(counter))
        transition_output = 'go_back_scan'
        if len(berry_detect_list) == 0:
            transition_output = 'go_back_scan'
        elif counter < 6:
            transition_output = 'go_back'
        else:
            transition_output = 'stop'

        return transition_output

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
    msg = soft_arm()
    msg.bend_press = 0
    msg.rot_1_press = 0
    msg.rot_2_press = 0
    soft_arm_pub.publish(msg)
    msg = extrusion()
    msg.length_cm = 0
    stepper_control_pub.publish(msg)
    global cur_arm_position
    _position = [0.3, 0.0, 0.6]
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
            smach.StateMachine.add('DETECTION_ALGORITHM',DETECTION_ALGORITHM(), {'detected':'STOP1', 'not_detected':'PAN_TILT'})

        smach.StateMachine.add('BERRY_DETECT_SM',  sm_berry_detection, {'STOP1':'REACH_BERRY_SM'})

        sm_reach_berry = smach.StateMachine(outcomes=['STOP2', 'go_back_scan'])

        with sm_reach_berry:

            # Create and add the SERVO_CONROLLER_ARM SMACH state
            smach.StateMachine.add('SERVO_CONTROLLER_ARM', SERVO_CONTROLLER_ARM(), {'error_check_rigid_arm':'CHECK_VISUAL_SERVO'})

            # check visual servo state
            smach.StateMachine.add('CHECK_VISUAL_SERVO', CHECK_VISUAL_SERVO(), {'centered':'REACH_BERRY', 'not_centered':'MOVE_GRIPPER', 'not_detected':'go_back_scan'})
            # move to the desired position
            smach.StateMachine.add('MOVE_GRIPPER', MOVE_GRIPPER(), {'check' :'CHECK_VISUAL_SERVO'})
            # once reached, go back
            smach.StateMachine.add('REACH_BERRY', REACH_BERRY(), {'stop':'STOP2', 'go_back_scan':'go_back_scan', 'go_back':'SERVO_CONTROLLER_ARM'})

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
