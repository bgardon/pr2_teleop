#! /usr/bin/env python

import rospy 
import roslib 
import math

roslib.load_manifest('actionlib_msgs')
roslib.load_manifest('teleop_web_app')
roslib.load_manifest('pr2_controllers_msgs')

from actionlib import *;
from actionlib_msgs.msg import *
from trajectory_msgs.msg import *
from pr2_controllers_msgs.msg import *
from teleop_web_app.msg import *

# Joint names
joint_names = ["shoulder_pan",
               "shoulder_lift",
               "upper_arm_roll",
               "elbow_flex",
               "forearm_roll",
               "wrist_flex",
               "wrist_roll" ]

# Trajectories from pr2_tuck_arms_action
l_arm_tucked   = [0.06024, 1.248526, 1.789070, -1.683386, -1.7343417, -0.0962141, -0.0864407]
r_arm_tucked   = [-0.023593, 1.1072800, -1.5566882, -2.124408, -1.4175, -1.8417, 0.21436]
l_arm_untucked = [ 0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]
r_arm_untucked = [-0.4,  1.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]
r_arm_approach = [0.039, 1.1072, 0.0, -2.067, -1.231, -1.998, 0.369]
r_arm_up_traj  = [[-0.4,  0.0,   0.0,  -2.05,  0.0,  -0.1,  0.0]]

# Trajectories from point_arms_down.py
r_arm_pointed = [-1.0, 1.3, -0.75, -1.0, -3, -0.1, 0]
l_arm_pointed = [ 1.0, 1.3, 0.75, -1.0, 3, -0.1, 0]
r_arm_pointed_tight = [-1.0, 1.3, -1.25, -1.0, -3, -0.1, 0]
l_arm_pointed_tight = [ 1.0, 1.3, 1.25, -1.0, 3, -0.1, 0]

class GetArmStateActionServer:
    # arm state: -1 unknown, 0 tucked, 1 untucked, 2 pointed, 3 pointed tight
    l_arm_state = -1
    r_arm_state = -1

    r_received = False
    l_received = False

    states = ["tucked", "untucked", "pointed", "pointed-tight", "unknown"]

    def __init__(self, node_name):
        self.node_name = node_name

         # Connect to controller state
        rospy.Subscriber('l_arm_controller/state', JointTrajectoryControllerState ,self.stateCb)
        rospy.Subscriber('r_arm_controller/state', JointTrajectoryControllerState ,self.stateCb)

        self.action_server = SimpleActionServer(node_name, GetArmStateAction, self.executeCB)

    def executeCB(self, goal):
        # Make sure we received arm state
        while not self.r_received or not self.l_received:
            rospy.sleep(0.1)
            if rospy.is_shutdown():
                return

        result = GetArmStateResult()
        result.l_arm_state = self.l_arm_state
        result.r_arm_state = self.r_arm_state

        #rospy.loginfo('Left Arm State:  %s' % self.states[self.l_arm_state])
        #rospy.loginfo('Right Arm State: %s' % self.states[self.r_arm_state])

        self.action_server.set_succeeded(result)

    # Returns angle between -pi and + pi
    def angleWrap(self, angle):
        while angle > math.pi:
            angle -= math.pi*2.0
        while angle < -math.pi:
            angle += math.pi*2.0
        return angle

    # Determines if the arms are tucked or not
    def stateCb(self, msg):
        l_sum_tucked = 0
        l_sum_untucked = 0
        l_sum_pointed = 0
        l_sum_pointed_t = 0
        r_sum_tucked = 0
        r_sum_untucked = 0
        r_sum_pointed = 0
        r_sum_pointed_t = 0

        for name_state, name_desi, value_state, value_l_tucked, value_l_untucked, value_r_tucked, value_r_untucked, value_l_pointed, value_l_pointed_t, value_r_pointed, value_r_pointed_t in zip(msg.joint_names, joint_names, msg.actual.positions , l_arm_tucked, l_arm_untucked, r_arm_tucked, r_arm_untucked, l_arm_pointed, l_arm_pointed_tight, r_arm_pointed, r_arm_pointed_tight):

            value_state = self.angleWrap(value_state)

            if 'l_'+name_desi+'_joint' == name_state:
                self.l_received = True
                l_sum_tucked = l_sum_tucked + math.fabs(value_state - value_l_tucked)
                l_sum_untucked = l_sum_untucked + math.fabs(value_state - value_l_untucked)
                l_sum_pointed = l_sum_pointed + math.fabs(value_state - value_l_pointed)
                l_sum_pointed_t = l_sum_pointed_t + math.fabs(value_state - value_l_pointed_t)

            if 'r_'+name_desi+'_joint' == name_state:
                self.r_received = True
                r_sum_tucked = r_sum_tucked + math.fabs(value_state - value_r_tucked)
                r_sum_untucked = r_sum_untucked + math.fabs(value_state - value_r_untucked)
                r_sum_pointed = r_sum_pointed + math.fabs(value_state - value_r_pointed)
                r_sum_pointed_t = r_sum_pointed_t + math.fabs(value_state - value_r_pointed_t)

        # Determine the state of the left arm
        if l_sum_tucked > 0 and l_sum_tucked < 0.1:
            self.l_arm_state = 0
        elif l_sum_untucked > 0 and l_sum_untucked < 0.3:
            self.l_arm_state = 1
        elif l_sum_pointed > 0 and l_sum_pointed < 0.1:
            self.l_arm_state = 2
        elif l_sum_pointed_t > 0 and l_sum_pointed_t < 0.1:
            self.l_arm_state = 3
        elif l_sum_tucked >= 0.1 and l_sum_untucked >= 0.3 and l_sum_pointed >= 0.1 and l_sum_pointed_t >= 0.1:
            self.l_arm_state = -1    

        # Determine the state of the right arm
        if r_sum_tucked > 0 and r_sum_tucked < 0.1:
            self.r_arm_state = 0
        elif r_sum_untucked > 0 and r_sum_untucked < 0.3:
            self.r_arm_state = 1
        elif r_sum_pointed > 0 and r_sum_pointed < 0.1:
            self.r_arm_state = 2
        elif r_sum_pointed_t > 0 and r_sum_pointed_t < 0.1:
            self.r_arm_state = 3
        elif r_sum_tucked >= 0.1 and r_sum_untucked >= 0.3 and r_sum_pointed >= 0.1 and r_sum_pointed_t >= 0.1:
            self.r_arm_state = -1 

def main():
    action_name = 'get_arm_state'
    rospy.init_node(action_name)
    rospy.sleep(0.001)
    get_arm_state_action_server = GetArmStateActionServer(action_name)

    rospy.spin()

if __name__ == '__main__':
    main()
