#! /usr/bin/env python

import rospy; 
import roslib; 

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

# Spacious configuration
r_arm_pointed   = [-1.0, 1.3, -0.75, -1.0, -3, -0.1, 0]
l_arm_pointed   = [ 1.0, 1.3,  0.75, -1.0,  3, -0.1, 0]

# Tight configuration
r_arm_pointed_t = [-1.0, 1.3, -1.25, -1.0, -3, -0.1, 0]
l_arm_pointed_t = [ 1.0, 1.3,  1.25, -1.0,  3, -0.1, 0]

class PointArmsActionServer:
    # arm state: -1 unknown, 0 tucked, 1 untucked
    l_arm_state = 0
    r_arm_state = 0

    states = ["tucked", "untucked", "pointed", "pointed-tight", "unknown"]

    def __init__(self, node_name):
        self.node_name = node_name
        
        self.success = True

        l_name = 'l_arm_controller/joint_trajectory_action'
        r_name = 'r_arm_controller/joint_trajectory_action'

        self.move_duration = 1
        self.state_client = SimpleActionClient('get_arm_state', GetArmStateAction)
        self.l_client = SimpleActionClient(l_name, JointTrajectoryAction)
        self.r_client = SimpleActionClient(r_name, JointTrajectoryAction)

        if not self.state_client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("get_arm_state: action server did not come up within the time limit")
        if not self.l_client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("point_arms: l_client action server did not come up within time limit")
        if not self.r_client.wait_for_server(rospy.Duration(30)):
            rospy.logerr("point_arms: r_client action server did not come up within time limit")

        goal = GetArmStateGoal()
        self.state_client.send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0))
        state_result = self.state_client.get_result()

        self.l_arm_state = state_result.l_arm_state
        self.r_arm_state = state_result.r_arm_state

        rospy.loginfo('Left Arm State:  %s' % self.states[self.l_arm_state])
        rospy.loginfo('Right Arm State: %s' % self.states[self.r_arm_state])

        self.action_server = SimpleActionServer(node_name, PointArmsAction, self.executeCB)

    def updateArmState(self):
        state_goal = GetArmStateGoal()
        self.state_client.send_goal_and_wait(state_goal, rospy.Duration(30.0), rospy.Duration(5.0))
        state_result = self.state_client.get_result()

        self.l_arm_state = state_result.l_arm_state
        self.r_arm_state = state_result.r_arm_state

        rospy.loginfo('Left Arm State:  %s' % self.states[self.l_arm_state])
        rospy.loginfo('Right Arm State: %s' % self.states[self.r_arm_state])
        

    def executeCB(self, goal):
        self.updateArmState()

        result = PointArmsResult()

        lclient = self.pointL(goal.tight)
        rclient = self.pointR(goal.tight)
        lclient.wait_for_result()
        rclient.wait_for_result()

        self.updateArmState()

        if self.success:
            result.success = True
            self.action_server.set_succeeded(result)
        else:
            result.success = False
            self.action_server.set_aborted(result)


    def pointL(self, tight):
        if tight:
            return self.go('l', [l_arm_pointed_t])
        else:
            return self.go('l', [l_arm_pointed])
    
    def pointR(self, tight):
        if tight:
            return self.go('r', [r_arm_pointed_t])
        else:
            return self.go('r', [r_arm_pointed])

    def go(self, side, positions, wait = False):
        goal = JointTrajectoryGoal()
        goal.trajectory.joint_names = [side+"_"+name+"_joint" for name in joint_names]
        goal.trajectory.points = []
        for p, count in zip(positions, range(0,len(positions)+1)):
            goal.trajectory.points.append(
                JointTrajectoryPoint( positions = p,
                                      velocities = [],
                                      accelerations = [],
                                      time_from_start = rospy.Duration((count+1) * self.move_duration))
                )

        goal.trajectory.header.stamp = rospy.get_rostime() + rospy.Duration(0.01)

        if wait:
            if not {'l': self.l_client, 'r': self.r_client}[side].send_goal_and_wait(goal, rospy.Duration(30.0), rospy.Duration(5.0)):
                self.success = False
        else:
            {'l': self.l_client, 'r': self.r_client}[side].send_goal(goal)

        return {'l': self.l_client, 'r': self.r_client}[side]



def main():
    action_name = 'point_arms'
    rospy.init_node(action_name)
    rospy.sleep(0.001)
    point_arms_action_server = PointArmsActionServer(action_name)

    rospy.spin()

if __name__ == '__main__':
    main()
