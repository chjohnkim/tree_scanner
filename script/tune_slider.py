#!/usr/bin/env python

import sys
import rospy
import moveit_msgs.msg 
import moveit_commander 
import numpy as np


ROBOT_VELOCITY = 0.2 # Max robot speed scaled from 0 to 1

if __name__=='__main__':

    moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
    rospy.init_node('move_group_python_interface', anonymous=True) #initialize the node 
    robot = moveit_commander.RobotCommander() #define the robot
    scene = moveit_commander.PlanningSceneInterface() #define the scene
    group = moveit_commander.MoveGroupCommander("manipulator") #define the planning group (from the moveit packet 'manipulator' planning group)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1) #create a publisher that publishes a plan to the topic: '/move_group/display_planned_path'
    rospy.sleep(3)
    joint_init = group.get_current_joint_values() 
    joint_init = group.get_current_joint_values() 
    print(joint_init)

    joint_target = group.get_current_joint_values() 
    joint_target[6] -= np.pi/2
    joint_target[0] += 0.4
    print(joint_target)
    print(joint_init)
    group.set_joint_value_target(joint_target) 
    plan = group.plan() 
    group.execute(plan, wait=True) 
    group.stop()

    #group.set_joint_value_target(joint_init) 
    #plan = group.plan() 
    #group.execute(plan, wait=True) 
    #group.stop()

    moveit_commander.roscpp_shutdown() #shut down the moveit_commander