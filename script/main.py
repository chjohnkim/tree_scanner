#!/usr/bin/env python

import sys
import rospy
import moveit_msgs.msg 
import moveit_commander 
import scanner

MODE = 'sphere'
ROBOT_VELOCITY = 0.3 # Max robot speed scaled from 0 to 1
NUM_PLANNING_ATTEMPTS = 100
DISTANCE_BETWEEN_TRUNK_AND_CAMERA = None
SCAN_ANGLE_DEGREES = None
NUM_LEVELS = None
LEVEL_OFFSET = None
if MODE=='cylinder':
    DISTANCE_BETWEEN_TRUNK_AND_CAMERA = 0.5
    SCAN_RADIUS = 0.5
    SCAN_ANGLE_DEGREES = 80
    NUM_LEVELS = 4
    LEVEL_OFFSET = 0.2
    NUM_VIEWPOINTS = 32
elif MODE=='sphere':
    SCAN_RADIUS = 0.3
    SPHERE_LEVELS = [0.90]
    NUM_VIEWPOINTS = 16


if __name__=='__main__':

    moveit_commander.roscpp_initialize(sys.argv) #initialize the moveit commander
    rospy.init_node('move_group_python_interface', anonymous=True) #initialize the node 
    robot = moveit_commander.RobotCommander() #define the robot
    scene = moveit_commander.PlanningSceneInterface() #define the scene
    group = moveit_commander.MoveGroupCommander("manipulator") #define the planning group (from the moveit packet 'manipulator' planning group)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1) #create a publisher that publishes a plan to the topic: '/move_group/display_planned_path'
    rospy.sleep(3)

    tree_scanner = scanner.scanner(group=group,
                                   robot=robot, 
                                      dist_trunk=DISTANCE_BETWEEN_TRUNK_AND_CAMERA,
                                      scan_radius=SCAN_RADIUS,
                                      scan_angle=SCAN_ANGLE_DEGREES,
                                      num_levels=NUM_LEVELS,
                                      level_offset=LEVEL_OFFSET, 
                                      velocity=ROBOT_VELOCITY)
    if MODE=='cylinder':
        tree_scanner.go_home('bot')
        while True:
            plan, fraction = tree_scanner.generate_cylindrical_waypoints(NUM_LEVELS)
            scan = raw_input('Execute waypoints plan?: (Y/n)')
            if scan=='Y':
                tree_id = raw_input('Tree ID (example: 6-4):')
                tree_scanner.scan(plan, tree_id=tree_id, num_viewpoints=NUM_VIEWPOINTS)
                break
            elif scan=='x':
                moveit_commander.roscpp_shutdown() 
                tree_scanner.camera.close()
                raise KeyboardInterrupt
        tree_scanner.go_home('top')
        while True:
            plan, fraction = tree_scanner.generate_cylindrical_waypoints(NUM_LEVELS-1)
            scan = raw_input('Execute waypoints plan?: (Y/n)')
            if scan=='Y':
                tree_id = raw_input('Tree ID (example: 6-4):')
                tree_scanner.scan(plan, tree_id=tree_id, num_viewpoints=NUM_VIEWPOINTS)
                break
            elif scan=='x':
                moveit_commander.roscpp_shutdown() 
                tree_scanner.camera.close()
                raise KeyboardInterrupt

    elif MODE=='sphere':
        #tree_scanner.go_home('top')
        #tree_scanner.set_home()
        while True:
            plan, fraction = tree_scanner.generate_spherical_waypoints_from_tag(SPHERE_LEVELS)
            scan = raw_input('Execute waypoints plan?: (Y/n)')
            if scan=='Y':
                tree_id = raw_input('Tree ID (example: 6-4):')
                tree_scanner.scan(plan, tree_id=tree_id, num_viewpoints=NUM_VIEWPOINTS)
            elif scan=='x':
                moveit_commander.roscpp_shutdown() 
                tree_scanner.camera.close()
                raise KeyboardInterrupt
    '''
    if MODE=='cylinder':
        #tree_scanner.go_home('bot')
        tree_scanner.go_home('top')
    elif MODE=='sphere':
        pass
        #tree_scanner.set_home()
    while True:
        num_planning_attempts+=1
        if MODE=='cylinder':
            plan, fraction = tree_scanner.generate_cylindrical_waypoints()
        elif MODE=='sphere':
            plan, fraction = tree_scanner.generate_spherical_waypoints_from_tag(SPHERE_LEVELS)
        else:
            moveit_commander.roscpp_shutdown() 
            tree_scanner.camera.close()
            raise KeyboardInterrupt            
        scan = raw_input('Execute waypoints plan?: (Y/n)')
        if scan=='Y':
            tree_id = raw_input('Tree ID (example: 6-4):')
            tree_scanner.scan(plan, tree_id=tree_id, num_viewpoints=NUM_VIEWPOINTS)
            #tree_scanner.execute_waypoints(plan)
            #tree_scanner.execute_plan_by_joints(plan)
        elif scan=='x':
            moveit_commander.roscpp_shutdown() 
            tree_scanner.camera.close()
            raise KeyboardInterrupt
    '''
    moveit_commander.roscpp_shutdown() #shut down the moveit_commander
