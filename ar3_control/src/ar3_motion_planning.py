#! /usr/bin/env python
#This works 6/21/22
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np


moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('ar3_program_planning', anonymous=True)

robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = moveit_commander.MoveGroupCommander("manipulator")
display_trajectory_publisher = rospy.Publisher(
    '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=10)

joint_values = group.get_current_joint_values()

print("Current Pose:")
print(group.get_current_pose())
print("Current Joint Values")
print(joint_values)

#load text file with actions table  and convert to numpy array  
actions_table = np.loadtxt('C:/ar3_ws/src/ar3_control/src/actions_table5.txt', delimiter=',')
#trim the zeros from the end of the array
actions_table = actions_table[~np.all(actions_table == 0, axis=1)]

#loop through the actions table and assign a joint value to each row
for i in range(len(actions_table)):
    # for each action (9) execute the joint values 
    joint_values = group.get_current_joint_values()

    #for j in range(len(joint_values)):
    #    joint_values[j] = np.add(joint_values,actions_table[i,j])
    joint_values = np.add(joint_values,actions_table[i])
    print(joint_values)
    group.set_joint_value_target(joint_values)
    plan2 = group.plan()
    group.execute(plan2, wait=True)
    print(group.get_current_pose())
    #group.go(joint_values, wait=True)
    #group.stop()
    #rospy.sleep(2)

#add joint values to actions table
#actions_table = np.append(actions_table, joint_values, axis=0)

#joint_values[0] = -1
#joint_values[1] = 1
#group.set_joint_value_target(joint_values)


#plan2 = group.plan()
#group.go(joint_values, wait=True)
#group.stop()
rospy.sleep(5)
#group.stop()
moveit_commander.roscpp_shutdown()

