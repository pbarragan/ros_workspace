#!/usr/bin/env python
import roslib; roslib.load_manifest('ee_impulse_try1')
import rospy
import pr2_utils.arm_control

#here is a change i made on the robot

def impulse():
    
    arm = pr2_utils.arm_control.ArmControl("right_arm")
    #add_trajectory_point_to_force_control(x, y, z, ox, oy, oz, ow, 
    #                                          fx, fy, fz, tx, ty, tz, 
    #                                          isfx, isfy, isfz, istx, 
    #                                          isty, istz, time)

    arm.move_arm_to_side()

    arm.add_trajectory_point_to_force_control(0.5, 0, 0.20, 0, 0, 0, 1,\
                                     1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4)
    
    arm.add_trajectory_point_to_force_control(0.55, 0, -0.20, 0, 0.707, 0,\
                                     0.707, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4)

    goal_x_pos = 0.75
    
    arm.add_trajectory_point_to_force_control(goal_x_pos, 0, -0.20, 0, 0.707, 0,\
                                     0.707, 10, 1000, 1000, 30, 30, 30,\
                                     True, False, False, False, False,\
                                     False, 4)

    arm.executeForceControl(wait=False)

    keepGoing = 1
    
    while (keepGoing == 1):
        hand_pose = arm.get_hand_pose()
        hand_x = hand_pose.pose.position.x
        if (hand_x>=goal_x_pos):
            arm.stop_in_place()
            keepGoing = 0
            print hand_x

    arm.add_trajectory_point_to_force_control(0.5, 0, 0.20, 0, 0, 0, 1,\
                                     1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4)

if __name__ == '__main__':
    rospy.init_node('ee_impulse_testing_node')
    try:
        impulse()
    except rospy.ROSInterruptException: pass
