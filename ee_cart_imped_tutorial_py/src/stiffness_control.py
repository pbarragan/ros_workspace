#!/usr/bin/env python
import roslib; roslib.load_manifest('ee_cart_imped_tutorial')
import rospy
import ee_cart_imped_action

def main():
    control = ee_cart_imped_action.EECartImpedClient('right_arm')

    control.addTrajectoryPoint(0.1, 0, 0, 0, 0, 0.5, 0.866,
                               1000, 50, 1000, 30, 30, 30,
                               False, False, False, False, False,
                               False, 4, '/r_gripper_tool_frame');    
    control.addTrajectoryPoint(0.1, 0, 0, 0, 0, 0.5, 0.866,
                               1000, 50, 1000, 30, 30, 30,
                               False, False, False, False, False,
                               False, 20, '/r_gripper_tool_frame');
    control.addTrajectoryPoint(-0.1, 0, 0, 0, 0, 0, 1,
                               1000, 50, 1000, 30, 30, 30,
                               False, False, False, False, False,
                               False, 24, '/r_gripper_tool_frame');

    # control.addTrajectoryPoint(0.1, 0, 0, 0, 0, 0, 1,
    #                            1000, 50, 1000, 30, 30, 30,
    #                            False, False, False, False, False,
    #                            False, 4, '/r_gripper_tool_frame');
    # control.addTrajectoryPoint(0.0, 0, 0, 0, 0, 0, 1,
    #                            1000, 50, 1000, 30, 30, 30,
    #                            False, False, False, False, False,
    #                            False, 8, '/r_gripper_tool_frame');

    control.sendGoal()

if __name__ == '__main__':
    rospy.init_node('stiffness_control_test')
    main()