#!/usr/bin/env python
import roslib; roslib.load_manifest('ee_impulse_try1')
import rospy
import time
import pr2_utils.arm_control

from furniture.msg import *              #All_Hulls.msg
from cardboard.msg import *              #SceneHypothesis.msg

class Table_and_Objects:
    def __init__(self):
        self.tableZ = 0
        self.sbX = 0 #straw bowl
        self.sbY = 0
        self.gpX = 0 #green pear
        self.gpY = 0
        self.tcX = 0 #tiki cup
        self.tcY = 0
        self.taoFrame = ''

        self.convexSub = rospy.Subscriber("convex_hulls", All_Hulls, self.setTableParam)
        self.sceneSub = rospy.Subscriber("scene_hypothesis", SceneHypothesis, self.setObjParams)
        
    def setTableParam(self, msg):
        #print "inside"
        #Assume it's the first table
        nPts = len(msg.hulls[0].polygon.points) #number of points in the polygon
        total = 0
        for i in range(nPts):
            total = total + msg.hulls[0].polygon.points[i].z
        self.tableZ = total/nPts
        self.taoFrame =  msg.hulls[0].header.frame_id
        #print self.tableZ

    def setObjParams(self, msg):
        nObjs = len(msg.objects)
        oD = {}                  #this is a dictionary of the objects
        for i in range(nObjs):
            oD[msg.objects[i].name]=i

        #set the X and Y positions for the three objects
        self.sbX = msg.objects[oD['StrawBowl']].pose.position.x #straw bowl
        self.sbY = msg.objects[oD['StrawBowl']].pose.position.y
        self.gpX = msg.objects[oD['GreenPear']].pose.position.x #green pear
        self.gpY = msg.objects[oD['GreenPear']].pose.position.y
        self.tcX = msg.objects[oD['TikiCup']].pose.position.x #tiki cup
        self.tcY = msg.objects[oD['TikiCup']].pose.position.y

        #print(self.sbX, self.sbY, self.gpX, self.gpY, self.tcX, self.tcY)
        #print msg.header.frame_id

def impulse(TAO):
    
    arm = pr2_utils.arm_control.ArmControl("right_arm")
    #add_trajectory_point_to_force_control(x, y, z, ox, oy, oz, ow, 
    #                                          fx, fy, fz, tx, ty, tz, 
    #                                          isfx, isfy, isfz, istx, 
    #                                          isty, istz, time)

    #first position
    offZ1 = 0.20
    offZ2 = 0.03
    offX = -0.15 #radius around the bowl
    offXend = -0.05
    offY = 0.
    p1Z = TAO.tableZ + offZ1
    p1X = TAO.sbX + offX
    p1Y = TAO.sbY + offY

    p2Z = TAO.tableZ + offZ2
    
    arm.move_arm_to_side()

    arm.add_trajectory_point_to_force_control(p1X, p1Y, p1Z, 0, 0.707, 0, \
                                     0.707, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.taoFrame)
    
    arm.add_trajectory_point_to_force_control(p1X, p1Y, p2Z, 0, 0.707, 0, \
                                     0.707, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.taoFrame)
    
    arm.executeForceControl()
    #arm.add_trajectory_point_to_force_control(0.55, 0, -0.20, 0, 0.707, 0,\
    #                                 0.707, 1000, 1000, 1000, 30, 30, 30,\
    #                                 False, False, False, False, False,\
    #                                 False, 4)

    rospy.sleep(3)

    goal_x_pos = TAO.sbX + offXend

    arm.add_trajectory_point_to_force_control(goal_x_pos, p1Y, p2Z, 0, 0.707,\
                                     0, 0.707, 25, 1000, 1000, 30, 30, 30,\
                                     True, False, False, False, False,\
                                     False, 4, frame_id=TAO.taoFrame)

    arm.executeForceControl(wait=False)

    keepGoing = 1
    
    while (keepGoing == 1):
        hand_pose = arm.get_hand_pose()
        hand_x = hand_pose.pose.position.x
        if (hand_x>=goal_x_pos):
            arm.stop_in_place()
            keepGoing = 0
            print hand_x
    
    rospy.sleep(3)

    arm.add_trajectory_point_to_force_control(goal_x_pos, p1Y, p1Z, 0, 0.707,\
                                     0, 0.707, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.taoFrame)

    arm.executeForceControl()

    arm.move_arm_to_side()
    #arm.add_trajectory_point_to_force_control(0.5, 0, 0.20, 0, 0, 0, 1,\
    #                                 1000, 1000, 1000, 30, 30, 30,\
    #                                 False, False, False, False, False,\
    #                                 False, 4)

def waitForValues():
    print "waiting for convex_hulls"
    rospy.client.wait_for_message("convex_hulls", All_Hulls)
    print "recieved convex_hulls"
    print "waiting for scene_hypothesis"
    rospy.client.wait_for_message("scene_hypothesis", SceneHypothesis)
    print "recieved scene_hypothesis"

if __name__ == '__main__':
    rospy.init_node('ee_impulse_testing_node')
    try:
        TAO = Table_and_Objects()
        print "i got here at least"
        waitForValues()
        impulse(TAO)
    except rospy.ROSInterruptException: pass
