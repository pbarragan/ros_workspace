#!/usr/bin/env python
import roslib; roslib.load_manifest('ee_impulse_try1')
import rospy
import time
import pr2_utils.arm_control
import sensor_msgs.msg
import object_manipulation_msgs.msg
import geometry_msgs.msg
import actionlib

from furniture.msg import *              #All_Hulls.msg
from cardboard.msg import *              #SceneHypothesis.msg
from ee_impulse_try1.msg import *        #Cylinders.msg

class Table_and_Objects:
    def __init__(self):
        self.tableZ = 0
        #self.sbX = 0 #straw bowl
        #self.sbY = 0
        #self.gpX = 0 #green pear
        #self.gpY = 0
        #self.tcX = 0 #tiki cup
        #self.tcY = 0
        self.obj_frame_id = ''
        self.Objects_on_Table = []
        self.goalPost_left = [0,0]
        self.goalPost_left_Ind = 0
        self.goalPost_right = [0,0]
        self.goalPost_right_Ind = 0
        self.Target_Object_Current = [0,0]
        self.Target_Object_Current_Ind = 0


        #self.convexSub = rospy.Subscriber("convex_hulls", All_Hulls, self.setTableParam)
        #self.sceneSub = rospy.Subscriber("scene_hypothesis", SceneHypothesis, self.setObjParams)

        #self.objectSub = rospy.Subscriber("cylinders", Cylinders, self.setObjParams)
        
        hul_msg = rospy.client.wait_for_message("convex_hulls", All_Hulls)
        self.setTableParam(hul_msg)
        cyl_msg = rospy.client.wait_for_message("cylinders", Cylinders)
        self.setObjParams(cyl_msg)

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
        print "Table Height:"
        print self.tableZ

    def setObjParams(self, msg):
        numObjs = len(msg.cylinders)                    
       
        #check if you have three objects initially
        if numObjs != 3:
            rospy.loginfo("Only % objects. I'm supposed to have 3. I'm getting \
out of here." %numObjs)
        else:
            # if it is three, set which objects are which       
            self.Objects_on_Table = msg.cylinders
            self.obj_frame_id = msg.header.frame_id
            Xposes = [0]*numObjs
            Yposes = [0]*numObjs
            
            #get a list of x and y positions for the objects                    
            for i in range(numObjs):
                Xposes[i]=self.Objects_on_Table[i].x
                Yposes[i]=self.Objects_on_Table[i].y
                
            #find the target object. It should be the closest object initially.
            minX = min(Xposes)
            minInd = Xposes.index(minX) #index of the minimum X                 
            self.Target_Object_Current = [minX,Yposes[minInd]]  #X,Y of target object    
            self.Target_Object_Current_Ind = minInd

            #set the goal posts                                                 
            Xposes_copy = Xposes
            Yposes_copy = Yposes
            # remove the target object                                          
            Xposes_copy.remove(minX)
            Yposes_copy.remove(Yposes[minInd])

            #figure out which one is to the left. It should have max Y.         
            maxY = max(Yposes_copy)
            maxInd = Yposes_copy.index(maxY) #index of the max Y                
            self.goalPost_left = [Xposes_copy[maxInd],maxY]
            self.goalPost_left_Ind = maxInd

            if maxInd == 0:
                self.goalPost_right = [Xposes_copy[1],Yposes_copy[1]]
                self.goalPost_right_Ind = 1
            elif maxInd == 1:
                self.goalPost_right = [Xposes_copy[0],Yposes_copy[0]]
                self.goalPost_right_Ind = 0

            print "Left Goal Post:"
            print self.goalPost_left
            print "Right Goal Post:"
            print self.goalPost_right
            print "Target Object:"
            print self.Target_Object_Current
            print "Object Frame ID:"
            print self.obj_frame_id



        #oD = {}                  #this is a dictionary of the objects
        #for i in range(nObjs):
        #    oD[msg.objects[i].name]=i

        #set the X and Y positions for the three objects
        #self.sbX = msg.objects[oD['StrawBowl']].pose.position.x #straw bowl
        #self.sbY = msg.objects[oD['StrawBowl']].pose.position.y
        #self.gpX = msg.objects[oD['GreenPear']].pose.position.x #green pear
        #self.gpY = msg.objects[oD['GreenPear']].pose.position.y
        #self.tcX = msg.objects[oD['TikiCup']].pose.position.x #tiki cup
        #self.tcY = msg.objects[oD['TikiCup']].pose.position.y

        #print(self.sbX, self.sbY, self.gpX, self.gpY, self.tcX, self.tcY)
        #print msg.header.frame_id

def test_pickup(TAO):
    #create a fake object hovering in baselink
    #points = []
    #for x in range(60, 70):
    #    for y in range(-2, 2):
    #        for z in range(-5, 5):
    #            pt = geometry_msgs.msg.Point32(x/100.0, y/100.0, z/100.0)
    #            points.append(pt)
    
 #set everything for the target object
    #cloud = sensor_msgs.msg.PointCloud()
    #cloud.header.frame_id = 'torso_lift_link'
    #cloud.points = points
    #cloud_pub.publish(cloud)
    #cloud_pub.publish(cloud)
    #cloud_pub.publish(cloud)
    obj = object_manipulation_msgs.msg.GraspableObject()
    obj.cluster = TAO.Objects_on_Table[TAO.Target_Object_Current_Ind].cloud
    obj.reference_frame_id = obj.cluster.header.frame_id #TAO.obj_frame_id
    goal = object_manipulation_msgs.msg.PickupGoal()
    goal.arm_name = 'right_arm'
    goal.target = obj
    goal.collision_object_name = ''
    goal.collision_support_surface_name = ''
    goal.lift = object_manipulation_msgs.msg.GripperTranslation()
    goal.lift.direction.header.frame_id = "base_link" #TAO.obj_frame_id
    goal.lift.direction.vector = geometry_msgs.msg.Vector3(0, 0, 1)
    goal.lift.desired_distance = 0.1
    goal.lift.min_distance = 0.05
    pickup_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_pickup', object_manipulation_msgs.msg.PickupAction)
    print 'waiting for pickup server'
    pickup_client.wait_for_server()
    print 'Found server, sending goal'
    pickup_client.send_goal_and_wait(goal)
    print 'Seems to have worked?'

def resetArm(arm):
    arm.move_arm_to_side()

def impulse(TAO,arm):
    
    #arm = pr2_utils.arm_control.ArmControl("right_arm")
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

    #print "waiting for scene_hypothesis"
    #rospy.client.wait_for_message("scene_hypothesis", SceneHypothesis)
    #print "recieved scene_hypothesis"

    print "waiting for cylinders"
    rospy.client.wait_for_message("cylinders", Cylinders)
    print "recieved cylinders"

if __name__ == '__main__':
    rospy.init_node('ee_impulse_testing_node')
    arm = pr2_utils.arm_control.ArmControl("right_arm")
    resetArm(arm)
    TAO = Table_and_Objects()
    try:
        print "i got here at least"
        waitForValues()
        test_pickup(TAO)
        resetArm(arm)
    except rospy.ROSInterruptException: pass
