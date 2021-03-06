#!/usr/bin/env python
import roslib; roslib.load_manifest('ee_impulse_try1')
import rospy
import time
import pr2_utils.arm_control
import sensor_msgs.msg
import object_manipulation_msgs.msg
import geometry_msgs.msg
import actionlib
from pr2_pick_and_place_demos.pick_and_place_manager import *
from object_manipulator.convert_functions import *
import math
from numpy import *

from furniture.msg import *              #All_Hulls.msg
from cardboard.msg import *              #SceneHypothesis.msg
from ee_impulse_try1.msg import *        #Cylinders.msg

#this is the one that does linear regression


###################
# This initializes all the variables needed
###################
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
        self.Target_Object_Current_R = 0
        self.Target_Object_Current_Ind = 0
        self.reset_pose = [0,0]
        self.Target_Object_Before = [0,0]
        self.Target_Object_After = [0,0]
        self.papm = PickAndPlaceManager()

        #force, in this function the 4 forces are used with F being first
        self.F = 10 #[N]
        self.Flist = [10,15,20,25] #this will be used to do the measurements, original one, for 5 trials total
        #self.Flist = [10,12.5,15,17.5,20,22.5,25] #this will be used to do the measurements, for 8 trials total

        #regression
        self.zList = [] #this will be the list of measurements

        #learning parameters
        self.pr_mu = 0
        self.pr_var = 50
        self.lh_mu = 0
        self.lh_var = 0
        #this variance should be process noise + measurement noise
        self.model_var = 0.0001 #probably a few centimeters of measurement error
        self.po_mu = 0
        self.po_var = 0

        #do the file writing stuff for initialization
        f = open('./testFiles/testFileNEW.txt','a')
        f.write('\n\n\nThis is a new regression experiment\n\n\n')
        f.close()

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
            self.Target_Object_Current_R = self.Objects_on_Table[minInd].r
            self.reset_pose = self.Target_Object_Current

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

########################
# This never worked
########################
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
    goal.lift.direction.header.frame_id = "/base_link" #TAO.obj_frame_id
    goal.lift.direction.vector = geometry_msgs.msg.Vector3(0, 0, 1)
    goal.lift.desired_distance = 0.1
    goal.lift.min_distance = 0.05

    print "I got to the grasp stuff"
    ###################attempting to add grasp stuff
    #goal.desired_grasps = [object_manipulation_msgs.msg.Grasp()]
    #goal.desired_grasps[0].pre_grasp_posture = sensor_msgs.msg.JointState()
    #goal.desired_grasps[0].grasp_posture = sensor_msgs.msg.JointState()
    #goal.desired_grasps[0].grasp_pose = geometry_msgs.msg.Pose()

    #set frame id's
    #goal.desired_grasps[0].pre_grasp_posture.header.frame_id = obj.cluster.header.frame_id
    #goal.desired_grasps[0].grasp_posture.header.frame_id = obj.cluster.header.frame_id
    #goal.desired_grasps[0].pre_grasp_posture.header.frame_id = "/base_link"
    #goal.desired_grasps[0].grasp_posture.header.frame_id = "/base_link"

    #set position for pre_grasp
    #goal.desired_grasps[0].pre_grasp_posture.position = [TAO.Target_Object_Current[0],TAO.Target_Object_Current[1],TAO.tableZ+.1]

    #set position for grasp
    #goal.desired_grasps[0].grasp_posture.position = [TAO.Target_Object_Current[0],TAO.Target_Object_Current[1],TAO.tableZ+.05]

    #set grasp pose
    #goal.desired_grasps[0].grasp_pose.position.x = TAO.Target_Object_Current[0]
    #goal.desired_grasps[0].grasp_pose.position.y = TAO.Target_Object_Current[1]
    #goal.desired_grasps[0].grasp_pose.position.z = TAO.tableZ+.05
    #goal.desired_grasps[0].grasp_pose.orientation.x = 0
    #goal.desired_grasps[0].grasp_pose.orientation.y = 0.707
    #goal.desired_grasps[0].grasp_pose.orientation.z = 0
    #goal.desired_grasps[0].grasp_pose.orientation.w = 0.707

    #set other parameters maybe
    #goal.desired_grasps[0].success_probability = 0.9
    #goal.desired_grasps[0].desired_approach_distance = 0.05
    #goal.desired_grasps[0].min_approach_distance = 0.02

    ####################done adding grasp stuff

    pickup_client = actionlib.SimpleActionClient('/object_manipulator/object_manipulator_pickup', object_manipulation_msgs.msg.PickupAction)
    print 'waiting for pickup server'
    pickup_client.wait_for_server()
    print 'Found server, sending goal'
    pickup_client.send_goal_and_wait(goal)
    print 'Seems to have worked?'

def poseStampedFromList(pose):
    PoseS = geometry_msgs.msg.PoseStamped()
    PoseS.header.frame_id = "/base_footprint"
    PoseS.pose.position.x = pose[0]
    PoseS.pose.position.y = pose[1]
    PoseS.pose.position.z = pose[2]
    PoseS.pose.orientation.x = pose[3]
    PoseS.pose.orientation.y = pose[4]
    PoseS.pose.orientation.z = pose[5]
    PoseS.pose.orientation.w = pose[6]

    return PoseS

def test_pickup2(TAO):
    #exPose = [.1,-.60,0,-.5,.5,.5,.5]
    exPose = poseStampedFromList([.05,-.50,0.7,-.5,.5,.5,.5])

    TAO.papm.move_cartesian_step(0,exPose)
    rospy.sleep(1)
    TAO.papm.open_gripper(0)
    TAO.papm.close_gripper(0)

####################################
# This function picks up the target object
####################################
def pickup(TAO,arm):
    arm.move_arm_to_side()

    arm.add_trajectory_point_to_force_control(0.3, 0, TAO.tableZ+.3, -.5, .5, .5, \
                                     .5, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)

    #set the pre grasp pose
    pg_px = TAO.Target_Object_Current[0]-TAO.Target_Object_Current_R
    pg_py = TAO.Target_Object_Current[1]
    pg_pz = TAO.tableZ+.20
    pg_ox = -0.5
    pg_oy = 0.5
    pg_oz = 0.5
    pg_ow = 0.5

    #set the post grasp pose
    g_px = TAO.Target_Object_Current[0]-TAO.Target_Object_Current_R
    g_py = TAO.Target_Object_Current[1]
    g_pz = TAO.tableZ+.03
    g_ox = -0.5
    g_oy = 0.5
    g_oz = 0.5
    g_ow = 0.5

    arm.add_trajectory_point_to_force_control(pg_px, pg_py, pg_pz, pg_ox, pg_oy, pg_oz, \
                                     pg_ow, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)
    
    arm.executeForceControl()

    #we are above the bowl. open the gripper.
    TAO.papm.open_gripper(0)
    rospy.sleep(2)

    #lower down
    arm.add_trajectory_point_to_force_control(g_px, g_py, g_pz, g_ox, g_oy, g_oz, \
                                     g_ow, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)
    
    arm.executeForceControl()

    #close the gripper
    TAO.papm.close_gripper(0)
    rospy.sleep(2)

    #raise up to pre grasp pose
    arm.add_trajectory_point_to_force_control(pg_px, pg_py, pg_pz, pg_ox, pg_oy, pg_oz, \
                                     pg_ow, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)
    
    arm.executeForceControl()

    #for now just drop the bowl
    #TAO.papm.open_gripper(0)
    #rospy.sleep(2)
    #we are good, hang out for the next command

#################################
# This function puts the target object back where it came from
#################################
def resetTargetObject(TAO,arm):
    #arm.move_arm_to_side()

    r_x = TAO.reset_pose[0]-TAO.Target_Object_Current_R
    r_y = TAO.reset_pose[1]
    r_zA = TAO.tableZ+0.2
    r_zB = TAO.tableZ+0.03
    
    arm.add_trajectory_point_to_force_control(r_x, r_y, r_zA, -.5, .5, .5, \
                                     .5, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)

    arm.add_trajectory_point_to_force_control(r_x, r_y, r_zB, -.5, .5, .5, \
                                     .5, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)
    arm.executeForceControl()


    #we are on the table. open gripper
    TAO.papm.open_gripper(0)
    rospy.sleep(2)

    #raise up. set the clearing pose
    
    c_x = r_x
    c_y = r_y
    c_z = r_zA

    arm.add_trajectory_point_to_force_control(c_x, c_y, c_z, -.5, .5, .5, \
                                     .5, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)
    
    arm.executeForceControl()

    #close the gripper
    TAO.papm.close_gripper(0)
    rospy.sleep(2)

    #we're good. move arm to side and hang out.
    arm.move_arm_to_side()
    rospy.sleep(2)

###########################
# This function updates the target object position using the vision.
# step = 0 if the update should also update the before position and = 1 if it's for the after position. step should be set to something else (like -1) if neither should be changed.
###########################
def updateTargetObject(TAO,step):
    tarInd = 5 #which is not possible
    numIter = 0

    while tarInd == 5 and numIter < 100:
        cyl_msg = rospy.client.wait_for_message("cylinders", Cylinders)
        numObjs = len(cyl_msg.cylinders)                    
    
        #check if you have three objects initially
        if numObjs != 3:
            rospy.loginfo("Only % objects. I'm supposed to have 3. I'm getting \
out of here." %numObjs)
        else:
            # if it is three, set which objects are which

        ############ this all assumes you won't get one object closest to both posts#############
            TAO.Objects_on_Table = cyl_msg.cylinders
            TAO.obj_frame_id = cyl_msg.header.frame_id
            Xposes = [0]*numObjs
            Yposes = [0]*numObjs
        
        #get a list of x and y positions for the objects                    
            for i in range(numObjs):
                Xposes[i]=TAO.Objects_on_Table[i].x
                Yposes[i]=TAO.Objects_on_Table[i].y

        #find squared distances to the goal posts. 
        #you can probably combine some for loops
        #you can probably clean this up a lot
            sDistsL = [0]*numObjs
            sDistsR = [0]*numObjs
            for i in range(numObjs):
                sDistsL[i] = (TAO.goalPost_left[0]-Xposes[i])*(TAO.goalPost_left[0]-Xposes[i])+(TAO.goalPost_left[1]-Yposes[1])*(TAO.goalPost_left[1]-Yposes[1])
                sDistsR[i] = (TAO.goalPost_right[0]-Xposes[i])*(TAO.goalPost_right[0]-Xposes[i])+(TAO.goalPost_right[1]-Yposes[1])*(TAO.goalPost_right[1]-Yposes[1])
            
        #find the target object. It should be the farthest from the goal post.
        #you finally properly used farthest.
            minL = min(sDistsL)
            minLi = sDistsL.index(minL) #index of the minimum to the left   
        
            minR = min(sDistsR)
            minRi = sDistsR.index(minR) #index of the minimum to the right 
        
        #tarInd = 5 #which is not possible

            if minLi == 0:
                if minRi == 1:
                    tarInd = 2
                elif minRi == 2:
                    tarInd = 1
            elif minLi == 1:
                if minRi == 0:
                    tarInd = 2
                elif minRi == 2:
                    tarInd = 0
            elif minLi == 2:
                if minRi == 0:
                    tarInd = 1
                elif minRi == 1:
                    tarInd = 0

            numIter = numIter + 1
            print "num iter:"
            print numIter

    TAO.Target_Object_Current = [Xposes[tarInd],Yposes[tarInd]]  #X,Y of target object    
    TAO.Target_Object_Current_Ind = tarInd
    TAO.Target_Object_Current_R = TAO.Objects_on_Table[tarInd].r

    if step == 0:
        TAO.Target_Object_Before = TAO.Target_Object_Current
    elif step == 1:
        TAO.Target_Object_After = TAO.Target_Object_Current

###############################
#this just moves the arm out of the way
###############################
def resetArm(arm):
    arm.move_arm_to_side()

###############################
#this is the updated impulse function. It finds the bowl and strikes it and then moves the arm out of the way.
###############################
def impulse_new(TAO,arm):
    TAO.papm.close_gripper(0)
    rospy.sleep(2)


 #first position
    offZ1 = 0.20
    offZ2 = 0.03
    offX = -0.15 #radius around the bowl
    offXend = -0.05
    offY = 0.
    p1Z = TAO.tableZ + offZ1
    p1X = TAO.Target_Object_Current[0] + offX
    p1Y = TAO.Target_Object_Current[1] + offY

    p2Z = TAO.tableZ + offZ2
    
    arm.move_arm_to_side()

    arm.add_trajectory_point_to_force_control(p1X, p1Y, p1Z, 0, 0.707, 0, \
                                     0.707, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)
    
    arm.add_trajectory_point_to_force_control(p1X, p1Y, p2Z, 0, 0.707, 0, \
                                     0.707, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)
    
    arm.executeForceControl()
    #arm.add_trajectory_point_to_force_control(0.55, 0, -0.20, 0, 0.707, 0,\
    #                                 0.707, 1000, 1000, 1000, 30, 30, 30,\
    #                                 False, False, False, False, False,\
    #                                 False, 4)

    rospy.sleep(3)

    goal_x_pos = TAO.Target_Object_Current[0] + offXend

    #this orginally was 25 newtons. make sure it's not more than 50
    f_use = min(TAO.F,50)

    arm.add_trajectory_point_to_force_control(goal_x_pos, p1Y, p2Z, 0, 0.707,\
                                     0, 0.707, f_use, 1000, 1000, 30, 30, 30,\
                                     True, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)

    arm.executeForceControl(wait=False)

    keepGoing = 1
    
    while (keepGoing == 1):
        hand_pose = arm.get_hand_pose()
        hand_x = hand_pose.pose.position.x
        if (hand_x>=goal_x_pos):
            arm.stop_in_place()
            keepGoing = 0
            #print hand_x
    
    rospy.sleep(3)

    arm.add_trajectory_point_to_force_control(goal_x_pos, p1Y, p1Z, 0, 0.707,\
                                     0, 0.707, 1000, 1000, 1000, 30, 30, 30,\
                                     False, False, False, False, False,\
                                     False, 4, frame_id=TAO.obj_frame_id)

    arm.executeForceControl()

    arm.move_arm_to_side()

###################################
#this function just waits until we get the first messages to make sure something is coming in before we try to initialize.
###################################
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

###################################
# evaluates a 1D gaussian
###################################
def gauss1D(x,mu,sig):
    return (1/sqrt(2*math.pi*sig**2))*math.exp((-(x-mu)**2)/(2*sig**2))

###################################
# this function updates the parameter probabilities
# first try is assuming coulombic friction
###################################
def learnParams(TAO):
    
    #constant parameters
    g = 9.81 #[m/s^2]
    dt = .0013 #[s]

    z = TAO.Target_Object_After[0] - TAO.Target_Object_Before[0] #this is dX which is the measurement
    #at this point, F is what was used during the last trial
    TAO.lh_mu = 2*g*z/((TAO.F*dt)**2)

    TAO.lh_var = TAO.model_var*2*g/((TAO.F*dt)**2)
    TAO.po_mu = (TAO.lh_mu*TAO.pr_var+TAO.pr_mu*TAO.lh_var)/(TAO.pr_var+TAO.lh_var)
    TAO.po_var = TAO.lh_var*TAO.pr_var/(TAO.lh_var+TAO.pr_var)
    TAO.pr_mu = TAO.po_mu
    TAO.pr_var = TAO.po_var

    #calcuate the force required for the next trial
    #force will be reset so at this point it is no longer what was used before
    xDes = (TAO.goalPost_left[0]+TAO.goalPost_right[0])/2-TAO.Target_Object_Current[0] #desired distance
    xError = (TAO.goalPost_left[0]+TAO.goalPost_right[0])/2-TAO.Target_Object_After[0]
    print "lh_mu:"
    print TAO.lh_mu
    print "lh_var:"
    print TAO.lh_var
    print "xDes:"
    print xDes
    print "z:"
    print z
    print "xError:"
    print xError
    TAO.F = math.sqrt(2*xDes*g/(dt**2*TAO.po_mu))

    #print out some info
    print "posterior mu:"
    print TAO.po_mu
    print "posterior var:"
    print TAO.po_var
    print "new force:"
    print TAO.F

    #do the file writing stuff for each iteration
    f = open('./testFiles/testFileNEW.txt','a')
    f.write('Measurement z: '+str(z)+'\n')
    f.write('Error in x: '+str(xError)+'\n')
    f.write('Likelihood mu: '+str(TAO.lh_mu)+'\n')
    f.write('Likelihood mu: '+str(TAO.lh_mu)+'\n')
    f.write('Likelihood var: '+str(TAO.lh_var)+'\n')
    f.write('Posterior mu: '+str(TAO.po_mu)+'\n')
    f.write('Posterior var: '+str(TAO.po_var)+'\n')
    f.write('New desired x: '+str(xDes)+'\n')
    f.write('New Force: '+str(TAO.F)+'\n')   
    f.close()

###################################
# this function updates for the next regression trial
# saves the measurement for each strike
###################################
def saveLastStrike(TAO):
    z = TAO.Target_Object_After[0] - TAO.Target_Object_Before[0] #this is dX which is the measurement
    TAO.zList.append(z)
    print "last z:"
    print z
    
###################################
# saves the measurement for the final strike and calculates the error
###################################
def saveFinalStrike(TAO):
    z = TAO.Target_Object_After[0] - TAO.Target_Object_Before[0] #this is dX which is the measurement

    xError = (TAO.goalPost_left[0]+TAO.goalPost_right[0])/2-TAO.Target_Object_After[0]

    print "Final measurment z:"
    print z
    print "Final error xError:"
    print xError
    
    #do the file writing stuff for this final strike
    f = open('./testFiles/testFileNEW.txt','a')
    f.write('Final measurement z: '+str(z)+'\n')
    f.write('Final error xError: '+str(xError)+'\n') 
    f.close()

###################################
# this function updates for the next regression trial
# assuming features of F F^2 etc up to fourth order
###################################
def doRegression(TAO):
    
    #set up the measurements
    #Fs = [10, 13.56, 14.98, 15.64]
    Fs = TAO.Flist
    #zs = [0.1132, 0.1318, 0.1610, 0.1130]
    zs = TAO.zList

    #make them into arrays
    FsA = array(Fs)
    zsA = array(zs)

    #set up and do the regression
    oC = ones(len(Fs))
    X = c_[oC,FsA,FsA*FsA,FsA*FsA*FsA]
    lam = 1 #this parameter can be tuned
    colX = X.shape[1]
    thR = linalg.solve((lam*eye(colX)+dot(X.T,X)),dot(X.T,zsA))
    
    #find the new force to try to get closest to the goal position
    FvecA = linspace(min(FsA)-10,max(FsA)+10,500) #this gives an array automatically
    Xvec = c_[ones(len(FvecA)),FvecA,FvecA*FvecA,FvecA*FvecA*FvecA]
    yvecA = dot(Xvec,thR)
    
    #this is only named y because it is the y of the regression. 
    #This still means the x dimension in space for the robot.
    yDes = (TAO.goalPost_left[0]+TAO.goalPost_right[0])/2-TAO.Target_Object_Current[0] #desired distance
    
    yDiff = abs(yvecA-yDes)
    index = nonzero(yDiff==min(yDiff))
    Fuse = FvecA[index[0]][0]
    TAO.F = Fuse

    print "List of forces used:"
    print TAO.Flist
    print "List of measurements:"
    print TAO.zList
    print "Desired distance xDes for final strike:"
    print yDes
    print "Force to use next:"
    print Fuse
    print "Ridge regression parameters:"
    print thR
    
    #do the file writing stuff for the regression
    f = open('./testFiles/testFileNEW.txt','a')
    f.write('Measurement list zList: '+str(TAO.zList)+'\n')
    f.write('Force list Flist: '+str(TAO.Flist)+'\n')
    f.write('xDes distance for final strike: '+str(yDes)+'\n')
    f.write('Force to use for final strike: '+str(Fuse)+'\n')
    f.write('Ridge regression parameters: '+str(thR)+'\n')  
    f.close()


###################################
#main function
###################################
if __name__ == '__main__':
    rospy.init_node('ee_impulse_testing_node')
    arm = pr2_utils.arm_control.ArmControl("right_arm")
    resetArm(arm)
    TAO = Table_and_Objects()
    iterNum = 4 #don't need this

    try:
        print "i got here at least"
        print "Initially, prior mu:"
        print TAO.pr_mu
        print "Initially, prior var:"
        print TAO.pr_var
        print "Intially, force:"
        print TAO.F
        for i in range(len(TAO.Flist)):
            f = open('./testFiles/testFileNEW.txt','a')
            f.write('\n\nIteration '+str(i)+':\n\n')
            f.close()

            #set the force for this strike
            TAO.F = TAO.Flist[i]
            print "current force:"
            print TAO.F
            
            #do everything
            updateTargetObject(TAO,0)
            impulse_new(TAO,arm)
            updateTargetObject(TAO,1)
            pickup(TAO,arm)
            resetTargetObject(TAO,arm)
            resetArm(arm)
            saveLastStrike(TAO)
           
        
        #update object position after the reset
        updateTargetObject(TAO,0)
        doRegression(TAO)
        
        #execute the final strike
        impulse_new(TAO,arm)
        updateTargetObject(TAO,1)
        pickup(TAO,arm)
        resetTargetObject(TAO,arm)
        resetArm(arm)

        #save the final strike
        saveFinalStrike(TAO)

    except rospy.ROSInterruptException: pass
