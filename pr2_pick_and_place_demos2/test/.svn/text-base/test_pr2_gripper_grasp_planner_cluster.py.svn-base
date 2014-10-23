#!/usr/bin/python
'''
Test client for point_cluster_grasp_planner.  
Requires tabletop_node.launch to be running (in tabletop_object_detector).  
Add Markers of topic 'grasp_markers' to see the poses being tested.
'''

from __future__ import division
import roslib
roslib.load_manifest('pr2_pick_and_place_demos')
import rospy
from object_manipulation_msgs.srv import FindClusterBoundingBox, FindClusterBoundingBoxRequest
from tabletop_object_detector.srv import TabletopDetection, TabletopDetectionRequest
from object_manipulation_msgs.srv import GraspPlanning, GraspPlanningRequest
from visualization_msgs.msg import Marker
import tf.transformations
import numpy
import scipy
import time
import object_manipulator.draw_functions as draw_functions
from object_manipulator.convert_functions import *


#call the tabletop object detector to get the table and clusters in the scene
def call_object_detector():

    req = TabletopDetectionRequest()
    req.return_clusters = 1
    req.return_models = 0
    service_name = "/object_detection"
    rospy.loginfo("waiting for object_detection service")
    rospy.wait_for_service(service_name)
    serv = rospy.ServiceProxy(service_name, TabletopDetection)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling object_detection: %s"%e)  
        return 0
    return (res.detection.table, res.detection.clusters)


#call plan_point_cluster_grasp to get candidate grasps for a cluster
def call_plan_point_cluster_grasp(cluster):
    
    req = GraspPlanningRequest()
    req.target.reference_frame_id = "/base_link"
    req.target.cluster = cluster
    req.arm_name = "right_arm"
    req.collision_support_surface_name = "table"
    service_name = "plan_point_cluster_grasp"
    rospy.loginfo("waiting for plan_point_cluster_grasp service")
    rospy.wait_for_service(service_name)
    rospy.loginfo("service found")
    serv = rospy.ServiceProxy(service_name, GraspPlanning)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling plan_point_cluster_grasp: %s"%e)  
        return 0
    if res.error_code.value != 0:
        return []
    
    return res.grasps


#call evaluate_point_cluster_grasps to evaluate grasps on a cluster
def call_evaluate_point_cluster_grasps(cluster, grasps):

    req = GraspPlanningRequest()
    req.target.reference_frame_id = "/base_link"
    req.target.cluster = cluster
    req.arm_name = "right_arm"
    req.collision_support_surface_name = "table"
    req.grasps_to_evaluate = grasps
    service_name = "evaluate_point_cluster_grasps"
    rospy.loginfo("waiting for evaluate_point_cluster_grasps service")
    rospy.wait_for_service(service_name)
    rospy.loginfo("service found")
    serv = rospy.ServiceProxy(service_name, GraspPlanning)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling evaluate_point_cluster_grasps: %s"%e)  
        return 0
    if res.error_code.value != 0:
        return []
    
    probs = [grasp.success_probability for grasp in res.grasps]
    return probs


#call find_cluster_bounding_box to get the bounding box for a cluster
def call_find_cluster_bounding_box(cluster):
    
    req = FindClusterBoundingBoxRequest()
    req.cluster = cluster
    service_name = "find_cluster_bounding_box"
    rospy.loginfo("waiting for find_cluster_bounding_box service")
    rospy.wait_for_service(service_name)
    rospy.loginfo("service found")
    serv = rospy.ServiceProxy(service_name, FindClusterBoundingBox)
    try:
        res = serv(req)
    except rospy.ServiceException, e:
        rospy.logerr("error when calling find_cluster_bounding_box: %s"%e)  
        return 0
    if not res.error_code:
        return (res.pose, res.box_dims)
    else:
        return (None, None)


if __name__ == "__main__":

    #initialize the node, tf listener and broadcaster, and rviz drawing helper class
    rospy.init_node('test_point_cluster_grasp_planner', anonymous=True)
    tf_broadcaster = tf.TransformBroadcaster()
    tf_listener = tf.TransformListener()
    draw_functions = draw_functions.DrawFunctions('grasp_markers')

    #set params for planner
    #rospy.set_param("point_cluster_grasp_planner/overhead_grasps_only", True)
    #rospy.set_param("point_cluster_grasp_planner/include_high_point_grasps", False)
    #rospy.set_param("point_cluster_grasp_planner/pregrasp_just_outside_box", True)
    rospy.set_param("point_cluster_grasp_planner/overhead_grasps_only", False)
    rospy.set_param("point_cluster_grasp_planner/include_high_point_grasps", True)
    rospy.set_param("point_cluster_grasp_planner/pregrasp_just_outside_box", False)

    #call the tabletop object detector to detect clusters on the table
    (table, clusters) = call_object_detector()
    print "table:\n", table

    #keep going until the user says to quit (or until Ctrl-C is pressed)
    while(not rospy.is_shutdown()):

        #for each cluster in view in turn
        for cluster in clusters:
            draw_functions.clear_grasps(num=1000)

            #get the bounding box for this cluster and draw it in blue
            (box_pose, box_dims) = call_find_cluster_bounding_box(cluster)
            if box_pose == None:
                continue
            box_mat = pose_to_mat(box_pose.pose)
            box_ranges = [[-box_dims.x/2, -box_dims.y/2, -box_dims.z/2],
                          [box_dims.x/2, box_dims.y/2, box_dims.z/2]]
            draw_functions.draw_rviz_box(box_mat, box_ranges, 'base_link', \
                                         ns = 'bounding box', \
                                         color = [0,0,1], opaque = 0.25, duration = 60)
            
            print "Run the blue cluster? Enter y to run, enter to go to the next, q to exit"
            c = raw_input()

            if c == 'y':
                draw_functions.clear_rviz_points('grasp_markers')

                #call the grasp planner
                grasps = call_plan_point_cluster_grasp(cluster)
                grasp_poses = [grasp.grasp_pose for grasp in grasps]
                #pregrasp_poses = [grasp.pre_grasp_pose for grasp in grasps]

                #print the returned success probs
                original_probs = [grasp.success_probability for grasp in grasps]
                print "original probs:", pplist(original_probs)

                #zero out the original success probabilities
                for grasp in grasps:
                    grasp.success_probability = 0

                #ask the grasp planner to evaluate the grasps
                probs = call_evaluate_point_cluster_grasps(cluster, grasps)
                print "new probs:     ", pplist(probs)

                #draw the resulting grasps (all at once, or one at a time)
                print "number of grasps returned:", len(grasps)
                print "drawing grasps: press p to pause after each one in turn, enter to show all grasps at once"
                c = raw_input()
                if c == 'p':
                    #for (pregrasp_pose, grasp_pose) in zip(pregrasp_poses, grasp_poses):
                    #    draw_functions.draw_grasps([pregrasp_pose, grasp_pose], cluster.header.frame_id, pause = 1)
                    draw_functions.draw_grasps(grasp_poses, cluster.header.frame_id, pause = 1)
                else:
                    draw_functions.draw_grasps(grasp_poses, cluster.header.frame_id, pause = 0)
                print "done drawing grasps, press enter to continue"
                raw_input()

                #clear out the grasps drawn
                draw_functions.clear_grasps(num = 1000)

            elif c == 'q':
                break
        else:
            continue
        break

