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

#this is a test of the emergency broadcast system

###################################
# this function updates for the next regression trial
# assuming features of F F^2 etc up to fourth order
###################################
def doRegression():

    Fs = [10, 13.56, 14.98, 15.64]
    zs = [0.1132, 0.1318, 0.1610, 0.1130]
    FsA = array(Fs)
    oC = ones(len(Fs))
    X = c_[oC,FsA,FsA*FsA,FsA*FsA*FsA]
    lam = 1
    colX = X.shape[1]
    zsA = array(zs)
    thR = linalg.solve((lam*eye(colX)+dot(X.T,X)),dot(X.T,zsA))
    FvecA = linspace(min(FsA)-10,max(FsA)+10,500) #this gives an array automatically
    Xvec = c_[ones(len(FvecA)),FvecA,FvecA*FvecA,FvecA*FvecA*FvecA]
    yvecA = dot(Xvec,thR)
    yDes = 0.040

    yDiff = abs(yvecA-yDes)
    index = nonzero(yDiff==min(yDiff))
    Fuse = FvecA[index[0]][0]

    print "Desired distance yDes:"
    print yDes
    print "Force to use next:"
    print Fuse
    print "Ridge regression:"
    print thR

###################################
#main function
###################################
if __name__ == '__main__':
    doRegression()
