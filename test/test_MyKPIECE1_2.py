import sys
sys.path.append('../src/')
from openravepy import Environment, RaveCreateCollisionChecker
from MyKPIECE1_2 import *
import FrictionAndZMPConstraints
import numpy as np
import time
import random


NDOF = 6
THRESHOLD = 0.1
PROJECTION_CHOICE = 3
CELLSIZE = 0.05
DISTANCE_WEIGHT = 0.5 # this value is for joint space
PROPAGATION_STEPSIZE = 0.05
GOAL_BIAS = 0.05
RP_DIM = 3 # dimension of the random projecting space

OPENING_HEIGHT = 0.5
EASY_MODE = True # if EASY_MODE, there will be no ceiling

BOTTLEDIM = [0.03, 0.03, 0.08]
MU = 1.736

env = Environment()
env.SetViewer('qtcoin')
env.Load('../xml/env.xml')
collisionChecker = RaveCreateCollisionChecker(env, 'ode')
env.SetCollisionChecker(collisionChecker)

#------------------------- Environment Setting -------------------------#
robot = env.GetRobots()[0]
floor = env.GetKinBody('floor')
box1 = env.GetKinBody('box1')
box2 = env.GetKinBody('box2')
box3 = env.GetKinBody('box3')
box4 = env.GetKinBody('box4')
box5 = env.GetKinBody('box5')
bottle = env.GetKinBody('bottle')

floor.Enable(False)

box1.SetTransform(np.array([[  0.0,  1.0,  0.0,  0.00], 
                            [ -1.0,  0.0,  0.0,  0.38], 
                            [  0.0,  0.0,  1.0,  0.40], 
                            [  0.0,  0.0,  0.0,  1.00]]))

box2.SetTransform(np.array([[  1.0,  0.0,  0.0,  0.00], 
                            [  0.0,  1.0,  0.0, -0.285], 
                            [  0.0,  0.0,  0.0,  OPENING_HEIGHT + 0.32 + 0.2], 
                            [  0.0,  0.0,  0.0,  1.00]]))

box3.SetTransform(np.array([[  1.0,  0.0,  0.0,  0.00], 
                            [  0.0,  1.0,  0.0, -0.586], 
                            [  0.0,  0.0,  1.0,  0.16], 
                            [  0.0,  0.0,  0.0,  1.00]]))

box4.SetTransform(np.array([[  1.0,  0.0,  0.0,  0.00], 
                            [  0.0,  1.0,  0.0,  0.101],
                            [  0.0,  0.0,  1.0,  0.75],
                            [  0.0,  0.0,  0.0,  1.00]]))

box5.SetTransform(np.array([[  1.0,  0.0,  0.0,  0.00], 
                            [  0.0,  1.0,  0.0, -0.876], 
                            [  0.0,  0.0,  1.0,  0.40], 
                            [  0.0,  0.0,  0.0,  1.00]]))

if EASY_MODE:
    box2.SetVisible(False)
    box2.Enable(False)
    box4.SetVisible(False)
    box4.Enable(False)
    box5.SetVisible(False)
    box5.Enable(False)

#------------------------- Robot Setting -------------------------#
T = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
robot.SetTransform(T)

qddLimits = robot.GetDOFAccelerationLimits()
robot.SetDOFAccelerationLimits(0.4*qddLimits)

bottle.SetTransform(robot.GetLink('tray').GetTransform())
robot.Grab(bottle)

startvect = np.array([ -1.57599994e+00,   1.37400000e+00,   1.39095846e-10,
                       -7.61664885e-08,  -1.36751533e+00,   1.61500001e+00,
                        0, 0, 0, 0, 0, 0])
# goalvect = np.array([ -1.18390877e+00,   1.56777797e+00,  -4.72570831e-01,  
#                       -1.67003522e-07,  -1.05627525e+00,   1.61499996e+00,
#                        0., 0., 0., 0., 0., 0.])
goalvect = np.array([ -0.74269128,  1.34999999,  0.10000005, 
                      -0.00570361, -1.40754354,  1.60000003,
                       0., 0., 0., 0., 0., 0.])

robot.SetDOFValues(startvect[0:6])

max_running_time = 3600.*3.
ss = Plan(robot, startvect, goalvect, max_running_time, 
          threshold = THRESHOLD, 
          projection_choice = PROJECTION_CHOICE, 
          cellsize = CELLSIZE,
          distance_weight = DISTANCE_WEIGHT,
          propagation_stepsize = PROPAGATION_STEPSIZE, 
          goal_bias = GOAL_BIAS,
          rp_dim = RP_DIM, 
          N = NDOF)

"""
sol = ss.getSolutionPath()
sol.interpolate()
M = sol.printAsMatrix()
M = M.strip().split('\n')
raw_input('Press any key to visualize the solution')
for i in range(len(M)):
    w = M[i]
    w1 = [float(s) for s in w.split()]
    if NDOF < 6:
        robot.SetDOFValues(np.hstack((w1[0:NDOF], np.zeros(6 - NDOF))))
    else:
        robot.SetDOFValues(w1[0:6])
    time.sleep(0.1)

si = ss.getSpaceInformation()
g = ss.getGoal().getState()
f = sol.getStates()[-1]
print "Running time: {0} s.".format(ss.getLastPlanComputationTime())
print "Threshold = {0}".format(THRESHOLD)
print "Distance to goal = {0}".format(si.distance(f, g))

"""
