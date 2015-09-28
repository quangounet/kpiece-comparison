#!/usr/bin/env python
from ompl import util
from ompl import base
from ompl import geometric
from ompl import control

from openravepy import Environment, RaveCreateCollisionChecker
from FrictionAndZMPConstraints import (NextControlConstraints,
                                       SampleInPolygon,
                                       CheckConstraintsForConfiguration,
                                       GetLocalVector,
                                       X, Y, Z)
import numpy as np
import time
import random

# These values will be used if not set locally
THRESHOLD = 0.1
PROJECTION_CHOICE = 3
CELLSIZE = 0.1
DISTANCE_WEIGHT = 0.5 # this value is for q
PROPAGATION_STEPSIZE = 0.1
GOAL_BIAS = 0.1
RP_DIM = 3 # dimension of the random projecting space

BORDERFRACTION = 0.5

BOTTLEDIM = [0.03, 0.03, 0.08]
MU = 1.736

# State Space
class MyStateSpace(base.CompoundStateSpace):
    """
    MyStateSpace
    robot : the robot to be used in planning
    N     : the number of joints we want to do planning for
    Note that N <= robot.GetActiveDOF()
    """
    def __init__(self, robot): 
        super(MyStateSpace, self).__init__()
        self.ndof = robot.GetActiveDOF()
        self.dimension = self.ndof * 2

        self.qSpace = base.RealVectorStateSpace(self.ndof)
        qBounds = base.RealVectorBounds(self.ndof)
        qMinLimits = robot.GetDOFLimits()[0]
        qMaxLimits = robot.GetDOFLimits()[1]
        for i in range(self.ndof):
            qBounds.setLow(i, qMinLimits[i])
            qBounds.setHigh(i, qMaxLimits[i])
        self.qdSpace = base.RealVectorStateSpace(self.ndof)
        qdLimits = robot.GetDOFVelocityLimits()
        qdBounds = base.RealVectorBounds(self.ndof)
        for i in range(self.ndof):
            qdBounds.setLow(i, -qdLimits[i])
            qdBounds.setHigh(i, qdLimits[i])

        self.qSpace.setBounds(qBounds)
        self.qdSpace.setBounds(qdBounds)

        self.addSubspace(self.qSpace, 1.0)   # joint value space
        self.addSubspace(self.qdSpace, 1.0)  # joint velocity space

    def getDimension(self):
        return self.dimension
    
    def distance(self, state1, state2):
        q1 = np.zeros(self.ndof)
        qd1 = np.zeros(self.ndof)
        q2 = np.zeros(self.ndof)
        qd2 = np.zeros(self.ndof)
        for i in range(self.ndof):
            q1[i] = state1[0][i]
            qd1[i] = state1[1][i]
            q2[i] = state2[0][i]
            qd2[i] = state2[1][i]
        # Euclidean distance.
        # This passes self.sanityChecks().
        W1 = DISTANCE_WEIGHT
        W2 = 1 - DISTANCE_WEIGHT
        return np.sqrt(W1*np.dot(q1 - q2, q1 - q2) + 
                       W2*np.dot(qd1 - qd2, qd1 - qd2))


# Control Space (joint acceleration space)
class MyControlSpace(control.RealVectorControlSpace):
    def __init__(self, sSpace, robot):
        self.robot = robot
        self.ndof = self.robot.GetActiveDOF()
        super(MyControlSpace, self).__init__(sSpace, self.ndof)

        qddBounds = base.RealVectorBounds(self.ndof)
        qddLimits = robot.GetDOFAccelerationLimits()
        for i in range(self.ndof):
            qddBounds.setLow(i, -qddLimits[i])
            qddBounds.setHigh(i, qddLimits[i])
        self.setBounds(qddBounds)


# State Validity Checker
class MyStateValidityChecker(base.StateValidityChecker):
    """
    MyStateValidityChecker
    """
    def __init__(self, spaceInformation, robot):
        # Inheritted from base.StateValidityChecker
        super(MyStateValidityChecker, self).__init__(spaceInformation)
        self.si_ = spaceInformation
        self.robot = robot
        self.env = self.robot.GetEnv()
        self.ndof = self.robot.GetActiveDOF()

    def isValid(self, state):
        # Check joint limits
        withinbounds = self.si_.satisfiesBounds(state)
        if not withinbounds:
            return False

        # Check friction and ZMP constraints
        """
        Since we already select an admissible control for propagation,
        we do not have to check states along the way, provided that
        the number of propagation steps is not large.
        """
        passed = True
        q = np.zeros(6)
        qd = np.zeros(6)
        for i in range(self.ndof):
            q[i] = state[0][i]
            qd[i] = state[1][i]
        if not passed:
            return False

        # print "checking validity of the state (q, qd)"
        # print "q = {0}\nqd = {1}\n".format(q, qd)

        # Check collision
        with self.robot.GetEnv():
            self.robot.SetDOFValues(q)
            incollision = (self.env.CheckCollision(self.robot) or 
                           self.robot.CheckSelfCollision())
        if incollision:
            return False
        
        return True


# Projection Evaluator
class MyProjection(base.ProjectionEvaluator):
    """
    This class defines a projection to be used in planning. The
    inclusion of the norm of joint velocities is suggested in the
    KPIECE paper.
    
    Projection 1: projecting a state (q, qd) into (p, |qd|)
    Projection 2: projecting a state (q, qd) into (q, |qd|)
    Projection 3: Random projection into 3 dimensional space
    
    """
    def __init__(self, space, robot, projection_choice, cellSizes = None):
        # Inheritted from base.ProjectionEvaluator
        super(MyProjection, self).__init__(space)
        self.robot = robot
        self.ndof = self.robot.GetActiveDOF()
        
        self.PROJECTION_CHOICE = projection_choice
        if self.PROJECTION_CHOICE == 1:
            # tool tip (3) + norm of velocities (1)
            self.manip = self.robot.GetManipulator('Flange')
            self.dim = 4
        elif self.PROJECTION_CHOICE == 2:
            # projection 2 : (ndof) + norm of velocities (1)
            self.dim = self.ndof + 1
        elif self.PROJECTION_CHOICE == 3:
            # random projection
            self.dim = RP_DIM
            self.projectionMatrix = CreateRandomOrthonormalMatrix(2*self.ndof, self.dim)
            print "projection matrix:"
            print self.projectionMatrix
        else:
            raise ValueError
        
        if cellSizes is None:
            self.cellSizes_ = util.vectorDouble()
            self.cellSizes_.extend(CELLSIZE*np.ones(self.dim))
        else:
            self.cellSizes_ = util.vectorDouble()
            self.cellSizes_.extend(cellSizes*np.ones(self.dim))
        self.setCellSizes(self.cellSizes_)

    # Get projected space dimension
    def getDimension(self):
        return self.dim

    def project(self, state, projection):
        # Redirect to a specific projection function
        if self.PROJECTION_CHOICE == 1:
            return self.project1(state, projection)
        elif self.PROJECTION_CHOICE == 2:
            return self.project2(state, projection)
        elif self.PROJECTION_CHOICE == 3:
            return self.project3(state, projection)

    # Tool-tip position in the workspace as a projection.
    def project1(self, state, projection):
        q = np.zeros(6)
        qd = np.zeros(6)
        for i in range(self.ndof):
            q[i] = state[0][i]
            qd[i] = state[1][i]
        with self.robot.GetEnv():
            self.robot.SetDOFValues(q)
            p = self.manip.GetEndEffectorTransform()[0:3, 3]
            projection[0] = p[0]
            projection[1] = p[1]
            projection[2] = q[2]
            projection[3] = np.linalg.norm(qd)
        # handle.append(env.plot3(points = p, pointsize = 5, colors = [0, 1, 0]))

    # Projecting into no-velocity space
    def project2(self, state, projection):
        q = np.zeros(6)
        qd = np.zeros(6)
        for i in range(self.ndof):
            q[i] = state[0][i]
            qd[i] = state[1][i]
            projection[i] = q[i]
        projection[self.ndof] = np.linalg.norm(qd)
        
    # Random projection
    def project3(self, state, projection):
        q = np.zeros(6)
        qd = np.zeros(6)
        for i in range(self.ndof):
            q[i] = state[0][i]
            qd[i] = state[1][i]
        s = np.hstack((q[0:self.ndof], qd[0:self.ndof])) # the state
        proj = np.dot(self.projectionMatrix, s)
        for i in range(self.dim):
            projection[i] = proj[i]


# Control Sampler
class MyControlSampler(control.ControlSampler):
    def __init__(self, cSpace, robot):
        super(MyControlSampler, self).__init__(cSpace)
        self.RNG = random.SystemRandom()
        self.robot = robot
        self.ndof = self.robot.GetActiveDOF()
        self.space_ = cSpace
        self.controlBounds = []
        l = np.zeros(self.ndof)
        h = np.zeros(self.ndof)
        bounds = self.space_.getBounds()
        for i in xrange(self.ndof):
            l[i] = bounds.low[i]
            h[i] = bounds.high[i]
        self.controlBounds.append(l)
        self.controlBounds.append(h)

    # # Given 'state', sample 'control'
    # def sample(self, newcontrol):
    #     print "sample control",
    #     for i in range(self.ndof):
    #         h = self.space_.getBounds().high[i]
    #         l = self.space_.getBounds().low[i]
    #         newcontrol[i] = self.RNG.uniform(l, h)
    #         # print newcontrol[i], 
    #     # print '\n'

    # KPIECE planner uses only sampleNext.
    def sampleNext(self, newcontrol, prevcontrol, prevstate):

        # print "sample control with constraints"
        # Obtain the current state
        q = np.zeros(6)
        qd = np.zeros(6)
        qdd = np.zeros(6)
        for i in range(self.ndof):
            q[i] = prevstate[0][i]
            qd[i] = prevstate[1][i]
            qdd[i] = prevcontrol[i]
            
        configlist = [q, qd, qdd]

        # passed = CheckConstraintsForConfiguration(self.robot, configlist, BOTTLEDIM, MU)
        # try:
        #     assert(passed)
        # except AssertionError:
        #     dt = 0.05
        #     qdprev = qd - dt*qdd
        #     qprev = q - dt*qdprev - 0.5*dt*dt*qdd
        #     passed = CheckConstraintsForConfiguration(self.robot, [qprev, qdprev, qdd], BOTTLEDIM, MU)
        #     assert(passed)
        ##########
        qddBounds = self.robot.GetDOFAccelerationLimits()
        newqdd = np.zeros(6)
        for i in xrange(6):
            lim = qddBounds[i]
            newqdd[i] = self.RNG.uniform(-lim, lim)
        i_ = 0
        nTries = 2000
        while ((not (CheckConstraintsForConfiguration(self.robot, [q, qd, newqdd], BOTTLEDIM, MU)))
               and i_ < nTries):
            i_ += 1
            newqdd = np.zeros(6)
            for i in xrange(6):
                lim = qddBounds[i]
                newqdd[i] = self.RNG.uniform(-lim, lim)
        if not (CheckConstraintsForConfiguration(self.robot, [q, qd, newqdd], BOTTLEDIM, MU)):
            print "No feasible control found after {0} tries.".format(nTries)
            print "Set the new control to be cSpace.nullControl"
            self.space_.nullControl(newcontrol)
            print newcontrol[0], newcontrol[1], newcontrol[2], newcontrol[3], newcontrol[4], newcontrol[5]
        else:
            for i in xrange(self.ndof):
                newcontrol[i] = newqdd[i]
            print newcontrol[0], newcontrol[1], newcontrol[2], newcontrol[3], newcontrol[4], newcontrol[5]
        
        # print "=========="
        # print "newqdd = {0}".format(VectorToString(newqdd))
        ##########
        
        # passed = CheckConstraintsForConfiguration(self.robot, configlist, BOTTLEDIM, MU)
        # print "Check constraints = {0}".format(passed)
        
        # (A, b) = NextControlConstraints(self.robot, configlist, BOTTLEDIM, MU)
        # # The next control, qdd, has to satisfy A qdd <= b.

        # newqdd = SampleInPolygon(A, b, self.controlBounds, prevcontrol = qdd)

        # ##########
        # with self.robot:
        #     tray = self.robot.GetLink('tray')
        #     bottle = self.robot.GetGrabbed()[0].GetLink('bottle_lowerpart')
        #     nlink = len(self.robot.GetLinks())
        #     bottlelinkindex = nlink - 1
            
        #     g = self.robot.GetEnv().GetPhysicsEngine().GetGravity()
        #     self.robot.SetDOFValues(q)
        #     self.robot.SetDOFVelocities(qd)
        #     pt = tray.GetGlobalCOM()
        #     pb = bottle.GetGlobalCOM()
            
        #     # Jacobian
        #     Jv = self.robot.CalculateJacobian(bottlelinkindex, pb)
        #     Jw = self.robot.CalculateAngularVelocityJacobian(bottlelinkindex)
        
        #     # Acceleration of the tray
        #     acc = self.robot.GetLinkAccelerations(newqdd)[tray.GetIndex()]
        #     at = np.array(acc[0:3])      # linear
        #     alpha = np.array(acc[3:6])   # angular
        #     # Angular velocity of the tray (and thus of the bottle)
        #     w = self.robot.GetLinkVelocities()[tray.GetIndex()][3:6]
            
        #     # Linear acceleration of the bottle
        #     ab = at + np.cross(w, np.cross(w, pb - pt)) + np.cross(alpha, pb - pt)

        #     nx = GetLocalVector(bottle, X)
        #     ny = GetLocalVector(bottle, Y)
        #     nz = GetLocalVector(bottle, Z)
        #     m = bottle.GetMass()
        #     u2 = MU/np.sqrt(2)
            
        #     A1 = np.vstack((-m*nz,
        #                      -u2*nz + nx,
        #                      -u2*nz - nx,
        #                      -u2*nz + ny,
        #                      -u2*nz - ny))
        #     print "Check Constraint : {0}".format(np.alltrue(np.dot(A1, ab - g) <= 0))

        # ##########

        # if newqdd is None:
        #     print "invalid prevcontrol"
        #     self.space_.nullControl(newcontrol)
        # else:
        #     configlist = [q, qd, newqdd]
        #     passed1 = CheckConstraintsForConfiguration(self.robot, configlist, BOTTLEDIM, MU)
        #     passed2 = np.alltrue(np.dot(A, newqdd) <= b)
        #     print "Check constraints 2 = {0}, {1}\n".format(passed1, passed2)
        #     if not (passed1 and passed2):
        #         print "Fatal error"
        #         print "q = " + VectorToString(q)
        #         print "qd = " + VectorToString(qd)
        #         print "qdd = " + VectorToString(qdd)
        #         print "newqdd = " + VectorToString(newqdd)
        #         raw_input()
        #     for i in range(self.ndof):
        #         newcontrol[i] = newqdd[i]


# Control Sampler Allocator
def MyControlSamplerAllocator(cSpace):
    return MyControlSampler(cSpace, cSpace.robot)
        
        
# State Propagator
class MyStatePropagator(control.StatePropagator):
    def __init__(self, spaceInformation, robot):
        super(MyStatePropagator, self).__init__(spaceInformation)
        self.robot = robot
        self.ndof = self.robot.GetActiveDOF()

    # propagate propagates the state according to a forward
    # integration rules.
    def propagate(self, state, control, duration, result):
        q = np.zeros(6)
        qd = np.zeros(6)
        qdd = np.zeros(6)
        qnext = np.zeros(6)
        qdnext = np.zeros(6)
        for i in range(self.ndof):
            q[i] = state[0][i]
            qd[i] = state[1][i]
            qdd[i] = control[i]
            
            # Integrate forward
            result[0][i] = q[i] + qd[i]*duration + 0.5*qdd[i]*(duration**2)
            result[1][i] = qd[i] + (qdd[i]*duration)
            qnext[i] = result[0][i]
            qdnext[i] = result[1][i]
            
        # print "dt     = {0}".format(duration)
        # print "q      = {0}".format(VectorToString(q))
        # print "qnext  = {0}".format(VectorToString(qnext))
        # print "qd     = {0}".format(VectorToString(qd))
        # print "qdnext = {0}".format(VectorToString(qdnext))


# Utilities (for random projection)
def Project(u, v):
    """
    Project projects v onto the line spanned by u.
    """
    return (np.dot(v, u) / np.dot(u, u)) * u    


def GrammSchmidt(V):
    """
    GrammSchmidt orthonormalizes columns of V.
    """
    U = np.array(V)
    for i in xrange(1, V.shape[1]):
        for j in xrange(i):
            U[:,i] -= Project(U[:, j], V[:, i])
    # Normalize Column
    den = (U**2).sum(axis = 0)**0.5
    E = U/den
    return E


def CreateRandomOrthonormalMatrix(dimFrom, dimTo):
    """
    CreateRandomOrthonormalMatrix creates a random orthonormal matrix
    V such that V^T : R^{dimFrom} -> R^{dimTo}.

    The notation (using V^T) follows from the KPIECE paper.
    """
    P = []
    RNG = random.SystemRandom()
    for i in xrange(dimTo):
        temp = []
        for j in xrange(dimFrom):
            temp.append(RNG.normalvariate(0, 1))
        P.append(np.asarray(temp))
    V = GrammSchmidt(np.asarray(P).T)
    return V.T    
##############################

        
# Planning
def Plan(robot, startvect, goalvect, runningtime, 
         threshold = THRESHOLD, 
         projection_choice = PROJECTION_CHOICE,
         cellsize = CELLSIZE,
         distance_weight = DISTANCE_WEIGHT,
         propagation_stepsize = PROPAGATION_STEPSIZE,
         goal_bias = GOAL_BIAS,
         rp_dim = RP_DIM,
         N = 2):
    
    ndof = N
    robot.SetActiveDOFs(range(ndof))
    print "MyKPIECE1_2 : bottle-and-tray"
    
    # Construct a state-space
    sSpace = MyStateSpace(robot)
    
    # Construct a control-space
    cSpace = MyControlSpace(sSpace, robot)

    # Create a control sampler
    calloc = control.ControlSamplerAllocator(MyControlSamplerAllocator)
    cSpace.setControlSamplerAllocator(calloc)

    # Create a control space information
    cInfo = control.SpaceInformation(sSpace, cSpace)

    # Create a state propagator
    cInfo.setPropagationStepSize(propagation_stepsize)
    cInfo.setMinMaxControlDuration(1, 1)
    # cInfo.setMinMaxControlDuration(1, 10) # default
    propagator = MyStatePropagator(cInfo, robot)
    cInfo.setStatePropagator(propagator)

    # Create a state validity checker
    vc = MyStateValidityChecker(cInfo, robot)
    cInfo.setStateValidityChecker(vc)

    cInfo.setup()
    
    # SimpleSetup
    ss = control.SimpleSetup(cInfo)
    
    # Create start and goal
    start = base.State(sSpace)
    goal = base.State(sSpace)
    for i in range(ndof):
        start()[0][i] = startvect[i]
        goal()[0][i] = goalvect[i]
    for i in range(ndof, 2*ndof):
        start()[1][i - ndof] = startvect[i]
        goal()[1][i - ndof] = goalvect[i]

    ss.setStartAndGoalStates(start, goal, threshold)

    # Create a planner
    planner = control.KPIECE1(cInfo)
    
    # Create and set ProjectionEvaluator
    proj = MyProjection(sSpace, robot, projection_choice, cellsize)
    planner.setProjectionEvaluator(proj)

    planner.setGoalBias(goal_bias) # the default value is 0.05
    ##planner.setBorderFraction(BORDERFRACTION)
    planner.setup()
    ss.setPlanner(planner)
    if ss.solve(runningtime) and ss.haveExactSolutionPath():
        print "Problem Solved"
    else:
        print "The planner has not found any solution in {0} s.".\
        format(ss.getLastPlanComputationTime())
    return ss


def VectorToString(q):
    res = "np.array([ "
    separator = ""
    for qi in q:
        res += separator
        res += str(qi)
        separator = ", "
    res += "])"
    return res
