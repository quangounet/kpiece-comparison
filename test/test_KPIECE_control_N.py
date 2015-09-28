#!/usr/bin/env python
from ompl import util
from ompl import base
from ompl import geometric
from ompl import control

from openravepy import Environment, RaveCreateCollisionChecker
import numpy as np
import time
import random


# Planning for the first N joint(s)
N = 3
RUNNING_TIME = 180
THRESHOLD = 0.1
PROJECTION_CHOICE = 3
CELLSIZE = 0.1
DISTANCE_WEIGHT = 0.5 # this value is for q
PROPAGATION_STEPSIZE = 0.1
GOAL_BIAS = 0.05
RP_DIM = 3 # dimension of the random projecting space
handle = []

# State Space
class MyStateSpace(base.CompoundStateSpace):
    def __init__(self, robot): 
        super(MyStateSpace, self).__init__()
        self.ndof = N
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
        self.ndof = N
        self.robot = robot
        self.SCALINGFACTOR = 0.3 # to scale the acceleration down
        super(MyControlSpace, self).__init__(sSpace, self.ndof)

        qddBounds = base.RealVectorBounds(self.ndof)
        qddLimits = robot.GetDOFAccelerationLimits() * self.SCALINGFACTOR
        for i in range(self.ndof):
            qddBounds.setLow(i, -qddLimits[i])
            qddBounds.setHigh(i, qddLimits[i])
        self.setBounds(qddBounds)



# State Validity Checker
class MyStateValidityChecker(base.StateValidityChecker):
    def __init__(self, spaceInformation, robot):
        # Inheritted from base.StateValidityChecker
        super(MyStateValidityChecker, self).__init__(spaceInformation)
        self.spaceInformation = spaceInformation
        self.robot = robot
        self.env = self.robot.GetEnv()
        self.ndof = N

    def isValid(self, state):
        # TODO: add fritcion constraints checking here        
        # Check joint limits
        withinbounds = self.spaceInformation.satisfiesBounds(state)
        if not withinbounds:
            return False

        # Check friction and ZMP constraints
        passed = True
        q = np.zeros(6)
        qd = np.zeros(6)
        for i in range(self.ndof):
            q[i] = state[0][i]
            qd[i] = state[1][i]

        if not passed:
            return False

        # Check collision
        robot.SetDOFValues(q)
        incollision = env.CheckCollision(robot) or robot.CheckSelfCollision()
        if incollision:
            return False
        
        return True


# Projection Evaluator
class MyProjection(base.ProjectionEvaluator):
    """
    This class defines a projection to be used in planning. The
    inclusion of the norm of joint velocities is suggested in the
    KPIECE paper.
    """
    def __init__(self, space, robot, cellSizes = None):
        # Inheritted from base.ProjectionEvaluator
        super(MyProjection, self).__init__(space)
        self.robot = robot
        self.manip = self.robot.GetManipulator('Flange')
        self.ndof = N
        if PROJECTION_CHOICE == 1:
            # tool tip (3) + norm of velocities (1)
            self.dim = 4
        elif PROJECTION_CHOICE == 2:
            # projection 2 : (ndof) + norm of velocities (1)
            self.dim = self.ndof + 1
        elif PROJECTION_CHOICE == 3:
            # random projection
            self.dim = RP_DIM
            self.projectionMatrix = CreateRandomOrthonormalMatrix(2*self.ndof, self.dim)
            # print "projection matrix:"
            # print self.projectionMatrix
        else:
            raise ValueError
        if cellSizes is None:
            self.cellSizes_ = util.vectorDouble()
            self.cellSizes_.extend(CELLSIZE*np.ones(self.dim))
        else:
            self.cellSizes_ = cellSizes
        self.setCellSizes(self.cellSizes_)

    # Get projected space dimension
    def getDimension(self):
        return self.dim

    def project(self, state, projection):
        if PROJECTION_CHOICE == 1:
            return self.project1(state, projection)
        elif PROJECTION_CHOICE == 2:
            return self.project2(state, projection)
        elif PROJECTION_CHOICE == 3:
            return self.project3(state, projection)

    # Tool-tip position in the workspace as a projection.
    def project1(self, state, projection):
        q = np.zeros(6)
        qd = np.zeros(6)
        for i in range(self.ndof):
            q[i] = state[0][i]
            qd[i] = state[1][i]
        with self.robot:
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
        self.ndof = N

    # Given 'state', sample 'control'
    def sample(self, control, state):
        print "sample control", 
        for i in range(self.ndof):
            h = self.space_.getBounds().high[i]
            l = self.space_.getBounds().low[i]
            control[i] = self.RNG.uniform(l, h)
        #     print control[i]
        # print '\n'

    # KPIECE planner uses only sampleNext.
    def sampleNext(self, newcontrol, prevcontrol, prevstate):
        pass


# State Propagator
class MyStatePropagator(control.StatePropagator):
    def __init__(self, spaceInformation, robot):
        super(MyStatePropagator, self).__init__(spaceInformation)
        self.robot = robot
        self.ndof = N

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
            
        # self.robot.SetDOFValues(q)
        # time.sleep(0.01)
        # print "duration", duration
        # print "qdd =", qdd
        # print "q = {0},\nqnext = {1},\nqd = {2},\nqdnext = {3}\n".format\
        # (q, qnext, qd, qdnext)


# Utilities
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


# Planning
def Plan(robot, startvect, goalvect, runningtime):
    ndof = N
    
    # Construct a state-space
    sSpace = MyStateSpace(robot)
    
    # Construct a control-space
    cSpace = MyControlSpace(sSpace, robot)
    cInfo = control.SpaceInformation(sSpace, cSpace)

    # Create a state propagator
    cInfo.setPropagationStepSize(PROPAGATION_STEPSIZE)
    cInfo.setMinMaxControlDuration(1, 20)    
    propagator = MyStatePropagator(cInfo, robot)
    cInfo.setStatePropagator(propagator)

    # Create a state validity checker
    vc = MyStateValidityChecker(cInfo, robot)
    cInfo.setStateValidityChecker(vc)

    # Create a control sampler
    csampler = MyControlSampler(cSpace, robot)

    cInfo.setup()
    
    # SimpleSetup
    ss = control.SimpleSetup(cInfo)
    
    # Create start and goal
    start = base.State(sSpace)
    goal = base.State(sSpace)
    for i in range(2 * ndof):
        start[i] = startvect[i]
        goal[i] = goalvect[i]

    ss.setStartAndGoalStates(start, goal, THRESHOLD)

    # Create a planner
    planner = control.KPIECE1(cInfo)
    
    # Create and set ProjectionEvaluator
    proj = MyProjection(sSpace, robot)
    planner.setProjectionEvaluator(proj)

    planner.setGoalBias(GOAL_BIAS) # the default value is 0.05
    planner.setup()
    ss.setPlanner(planner)
    if ss.solve(runningtime):
        print "Problem Solved"
        return ss


def RunTest(robot, startvect, goalvect, runningtime, n):
    good = 0.0
    time = 0.0
    for i in range(n):
        ss = Plan(robot, startvect, goalvect, runningtime)
        if ss.haveExactSolutionPath():
            good = good + 1
            time += ss.getLastPlanComputationTime()
    avgtime = time/float(good)
    return (good, avgtime)


if __name__ == "__main__":
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('../../robots/denso_base.xml')
    robot = env.GetRobots()[0]
    ndof = N

    startvect = np.zeros(2 * ndof)
    goalvect = np.hstack((np.ones(ndof), np.zeros(ndof)))

    n = 20
    (good, avgtime) = RunTest(robot, startvect, goalvect, RUNNING_TIME, n)
    

    print "==================== INFO ===================="
    print "ndof                 = {0}".format(N)
    print "THRESHOLD            = {0}".format(THRESHOLD)
    print "CELLSIZE             = {0}".format(CELLSIZE)
    print "PROPAGATION_STEPSIZE = {0}".format(PROPAGATION_STEPSIZE)
    print "GOAL_BIAS            = {0}".format(GOAL_BIAS)
    print "=============================================="

    print "Success rate = {0}%".format(100.*good/float(n))
    print "Average running time = {0} s.".format(avgtime)
    
    # ss = Plan(robot, startvect, goalvect, RUNNING_TIME)
    # print "Finish running"
    
    # sol = ss.getSolutionPath()
    # sol.interpolate()
    # M = sol.printAsMatrix()
    # M = M.strip().split('\n')
    # raw_input('Press any key to visualize the solution')
    # for i in range(len(M)):
    #     w = M[i]
    #     w1 = [float(s) for s in w.split()]
    #     if ndof < 6:
    #         robot.SetDOFValues(np.hstack((w1[0:ndof], np.zeros(6 - ndof))))
    #     else:
    #         robot.SetDOFValues(w1[0:6])
    #     time.sleep(0.01)

    # si = ss.getSpaceInformation()
    # g = ss.getGoal().getState()
    # f = sol.getStates()[-1]
    # print "Running time: {0} s.".format(ss.getLastPlanComputationTime())
    # print "Threshold = {0}".format(THRESHOLD)
    # print "Distance to goal = {0}".format(si.distance(f, g))
        
