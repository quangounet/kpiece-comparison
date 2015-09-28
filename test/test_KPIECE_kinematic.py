#!/usr/bin/env python
from ompl import util
from ompl import base
from ompl import geometric
from ompl import control

from openravepy import Environment, RaveCreateCollisionChecker
import numpy as np
import time

# State space
class MyStateSpace(base.CompoundStateSpace):
    def __init__(self, robot):
        super(MyStateSpace, self).__init__()
        self.ndof = robot.GetActiveDOF()
        self.dimension = self.ndof * 2

        self.jointvals = base.RealVectorStateSpace(self.ndof)
        jointvalbounds = base.RealVectorBounds(self.ndof)
        jointvalminlimits = robot.GetDOFLimits()[0]
        jointvalmaxlimits = robot.GetDOFLimits()[1]
        for i in range(self.ndof):
            jointvalbounds.setLow(i, jointvalminlimits[i])
            jointvalbounds.setHigh(i, jointvalmaxlimits[i])
        self.jointvels = base.RealVectorStateSpace(self.ndof)
        jointvellimits = robot.GetDOFVelocityLimits()
        jointvelbounds = base.RealVectorBounds(self.ndof)
        for i in range(self.ndof):
            jointvelbounds.setLow(i, -jointvellimits[i])
            jointvelbounds.setHigh(i, jointvellimits[i])

        self.jointvals.setBounds(jointvalbounds)
        self.jointvels.setBounds(jointvelbounds)

        self.addSubspace(self.jointvals, 1.0)
        self.addSubspace(self.jointvels, 1.0)

    def getDimension(self):
        return self.dimension
    
    def distance(self, state1, state2):
        s1 = np.zeros(self.ndof)
        v1 = np.zeros(self.ndof)
        s2 = np.zeros(self.ndof)
        v2 = np.zeros(self.ndof)
        for i in range(self.ndof):
            s1[i] = state1[0][i]
            v1[i] = state1[1][i]
            s2[i] = state2[0][i]
            v2[i] = state2[1][i]
        return np.sqrt(np.dot(s1 - s2, s1 - s2) + np.dot(v1 - v2, v1 - v2))


class MyControlSpace(control.RealVectorControlSpace):
    def __init__(self, sspace, robot):
        self.ndof = robot.GetActiveDOF()
        super(MyControlSpace, self).__init__(sspace, self.ndof)

        jointaccbounds = base.RealVectorBounds(self.ndof)
        jointacclimits = robot.GetDOFAccelerationLimits() * 0.3
        for i in range(self.ndof):
            jointaccbounds.setLow(-jointacclimits[i])
            jointaccbounds.setHigh(jointacclimits[i])
            
        self.setBounds(jointaccbounds)


# State Validity Checker for OMPL Planners
class MyStateValidityChecker(base.StateValidityChecker):
    def __init__(self, si, robot):
        # Inheritted from base.StateValidityChecker
        super(MyStateValidityChecker, self).__init__(si)
        self.spaceInformation = si
        self.robot = robot
        self.env = self.robot.GetEnv()
        self.ndof = self.robot.GetActiveDOF()

    def isValid(self, state):
        # TODO: add fritcion constraints checking here
        
        # Check joint limits
        withinbounds = self.spaceInformation.satisfiesBounds(state)
        if not withinbounds:
            return withinbounds

        # Check friction constraints
        passed = True
        jointvals = np.zeros(self.ndof)
        jointvels = np.zeros(self.ndof)
        for i in range(self.ndof):
            jointvals[i] = state[0][i]
            jointvels[i] = state[1][i]

        if not passed:
            return passed

        # Check collision
        incollision = env.CheckCollision(robot) or robot.CheckSelfCollision()
        if incollision:
            return incollision
        
        return True


# Define a projection for this problem.
class MyProjection(base.ProjectionEvaluator):
    def __init__(self, space, robot, cellSizes = None):
        # Inheritted from base.ProjectionEvaluator
        super(MyProjection, self).__init__(space)
        self.robot = robot
        self.manip = self.robot.GetManipulator('Flange')
        self.ndof = self.robot.GetActiveDOF()
        if cellSizes is None:
            self.cellSizes = util.vectorDouble()
            self.cellSizes.extend(0.1*np.ones(3))
        else:
            self.cellSizes = cellSizes
        self.setCellSizes(self.cellSizes)

    # Get projected space dimension
    def getDimension(self):
        return 3

    # We use tool-tip position in the workspace as a projection.
    def project(self, state, projection):
        subSpaceIndex = 0
        jointvals = np.zeros(self.ndof)
        for i in range(self.ndof):
            jointvals[i] = state[subSpaceIndex][i]
        with self.robot:
            self.robot.SetDOFValues(jointvals)
            pos = self.manip.GetEndEffectorTransform()[0:3, 3]
            projection[0] = pos[0]
            projection[1] = pos[1]
            projection[2] = pos[2]


def Plan(robot, startvect, goalvect, runningtime):
    ndof = robot.GetActiveDOF()
    
    # Construct the robot state space in which we are planning.
    # We are planning with a 6-DOF (Denso) robot. So State = Pos(6) x Vel(6).
    sspace = MyStateSpace(robot) # sspace for state-space

    # Create Space Information object
    si = base.SpaceInformation(sspace)

    # Create and set StateValidityChecker
    vc = MyStateValidityChecker(si, robot)
    si.setStateValidityChecker(vc)

    si.setup()
    
    # Create and set ProjectionEvaluator
    proj = MyProjection(sspace, robot)

    # Create start and goal
    start = base.State(sspace)
    goal = base.State(sspace)
    for i in range(2 * ndof):
        start[i] = startvect[i]
        goal[i] = goalvect[i]

    # Create a problem definition
    pdef = base.ProblemDefinition(si)

    pdef.setStartAndGoalStates(start, goal)

    # Create a new planner
    planner = geometric.KPIECE1(si)
    planner.setProblemDefinition(pdef)
    planner.setProjectionEvaluator(proj)
    planner.setup()

    if planner.solve(runningtime):
        print "Problem solved"
        sol = pdef.getSolutionPath()
        return (True, sol)
    else:
        print "Planning failed"
        return (False, '')


if __name__ == "__main__":
    env = Environment()
    env.SetViewer('qtcoin')
    env.Load('../../robots/denso_base.xml')
    robot = env.GetRobots()[0]
    ndof = robot.GetActiveDOF()
    startvect = np.zeros(2*ndof)
    VAL = 0.8
    goalvect = np.hstack((VAL*np.ones(ndof), np.zeros(ndof)))
    runningtime = 10.

    res = Plan(robot, startvect, goalvect, runningtime)
    if (res[0]):
        sol = res[1]
        sol.interpolate()
        M = sol.printAsMatrix()
        M = M.strip().split('\n')
        raw_input('Press any key to visualize the solution')
        for i in range(len(M)):
            w = M[i]
            w1 = [float(s) for s in w.split()]
            robot.SetDOFValues(w1[0:6])
            time.sleep(0.01)
        
