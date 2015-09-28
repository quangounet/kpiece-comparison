#!/usr/bin/env python

######################################################################
# Software License Agreement (BSD License)
#
#  Copyright (c) 2010, Rice University
#  All rights reserved.
#
#  Redistribution and use in source and binary forms, with or without
#  modification, are permitted provided that the following conditions
#  are met:
#
#   * Redistributions of source code must retain the above copyright
#     notice, this list of conditions and the following disclaimer.
#   * Redistributions in binary form must reproduce the above
#     copyright notice, this list of conditions and the following
#     disclaimer in the documentation and/or other materials provided
#     with the distribution.
#   * Neither the name of the Rice University nor the names of its
#     contributors may be used to endorse or promote products derived
#     from this software without specific prior written permission.
#
#  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
#  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
#  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
#  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
#  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
#  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
#  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
#  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
#  POSSIBILITY OF SUCH DAMAGE.
######################################################################

# Author: Mark Moll

import sys
from os.path import abspath, dirname, join
sys.path.insert(0, join(dirname(dirname(dirname(abspath(__file__)))),'py-bindings') )
from functools import partial
from os.path import dirname
from time import clock
from math import fabs, sqrt
import unittest
import copy
import ompl.util as ou
import ompl.base as ob
import ompl.control as oc
from ompl.util import setLogLevel, LogLevel

SOLUTION_TIME = 120
MAX_VELOCITY = 3.0

class Environment(object):
    def __init__(self, fname):
        fp = open(fname, 'r')
        lines = fp.readlines()
        fp.close()
        self.width, self.height = [int(i) for i in lines[0].split(' ')[1:3]]
        self.grid = []
        self.start = [int(i) for i in lines[1].split(' ')[1:3]]
        self.goal = [int(i) for i in lines[2].split(' ')[1:3]]
        for i in range(self.width):
            self.grid.append(
                [int(i) for i in lines[4+i].split(' ')[0:self.height]])
        self.char_mapping = ['__', '##', '()', 'XX']

    def __str__(self):
        result = ''
        for line in self.grid:
            result = result + ''.join([self.char_mapping[c] for c in line]) + '\n'
        return result

def isValid(grid, state):
    # planning is done in a continuous space, but our collision space
    # representation is discrete
    x = int(state[0])
    y = int(state[1])
    if x<0 or y<0 or x>=len(grid) or y>=len(grid[0]):
        return False
    return grid[x][y] == 0 # 0 means valid state

class MyStateSpace(ob.RealVectorStateSpace):
    def __init__(self):
        super(MyStateSpace, self).__init__(4)

    def distance(self, state1, state2):
        x1 = int(state1[0])
        y1 = int(state1[1])
        x2 = int(state2[0])
        y2 = int(state2[1])
        # return sqrt((x1 - x2)**2 + (y1 - y2)**2)
        return fabs(x1-x2) + fabs(y1-y2)

class MyProjectionEvaluator(ob.ProjectionEvaluator):
    def __init__(self, space, cellSizes):
        super(MyProjectionEvaluator, self).__init__(space)
        self.setCellSizes(cellSizes)

    def getDimension(self):
        return 2

    def project(self, state, projection):
        projection[0] = state[0]
        projection[1] = state[1]

class MyStatePropagator(oc.StatePropagator):
    def __init__(self, spaceInformation):
        super(MyStatePropagator, self).__init__(spaceInformation)

    def propagate(self, state, control, duration, result):
        result[0] = state[0] + duration*control[0]
        result[1] = state[1] + duration*control[1]
        result[2] = control[0]
        result[3] = control[1]

class KPIECE1Test(object):

    def execute(self, env, time, pathLength, show = False):
        result = True

        sSpace = MyStateSpace()
        sbounds = ob.RealVectorBounds(4)
        # dimension 0 (x) spans between [0, width)
        # dimension 1 (y) spans between [0, height)
        # since sampling is continuous and we round down, we allow values until
        # just under the max limit
        # the resolution is 1.0 since we check cells only
        sbounds.low = ou.vectorDouble()
        sbounds.low.extend([0.0, 0.0, -MAX_VELOCITY, -MAX_VELOCITY])
        sbounds.high = ou.vectorDouble()
        sbounds.high.extend([float(env.width) - 0.000000001,
            float(env.height) - 0.000000001,
            MAX_VELOCITY, MAX_VELOCITY])
        sSpace.setBounds(sbounds)

        cSpace = oc.RealVectorControlSpace(sSpace, 2)
        cbounds = ob.RealVectorBounds(2)
        cbounds.low[0] = -MAX_VELOCITY
        cbounds.high[0] = MAX_VELOCITY
        cbounds.low[1] = -MAX_VELOCITY
        cbounds.high[1] = MAX_VELOCITY
        cSpace.setBounds(cbounds)
        cInfo = oc.SpaceInformation(sSpace, cSpace)

        # cInfo.setPropagationStepSize(0.1)
        # cInfo.setMinMaxControlDuration(1, 20)

        ss = oc.SimpleSetup(cInfo)
        isValidFn = ob.StateValidityCheckerFn(partial(isValid, env.grid))
        ss.setStateValidityChecker(isValidFn)
        propagator = MyStatePropagator(ss.getSpaceInformation())
        ss.setStatePropagator(propagator)

        planner = self.newplanner(ss.getSpaceInformation())
        ss.setPlanner(planner)

        # the initial state
        start = ob.State(sSpace)
        start()[0] = env.start[0]
        start()[1] = env.start[1]
        start()[2] = 0.0
        start()[3] = 0.0

        goal = ob.State(sSpace)
        goal()[0] = env.goal[0]
        goal()[1] = env.goal[1]
        goal()[2] = 0.0
        goal()[3] = 0.0

        ss.setStartAndGoalStates(start, goal, 0.05)

        startTime = clock()
        if ss.solve(SOLUTION_TIME) and ss.haveExactSolutionPath():
            elapsed = clock() - startTime
            time = time + elapsed
            if show:
                print 'Found solution in {0} seconds!'.format(elapsed)

            path = ss.getSolutionPath()
            path.interpolate()
            if not path.check():
                return (False, time, pathLength)
            pathLength = pathLength + path.length()

            if show:
                print env
                temp = copy.deepcopy(env)
                states = path.getStates()
                for i in range(len(states)):
                    x = int(states[i][0])
                    y = int(states[i][1])
                    if temp.grid[x][y] in [0,2]:
                        temp.grid[x][y] = 2
                    else:
                        temp.grid[x][y] = 3
                print temp
        else:
            result = False

        return (result, time, pathLength)

    def newplanner(self, si):
        planner = oc.KPIECE1(si)
        cdim = ou.vectorDouble()
        cdim.extend([1, 1])
        ope = MyProjectionEvaluator(si.getStateSpace(), cdim)
        planner.setProjectionEvaluator(ope)
        return planner

if __name__ == "__main__":
    GREETING_TEXT = 'Choose the environment file:\n'
    GREETING_TEXT += '1 : env1.txt\n'
    GREETING_TEXT += '2 : env2.txt\n'
    GREETING_TEXT += '3 : env3.txt\n'
    GREETING_TEXT += '4 : env4.txt\n'
    env_num = raw_input(GREETING_TEXT)
    choices = ['1', '2', '3', '4']
    while env_num not in choices:
        print "Invalid input!"
        env_num = raw_input(GREETING_TEXT)
    env_name = '../src/env' + env_num + '.txt'
    env = Environment(env_name)
    
    planner = KPIECE1Test()
    time = 0.0
    length = 0.0
    good = 0
    
    N = 25
    for i in range(N):
        (result, time, length) = planner.execute(env, time, length, False)
        if result:
            good += 1
    success = 100.0 * float(good) / float(N)
    avgruntime = time / float(N)
    avglength = length / float(N)

    print('    Success rate: %f%%' % success)
    print('    Average runtime: %f' % avgruntime)
    print('    Average path length: %f' % avglength)

