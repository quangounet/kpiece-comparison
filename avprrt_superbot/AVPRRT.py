import numpy as np
import time
import string
import random

from openravepy import CollisionReport

import TOPP
import Utils
import Heap
import copy

# suppress openrave complain
CLA_NOTHING = 0
# global variables
FW = 0
BW = 1
NOTINTERSECT = -2
INCOLLISION = -1
OK = 1

_enableBW = False

class Config(object):
    def __init__(self, q, qs = None, qss = None):
        self.q = q
        if qs is None:
            self.qs = np.zeros(len(self.q))
        else:
            self.qs = qs
        if qss is None:
            self.qss = np.zeros(len(self.q))
        else:
            self.qss = qss


class Vertex(object):
    def __init__(self, config, vertextype = FW):
        self.config = config
        self.type = vertextype
        self.parentindex = None # this is to be assigned when added to a tree
        self.trajectorystring = '' # this is to be assigned when added to a tree
        self.index = 0 # this is to be assigned when added to a tree
        self.level = 0
        # AVP-related
        self.sdmin = 0.0
        self.sdmax = 0.0


class Tree(object):
    def __init__(self, treetype = FW, vroot = None):
        self.verticeslist = []
        if vroot is not None:
            self.verticeslist.append(vroot)
            self.length = 1
        else:
            self.length = 0
        self.type = treetype

    def __len__(self):
        return len(self.verticeslist)

    def __getitem__(self, index):
        return self.verticeslist[index]        
    
    def AddVertex(self, parentindex, vnew, trajectorystring):
        parent = self.verticeslist[parentindex]
        vnew.parentindex = parentindex
        vnew.level = parent.level + 1
        vnew.trajectorystring = trajectorystring
        vnew.index = self.length
        self.verticeslist.append(vnew)
        self.length += 1
        # check soundness
        assert(self.length == len(self.verticeslist))

    def GenerateTrajectoryString(self):
        trajslist = []
        vertex = self.verticeslist[-1]
        while vertex.parentindex is not None:
            parent = self.verticeslist[vertex.parentindex]
            trajslist.append(vertex.trajectorystring)
            vertex = parent

        if (self.type == FW):
            trajslist = trajslist[::-1]
        
        restraj = '' # resulting trajectory
        separator = ""
        for i in range(len(trajslist)):
            restraj += separator
            restraj += trajslist[i]
            separator = "\n"
        return restraj


class RRTPlanner(object):
    REACHED = 0
    ADVANCED = 1
    TRAPPED = 2

    def __init__(self, v_start, v_goal, robot):
        self.treestart = Tree(FW, v_start)
        self.treeend = Tree(BW, v_goal)
        self.robot = robot
        self.connectingstring = ''
        self.runningtime = 0.0
        self.nn = -1
        self.iterations = 0
        self.found = False

        self._RNG = random.SystemRandom()
        # default parameters
        self._stepsize = 0.8
        self._interation_duration = 0.5
        self._max_repeat_interation = 4
        self._factor = 0.8

    def __str__(self):
        s = 'Total running time: {0} s.\n'.format(self.runningtime)
        s += 'Total number of iterations: {0}\n'.format(self.iterations)
        return s

    def RandomJointValues(self, ndof):
        # assign lower and upper limits of joint values
        [lowerlimits, upperlimits] = self.robot.GetDOFLimits()
        
        # ## CHANGE JOINT LIMITS
        # PI = np.pi
        # lowerlimits[0] = -2 #lowerlimits[0]
        # lowerlimits[1] = 1 #lowerlimits[0]
        # lowerlimits[2] = -0.1 #lowerlimits[2]
        # lowerlimits[3] = -PI #lowerlimits[3]
        # lowerlimits[4] = lowerlimits[4]
        # lowerlimits[5] = PI/4 #lowerlimits[5]
        
        # upperlimits[0] = 2
        # upperlimits[1] = 2
        # upperlimits[2] = 0.5 #upperlimits[2]
        # upperlimits[3] = PI/3 #upperlimits[3]
        # upperlimits[4] = 0 #upperlimits[4]
        # upperlimits[5] = 3*PI/4 #upperlimits[5]
        
        passed = False
        while (not passed):
            q_rand = np.zeros(ndof)
            for i in xrange(ndof):
                q_rand[i] = self._RNG.uniform(lowerlimits[i], upperlimits[i])

            passed = True # do not have any other constraint for now.

        return q_rand

    def RandomVelocity(self, ndof):
        # assign lower and upper limits of joint values
        upperlimits = self.robot.GetDOFVelocityLimits()
        lowerlimits = -1.0*upperlimits
        
        # ## CHANGE JOINT VELOCITY LIMITS
        # lowerlimits[0] = -0.5 #lowerlimits[0]
        # lowerlimits[1] = -0.5 # lowerlimits[0]
        # lowerlimits[2] = -0.5 # lowerlimits[2]
        # lowerlimits[3] = -0.5 # lowerlimits[3]
        # lowerlimits[4] = lowerlimits[4]
        # lowerlimits[5] = -0.5 # lowerlimits[5]
        
        # upperlimits[0] = upperlimits[0]
        # upperlimits[1] = 0.5 # upperlimits[1]
        # upperlimits[2] = 0.5 # upperlimits[2]
        # upperlimits[3] = 0.5 # upperlimits[3]
        # upperlimits[4] = 0.5 # upperlimits[4]
        # upperlimits[5] = 0.5# upperlimits[5]

        qd_rand = np.zeros(ndof)
        for i in xrange(ndof):
            qd_rand[i] = self._RNG.uniform(lowerlimits[i], upperlimits[i])

        return qd_rand

    def Extend(self, c_rand):
        raise RRTException("Virtual method not implemented.")


    def Connect(self):
        raise RRTException("Virtual method not implemented.")


    def IsFeasibleConfiguration(self, c_rand):
        """IsFeasibleConfig checks feasibility of the given Config object. 
        Feasibility conditions are to be determined by each RRT planner.
        """
        raise RRTException("Virtual method not implemented.")
       

    def IsFeasibleTrajectory(self, trajectorystring):
        """IsFeasibleTrajectory checks feasibility of the given trajectorystring.
        Feasibility conditions are to be determined by each RRT planner.
        """
        raise RRTException("Virtual method not implemented.")

    def Run(self, allottedtime):
        if (self.found):
            print "The planner has already found a path."
            return True
        
        t = 0.0 # total running time for this run
        prev_iter = self.iterations

        ndof = self.robot.GetActiveDOF()
        
        while (t < allottedtime):
            self.iterations += 1
            print "\033[1;34miteration:", self.iterations, "\033[0m"
            t_begin = time.time()

            q_rand = self.RandomJointValues(ndof)
            qs_rand = Utils.Normalize(self.RandomVelocity(ndof))
            qss_rand = np.zeros(ndof)
            c_rand = Config(q_rand, qs_rand, qss_rand)
            
            if (self.Extend(c_rand) != self.TRAPPED):
                print "\033[1;32mTree start : ", len(self.treestart.verticeslist), 
                print "; Tree end : ", len(self.treeend.verticeslist), "\033[0m"
                if (self.Connect() == self.REACHED):
                    print "\033[1;32mPath found"
                    print "    Total number of iterations: {0}".format(self.iterations)
                    t_end = time.time()
                    t += t_end - t_begin
                    self.runningtime += t
                    print "    Total running time: {0} s.\033[0m".format(self.runningtime)
                    self.found = True
                    return True # running result
                
            t_end = time.time()
            t += t_end - t_begin
            self.runningtime += t_end - t_begin
            
        print "\033[1;31mAllotted time ({0} s. is exhausted after {1} iterations\033[0m"\
        .format(allottedtime, self.iterations - prev_it)
        return False

    def Distance(self, c1, c2, metrictype = 1):
        delta_q = c1.q - c2.q
        if (metrictype == 1):
            # norm-2 squared
            return np.dot(delta_q, delta_q)
        elif (metrictype == 2):
            # norm-1
            return np.linalg.norm(delta_q, 1)        
        elif (metrictype == 3):
            # special
            return np.dot(delta_q, delta_q - (c1.qs + c2.qs)/np.linalg.norm(delta_q))
        else:
            raise RRTException("Unknown Distance Metric.")

    def NearestNeighborIndices(self, c_rand, treetype, custom_nn = 0):
        """NearestNeighborIndices returns indices of self.nn nearest
           neighbors of c_rand on the tree specified by treetype.
        """ 
        if (treetype == FW):
            tree = self.treestart
            nv = len(tree)
        else:
            tree = self.treeend
            nv = len(tree)
            
        distancelist = [self.Distance(c_rand, v.config, self.metrictype) 
                        for v in tree.verticeslist]
        distanceheap = Heap.Heap(distancelist)
        
        if (custom_nn == 0):
            nn = self.nn
        else:
            nn = custom_nn
        
        if (nn == -1):
            nn = nv
        else:
            nn = min(self.nn, nv)
        nnindices = [distanceheap.ExtractMin()[0] for i in range(nn)]
        return nnindices

    def GenerateFinalTrajectoryString(self):
        if (not self.found):
            print "The Planner has not found any trajectory from start to goal."
            return ''
        
        trajectorystring = ''
        trajectorystring += self.treestart.GenerateTrajectoryString()
        if not (self.connectingstring == ''):
            if not (trajectorystring == ''):
                trajectorystring += "\n"
            trajectorystring += self.connectingstring
        trajstring_treeend = self.treeend.GenerateTrajectoryString()
        if (not trajstring_treeend == ''):
            trajectorystring += "\n"
            trajectorystring += trajstring_treeend
        
        return trajectorystring


class RRTException(Exception):
    """Base class for exceptions for RRT planners"""
    pass


class AVPBiRRTPlanner(RRTPlanner):
    def __init__(self, v_start, v_goal, robot, problemtype, constraintsstring, 
                 tuningsstring, nn = -1, metrictype = 1, polynomialdegree = 5):
        super(AVPBiRRTPlanner, self).__init__(v_start, v_goal, robot)
        
        self.problemtype = problemtype
        self.constraintsstring = constraintsstring
        
        self.discrtimestep = float(string.split(constraintsstring)[0])
        params = [float(x) for x in string.split(tuningsstring)]
        self.integrationtimestep = params[0]
        self.reparamtimestep = params[1]
        self.passswitchpointnsteps = params[2]

        self.nn = nn
        self.metrictype = metrictype
        self.polynomialdegree = polynomialdegree

        self._max_repeat = -1

    def Extend(self, c_rand):
        if not _enableBW:
            return self.ExtendFW(c_rand)
        if (np.mod(self.iterations - 1, 2) == FW):
            return self.ExtendFW(c_rand)
        else:
            return self.ExtendBW(c_rand)

    def ExtendFW(self, c_rand):
        nnindices = self.NearestNeighborIndices(c_rand, FW)
        for index in nnindices:
            print "Extending forward from index = {0}".format(index)
            v_near = self.treestart.verticeslist[index]
            
            q_beg = v_near.config.q
            qs_beg = v_near.config.qs
            qss_beg = v_near.config.qss            
            # check if c_rand is too far from vnear
            delta = self.Distance(v_near.config, c_rand)
            if (delta <= self._stepsize):
                q_end = c_rand.q
                status = self.REACHED
            else:
                q_end = q_beg + self._stepsize*(c_rand.q - q_beg)/np.sqrt(delta)
                status = self.ADVANCED    
            qs_end = c_rand.qs
            qss_end = c_rand.qss
            # new config to be extended to
            c_new = Config(q_end, qs_end, qss_end)

            # check feasibility of c_new
            print "\tCheck feasibility"
            if not self.IsFeasibleConfiguration(c_new):
                status = self.TRAPPED
                continue
            # interpolate a trajectory from v_near.config to c_new
            print "\tInterpolate a trajectory"
            for rep in xrange(self._max_repeat_interation):
                # interpolation duration is shorter every time it fails
                T = (self._factor**rep)*self._interation_duration
                if (self.polynomialdegree == 3):
                    trajectorystring = Utils.TrajString3rdDegree(q_beg, q_end,
                                                                 qs_beg, qs_end, T)
                elif (self.polynomialdegree == 5):
                    trajectorystring = Utils.TrajString5thDegree(q_beg, q_end,
                                                                 qs_beg, qs_end, 
                                                                 qss_beg, qss_end, T)
                # check DOF limits
                inlimits = Utils.CheckDOFLimits(self.robot, trajectorystring)
                if (not inlimits):
                    continue
                # check feasibility (AVP + collision checking)
                print "\tCheck trajectory feasibility"
                result = self.IsFeasibleTrajectory(trajectorystring, FW, 
                                                   v_near.sdmin, v_near.sdmax)
                if not (result[0] == 1):
                    continue
                # this passes every check, extension is now successful
                v_new = Vertex(c_new)
                v_new.sdmin = result[1]
                v_new.sdmax = result[2]
                v_new.level = v_near.level + 1
                self.treestart.AddVertex(index, v_new, trajectorystring)
                print "Successful extension"
                return status
            status = self.TRAPPED
        return status
    
    def ExtendBW(self, c_rand):
        nnindices = self.NearestNeighborIndices(c_rand, BW)
        for index in nnindices:
            print "Extending backward from index = {0}".format(index)
            v_near = self.treeend.verticeslist[index]
            
            q_end = v_near.config.q
            qs_end = v_near.config.qs
            qss_end = v_near.config.qss
            # check if c_rand is too far from vnear
            delta = self.Distance(v_near.config, c_rand)
            if (delta <= self._stepsize):
                q_beg = c_rand.q
                status = self.REACHED
            else:
                q_beg = q_end + self._stepsize*(c_rand.q - q_end)/np.sqrt(delta)
                status = self.ADVANCED    
            qs_beg = c_rand.qs
            qss_beg = c_rand.qss
            # new config to be extended to
            c_new = Config(q_beg, qs_beg, qss_beg)

            # check feasibility of c_new
            print "\tCheck feasibility"
            if not self.IsFeasibleConfiguration(c_new):
                status = self.TRAPPED
                continue
            # interpolate a trajectory from v_near.config to c_new
            print "\tInterpolate a trajectory"
            for rep in xrange(self._max_repeat_interation):
                # interpolation duration is shorter every time it fails
                T = (self._factor**rep)*self._interation_duration
                if (self.polynomialdegree == 3):
                    trajectorystring = Utils.TrajString3rdDegree(q_beg, q_end,
                                                                 qs_beg, qs_end, T)
                elif (self.polynomialdegree == 5):
                    trajectorystring = Utils.TrajString5thDegree(q_beg, q_end,
                                                                 qs_beg, qs_end, 
                                                                 qss_beg, qss_end, T)
                # check DOF limits
                inlimits = Utils.CheckDOFLimits(self.robot, trajectorystring)
                if (not inlimits):
                    continue
                # check feasibility (AVP + collision checking)
                print "\tCheck trajectory feasibility"
                result = self.IsFeasibleTrajectory(trajectorystring, BW, 
                                                   v_near.sdmin, v_near.sdmax)
                if not (result[0] == 1):
                    continue
                # this passes every check, extension is now successful
                v_new = Vertex(c_new)
                v_new.sdmin = result[1]
                v_new.sdmax = result[2]
                v_new.level = v_near.level + 1
                self.treeend.AddVertex(index, v_new, trajectorystring)
                print "Successful extension"
                return status
            status = self.TRAPPED
        return status
    
    def Connect(self):
        if not _enableBW:
            return self.ConnectBW()
        if (np.mod(self.iterations - 1, 2) == FW):
            return self.ConnectBW()
        else:
            return self.ConnectFW()

    def ConnectFW(self):
        v_test = self.treeend.verticeslist[-1]
        nnindices = self.NearestNeighborIndices(v_test.config, FW)
        for index in nnindices:
            v_near = self.treestart.verticeslist[index]
            
            q_beg = v_near.config.q
            qs_beg = v_near.config.qs
            qss_beg = v_near.config.qss
            
            q_end = v_test.config.q
            qs_end = v_test.config.qs
            qss_end = v_test.config.qss
            
            # interpolate a trajectory
            for rep in xrange(self._max_repeat_interation):
                # interpolation duration is shorter every time it fails
                T = (self._factor**rep)*self._interation_duration
                if (self.polynomialdegree == 3):
                    trajectorystring = Utils.TrajString3rdDegree(q_beg, q_end,
                                                                 qs_beg, qs_end, T)
                elif (self.polynomialdegree == 5):
                    trajectorystring = Utils.TrajString5thDegree(q_beg, q_end,
                                                                 qs_beg, qs_end, 
                                                                 qss_beg, qss_end, T)
                # check DOF limits
                inlimits = Utils.CheckDOFLimits(self.robot, trajectorystring)
                if (not inlimits):
                    continue
                # check feasibility (AVP + collision checking)
                result = self.IsFeasibleTrajectory(trajectorystring, FW, 
                                                   v_near.sdmin, v_near.sdmax,
                                                   True, [v_test.sdmin, v_test.sdmax])
                if not (result[0] == 1):
                    continue
                # this passes every test, connection is now successful
                self.treestart.verticeslist.append(v_near)
                self.connectingstring = trajectorystring
                status = self.REACHED
                return status
        status = self.TRAPPED
        return status

    def ConnectBW(self):
        v_test = self.treestart.verticeslist[-1]
        nnindices = self.NearestNeighborIndices(v_test.config, BW)
        for index in nnindices:
            v_near = self.treeend.verticeslist[index]
            
            q_end = v_near.config.q
            qs_end = v_near.config.qs
            qss_end = v_near.config.qss
            
            q_beg = v_test.config.q
            qs_beg = v_test.config.qs
            qss_beg = v_test.config.qss
            
            # interpolate a trajectory
            for rep in xrange(self._max_repeat_interation):
                # interpolation duration is shorter every time it fails
                T = (self._factor**rep)*self._interation_duration
                if (self.polynomialdegree == 3):
                    trajectorystring = Utils.TrajString3rdDegree(q_beg, q_end,
                                                                 qs_beg, qs_end, T)
                elif (self.polynomialdegree == 5):
                    trajectorystring = Utils.TrajString5thDegree(q_beg, q_end,
                                                                 qs_beg, qs_end, 
                                                                 qss_beg, qss_end, T)
                # check DOF limits
                inlimits = Utils.CheckDOFLimits(self.robot, trajectorystring)
                if (not inlimits):
                    continue
                # check feasibility (AVP + collision checking)
                result = self.IsFeasibleTrajectory(trajectorystring, BW, 
                                                   v_near.sdmin, v_near.sdmax,
                                                   True, [v_test.sdmin, v_test.sdmax])
                if not (result[0] == 1):
                    continue
                # this passes every test, connection is now successful
                self.treeend.verticeslist.append(v_near)
                self.connectingstring = trajectorystring
                status = self.REACHED
                return status
        status = self.TRAPPED
        return status

    def IsFeasibleConfiguration(self, c_rand):
        feasible = True
        env = self.robot.GetEnv()
        with self.robot:
            self.robot.SetActiveDOFValues(c_rand.q)
            incollision = (env.CheckCollision(self.robot, CollisionReport()) or 
                           self.robot.CheckSelfCollision(CollisionReport()))
        if incollision:
            feasible = False
            return feasible
        if (self._max_repeat == -1):
            # no further checking
            return feasible

    def IsFeasibleTrajectory(self, trajectorystring, direction, sdmin, sdmax,
                             checkintersection = False, sdinterval = []):
        # run AVP
        x = TOPP.TOPPbindings.TOPPInstance(self.robot, self.problemtype, 
                                           self.constraintsstring, trajectorystring)
        x.integrationtimestep = self.integrationtimestep
        x.reparamtimestep = self.reparamtimestep
        x.passswitchpointnsteps = self.passswitchpointnsteps
        if (direction == FW):
            res = x.RunVIP(sdmin, sdmax)
        else:
            res = x.RunVIPBackward(sdmin, sdmax)
        if not (res == 1):
            return [res, -1, -1]
        
        # check intersection
        if checkintersection:
            if (direction == FW):
                intersect = Utils.CheckIntersection([x.sdendmin, x.sdendmax], sdinterval)
            else:
                intersect = Utils.CheckIntersection([x.sdbegmin, x.sdbegmax], sdinterval)
            if not intersect:
                return [NOTINTERSECT, -1, -1]
            
        # check collision
        env = self.robot.GetEnv()
        ndof = self.robot.GetActiveDOF()
        traj = TOPP.Trajectory.PiecewisePolynomialTrajectory.FromString(trajectorystring)
        for s in np.arange(0, traj.duration, self.discrtimestep):
            with self.robot:
                self.robot.SetActiveDOFValues(traj.Eval(s))
                incollision = (env.CheckCollision(self.robot, CollisionReport()) or 
                               self.robot.CheckSelfCollision(CollisionReport()))
            if incollision:
                return [INCOLLISION, -1, -1]
            
        with self.robot:
            self.robot.SetActiveDOFValues(traj.Eval(traj.duration))
            incollision = (env.CheckCollision(self.robot, CollisionReport()) or 
                           self.robot.CheckSelfCollision(CollisionReport()))
        if incollision:
            return [INCOLLISION, -1, -1]
        
        if (direction == FW):
            return [OK, x.sdendmin, x.sdendmax]
        else:
            return [OK, x.sdbegmin, x.sdbegmax]
