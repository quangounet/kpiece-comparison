from openravepy import *
from pylab import *

from TOPP import TOPPbindings
from TOPP import Trajectory
from TOPP import Utilities

import string
import numpy as np
import random

INF = np.infty
EPS = 1e-12
CLA_NOTHING = 0

############################## POLYNOMIALS ##############################
"""
NB: we adopt the weak-term-first convention for inputs
"""

def FindPolynomialCriticalPoints(coeff_list, interval = None):
    p = np.poly1d(coeff_list[::-1])
    pd = np.polyder(p)
    critical_points = pd.r
    pointslist = []
    if interval == None:
        interval = [-INF, INF]
    for x in critical_points:
        if (abs(x.imag) < EPS):
            if (x.real <= interval[1]) and (x.real >= interval[0]):
                pointslist.append(x.real)
    return pointslist


def TrajString3rdDegree(q_beg, q_end, qs_beg, qs_end, duration):
    trajectorystring = ''
    ndof = len(q_beg)
    trajectorystring += "%f\n%d"%(duration, ndof)
    for k in range(ndof):
        a, b, c, d = Utilities.Interpolate3rdDegree(q_beg[k], q_end[k], qs_beg[k], qs_end[k], duration)
        trajectorystring += "\n%f %f %f %f"%(d, c, b, a)
    return trajectorystring


def TrajString3rdDegree_2FW(q_beg, q_end, qs_beg, qss_beg, duration):
    trajectorystring = ''
    ndof = len(q_beg)
    trajectorystring += "%f\n%d"%(duration, ndof)
    for k in range(ndof):
        a, b, c, d = Interpolate3rdDegree_2FW(q_beg[k], q_end[k], qs_beg[k], qss_beg[k], duration)
        trajectorystring += "\n%f %f %f %f"%(d, c, b, a)
    return trajectorystring


def Interpolate3rdDegree_2FW(q0, q1, qd0, qdd0, T):
    b = 0.5*qdd0
    c = qd0
    d = q0
    a = (q1 - d - c*T - b*T*T)/(T**3)
    return a, b, c, d


def TrajString3rdDegree_2BW(q_beg, q_end, qs_end, qss_end, duration):
    trajectorystring = ''
    ndof = len(q_beg)
    trajectorystring += "%f\n%d"%(duration, ndof)
    for k in range(ndof):
        a, b, c, d = Interpolate3rdDegree_2FW(q_beg[k], q_end[k], qs_end[k], qss_end[k], duration)
        trajectorystring += "\n%f %f %f %f"%(d, c, b, a)
    return trajectorystring


def Interpolate3rdDegree_2BW(q0, q1, qd1, qdd1, T):
    a = ((q1 - q0) - T*qd1 + 0.5*(T**2)*qdd1)/(T**3)
    b = (3*(q1 - q0) + 3*T*qd1)/(T**2) - qdd1
    c = 3*(q1 - q0)/T - 2*qd1 + 0.5*T*qdd1
    d = q0
    return a, b, c, d
    


def TrajString5thDegree(q_beg, q_end, qs_beg, qs_end, qss_beg, qss_end, duration):
    trajectorystring = ''
    ndof = len(q_beg)
    trajectorystring += "%f\n%d"%(duration, ndof)
    for k in range(ndof):
        a, b, c, d, e, f = Utilities.Interpolate5thDegree(q_beg[k], q_end[k], qs_beg[k], 
                                                          qs_end[k], qss_beg[k], qss_end[k], duration)
        trajectorystring += "\n%f %f %f %f %f %f"%(f, e, d, c, b, a)
    return trajectorystring


def CheckDOFLimits(robot, trajectorystring):
    trajinfo = string.split(trajectorystring, "\n")
    dur = float(trajinfo[0])
    ndof = int(trajinfo[1])
    
    for i in range(ndof):
        coeff_list = [float(j) for j in string.split(trajinfo[i + 2])]
        q = np.poly1d(coeff_list[::-1])
        qd = np.polyder(q)
        criticalqlist = FindPolynomialCriticalPoints(coeff_list, [0.0, dur])
        criticalqdlist = FindPolynomialCriticalPoints(qd.coeffs[::-1], [0.0, dur])
        
        if (len(criticalqlist) == 0):
            continue
        else:
            # check DOF values
            for j in criticalqlist:
                if (not (abs(q(j)) <= robot.GetDOFLimits()[1][i])):
                    return False
            # if (len(criticalqdlist) == 0):
            #     continue
            # else:
            #     # check DOF velocities
            #     for k in criticalqdlist:
            #         if (not (abs(qd(k)) <= robot.GetDOFVelocityLimits()[i])):
            #             return False
    
    return True


######################### COLLISION CHECKING #########################
def CheckCollisionConfig(robot, config):
    """CheckCollisionConfig accepts a robot and an RRT.Config as its inputs.
       It returns True if the config is IN-COLLISION and returns False, otherwise.
    """
    env = robot.GetEnv()
    with self.robot:
        robot.SetDOFValues(config.q, range(robot.GetDOF()), CLA_NOTHING)
        isincollision = (env.CheckCollision(robot, CollisionReport()) or 
                         robot.CheckSelfCollision(CollisionReport()))
    if (isincollision):
        return True
    
    
def CheckCollisionTraj(robot, traj, checkcollisiontimestep = 1e-3):
    """CheckCollisionTraj accepts a robot and a trajectory object as its inputs.
       (checkcollisiontimestep is set to 1e-3 as a default value)
       It returns True if any config along the traj is IN-COLLISION.
    """
    env = robot.GetEnv()
    ndof = robot.GetDOF()
    for s in np.arange(0, traj.duration, checkcollisiontimestep):
        with robot:
            robot.SetDOFValues(traj.Eval(s), range(ndof), CLA_NOTHING)
            isincollision = (env.CheckCollision(robot, CollisionReport()) or 
                             robot.CheckSelfCollision(CollisionReport()))
            if (isincollision):
                return True
            
    s = traj.duration
    with robot:
        robot.SetDOFValues(traj.Eval(s), range(ndof), CLA_NOTHING)
        isincollision = (env.CheckCollision(robot, CollisionReport()) or 
                         robot.CheckSelfCollision(CollisionReport()))
    if (isincollision):
        return True
    else:
        return False


############################## ETC ##############################
def Shortcut(robot, problemtype, constraintsstring, trajectorystring, tuningsstring, 
             maxiter, estimate = False, meanduration = 0, upperlimit = -1):
    """
    SHORTCUTTING implementation detail:
    1. pick 2 time points at random and force them to differ from each other more than 1 seconds
    2. newly created traj. polynomial always has T = 1
    3. check for collision first and if collision-free, retime-parameterize the whole traj. (not just newly created portion)
    4. if shortcut traj. has duration less than 2 seconds, terminate.
    """
    
    traj = Trajectory.PiecewisePolynomialTrajectory.FromString(trajectorystring)
    if upperlimit < 0:
        dur = traj.duration
        upperlimit = traj.duration
    else:
        dur = upperlimit
    
    attempt = 0
    tunings = string.split(tuningsstring)
    integrationtimestep = float(tunings[0])
    reparamtimestep = float(tunings[1])
    passswitchpointnsteps = int(tunings[2])

    discrtimestep = float(string.split(constraintsstring)[0])
    assert(dur > 10.0*discrtimestep)
    
    ncollision = 0
    nnotretimable = 0 
    nnotshorter = 0

    for it in range(maxiter):
        if (dur < discrtimestep):
            print "[Utils::Shortcut] trajectory duration is less than discrtimestep.\n"
            break ## otherwise, this will cause an error in TOPP        
        
        ## select an interval for shortcutting
        t0 = np.random.rand()* dur

        if meanduration == 0:
            meanduration = dur - t0

        T = np.random.rand()*min(meanduration, dur - t0)
        t1 = t0 + T
        while (T < 2.0*discrtimestep):
            t0 = np.random.rand()*dur
            if meanduration == 0:
                meanduration = dur - t0
                
            T = np.random.rand()*min(meanduration, dur - t0)
            t1 = t0 + T

            if t1 > upperlimit:
                t1 = upperlimit
                if (t1 < t0):
                    temp = t0
                    t0 = t1
                    t1 = temp
                    T = t1 - t0

        print "\n\nShortcutting iteration", it + 1
        print t0, t1, t1- t0
        
        q0 = traj.Eval(t0)
        q1 = traj.Eval(t1)
        qs0 = traj.Evald(t0)
        qs1 = traj.Evald(t1)
        
        if (traj.degree == 3):
            shortcuttrajstring = TrajString3rdDegree(q0, q1, qs0, qs1, T)
        elif (traj.degree == 5) or (traj.degree > 3):
            qss0 = traj.Evald(t0)
            qss1 = traj.Evald(t1)
            shortcuttrajstring = TrajString5thDegree(q0, q1, qs0, qs1, qss0, qss1, T)
        
        ## check feasibility first (check only for the new portion)
        x = TOPPbindings.TOPPInstance(robot, problemtype, constraintsstring, shortcuttrajstring)
        x.integrationtimestep = integrationtimestep
        x.reparamtimestep = reparamtimestep
        x.passswitchpointnsteps = passswitchpointnsteps
        
        ret = x.RunComputeProfiles(1, 1)
        if (ret == 1):
            ## check whether the new one has shorter duration
            if (x.resduration < T):
                shortcuttraj = Trajectory.PiecewisePolynomialTrajectory.FromString(shortcuttrajstring)
                isincollision = CheckCollisionTraj(robot, shortcuttraj, discrtimestep)
                if (not isincollision):
                    newtraj = ReplaceTrajectorySegment(traj, shortcuttraj, t0, t1)
                    ## newtraj = Trajectory.InsertIntoTrajectory(traj, shortcuttraj, t0, t1)
                    traj = newtraj
                    dur = traj.duration
                    print "*******************************************"
                    print "Success:", t0, t1, t1 - t0, x.resduration
                    print "*******************************************"
                    attempt += 1
                else:
                    print "Collision"
                    ncollision += 1
            else: 
                print "Not shorter"
                nnotshorter += 1
        else:
            print "Not retimable"
            nnotretimable += 1

        print "T:", nnotretimable, "; S:", nnotshorter , "; C:", ncollision , "; OK:", attempt

        # ## check feasibility first (check for the whole traj)
        # shortcuttraj = Trajectory.PiecewisePolynomialTrajectory.FromString(shortcuttrajstring)
        # newtraj =  ReplaceTrajectorySegment(traj, shortcuttraj, t0, t1)
        # x = TOPPbindings.TOPPInstance(robot, problemtype, constraintsstring, str(newtraj))
        # x.integrationtimestep = integrationtimestep
        # x.reparamtimestep = reparamtimestep
        # x.passswitchpointnsteps = passswitchpointnsteps
        
        # ret = x.RunComputeProfiles(0, 0)
        # if (ret == 1):
        #     ## check whether the new one has shorter duration
        #     if (x.resduration < dur):
        #         ## check collision (check for only the new portion)
        #         isincollision = CheckCollisionTraj(robot, shortcuttraj, discrtimestep)
        #         if (not isincollision):
        #             traj = newtraj
        #             dur = traj.duration
        #             attempt += 1
            
    ## retime the overall traj. for traj.duration estimation (optional)
    if (estimate):
        x = TOPPbindings.TOPPInstance(robot, problemtype, constraintsstring, str(traj))                
        x.integrationtimestep = integrationtimestep
        x.reparamtimestep = reparamtimestep
        x.passswitchpointnsteps = passswitchpointnsteps
        ret = x.RunComputeProfiles(0, 0)
        if (ret == 1):
            x.ReparameterizeTrajectory()
            x.WriteResultTrajectory()
            print "Trajectory duartion before shortcutting:", traj.duration, "seconds."
            print "Estimated Trajectory duration after shortcutting:", x.resduration, "seconds."
            print "Successful shortcutting attempts:", attempt, "times."
        else:
            print "Successful shortcutting attempts:", attempt, "times."
            print "Failed to retime-parameterize the final trajectory."
    else:
        print "Successful shortcutting attempts:", attempt, "times."
    return str(traj)


def ExtractTrajectory(traj, t0, t1):
    """ExtractTrajectory extracts a trajectory segment from t = t0 to t = t1 in traj
    """
    assert(t1 > t0)
    
    newchunkslist = []
    i0, rem0 = traj.FindChunkIndex(t0)
    i1, rem1 = traj.FindChunkIndex(t1)

    inthesamechunk = (i0 == i1)

    ## append the traj segment from t0 to the end of chunk[i0]
    newpoly_list = []
    for p in traj.chunkslist[i0].polynomialsvector:
        ## perform variable changing of p(x) = a_n(x)^n + a_(n-1)(x)^(n-1) + ...
        ## by x = y + rem0
        
        a = p.q ## coefficient vector with python convention (highest degree first)
        ## a is a poly1d object
        r = a.r ## polynomial roots
        for i in range(len(r)):
            r[i] = r[i] - rem0
        b = np.poly1d(r, True) ## reconstruct a new polynomial from roots
        ## b is a poly1d object
        b = b*a.coeffs[0] ## multiply back by a_n *** this multiplication does not commute
        
        newpoly = Trajectory.Polynomial(b.coeffs.tolist()[::-1]) ## TOPP convention is weak-term-first
        newpoly_list.append(newpoly)
        
    if (inthesamechunk):
        remchunk0 = Trajectory.Chunk(rem1 - rem0, newpoly_list)
        newchunkslist.append(remchunk0)
    else:
        remchunk0 = Trajectory.Chunk(traj.chunkslist[i0].duration - rem0, newpoly_list)
        newchunkslist.append(remchunk0)

        ## append all chunks in between chunk[i0] and chunk[i1]
        for c in traj.chunkslist[i0 + 1: i1]:
            newchunkslist.append(c)

        ## append the traj segment from the beginning of chunk[i1] to t1
        remchunk1 = Trajectory.Chunk(rem1, traj.chunkslist[i1].polynomialsvector)
        newchunkslist.append(remchunk1)

    return Trajectory.PiecewisePolynomialTrajectory(newchunkslist)


def LinearlyScaleVelocities(traj, factor):
    """LinearlyScaleVelocities scales velocities of every dof by factor, i.e.
    new_traj_duration = original_traj_duration/factor
    """
    
    assert(not(factor == 0))
    newchunkslist = []
    
    for c in traj.chunkslist:
        # newpoly_list = []
        # for p in c.polynomialsvector:
        #     # coeffsvect = []
        #     # coeffsvect.append(p.coeff_list[0])
        #     # coeffsvect.append(p.coeff_list[1])
        #     # for i in range(2, len(p.coeff_list)):
        #     #     coeffsvect.append(p.coeff_list[i]/(factor**(i - 1)))
        #     # newpoly_list.append(Trajectory.Polynomial(coeffsvect))
        #     newpoly_list.append(p)
        # tempchunk = Trajectory.Chunk(c.duration/factor, newpoly_list)
        tempchunk = Trajectory.Chunk(c.duration/factor, c.polynomialsvector)
        newchunkslist.append(tempchunk)
    
    return Trajectory.PiecewisePolynomialTrajectory(newchunkslist)
        

def ReplaceTrajectorySegment(originaltraj, trajsegment, t0, t1):
    """ReplaceTrajectorySegment replaces the segment (t0, t1) in the (arbitrary degree) originaltraj 
    with an (arbitrary degree) trajsegment.
    """
    assert(t1 > t0)
    
    newchunkslist = []
    i0, rem0 = originaltraj.FindChunkIndex(t0)
    i1, rem1 = originaltraj.FindChunkIndex(t1)
             
    ## check if t0 falls in the first chunk. 
    ## if not, insert chunk 0 to chunk i0 - 1 into newchunkslist
    if i0 > 0:
        for c in originaltraj.chunkslist[0: i0]:
            newchunkslist.append(c)

    ## remainderchunk0
    remchunk0 = Trajectory.Chunk(rem0, originaltraj.chunkslist[i0].polynomialsvector)
    newchunkslist.append(remchunk0)

    ## insert trajsegment
    for c in trajsegment.chunkslist:
        newchunkslist.append(c)

    ## remainderchunk1
    newpoly_list = []
    for p in originaltraj.chunkslist[i1].polynomialsvector:
        ## perform variable changing of p(x) = a_n(x)^n + a_(n-1)(x)^(n-1) + ...
        ## by x = y + rem1
        
        a = p.q ## coefficient vector with python convention (highest degree first)
        ## a is a poly1d object
        r = a.r ## polynomial roots
        for i in range(len(r)):
            r[i] = r[i] - rem1
        b = np.poly1d(r, True) ## reconstruct a new polynomial from roots
        ## b is a poly1d object
        b = b*a.coeffs[0] ## multiply back by a_n *** this multiplication does not commute
        
        newpoly = Trajectory.Polynomial(b.coeffs.tolist()[::-1]) ## TOPP convention is weak-term-first
        newpoly_list.append(newpoly)
    remchunk1 = Trajectory.Chunk(originaltraj.chunkslist[i1].duration - rem1, newpoly_list)
    newchunkslist.append(remchunk1)
    
    ## insert remaining chunks
    if i1 < len(originaltraj.chunkslist) - 1:
        for c in originaltraj.chunkslist[i1 + 1: len(originaltraj.chunkslist)]:
            newchunkslist.append(c)

    return Trajectory.PiecewisePolynomialTrajectory(newchunkslist)


def CheckIntersection(interval0, interval1):
    """CheckIntersection checks whether interval0 intersects interval1.
    """
    
    if (np.max(interval0) < np.min(interval1)):
        return False

    elif (np.max(interval1) < np.min(interval0)):
        return False
    
    else:
        return True


def Normalize(vect0):
    vect_norm = np.linalg.norm(vect0)
    assert(not vect_norm == 0)
    return vect0/vect_norm


def OrthogonalSampling(vect0):
    n = len(vect0)
    r = random.SystemRandom()
    orthvect = []
    for i in range(n - 1):
        orthvect.append(2*r.random() - 1)
        
    ## compute the last element that make the equation
    ##     x0y0 + x1y1 + x2y2 + ... + xnyn = 0
    ## satisfied.
    orthvect.append(-(1.0/vect0[n - 1])*np.dot(np.asarray(orthvect), vect0[0:n - 1]))
    
    return Normalize(np.asarray(orthvect))


def IsInvertible(a):
    """
    Determine invertibility by Gaussian elimination
    code from stackexchange
    """
    a = np.array(a, dtype = np.bool_)
    n = a.shape[0]
    for i in range(n):
        pivots = np.where(a[i:, i])[0]
        if len(pivots) == 0:
            return False

        # swap pivot
        piv = i + pivots[0]
        row = a[piv, i:].copy()
        a[piv, i:] = a[i, i:]
        a[i, i:] = row

        # eliminate
        a[i + 1:, i:] -= a[i + 1:, i, None]*row[None, :]

    return True


############################## PLOTTING ##############################

def PlotDOF(robot, traj, dt, dof = -1):
    ndof = traj.dimension
    lowerdof_lim = robot.GetDOFLimits()[0]
    upperdof_lim = robot.GetDOFLimits()[1]
    q = []
    T = arange(0, traj.duration, dt)
    for t in T:
        q.append(traj.Eval(t))
    
    if (dof == -1):
        for i in range(ndof):
            plt.figure()
            x = [k[i] for k in q]
            plt.plot(T, x)
            plt.show(False)
            plt.hold(True)
            plt.plot([T[0], T[-1]], [lowerdof_lim[i], lowerdof_lim[i]], '--')
            plt.plot([T[0], T[-1]], [upperdof_lim[i], upperdof_lim[i]], '--')
    else:
        plt.figure()
        x = [k[dof] for k in q]
        plt.plot(T, x)
        plt.show(False)
        plt.hold(True)
        plt.plot([T[0], T[-1]], [lowerdof_lim[i], lowerdof_lim[i]], '--')
        plt.plot([T[0], T[-1]], [upperdof_lim[i], upperdof_lim[i]], '--')
            

def PlotdDOF(robot, traj, dt, dof = -1):
    ndof = traj.dimension
    vel_lim = robot.GetDOFVelocityLimits()
    qd = []
    T = arange(0, traj.duration, dt)
    for t in T:
        qd.append(traj.Evald(t))
    
    if (dof == -1):
        for i in range(ndof):
            plt.figure()
            xd = [k[i] for k in qd]
            plt.plot(T, xd)
            plt.show(False)
            plt.hold(True)
            plt.plot([T[0], T[-1]], [-vel_lim[i], -vel_lim[i]], '--')
            plt.plot([T[0], T[-1]], [vel_lim[i], vel_lim[i]], '--')
    else:
        plt.figure()
        xd = [k[dof] for k in qd]
        plt.plot(T, xd)
        plt.show(False)
        plt.hold(True)
        plt.plot([T[0], T[-1]], [-vel_lim[dof], -vel_lim[dof]], '--')
        plt.plot([T[0], T[-1]], [vel_lim[dof], vel_lim[dof]], '--')
        

def PlotddDOF(robot, traj, dt, dof = -1):
    ndof = traj.dimension
    acc_lim = robot.GetDOFAccelerationLimits()
    qdd = []
    T = arange(0, traj.duration, dt)
    for t in T:
        qdd.append(traj.Evaldd(t))
    
    if (dof == -1):
        for i in range(ndof):
            plt.figure()
            xdd = [k[i] for k in qdd]
            plt.plot(T, xdd)
            plt.show(False)
            plt.hold(True)
            plt.plot([T[0], T[-1]], [-acc_lim[i], -acc_lim[i]], '--')
            plt.plot([T[0], T[-1]], [acc_lim[i], acc_lim[i]], '--')
    else:
        plt.figure()
        xdd = [k[dof] for k in qdd]
        plt.plot(T, xdd)
        plt.show(False)
        plt.hold(True)
        plt.plot([T[0], T[-1]], [-acc_lim[dof], -acc_lim[dof]], '--')
        plt.plot([T[0], T[-1]], [acc_lim[dof], acc_lim[dof]], '--')
        
