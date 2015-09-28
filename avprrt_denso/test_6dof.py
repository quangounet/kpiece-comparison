from openravepy import *
import numpy as np
import time
import TOPP
import AVPRRT
import Utils

env = Environment()
env.Load('../../robots/denso_base.xml')

collisionchecker = RaveCreateCollisionChecker(env, 'ode')
env.SetCollisionChecker(collisionchecker)
env.SetViewer('qtcoin')

robot = env.GetRobots()[0]
ndof = robot.GetActiveDOF()
vmax = robot.GetDOFVelocityLimits()
amax = robot.GetDOFAccelerationLimits()
robot.SetDOFAccelerationLimits(0.4*amax)
amax = robot.GetDOFAccelerationLimits()

integrationtimestep0 = 0.005
reparamtimestep0 = 0.01
passswitchpointnsteps0 = 5
tuningsstring0 = "%f %f %d"%(integrationtimestep0, 
                             reparamtimestep0, 
                             passswitchpointnsteps0)

discrtimestep0 = 0.0005
constraintsstring0 = str(discrtimestep0)
constraintsstring0 += "\n" + ' '.join([str(v) for v in vmax])
constraintsstring0 += "\n" + ' '.join([str(a) for a in amax])

q_start = np.zeros(ndof)
q_goal = np.ones(ndof)
robot.SetDOFValues(q_start)

v = 1e-6
qd_start = v*np.ones(ndof)
qd_goal = v*np.ones(ndof)

qdd_start = np.zeros(ndof)
qdd_goal = np.zeros(ndof)

problemtype = "KinematicLimits"
sdbeg = 0.0
sdend = 0.0
allottedtime = 600.0
nn = 10
metrictype = 1
polynomialdegree = 5

c_start = AVPRRT.Config(q_start, qd_start, qdd_start)
v_start = AVPRRT.Vertex(c_start, AVPRRT.FW)
v_start.sdmin = sdbeg
v_start.sdmax = sdbeg

c_goal = AVPRRT.Config(q_goal, qd_goal, qdd_goal)
v_goal = AVPRRT.Vertex(c_goal, AVPRRT.BW)
v_goal.sdmin = sdend
v_goal.sdmax = sdend

nruns = 50
plantime = []
topptime = []
for i in xrange(nruns):
    print "iteration: {0}".format(i + 1)
    birrtinstance = AVPRRT.AVPBiRRTPlanner(v_start, v_goal, robot, problemtype, 
                                           constraintsstring0, tuningsstring0, 
                                           nn, metrictype, polynomialdegree)
    birrtinstance.Run(allottedtime)
    plantime.append(birrtinstance.runningtime)
    time_topp_start = time.time()

    trajectorystring0 = birrtinstance.GenerateFinalTrajectoryString()
    #traj0 = TOPP.Trajectory.PiecewisePolynomialTrajectory.FromString(trajectorystring0)

    #-------------------- TOPP --------------------
    integrationtimestep1 = 0.0005
    reparamtimestep1 = 0.01
    passswitchpointnsteps1 = 5
    discrtimestep1 = 0.0005
    constraintsstring1 = str(discrtimestep1)
    constraintsstring1 += "\n" + ' '.join([str(v) for v in vmax])
    constraintsstring1 += "\n" + ' '.join([str(a) for a in amax])

    x = TOPP.TOPPbindings.TOPPInstance(robot, problemtype,
                                       constraintsstring1, trajectorystring0)
    x.integrationtimestep = integrationtimestep1
    x.reparamtimestep = reparamtimestep1
    x.extrareps = 5
    x.passswitchpointnsteps = passswitchpointnsteps1

    ret = x.RunComputeProfiles(0.0, 0.0)
    if (ret == 1):
        x.ReparameterizeTrajectory()
    else:
        print "ERROR"
        raw_input()

    x.WriteProfilesList()
    x.WriteSwitchPointsList()
    # profileslist = TOPP.TOPPpy.ProfilesFromString(x.resprofilesliststring)
    # switchpointslist = TOPP.TOPPpy.SwitchPointsFromString(x.switchpointsliststring)
    # TOPP.TOPPpy.PlotProfiles(profileslist, switchpointslist, 4)
    # from matplotlib import pyplot as plt
    # plt.show(False)

    if (ret == 1):
        x.WriteResultTrajectory()
        traj1 = TOPP.Trajectory.PiecewisePolynomialTrajectory.FromString\
        (x.restrajectorystring)
    time_topp_end = time.time()
    topptime.append(time_topp_end - time_topp_start)
    print "TOPP running time = {0} s.".format(topptime[-1])

avgplantime = np.sum(plantime)/len(plantime)
avgtopptime = np.sum(topptime)/len(topptime)
runningtime = [plantime[i] + topptime[i] for i in range(nruns)]
avgrunningtime = avgplantime + avgtopptime
print "Average planning time = {0} s.".format(avgplantime)
print "Average reparamaterization time = {0} s.".format(avgtopptime)
print "Average running time = {0} s.".format(avgrunningtime)

sd = np.std(runningtime)

import pickle
with open('data/test_6dof_raw.data', 'w') as f:
    pickle.dump([plantime, topptime, runningtime], f, pickle.HIGHEST_PROTOCOL)

# for t in np.arange(0.0, traj.duration, discrtimestep0):
#     robot.SetDOFValues(traj.Eval(t))
#     time.sleep(discrtimestep0)
# robot.SetDOFValues(traj.Eval(traj.duration))


"""
integrationtimestep0 = 0.005
reparamtimestep0 = 0.01
passswitchpointnsteps0 = 5
discrtimestep0 = 0.0005
integrationtimestep1 = 0.0005
reparamtimestep1 = 0.01
passswitchpointnsteps1 = 5
discrtimestep1 = 0.0005

results
Average planning time = 0.543576159477 s.
Average reparamaterization time = 1.39376610756 s.
Average running time = 1.93734226704 s.
"""
