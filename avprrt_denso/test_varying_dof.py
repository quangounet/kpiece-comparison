import openravepy as orpy
import numpy as np
import time
import TOPP
import AVPRRT
import Utils

showviewer = False
envname = '../../cri1/robots/denso_base.xml'
ccheckername = 'ode' 

env = orpy.Environment()
env.Load(envname)

collisionchecker = orpy.RaveCreateCollisionChecker(env, ccheckername)
env.SetCollisionChecker(collisionchecker)
if showviewer:
    env.SetViewer('qtcoin')

robot = env.GetRobots()[0]
[qL, qU] = robot.GetDOFLimits()
vmax = robot.GetDOFVelocityLimits()
amax_prev = robot.GetDOFAccelerationLimits()
ascale = 0.4
robot.SetDOFAccelerationLimits(ascale*amax_prev)
amax = robot.GetDOFAccelerationLimits()

ndof = int(raw_input("Please enter the number of DOFs: "))

if (ndof > 6):
    for i in xrange(6, ndof):
        np.asarray(qU.tolist().append(qU[i - 6]))
        np.asarray(qL.tolist().append(qU[i - 6]))
        np.asarray(vmax.tolist().append(vmax[i - 6]))
        np.asarray(amax.tolist().append(amax[i - 6]))
else:
    qU = qU[0:ndof]
    qL = qL[0:ndof]
    vmax = vmax[0:ndof]
    amax = amax[0:ndof]
robot.SetActiveDOFs(xrange(ndof))

integrationtimestep0 = 5e-3
reparamtimestep0 = 1e-2
passswitchpointnsteps0 = 5
tuningsstring0 = "%f %f %d"%(integrationtimestep0, reparamtimestep0, passswitchpointnsteps0)

discrtimestep0 = 0.5e-3
constraintsstring0 = str(discrtimestep0)
constraintsstring0 += "\n" + ' '.join([str(v) for v in vmax])
constraintsstring0 += "\n" + ' '.join([str(a) for a in amax])

q_start = np.zeros(ndof)
q_goal = np.ones(ndof)

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
    integrationtimestep1 = 1e-3
    reparamtimestep1 = 1e-2
    passswitchpointnsteps1 = 5
    discrtimestep1 = 0.5e-3
    constraintsstring1 = str(discrtimestep1)
    constraintsstring1 += "\n" + ' '.join([str(v) for v in vmax])
    constraintsstring1 += "\n" + ' '.join([str(a) for a in amax])

    x = TOPP.TOPPbindings.TOPPInstance(None, problemtype,
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
avgrunningtime = avgplantime + avgtopptime
print "Average planning time = {0} s.".format(avgplantime)
print "Average reparamaterization time = {0} s.".format(avgtopptime)
print "Average running time = {0} s.".format(avgrunningtime)

FILENAME = 'data/AVPRRT_{0}DOF.data'.format(ndof)

# with open(FILENAME, 'r') as f:
#     datastring = f.read()

datastring = "{0} {1} {2} {3}\n".format(ndof, avgplantime, avgtopptime, avgrunningtime)
with open(FILENAME, 'w') as f:
    f.write(datastring)

