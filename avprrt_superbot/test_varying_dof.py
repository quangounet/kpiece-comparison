import openravepy as orpy
import numpy as np
import time
import TOPP
import AVPRRT
import Utils

showviewer = False
envname = '../xml/superbot.xml'
ccheckername = 'ode'

problemtype = "KinematicLimits"
integrationtimestep0 = 1e-3
reparamtimestep0 = 1e-2
passswitchpointnsteps0 = 5
discrtimestep0 = 0.5e-3
integrationtimestep1 = 1e-3
reparamtimestep1 = 1e-2
passswitchpointnsteps1 = 5
discrtimestep1 = 0.5e-3
sdbeg = 0.0
sdend = 0.0

allottedtime = 600.0
nn = 10
metrictype = 1
polynomialdegree = 5
nRuns = 50

env = orpy.Environment()
env.Load(envname)

collisionchecker = orpy.RaveCreateCollisionChecker(env, ccheckername)
env.SetCollisionChecker(collisionchecker)
if showviewer:
    env.SetViewer('qtcoin')

robot = env.GetRobots()[0]
robot.SetActiveManipulator('second_Flange')

[xMin, xMax] = robot.GetDOFLimits()
vMax = robot.GetDOFVelocityLimits()
realAcc = robot.GetDOFAccelerationLimits()
aScale = 0.4
aMax = aScale * realAcc
robot.SetDOFAccelerationLimits(aMax)

ndofs = xrange(2, 13)
for ndof in ndofs:
    print "nDOF = {0}".format(ndof)
    robot.SetActiveDOFs(range(ndof))
    
    qL = xMin[0:ndof]
    qU = xMax[0:ndof]
    qdU = vMax[0:ndof]
    qddU = aMax[0:ndof]
    
    tuningsstring0 = "%f %f %d"%(integrationtimestep0, reparamtimestep0, passswitchpointnsteps0)    
    constraintsstring0 = str(discrtimestep0)
    constraintsstring0 += "\n" + ' '.join([str(v) for v in qdU])
    constraintsstring0 += "\n" + ' '.join([str(a) for a in qddU])

    qStart = np.zeros(ndof)
    qGoal = np.ones(ndof)

    v = 1e-6
    qdStart = v*np.ones(ndof)
    qdGoal = v*np.ones(ndof)

    qddStart = np.zeros(ndof)
    qddGoal = np.zeros(ndof)
    
    plantime = []
    topptime = []
    for i in xrange(nRuns):
        print "iteration {0}".format(i + 1)
        
        cStart = AVPRRT.Config(qStart, qdStart, qddStart)
        vStart = AVPRRT.Vertex(cStart, AVPRRT.FW)
        vStart.sdmin = sdbeg
        vStart.sdmax = sdbeg
        
        cGoal = AVPRRT.Config(qGoal, qdGoal, qddGoal)
        vGoal = AVPRRT.Vertex(cGoal, AVPRRT.BW)
        vGoal.sdmin = sdend
        vGoal.sdmax = sdend
        
        birrtinstance = AVPRRT.AVPBiRRTPlanner(vStart, vGoal, robot, problemtype, 
                                               constraintsstring0, tuningsstring0, 
                                               nn, metrictype, polynomialdegree)
        birrtinstance.Run(allottedtime)
        plantime.append(birrtinstance.runningtime)
        if not birrtinstance.found:
            continue

        tsTOPP = time.time()
        trajectorystring0 = birrtinstance.GenerateFinalTrajectoryString()
        constraintsstring1 = str(discrtimestep1)
        constraintsstring1 += "\n" + ' '.join([str(v) for v in qdU])
        constraintsstring1 += "\n" + ' '.join([str(a) for a in qddU])

        x = TOPP.TOPPbindings.TOPPInstance(robot, problemtype, constraintsstring1, trajectorystring0)
        x.integrationtimestep = integrationtimestep1
        x.reparamtimestep = reparamtimestep1
        x.extrareps = 5
        x.passswitchpointnsteps = passswitchpointnsteps1
        
        ret = x.RunComputeProfiles(sdbeg, sdend)
        if (ret == 1):
            x.ReparameterizeTrajectory()
        else:
            continue

        x.WriteProfilesList()
        x.WriteSwitchPointsList()
        x.WriteResultTrajectory()
        traj1 = TOPP.Trajectory.PiecewisePolynomialTrajectory.FromString(x.restrajectorystring)
        teTOPP = time.time()

        plantime.append(birrtinstance.runningtime)
        topptime.append(teTOPP - tsTOPP)

        print "AVPRRT running time = {0} sec.".format(plantime[-1])
        print "TOPP running time = {0} sec.".format(topptime[-1])

    avgplantime = np.average(plantime)
    avgtopptime = np.average(topptime)
    avgrunningtime = avgplantime + avgtopptime
    print "nDOF = {0}".format(ndof)
    print "Average planning time = {0} s.".format(avgplantime)
    print "Average reparamaterization time = {0} s.".format(avgtopptime)
    print "Average running time = {0} s.".format(avgrunningtime)

    FILENAME = 'data/AVPRRT_{0}DOFs.data'.format(ndof)
    datastring = "{0}\n".format(ndof)
    separator = ""
    for t in plantime:
        datastring += separator
        separator = " "
        datastring += str(t)

    separator = "\n"
    for t in topptime:
        datastring += separator
        separator = " "
        datastring += str(t)

    # Write a separate file for each DOF
    with open(FILENAME, 'w') as f:
        f.write(datastring)

