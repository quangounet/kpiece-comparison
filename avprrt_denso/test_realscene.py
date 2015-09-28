from openravepy import *
import numpy as np
import time
import TOPP
import AVPRRT
import Utils

env = Environment()
env.Load('../xml/env3.xml')

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

# q_start = np.zeros(ndof)
# q_goal = np.ones(ndof)
q_start = np.array([ -1.57599994e+00,   1.37400000e+00,   1.39095846e-10,
                      -7.61664885e-08,  -1.36751533e+00,   1.61500001e+00])
q_goal = np.array([  1.55000001,  1.34999999,  0.10000005, 
                    -0.00570361, -1.40754354,  1.60000003])
robot.SetDOFValues(q_start)

v = 1e-6
qd_start = v*np.ones(ndof)
qd_goal = v*np.ones(ndof)

qdd_start = np.zeros(ndof)
qdd_goal = np.zeros(ndof)

problemtype = "KinematicLimits"
sdbeg = 0.0
sdend = 0.0
allottedtime = 1000.0
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

nruns = 10
plantime = []
topptime = []
nnodes = []
ntrajs = []
for i in xrange(nruns):
    print "iteration: {0}".format(i + 1)
    birrtinstance = AVPRRT.AVPBiRRTPlanner(v_start, v_goal, robot, problemtype, 
                                           constraintsstring0, tuningsstring0, 
                                           nn, metrictype, polynomialdegree)
    birrtinstance.Run(allottedtime)
    nnodes.append(len(birrtinstance.treestart) + 1) # include the goal node in treeend
    ntrajs.append(birrtinstance.treestart[-1].level + 1)
    time_topp_start = time.time()

    trajectorystring0 = birrtinstance.GenerateFinalTrajectoryString()
    #traj0 = TOPP.Trajectory.PiecewisePolynomialTrajectory.FromString(trajectorystring0)

    #-------------------- TOPP --------------------
    integrationtimestep1 = 0.001
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
        print "Reparameterization Error!"

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

    plantime.append(birrtinstance.runningtime)
    topptime.append(time_topp_end - time_topp_start)
    print "TOPP running time = {0} s.".format(topptime[-1])

avgplantime = np.sum(plantime)/len(plantime)
avgtopptime = np.sum(topptime)/len(topptime)
runningtime = [plantime[i] + topptime[i] for i in range(nruns)]
avgrunningtime = avgplantime + avgtopptime
print "Average planning time = {0} s.".format(avgplantime)
print "Average reparamaterization time = {0} s.".format(avgtopptime)
print "Average running time = {0} s.".format(avgrunningtime)

manip = robot.GetManipulator('Flange')
P = []
for t in np.arange(0.0, traj1.duration, discrtimestep0):
    robot.SetDOFValues(traj1.Eval(t))
    P.append(manip.GetTransform()[0:3, 3])
    time.sleep(discrtimestep0)
robot.SetDOFValues(traj1.Eval(traj1.duration))
handle = env.drawlinestrip(points = np.vstack((P)), linewidth = 3)

# import pickle
# with open('data/test_realscene_raw.data', 'w') as f:
#     pickle.dump([plantime, topptime, runningtime], f, pickle.HIGHEST_PROTOCOL)

# import pickle
# with open('data/test_realscene_raw.pkl', 'wb') as f:
#     pickle.dump([plantime, topptime, runningtime, nnodes, ntrajs], f,
#                 pickle.HIGHEST_PROTOCOL)
"""
import pickle
with open('data/test_realscene_raw.pkl2', 'wb') as f:
    pickle.dump([plantime, topptime, runningtime, nnodes, ntrajs], f,
                pickle.HIGHEST_PROTOCOL)
"""
"""
average values over 50 runs (previous runs)

runningtime = [5.7289955615997314, 3.728551149368286, 5.085183143615723, 42.61051297187805, 1.9713568687438965, 10.060971975326538, 40.47702503204346, 6.290570974349976, 1.9281201362609863, 20.104873180389404, 2.8543167114257812, 20.316935539245605, 7.833720684051514, 1.9339323043823242, 16.555227994918823, 3.9452900886535645, 1.8890738487243652, 28.509686946868896, 17.146886348724365, 6.281855821609497, 20.728729963302612, 7.276635408401489, 5.439096450805664, 5.087064027786255, 2.982445001602173, 1.8686509132385254, 1.8894572257995605, 1.8750789165496826, 1.8941318988800049, 8.264504194259644, 1.9643259048461914, 2.8591365814208984, 5.687114000320435, 4.818651437759399, 13.346293687820435, 44.32132053375244, 6.816420793533325, 1.9035229682922363, 21.507760763168335, 30.70221495628357, 1.891308069229126, 13.07431411743164, 1.8909273147583008, 9.522746562957764, 1.9804646968841553, 4.918659687042236, 1.956864833831787, 1.9205901622772217, 23.1201114654541, 4.372236251831055]
"""

"""
example trajectorystring for the case when the opening is translated upward (+13 cm.)
(env3.xml)

average:
runningtime = 68.67
nodes = 60.15
trajs = 8.6

trajectorystring = '0.500000\n6\n-1.576000 0.000001 0.000000 -11.509616 37.445722 -31.123328\n1.374000 0.000001 0.000000 16.408677 -51.455608 42.056317\n0.000000 0.000001 0.000000 7.798584 -19.675050 14.251760\n-0.000000 0.000001 0.000000 -54.386006 167.406195 -135.624227\n-1.367515 0.000001 0.000000 31.896498 -95.415425 76.222712\n1.615000 0.000001 0.000000 31.641596 -99.232343 81.108895\n0.500000\n6\n-1.646948 0.364610 0.000000 28.085354 -91.009963 75.509531\n1.523370 -0.278696 0.000000 -5.005301 19.635054 -17.555704\n0.190500 0.465089 0.000000 -13.856172 36.763821 -27.489179\n-0.573620 0.531023 0.000000 -40.974481 120.339809 -95.238394\n-0.961957 0.034260 0.000000 -0.061395 2.137092 -2.490837\n1.902832 -0.538443 0.000000 74.965602 -226.476704 181.813322\n0.500000\n6\n-1.282423 -0.479627 0.000000 -21.782038 72.170011 -60.465568\n1.436934 0.298698 0.000000 -6.180304 17.644439 -13.756963\n0.129725 -0.135498 0.000000 28.229515 -85.768534 69.046823\n-0.884881 0.208069 0.000000 18.938735 -55.485292 43.855869\n-0.896772 0.278373 0.000000 -12.919004 37.308645 -29.267569\n2.531183 -0.735930 0.000000 49.849515 -137.449234 105.119663\n0.500000\n6\n-1.623915 0.373360 0.000000 33.206942 -105.477477 86.724642\n1.486617 0.186639 0.000000 -20.199553 56.626269 -43.712059\n0.387845 -0.270497 0.000000 -20.554183 66.071324 -54.620568\n-0.510839 0.374433 0.000000 19.457083 -57.804942 46.017431\n-0.955282 0.097327 0.000000 10.591648 -31.223510 24.758235\n3.088819 0.776484 0.000000 -11.595772 34.063907 -26.961762\n0.500000\n6\n-1.168564 -0.358722 0.000000 -9.912842 34.982758 -30.083898\n1.228133 -0.309910 0.000000 -36.670134 114.919645 -93.899413\n0.105889 0.280600 0.000000 0.787049 -8.104147 8.780518\n-0.066251 0.445221 0.000000 42.608337 -135.704358 111.715226\n-0.760437 0.166256 0.000000 -0.232186 3.769973 -4.245345\n3.314029 0.686058 0.000000 -4.816958 10.441537 -6.749495\n0.500000\n6\n-1.340730 0.296807 0.000000 23.716601 -70.740145 56.428252\n0.737532 0.303746 0.000000 -27.323038 78.262558 -61.127424\n0.112452 -0.437275 0.000000 20.764125 -56.897645 43.360224\n0.491980 -0.539697 0.000000 55.221374 -164.125833 130.685351\n-0.603376 0.550433 0.000000 -29.032630 84.716937 -66.821169\n3.496612 0.184891 0.000000 7.043918 -16.349315 11.166477\n0.500000\n6\n-0.885627 0.348015 0.000000 31.396099 -95.009163 76.335677\n0.455203 -0.159574 0.000000 17.735908 -47.347205 35.533557\n0.288234 0.237066 0.000000 12.889286 -41.260454 34.045401\n0.950856 -0.347411 0.000000 14.908932 -37.643054 27.280947\n-0.750591 0.252814 0.000000 15.886655 -47.157639 37.525180\n3.796668 0.782695 0.000000 -46.226304 136.124857 -107.878263\n0.500000\n6\n-0.339690 0.245406 0.000000 27.243874 -86.685401 71.329833\n0.743628 0.572991 0.000000 -13.056698 34.324798 -25.521721\n0.503068 -0.087008 0.000000 34.063754 -102.856019 82.550717\n1.140606 0.538057 0.000000 43.861542 -140.039260 115.413262\n-0.413042 0.315605 0.000000 -2.771926 4.037225 -1.518358\n3.546335 0.463438 0.000000 -11.968627 26.468008 -17.399257\n0.400000\n6\n-0.000283 -0.373816 0.000000 87.966710 -333.338635 335.070372\n0.745782 -0.032671 0.000000 -27.674364 103.810052 -103.825646\n0.868742 -0.170103 0.000000 -51.177699 202.405596 -207.650208\n1.746537 -0.518773 0.000000 73.397038 -263.959114 258.319226\n-0.396852 -0.219214 0.000000 -14.776309 57.880801 -59.115623\n3.392500 -0.716296 0.000000 113.337503 -423.044277 422.058597\n0.256000\n6\n0.377711 -0.595479 0.000000 753.256268 -4378.117597 6813.079568\n0.555917 -0.030675 0.000000 476.118682 -2787.929463 4354.711317\n0.580574 0.501208 0.000000 -332.331114 1917.378366 -2972.564436\n2.124274 0.203133 0.000000 -1288.163226 7535.723780 -11765.109327\n-0.553817 -0.061157 0.000000 -503.261637 2952.443936 -4616.041527\n3.851528 -0.590129 0.000000 -1287.987502 7581.976238 -11874.317926'

"""
