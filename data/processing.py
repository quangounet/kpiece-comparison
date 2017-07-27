import os

# AVPRRT
# data format:
# first line : nDOF
# second line: planning time for each run
# third line : topp time for each run
rawdata1 = dict()
avgplanningtime1 = dict()
avgtopptime1 = dict()
avgtotaltime1 = dict()
for i in xrange(2, 13):
    filename = "AVPRRT/AVPRRT_{0}DOFs.data".format(i)
    with open(filename, 'r') as f:
        rawdata1[i] = f.read().split('\n')

    assert(int(rawdata1[i][0]) == i)
    # Due to error in the running script, a planning time for each run
    # is recorded twice.
    
    # Also, since we only conduct 20 runs for each of KPIECE case, we
    # use only the first 20 entries of AVPRRT results.
    nRuns = 20
    plantime1 = rawdata1[i][1].split()
    topptime1 = rawdata1[i][2].split()
    assert(len(plantime1) == 2*len(topptime1))

    tp = 0
    tt = 0
    for j in xrange(nRuns):
        assert(plantime1[2*j] == plantime1[2*j + 1]) # check soundness
        tp += float(plantime1[2*j])
        tt += float(topptime1[j])

    avgplanningtime1[i] = tp/nRuns
    avgtopptime1[i] = tt/nRuns
    avgtotaltime1[i] = (tp + tt)/nRuns

# KPIECE
# data format:
# first line : parameter names
# second line: parameter values
# third line : running time for each successful run
"""
Parameters:
NDOF THRESHOLD PROJECTION_CHOICE CELL_SIZE PROPAGATION_STEPSIZE GOAL_BIAS MAX_TIME AVG_RUNNING_TIME SUCCESS_RATE NRUNS

"""
rawdata2 = dict()
data2 = dict()
cellSizes = [0.05, 0.1, 1]
for cellSize in cellSizes:
    prevExists = True
    rawdata2[cellSize] = dict()
    data2[cellSize] = dict()
    for i in xrange(2, 13):
        filename = "KPIECE/KPIECE_{0}_{1:d}DOFs.data".format(cellSize, i)
        if not os.path.isfile(filename):
            # If for DOF = n there is no data, the program should
            # already terminate, i.e., there shuold be no data for
            # higher DOFs.
            prevExists = False
            continue
        else:
            assert(prevExists)

            data2[cellSize][i] = dict()
            
            # Start extracting data
            with open(filename, 'r') as f:
                rawdata2[cellSize][i] = f.read().split('\n')
            data2[cellSize][i]['runningtime'] = [float(x) for x in rawdata2[cellSize][i][2].split()]
            nSuccess = len(data2[cellSize][i]['runningtime'])
            data2[cellSize][i]['successrate'] = nSuccess/20.0
            if nSuccess > 0:
                data2[cellSize][i]['avgrunningtime'] = sum(data2[cellSize][i]['runningtime'])/nSuccess


import matplotlib.pyplot as plt
import pylab; pylab.ion()


KPIECEavgtime = []
dofindices = []
for (i, cellSize) in enumerate(cellSizes):
    KPIECEavgtime.append([])
    dofindices.append([])
    for j in xrange(2, 13):
        if data2[cellSize].has_key(j):
            if data2[cellSize][j].has_key('avgrunningtime'):
                dofindices[i].append(j)
                KPIECEavgtime[i].append(data2[cellSize][j]['avgrunningtime'])
            else:
                break
        else:
            break

fig = plt.figure(1)
ax = fig.add_subplot(1, 1, 1)
plt.plot(dofindices[0], KPIECEavgtime[0], 'r^-', linewidth=2, markersize=10, label="KPIECE (cell size = 0.05)")
plt.plot(dofindices[1], KPIECEavgtime[1], 'go:', linewidth=2, markersize=10, label="KPIECE (cell size = 0.1)")
plt.plot(dofindices[2], KPIECEavgtime[2], 'b*--', linewidth=2, markersize=10, label="KPIECE (cell size = 1.0)")

dofindices = range(2, 13)
avpavgtime = [avgtotaltime1[x] for x in dofindices]
plt.plot(range(2, len(avgtotaltime1) + 2), avpavgtime, 'ks-.', linewidth=2, markersize=10, label="AVP-RRT")

plt.plot([1.5, 12.5], [200, 200], 'm', linewidth=2, label="time limlit")
ax.set_yscale('log')
plt.legend(loc=4)

fontsize = 22
labelpad = 8
labelsize = 20
plt.xlabel("n DOFs", fontsize=fontsize, labelpad=labelpad)
plt.ylabel("Average running time (s.)", fontsize=fontsize, labelpad=labelpad)
g = plt.gca()
g.tick_params(axis='x', labelsize=labelsize)
g.tick_params(axis='y', labelsize=labelsize)
plt.axis([1.5, 12.5, 0.1, 1000])
plt.tight_layout()



plt.figure(2)
successrate1 = [100*data2[0.05][x]['successrate'] for x in range(2, 13) if data2[0.05].has_key(x)]
successrate2 = [100*data2[0.1][x]['successrate'] for x in range(2, 13) if data2[0.1].has_key(x)]
successrate3 = [100*data2[1.0][x]['successrate'] for x in range(2, 13) if data2[1.0].has_key(x)]

while len(successrate1) < 11:
    successrate1.append(0)
while len(successrate2) < 11:
    successrate2.append(0)
while len(successrate3) < 11:
    successrate3.append(0)

dofindices = range(2, 13)
plt.plot(range(2, 13), successrate1, 'r^-', linewidth=2, markersize=10, label="KPIECE (cell size = 0.05)")
plt.plot(range(2, 13), successrate2, 'go:', linewidth=2, markersize=10, label="KPIECE (cell size = 0.1)")
plt.plot(range(2, 13), successrate3, 'b*--', linewidth=2, markersize=10, label="KPIECE (cell size = 1.0)")
plt.plot(range(2, 13), [100]*11, 'ks-.', linewidth=2, markersize=10, label="AVP-RRT")
plt.legend(loc=3)

fontsize = 22
labelpad = 8
labelsize = 20
plt.xlabel("n DOFs", fontsize=fontsize, labelpad=labelpad)
plt.ylabel("Success rate (%)", fontsize=fontsize, labelpad=labelpad)
g = plt.gca()
g.tick_params(axis='x', labelsize=labelsize)
g.tick_params(axis='y', labelsize=labelsize)
plt.axis([1.5, 12.5, 0, 100])
plt.tight_layout()
