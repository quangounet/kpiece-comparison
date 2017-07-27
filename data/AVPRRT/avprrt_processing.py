import numpy as np

rawdata = dict()
avgplanningtime = dict()
avgtopptime = dict()
avgtotaltime = dict()
for i in xrange(2, 13):
    filename = "AVPRRT_{0}DOFs.data".format(i)
    with open(filename, 'r') as f:
        rawdata[i] = f.read().split('\n')

    assert(int(rawdata[i][0]) == i)
    # Due to error in the running script, a planning time for each run
    # is recorded twice.
    
    # Also, since we only conduct 20 runs for each of KPIECE case, we
    # use only the first 20 entries of AVPRRT results.
    nRuns = 20
    plantime = rawdata[i][1].split()
    topptime = rawdata[i][2].split()
    assert(len(plantime) == 2*len(topptime))

    tp = 0
    tt = 0
    for j in xrange(nRuns):
        assert(plantime[2*j] == plantime[2*j + 1]) # check soundness
        tp += float(plantime[2*j])
        tt += float(topptime[j])

    avgplanningtime[i] = tp/nRuns
    avgtopptime[i] = tt/nRuns
    avgtotaltime[i] = (tp + tt)/nRuns
        
    
