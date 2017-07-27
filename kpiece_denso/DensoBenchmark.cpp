/* Author: Puttichai Lertkultanon */

#include <cstring>
#include <sstream>
#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <ompl/tools/benchmark/Benchmark.h>
#include "DensoStateSpace.cpp"
#include "DensoProjection.h"
#include "DensoStatePropagator.cpp"
#include "DensoControlSpace.cpp"
#include "DensoSetup.cpp"

#include <unistd.h>
#include <sys/stat.h>
#include <fstream>

inline bool exists(const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

int main(int argc, char *argv[])
{
    std::string scenefilename = "../../../cri1/robots/denso_base.xml";
    std::string viewername = "qtcoin";
    std::string collisioncheckername = "ode";

    // Start OpenRAVE coreg
    OpenRAVE::RaveInitialize(true);
    EnvironmentBasePtr penv = RaveCreateEnvironment();
    CollisionCheckerBasePtr pchecker = RaveCreateCollisionChecker(penv, collisioncheckername);
    penv->SetCollisionChecker(pchecker);
    penv->Load(scenefilename); // load the scene

    
    std::vector<RobotBasePtr> probots;
    penv->GetRobots(probots);
    RobotBasePtr probot = probots.at(0);

    std::vector<double> qddLimits;
    probot->GetDOFAccelerationLimits(qddLimits);
    double accelerationScalingFactor = 0.4;
    for (unsigned int i = 0; i < qddLimits.size(); i++)
	qddLimits[i] = qddLimits[i] * accelerationScalingFactor;
    probot->SetDOFAccelerationLimits(qddLimits);

    int ndof = atoi(argv[1]);
    std::vector<int> activedofs;
    for (int i = 0; i < ndof; i++)
	activedofs.push_back(i);
    probot->SetActiveDOFs(activedofs);
    std::cout << "nActiveDOFs is " << probot->GetActiveDOF() << "\n";
    
    // DensoSetup ds(probot);
    // ds.print();
    double maxTime = 600.0;
    unsigned int nRuns = 50;

    // // benchmark
    // ompl::tools::Benchmark b(ds, "Denso Experiment");
    // b.addPlanner(ob::PlannerPtr(new oc::KPIECE1(ds.getSpaceInformation())));
    // b.benchmark(ompl::tools::Benchmark::Request(maxTime, 10000.0, nRuns));
    // b.saveResultsToFile("kpiece1_benchmark.data");

    std::cout << "====================\n";
    std::cout << "ndof                   " << ndof << "\n";
    std::cout << "theshold               " << threshold << "\n";
    std::cout << "projection type        " << projectionType << "\n";
    std::cout << "cell size              " << cellSize << "\n";
    std::cout << "propagation step size  " << propagationStepSize << "\n";
    std::cout << "goal bias              " << goalBias << "\n";
    std::cout << "====================\n";

    std::vector<double> runningTime;
    runningTime.resize(0);
    double avgRunningTime = 0.0;
    unsigned int nSuccess = 0;
    double successRate;
    
    std::stringstream outfilename;
    if (cellSize == 0.05) {
	outfilename << "data/KPIECE1_" << ndof << "DOF.data";
    }
    else if (cellSize == 0.1) {    
	outfilename << "data/KPIECE2_" << ndof << "DOF.data";
    }
    else if (cellSize == 1.0) { 
	outfilename << "data/KPIECE3_" << ndof << "DOF.data";
    }
    else {
	outfilename << "data/KPIECE4_" << ndof << "DOF.data";
    }
    std::cout << outfilename.str() << std::endl;

    for (unsigned int r = 0; r < nRuns; r++)
    {
	DensoSetup ds(probot);
	std::cout << "iteration " << r + 1 << "\n";
	if (ds.solve(maxTime) && ds.haveExactSolutionPath())
	{
	    nSuccess = nSuccess + 1;
	    runningTime.push_back(ds.getLastPlanComputationTime());
	    avgRunningTime = avgRunningTime + ds.getLastPlanComputationTime();
	}	
    }
    if (nSuccess > 0)
    {
	avgRunningTime = avgRunningTime / nSuccess;
    }
    successRate = nSuccess / nRuns;
    
    std::string outputFileName = outfilename.str();//"../data/KPIECE_var_dof.data";
    std::ofstream outFile;
    outFile.open(outputFileName.c_str(), std::ios::app);
    std::stringstream newData;
    if (!exists(outputFileName))
    {
	std::stringstream desc;
	desc << "NDOF" 
	     << " THRESHOLD" 
	     << " PROJECTION_CHOICE" 
	     << " CELL_SIZE" 
	     << " PROPAGATION_STEPSIZE" 
	     << " GOAL_BIAS" 
	     << " MAX_TIME" 
	     << " AVG_RUNNING_TIME"
	     << " SUCCESS_RATE"
	     << " NRUNS\n";
	newData << desc.str();
    }
    
    std::string separator = " ";
    newData << ndof 
	    << separator << threshold
	    << separator << projectionType
	    << separator << cellSize
	    << separator << propagationStepSize
	    << separator << goalBias
	    << separator << maxTime
	    << separator << avgRunningTime
	    << separator << successRate
	    << separator << nRuns << "\n";
    separator = "";
    for (unsigned int i = 0; i < nRuns; i++)
    {
	newData << separator << runningTime[i];
	separator = ", ";
    }
    newData << "\n";
    // std::cout << newData.str() << "\n";
    outFile << newData.str();
    // no need for explicit file closing

    std::cout << "====================\n";
    std::cout << "ndof                   " << ndof << "\n";
    std::cout << "theshold               " << threshold << "\n";
    std::cout << "projection type        " << projectionType << "\n";
    std::cout << "cell size              " << cellSize << "\n";
    std::cout << "propagation step size  " << propagationStepSize << "\n";
    std::cout << "goal bias              " << goalBias << "\n";
    std::cout << "average running time   " << propagationStepSize << "\n";
    std::cout << "success rate           " << successRate << "\n";
    std::cout << "====================\n";
    
    return 0;
}
