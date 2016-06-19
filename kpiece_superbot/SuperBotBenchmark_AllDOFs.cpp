/* Author: Puttichai Lertkultanon */

#include <cstring>
#include <sstream>
#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include <ompl/tools/benchmark/Benchmark.h>
#include "SuperBotStateSpace.cpp"
#include "SuperBotProjection.h"
#include "SuperBotStatePropagator.cpp"
#include "SuperBotControlSpace.cpp"
#include "SuperBotSetup.cpp"

#include <unistd.h>
#include <sys/stat.h>
#include <fstream>

inline bool exists(const std::string& name) {
  struct stat buffer;   
  return (stat (name.c_str(), &buffer) == 0); 
}

int main(int argc, char *argv[]) {
    std::string scenefilename = "../../xml/superbot.xml";
    std::string viewername = "qtcoin";
    std::string collisioncheckername = "ode";
    std::string manipulatorname = "second_Flange";

    // Start OpenRAVE core
    OpenRAVE::RaveInitialize(true);
    EnvironmentBasePtr penv = RaveCreateEnvironment();
    CollisionCheckerBasePtr pchecker = RaveCreateCollisionChecker(penv, collisioncheckername);
    penv->SetCollisionChecker(pchecker);
    penv->Load(scenefilename); // load the scene

    std::vector<RobotBasePtr> probots;
    penv->GetRobots(probots);
    RobotBasePtr probot = probots.at(0);
    probot->SetActiveManipulator(manipulatorname);

    std::vector<double> qddLimits;
    probot->GetDOFAccelerationLimits(qddLimits);
    double accelerationScalingFactor = 0.4;
    for (unsigned int i = 0; i < qddLimits.size(); i++)
	qddLimits[i] = qddLimits[i] * accelerationScalingFactor;
    probot->SetDOFAccelerationLimits(qddLimits);
    
    // Parameters
    double maxTime = 600.0;
    unsigned int nRuns = 50;

    // cellSize is parsed from the command line
    double cellSize = atof(argv[1]);
    std::vector<int> ndofs;
    ndofs.push_back(2);
    ndofs.push_back(3);
    ndofs.push_back(4);
    ndofs.push_back(5);
    ndofs.push_back(6);
    ndofs.push_back(7);
    ndofs.push_back(8);
    ndofs.push_back(9);
    ndofs.push_back(10);
    ndofs.push_back(11);
    ndofs.push_back(12);
    
    for (size_t j = 0; j < ndofs.size(); ++j) {
	int ndof = ndofs[j];
	
	std::vector<int> activeDOFs;
	for (int i = 0; i < ndof; ++i)
	    activeDOFs.push_back(i);
	probot->SetActiveDOFs(activeDOFs);
	std::cout << "====================\n";
	std::cout << "ndof                   " << probot->GetActiveDOF() << "\n";
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

	// Actual runs
	for (unsigned int r = 0; r < nRuns; ++r) {
	    SuperBotSetup ss(probot);
	    std::cout << "iteration " << r + 1 << "\n";
	    if (ss.solve(maxTime) && ss.haveExactSolutionPath()) {
		nSuccess = nSuccess + 1;
		runningTime.push_back(ss.getLastPlanComputationTime());
		avgRunningTime = avgRunningTime + ss.getLastPlanComputationTime();
	    }
	}

	if (nSuccess > 0) {
	    avgRunningTime = avgRunningTime / nSuccess;
	}
	successRate = nSuccess / nRuns;

	std::stringstream ssFileName;
	ssFileName << "data/KPIECE_" << cellSize << "_" << ndof << "DOFs.data";
	std::string outputFileName = ssFileName.str();
	std::cout << "Saving data into " << outputFileName << std::endl;

	/*
	  Here is the format that we use:

	  first line: parameter names
	  second line: parameter values
	  third line: running time for each successful run
	 */

	std::stringstream newData;
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
	for (unsigned int i = 0; i < nSuccess; i++) {
	    newData << separator << runningTime[i];
	    separator = " ";
	}

	newData << "\n";

	// Write results to a file
	std::ofstream outFile;
	outFile.open(outputFileName.c_str(), std::ios::app);
	outFile << newData.str();

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
    } // end for each DOF
    
    return 0;
}
