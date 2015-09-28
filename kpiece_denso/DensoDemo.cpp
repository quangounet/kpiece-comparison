/* Author: Puttichai Lertkultanon */

#include <cstring>
#include <sstream>
#include <vector>
#include <boost/thread/thread.hpp>
#include <boost/bind.hpp>
#include "DensoStateSpace.cpp"
#include "DensoProjection.h"
#include "DensoStatePropagator.cpp"
#include "DensoControlSpace.cpp"
#include "DensoSetup.cpp"

namespace ob = ompl::base;
using namespace OpenRAVE;


void SetViewer(EnvironmentBasePtr penv, const std::string& viewername)
{
    ViewerBasePtr viewer = RaveCreateViewer(penv,viewername);
    penv->AddViewer(viewer);
    viewer->main(true);
}


int main(int argc, char **argv)
{
    std::string scenefilename = "../../robots/denso_base.xml";
    std::string viewername = "qtcoin";

    // Start OpenRAVE core
    RaveInitialize(true);
    EnvironmentBasePtr penv = RaveCreateEnvironment();
    boost::thread thviewer(boost::bind(SetViewer, penv, viewername));
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

    std::vector<int> activedofs;
    int n = 2;
    if (argc == 2)
    {
	n = atoi(argv[1]);
    }
    for (int i = 0; i < n; i++)
	activedofs.push_back(i);
    probot->SetActiveDOFs(activedofs);
    std::cout << "nActiveDOFs is " << probot->GetActiveDOF() << "\n";

    // DensoStateSpace *sSpace = new DensoStateSpace(probot);
    // DensoControlSpace *cSpace = new DensoControlSpace(probot);
    DensoSetup ds(probot);
    ds.print();
    double runningTime = 600;

    if (ds.solve(runningTime) && ds.haveExactSolutionPath())
    {
	std::cout << "Problem Solved in " << ds.getLastPlanComputationTime() << "s.\n";
	std::string outputFile = "kpiece_traj.data";
	std::ofstream out(outputFile.c_str());
        oc::PathControl path(ds.getSolutionPath());
	const ob::ProblemDefinitionPtr& pdef = ds.getProblemDefinition();
        path.interpolate();
	{
	    double difference = 0.0;
	    pdef->getGoal()->isSatisfied(path.getStates().back(), &difference);
	    std::cout << "difference = " << difference << "\n";
	}
	std::size_t nStates = path.getStateCount();
	std::cout << "nStates = " << nStates << "\n";
	for (unsigned int i = 0; i < nStates; i++)
	{
	    const ob::CompoundStateSpace::StateType *s = \
		path.getState(i)->as<ob::CompoundStateSpace::StateType>();
	    const double* q = s->as<ob::RealVectorStateSpace::StateType>(0)->values;
	    
	    std::string separator = "";
	    std::vector<double> jointValues;
	    jointValues.resize(0);
	    for (int j = 0; j < n; j++)
	    {
		jointValues.push_back(q[j]);
		std::cout << separator << jointValues[j];
		separator = ", ";
	    }
	    for (int j = n; j < probot->GetDOF(); j++)
	    {
		jointValues.push_back(0.0);
		std::cout << separator << jointValues[j];
		separator = ", ";
	    }
	    std::cout << "\n";
	    {
		EnvironmentMutex::scoped_lock lock(probot->GetEnv()->GetMutex());
		probot->SetDOFValues(jointValues);
		boost::this_thread::sleep(boost::posix_time::milliseconds(100));
	    }
	}
    }
    thviewer.join(); // wait for the viewer thread to exit

    penv->Destroy();
    return 0;
}
