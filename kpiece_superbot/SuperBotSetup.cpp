/* Author: Puttichai Lertkultanon */

#include <openrave-core.h>
#include <openrave/openrave.h>
#include "SuperBotConfig.h"
#include "SuperBotSetup.h"
#include "SuperBotStateSpace.h"
#include "SuperBotControlSpace.h"
#include "SuperBotStatePropagator.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <cstring>

namespace ob = ompl::base;
namespace oc = ompl::control;

class SuperBotStateValidityChecker : public ob::StateValidityChecker
{
public:
    SuperBotStateValidityChecker(const ob::SpaceInformationPtr &si) : ob::StateValidityChecker(si) {
    }

    virtual bool isValid(const ob::State *state) const {
	if (si_->satisfiesBounds(state) == false)
	    return false;
	
	RobotBasePtr probot;
	probot = si_->getStateSpace()->as<SuperBotStateSpace>()->_probot;

	const ob::CompoundStateSpace::StateType *s = 
	    state->as<ob::CompoundStateSpace::StateType>();
	const double* q = s->as<ob::RealVectorStateSpace::StateType>(0)->values;
	const double* qd = s->as<ob::RealVectorStateSpace::StateType>(1)->values;
	std::vector<double> jointvalues;
	for (int i = 0; i < int(probot->GetActiveDOF()); i++)
	    jointvalues.push_back(q[i]);
	for (int i = int(probot->GetActiveDOF()); i < int(probot->GetDOF()); i++)
	    jointvalues.push_back(0.0);
	{
	    EnvironmentMutex::scoped_lock lock(probot->GetEnv()->GetMutex());
	    probot->SetDOFValues(jointvalues, CLA_NOTHING);
	    if (probot->CheckSelfCollision())
	    	return false;
	    if (probot->GetEnv()->CheckCollision(probot))
	    	return false;
	    else
	    	return true;
	}
    }
};


SuperBotSetup::SuperBotSetup(RobotBasePtr probot) : oc::SimpleSetup(oc::ControlSpacePtr(new SuperBotControlSpace(probot))) {
    std::cout << "Planning for the first " << probot->GetActiveDOF() << " DOFs\n";
    initialize(probot);
}

void SuperBotSetup::initialize(RobotBasePtr probot)
{
    const ob::StateSpacePtr &sSpace = getStateSpace();
    sSpace->setup();

    ob::ScopedState<> start(sSpace);
    ob::ScopedState<> goal(sSpace);
    std::vector<double> startVec(sSpace->getDimension(), 0.);
    std::vector<double> goalVec(sSpace->getDimension(), 0.);
    
    unsigned int ndof = si_->getStateSpace()->as<SuperBotStateSpace>()->_probot->GetActiveDOF();
    for (unsigned int i = 0; i < ndof; i++)
	goalVec[i] = 1.0;
    
    sSpace->copyFromReals(start.get(), startVec);
    sSpace->copyFromReals(goal.get(), goalVec);
    // set start, goal, and threshold
    setStartAndGoalStates(start, goal, threshold);
    
    // set propagation step size and min-max propagation steps
    si_->setPropagationStepSize(propagationStepSize);
    si_->setMinMaxControlDuration(propagationMinSteps, propagationMaxSteps);
    
    // initialize a new KPIECE planner
    setPlanner(ob::PlannerPtr(new oc::KPIECE1(si_)));
    
    // set default parameters
    getPlanner()->as<oc::KPIECE1>()->setMaxCloseSamplesCount(maxCloseSamplesCount);
    getPlanner()->as<oc::KPIECE1>()->setGoalBias(goalBias);
    
    // set a state validity checker
    setStateValidityChecker(ob::StateValidityCheckerPtr(new SuperBotStateValidityChecker(si_)));
    si_->setStateValidityCheckingResolution(0.05); //5%
    
    // set a state propagator
    setStatePropagator(oc::StatePropagatorPtr(new SuperBotStatePropagator(si_)));
}
