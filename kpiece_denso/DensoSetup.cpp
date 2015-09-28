/* Author: Puttichai Lertkultanon */

#include <openrave-core.h>
#include <openrave/openrave.h>
#include "DensoConfig.h"
#include "DensoSetup.h"
#include "DensoStateSpace.h"
#include "DensoControlSpace.h"
#include "DensoStatePropagator.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <cstring>

namespace ob = ompl::base;
namespace oc = ompl::control;

class DensoStateValidityChecker : public ob::StateValidityChecker
{
public:
    DensoStateValidityChecker(const ob::SpaceInformationPtr &si) : 
	ob::StateValidityChecker(si)
    {
    }

    virtual bool isValid(const ob::State *state) const
    {
	if (si_->satisfiesBounds(state) == false)
	    return false;
	
	RobotBasePtr probot;
	probot = si_->getStateSpace()->as<DensoStateSpace>()->_probot;

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


DensoSetup::DensoSetup(RobotBasePtr probot) :
    oc::SimpleSetup(oc::ControlSpacePtr(new DensoControlSpace(probot)))
{
    std::cout << "Planning for the first " << probot->GetActiveDOF() << " DOFs\n";
    initialize(probot);
}

void DensoSetup::initialize(RobotBasePtr probot)
{
    const ob::StateSpacePtr &sSpace = getStateSpace();
    sSpace->setup();

    ob::ScopedState<> start(sSpace);
    ob::ScopedState<> goal(sSpace);
    std::vector<double> startVec(sSpace->getDimension(), 0.);
    std::vector<double> goalVec(sSpace->getDimension(), 0.);
    
    unsigned int ndof = si_->getStateSpace()->as<DensoStateSpace>()->
	_probot->GetActiveDOF();
    for (unsigned int i = 0; i < ndof; i++)
	goalVec[i] = 1.0;

    if (_useRealScene)
    {
	startVec[0] = -1.576;
	startVec[1] = 1.374;
	startVec[2] = 0.0;
	startVec[3] = 0.0;
	startVec[4] = -1.36751533;
	startVec[5] = 1.615;
	goalVec[0] = 1.55;
	goalVec[1] = 1.35;
	goalVec[2] = 0.1;
	goalVec[3] = -000057;
	goalVec[4] = -1.40754;
	goalVec[5] = 1.6;
    }
    
    sSpace->copyFromReals(start.get(), startVec);
    sSpace->copyFromReals(goal.get(), goalVec);
    /* set start, goal, and threshold*/
    setStartAndGoalStates(start, goal, threshold);
    /* set propagation step size and min-max propagation steps*/
    si_->setPropagationStepSize(propagationStepSize);
    si_->setMinMaxControlDuration(propagationMinSteps, propagationMaxSteps);
    /* initialize a new KPIECE planner */
    setPlanner(ob::PlannerPtr(new oc::KPIECE1(si_)));
    /* set default parameters */
    getPlanner()->as<oc::KPIECE1>()->setMaxCloseSamplesCount(maxCloseSamplesCount);
    getPlanner()->as<oc::KPIECE1>()->setGoalBias(goalBias);
    /* set a state validity checker */
    setStateValidityChecker(ob::StateValidityCheckerPtr
			    (new DensoStateValidityChecker(si_)));
    si_->setStateValidityCheckingResolution(0.05); //5%
    /* set a state propagator */
    setStatePropagator(oc::StatePropagatorPtr(new DensoStatePropagator(si_)));
}
