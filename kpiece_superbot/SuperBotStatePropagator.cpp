/* Author: Puttichai Lertkultanon */

#include "SuperBotConfig.h"
#include "SuperBotStatePropagator.h"
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <cstring>

namespace ob = ompl::base;
namespace oc = ompl::control;

void SuperBotStatePropagator::propagate(const ob::State *start, const oc::Control *control, 
					const double duration, ob::State *result) const {
    /* double integrator */
    // previous state
    const ob::CompoundStateSpace::StateType *state = start->as<ob::CompoundStateSpace::StateType>();
    const double* q = state->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double* qd = state->as<ob::RealVectorStateSpace::StateType>(1)->values;
    
    // resulting state
    const ob::CompoundStateSpace::StateType *res = result->as<ob::CompoundStateSpace::StateType>();
    double* q_res = res->as<ob::RealVectorStateSpace::StateType>(0)->values;
    double* qd_res = res->as<ob::RealVectorStateSpace::StateType>(1)->values;
    
    unsigned int dim = si_->getStateDimension();

    if (_controlViaAcceleration) {
	// control via acceleration
	// new control
	const double* qdd = control->as<oc::RealVectorControlSpace::ControlType>()->values;
	// update state
	for (int i = 0; i < int(dim/2); i++) {
	    q_res[i] = q[i] + duration*qd[i] + 0.5*duration*duration*qdd[i];
	    qd_res[i] = qd[i] + duration*qdd[i];
	}	
    }
    else {
	// control via velocity
	// new control
	const double* u = control->as<oc::RealVectorControlSpace::ControlType>()->values;
	for (int i = 0; i < int(dim/2); i++) {
	    q_res[i] = q[i] + duration*u[i];
	    qd_res[i] = u[i];
	}
    }
}
