/* Author: Puttichai Lertkultanon */

#include <openrave-core.h>
#include <openrave/openrave.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/StateSpace.h>
#include "SuperBotConfig.h"
#include "SuperBotStateSpace.h"
#include "SuperBotControlSpace.h"
#include <vector>

namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace OpenRAVE;

void SuperBotControlSampler::sample(oc::Control *control) {
    const ob::RealVectorBounds &bounds = space_->as<oc::RealVectorControlSpace>()->getBounds();
    
    oc::RealVectorControlSpace::ControlType *rcontrol = control->as<oc::RealVectorControlSpace::ControlType>();

    unsigned int ndof = bounds.low.size();
    for (unsigned int i = 0; i < ndof; i++) {
	rcontrol->values[i] = _rng.uniformReal(bounds.low[i], bounds.high[i]);
    }
}

SuperBotControlSpace::SuperBotControlSpace(RobotBasePtr probot) : 
    oc::RealVectorControlSpace(ob::StateSpacePtr(new SuperBotStateSpace(probot)), probot->GetActiveDOF()),
    _probot(probot) {
    unsigned int ndof = _probot->GetActiveDOF();
    if (_controlViaAcceleration) {
	std::vector<double> qddBounds;
	_probot->GetDOFAccelerationLimits(qddBounds);
	for (unsigned int i = 0; i < ndof; i++) {
	    bounds_.setLow(i, -qddBounds[i]);
	    bounds_.setHigh(i, qddBounds[i]);
	}
    }
    else {
	std::vector<double> qdBounds;
	_probot->GetDOFVelocityLimits(qdBounds);
	for (unsigned int i = 0; i < ndof; i++) {
	    bounds_.setLow(i, -qdBounds[i]);
	    bounds_.setHigh(i, qdBounds[i]);
	}
    }
    }
