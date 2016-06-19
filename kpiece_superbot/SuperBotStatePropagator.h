/* Author: Puttichai Lertkultanon */

#ifndef DENSO_STATE_PROPAGATOR_
#define DENSO_STATE_PROPAGATOR_

#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/Control.h>
#include <ompl/control/ControlSpace.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/util/ClassForward.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class SuperBotStatePropagator : public oc::StatePropagator {
 public:    
    /* Constructor */
 SuperBotStatePropagator(oc::SpaceInformationPtr &si) : oc::StatePropagator(si) {
    }
    
    void propagate(const ob::State *start, const oc::Control *control, const double duration, ob::State *result) const;
    
    bool canPropagateBackward() const {
	return false;
    }
};

#endif
