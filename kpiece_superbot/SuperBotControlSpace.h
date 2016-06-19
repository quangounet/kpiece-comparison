/* Author: Puttichai Lertkultanon */

#ifndef SUPERBOT_CONTROL_SPACE_
#define SUPERBOT_CONTROL_SPACE_

#include <openrave-core.h>
#include <openrave/openrave.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace OpenRAVE;

class SuperBotControlSampler : public oc::ControlSampler {
 public:
 SuperBotControlSampler(const oc::ControlSpace *cSpace) : oc::ControlSampler(cSpace) {
    }
    
    virtual void sample(oc::Control *control);

    virtual void sample(oc::Control *control, const ob::State *state) {
	sample(control);
    }
    
    virtual void sampleNext(oc::Control *control, const oc::Control *prevControl, const ob::State *prevState) {
	sample(control);
    }
 protected:
    ompl::RNG _rng;
};


class SuperBotControlSpace : public oc::RealVectorControlSpace {
 public:
    /* Constructor */
    SuperBotControlSpace(RobotBasePtr probot);
    
    virtual oc::ControlSamplerPtr allocDefaultControlSampler() const {
	return oc::ControlSamplerPtr(new SuperBotControlSampler(this));
    }

 protected:
    RobotBasePtr _probot;
};

#endif
