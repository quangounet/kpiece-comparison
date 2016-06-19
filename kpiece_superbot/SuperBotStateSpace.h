/* Author: Puttichai Lertkultanon */

#ifndef SUPERBOT_STATE_SPACE_
#define SUPERBOT_STATE_SPACE_

#include <openrave-core.h>
#include <openrave/openrave.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace OpenRAVE;

class SuperBotStateSpace : public ompl::base::CompoundStateSpace {
 public:
    /* Constructor */
    SuperBotStateSpace(RobotBasePtr probot);
    
    /* Functionality */
    unsigned int getDimension();

    virtual void registerProjections(void);
    
    double distance(const ompl::base::State *state1, const ompl::base::State *state2) const;
    
    /* Members */
    unsigned int _dimension;
    unsigned int _ndof;
    RobotBasePtr _probot;
};

#endif
