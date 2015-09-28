/* Author: Puttichai Lertkultanon */

#ifndef DENSO_STATE_SPACE_
#define DENSO_STATE_SPACE_

#include <openrave-core.h>
#include <openrave/openrave.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace OpenRAVE;

class DensoStateSpace : public ompl::base::CompoundStateSpace
{
 public:
    /* Constructor */
    DensoStateSpace(RobotBasePtr probot);
    
    /* Functionality */
    unsigned int getDimension();

    virtual void registerProjections(void);
    
    double distance(const ompl::base::State *state1,
    		    const ompl::base::State *state2) const;
    
    /* Members */
    unsigned int _dimension;
    unsigned int _ndof;
    RobotBasePtr _probot;
};

#endif
