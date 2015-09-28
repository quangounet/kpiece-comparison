/* Author: Puttichai Lertkultanon */

#ifndef DENSO_SETUP_
#define DENSO_SETUP_

#include <openrave-core.h>
#include <openrave/openrave.h>
#include <ompl/control/SimpleSetup.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class DensoSetup : public oc::SimpleSetup
{
 public:
    DensoSetup(RobotBasePtr probot);

 private:
    void initialize(RobotBasePtr probot);
};

#endif
