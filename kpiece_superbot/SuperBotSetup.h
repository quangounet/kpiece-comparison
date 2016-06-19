/* Author: Puttichai Lertkultanon */

#ifndef SUPERBOT_SETUP_
#define SUPERBOT_SETUP_

#include <openrave-core.h>
#include <openrave/openrave.h>
#include <ompl/control/SimpleSetup.h>

namespace ob = ompl::base;
namespace oc = ompl::control;

class SuperBotSetup : public oc::SimpleSetup {
 public:
    SuperBotSetup(RobotBasePtr probot);

 private:
    void initialize(RobotBasePtr probot);
};

#endif
