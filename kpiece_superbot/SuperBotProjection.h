/* Author: Puttichai Lertkultanon */

#ifndef DENSO_PROJECTION_
#define DENSO_PROJECTION_

#include <openrave-core.h>
#include <openrave/openrave.h>
#include <cstring>
#include <cmath>
#include "SuperBotConfig.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace ob = ompl::base;
using namespace OpenRAVE;

#define CLA_NOTHING 0

class SuperBotProjection : public ob::ProjectionEvaluator {
 public:
 SuperBotProjection(const ob::StateSpace* sSpace, RobotBasePtr probot, unsigned int projectionType)
     : ob::ProjectionEvaluator(sSpace), _probot(probot), _projectionType(projectionType) {
	if (_projectionType == 1) {
	    /* Three DOFs for the end-effector position and another */
	    /* DOF for a velocity norm. */
	    _dimension = 4;
	    _pmanip = _probot->GetActiveManipulator();
	}
	else if (_projectionType == 2) {
	    /* ndof DOFs for the robot configuration and another DOF 
	       for a velocity norm. */
	    _dimension = int(sSpace->getDimension()/2) + 1;
	}
    }

    unsigned int getDimension() const {
	return _dimension;
    }

    void defaultCellSizes() {
	cellSizes_.resize(_dimension, 0.05);
    }

    void project(const ob::State *state, ob::EuclideanProjection &projection) const {
	if (_projectionType == 1)
	    project1(state, projection);
	else // if (_projectionType == 2)
	    project2(state, projection);
    }

    void project1(const ob::State *state, ob::EuclideanProjection &projection) const {
	const ob::CompoundStateSpace::StateType *s = state->as<ob::CompoundStateSpace::StateType>();
	
	const double* q = s->as<ob::RealVectorStateSpace::StateType>(0)->values;
	const double* qd = s->as<ob::RealVectorStateSpace::StateType>(1)->values;
	std::vector<double> jointValues;
	jointValues.resize(0);
	for (int i = 0; i < int(_probot->GetActiveDOF()); i++)
	    jointValues.push_back(q[i]);
	for (int i = int(_probot->GetActiveDOF()); i < int(_probot->GetDOF()); i++)
	    jointValues.push_back(0.0);
	{
	    EnvironmentMutex::scoped_lock lock(_probot->GetEnv()->GetMutex());
	    _probot->SetDOFValues(jointValues, CLA_NOTHING);
	    Transform t = _pmanip->GetEndEffectorTransform();
	    projection[0] = t.trans[0];
	    projection[1] = t.trans[1];
	    projection[2] = t.trans[2];
	}
	double norm = 0.0;
	for (int i = 0; i < _probot->GetActiveDOF(); ++i) {
	    norm = norm + (qd[i]*qd[i]);
	}
	projection[3] = sqrt(norm);
    }

    void project2(const ob::State *state, ob::EuclideanProjection &projection) const {
	const ob::CompoundStateSpace::StateType *s = state->as<ob::CompoundStateSpace::StateType>();
	
	const double* q = s->as<ob::RealVectorStateSpace::StateType>(0)->values;
	const double* qd = s->as<ob::RealVectorStateSpace::StateType>(1)->values;
	double norm = 0.0;
	for (unsigned int i = 0; i < _dimension - 1; ++i) {
	    projection[i] = q[i];
	    norm = norm + (qd[i]*qd[i]);
	}
	projection[_dimension - 1] = sqrt(norm);	
    }    

    RobotBasePtr _probot;
    RobotBase::ManipulatorPtr _pmanip;
    unsigned int _projectionType;
    unsigned int _dimension;
    
};

#endif
