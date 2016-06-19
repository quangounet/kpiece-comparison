/* Author: Puttichai Lertkultanon */

#include "SuperBotConfig.h"
#include "SuperBotStateSpace.h"
#include "SuperBotProjection.h"
#include <cmath>
#include <vector>
#include <sstream>

namespace ob = ompl::base;

SuperBotStateSpace::SuperBotStateSpace(RobotBasePtr probot) {
    _probot = probot;
    _ndof = _probot->GetActiveDOF();
    _dimension = 2*_ndof;
    
    ob::RealVectorStateSpace* qSpace(new ob::RealVectorStateSpace(_ndof));
    ob::RealVectorStateSpace* qdSpace(new ob::RealVectorStateSpace(_ndof));
    
    std::vector<double> qL, qU, qdU;
    _probot->GetDOFLimits(qL, qU);
    _probot->GetDOFVelocityLimits(qdU);

    ob::RealVectorBounds qBounds(_ndof);
    ob::RealVectorBounds qdBounds(_ndof);
    
    // Set joint value and joint velocity bounds
    for (unsigned int i = 0; i < _ndof; i++) {
    	qBounds.setLow(i, qL[i]);
    	qBounds.setHigh(i, qU[i]);
	qdBounds.setLow(i, -qdU[i]);
    	qdBounds.setHigh(i, qdU[i]);
    }

    qSpace->setBounds(qBounds);
    qdSpace->setBounds(qdBounds);

    // Add subspaces to SuperBotStateSpace
    addSubspace(ob::StateSpacePtr(qSpace), 0.5);
    addSubspace(ob::StateSpacePtr(qdSpace), 0.5);
}

unsigned int SuperBotStateSpace::getDimension() {
    return _dimension;
}

void SuperBotStateSpace::registerProjections() {
    registerDefaultProjection(ob::ProjectionEvaluatorPtr(new SuperBotProjection(this, _probot, projectionType)));
}

double SuperBotStateSpace::distance(const ob::State *state1, const ob::State *state2) const {
    const ob::CompoundStateSpace::StateType *s1 = state1->as<ob::CompoundStateSpace::StateType>();
    const ob::CompoundStateSpace::StateType *s2 = state2->as<ob::CompoundStateSpace::StateType>();
	
    const double* q1 = s1->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double* qd1 = s1->as<ob::RealVectorStateSpace::StateType>(1)->values;
    const double* q2 = s2->as<ob::RealVectorStateSpace::StateType>(0)->values;
    const double* qd2 = s2->as<ob::RealVectorStateSpace::StateType>(1)->values;
    double dist = 0.0;
    for (unsigned int i = 0; i < _ndof; i++)
    {
	double delta_q = (q1[i] - q2[i]);
	double delta_qd = 0;
	double wq = 0.5;
	double wqd = 0.5;
	if (_includeVelocityDistance)
	    delta_qd = delta_qd + (qd1[i] - qd2[i]);
	dist = dist + wq*(delta_q*delta_q) + wqd*(delta_qd*delta_qd);
    }
    return dist;
}
