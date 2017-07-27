/* Author: Puttichai Lertkultanon */

#ifndef SUPERBOT_CONFIG_
#define SUPERBOT_CONFIG_

#include <boost/math/constants/constants.hpp>
#include <cstring>

const bool _controlViaAcceleration = true;
const bool _includeVelocityDistance = true;

const double threshold = 0.1;

const double propagationStepSize = 0.05;
const double goalBias = 1.0;
const unsigned int propagationMinSteps = 1;
const unsigned int propagationMaxSteps = 10;
const unsigned int maxCloseSamplesCount = 100;
const unsigned int projectionType = 1;

#endif
