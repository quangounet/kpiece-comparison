/* Author: Puttichai Lertkultanon */

#ifndef DENSO_CONFIG_
#define DENSO_CONFIG_

#include <boost/math/constants/constants.hpp>
#include <cstring>

const bool _controlViaAcceleration = true;
const bool _includeVelocityDistance = true;
const bool _useRealScene = false;
//const bool _useRealScene = true;

const double threshold = 0.1;
const double cellSize = 0.05; // KPIECE1
/* const double cellSize = 0.1; // KPIECE2 */
/* const double cellSize = 1.0; // KPIECE3 */
const double propagationStepSize = 0.05;
const double goalBias = 0.2;
const unsigned int propagationMinSteps = 1;
const unsigned int propagationMaxSteps = 10;
const unsigned int maxCloseSamplesCount = 100;
const unsigned int projectionType = 1;

#endif
