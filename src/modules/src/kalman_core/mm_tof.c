/**
 * ,---------,       ____  _ __
 * |  ,-^-,  |      / __ )(_) /_______________ _____  ___
 * | (  O  ) |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * | / ,--'  |    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *    +------`   /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2021 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include "mm_tof.h"
#include "param.h"
#include "debug.h"
#include "log.h"

// Added Logging variables for state F update monitoring
float measuredDistanceF, predictedDistanceF;
float thresholdF, errorF;
// Added Logging variables for state R update monitoring
float measuredDistanceR, predictedDistanceR;
float thresholdR, errorR;

// Parameter for tuning the detection for F and R estimation in the tof update
// Factor multipled with the standard deviation of the measurement and compared to the prediction error
// Detection factor needs to be changed based on set height
static float detectionFactor = 3.5f;
// The value the large initial variance of F or R to be set to when an obstacle detection happens
static float variance_after_detection = 50; 
// Flag for turning the obstacle detection with F and R states - on and off
static bool use_detection = true;

// Default flow deck tof measurement function
void kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0){
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }
    //float predictedDistance = S[KC_STATE_Z] / cosf(angle);
    float predictedDistance = this->S[KC_STATE_Z] / this->R[2][2];
    float measuredDistance = tof->distance; // [m]

    //Measurement equation
    //
    // h = z/((R*z_b)\dot z_b) = z/cos(alpha)
    h[KC_STATE_Z] = 1 / this->R[2][2];
    //h[KC_STATE_Z] = 1 / cosf(angle);

    // Scalar update
    kalmanCoreScalarUpdate(this, &H, measuredDistance-predictedDistance, tof->stdDev);
  }
}

// Flow deck tof measurement function for state F estimation
void kalmanCoreUpdateWithTofUsingF(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0)
  {
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }

    float predictedDistance = (this->S[KC_STATE_Z]-this->S[KC_STATE_F]) / this->R[2][2];
    predictedDistanceF = predictedDistance; // Updates logging variable
    float measuredDistance = tof->distance; // [m]
    measuredDistanceF = measuredDistance;   // Updates logging variable

    float error = measuredDistance-predictedDistance;
    errorF = error; // Updates logging variable
    
    if (use_detection)
    { 
      float threshold = detectionFactor*(tof->stdDev);
      thresholdF = threshold; // Updates logging variable
      // Obstacle detection
      // If the error exceeds the threshold, it shows the presence of obstacle below the drone 
      // and the variable S[KC_STATE_F] of the F state needs to change
      if(error*error > threshold*threshold)
      {
        // Set the variance high and update the F state
        this->P[KC_STATE_F][KC_STATE_F] = variance_after_detection;
        this->S[KC_STATE_F] = this->S[KC_STATE_Z] - measuredDistance*this->R[2][2];
        // Kalman input error set to zero for every F update, as we directly update the F state above
        error = 0;  
        //DEBUG_PRINT("Inside update error\n");
      }
    }

    //Measurement equation
    // h = (z - f)/((R*z_b)\dot z_b) = z/cos(alpha)
    h[KC_STATE_Z] = 1 / this->R[2][2];

    h[KC_STATE_F] = -1 / this->R[2][2];
    
    // Scalar update
    kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
  }
}

// Flow deck tof measurement function for state R estimation
void kalmanCoreUpdateWithUpTofUsingR(kalmanCoreData_t* this, tofMeasurement_t *tof)
{
  // Updates the filter with a measured distance in the zb direction using the
  float h[KC_STATE_DIM] = {0};
  arm_matrix_instance_f32 H = {1, KC_STATE_DIM, h};

  // Initially state R is set as -1 and in the first function call, it gets set to first sensor measurement
  if(!this->stateRInitialized)
  {
    this->S[KC_STATE_R] = tof->distance - this->S[KC_STATE_Z]; // Just set it equal to the measurement
    this->stateRInitialized = true;
  }


  // Only update the filter if the measurement is reliable (\hat{h} -> infty when R[2][2] -> 0)
  if (fabs(this->R[2][2]) > 0.1 && this->R[2][2] > 0)
  {
    float angle = fabsf(acosf(this->R[2][2])) - DEG_TO_RAD * (15.0f / 2.0f);
    if (angle < 0.0f) {
      angle = 0.0f;
    }

    float predictedDistance = (this->S[KC_STATE_R] - this->S[KC_STATE_Z]) / this->R[2][2];
    predictedDistanceR = predictedDistance; // Updates logging variable
    float measuredDistance = tof->distance; // [m]
    measuredDistanceR = measuredDistance;   // Updates logging variable

    float error = measuredDistance-predictedDistance;
    errorR = error; // Updates logging variable

    if (use_detection)
    { 
      float threshold = detectionFactor*(tof->stdDev);
      thresholdR = threshold; // Updates logging variable
      // Obstacle detection
      // If the error exceeds the threshold, it shows the presence of obstacle above the drone 
      // and the variable S[KC_STATE_R] of the R state needs to change
      if(error*error > threshold*threshold)
      {
        // Set the variance high and update the R state
        this->P[KC_STATE_R][KC_STATE_R] = variance_after_detection;
        this->S[KC_STATE_R] = measuredDistance*this->R[2][2] + this->S[KC_STATE_Z];
        // Kalman input error set to zero for every R update, as we directly update the R state above
        error = 0;
      }
    }

    //Measurement equation
    // h = (r - z)/((R*z_b)\dot z_b) = z/cos(alpha)
    h[KC_STATE_Z] = -1 / this->R[2][2];
    //h[KC_STATE_Z] = -1 / cosf(angle);

    h[KC_STATE_R] = 1 / this->R[2][2];
    
    // Scalar update
    kalmanCoreScalarUpdate(this, &H, error, tof->stdDev);
  }
}

LOG_GROUP_START(kalman)
/**
 * @brief Measured distance sensor reading from flow deck for state F estimation
 */
  LOG_ADD(LOG_FLOAT, tofsensormeaF, &measuredDistanceF)
 /**
 * @brief Predicted distance from flow deck for state F estimation
 */
  LOG_ADD(LOG_FLOAT, tofsensorpreF, &predictedDistanceF)
 /**
 * @brief Error between measured and prediction values for state F estimation
 */
  LOG_ADD(LOG_FLOAT, toferrorF, &errorF)
 /**
 * @brief Threshold for state F update
 */
  LOG_ADD(LOG_FLOAT, tofthresF, &thresholdF)

 /**
 * @brief Measured distance sensor reading from flow deck for state R estimation
 */
  LOG_ADD(LOG_FLOAT, tofsensormeaR, &measuredDistanceR)
 /**
 * @brief Predicted distance from flow deck for state R estimation
 */
  LOG_ADD(LOG_FLOAT, tofsensorpreR, &predictedDistanceR)
 /**
 * @brief Error between measured and prediction values for state R estimation
 */
  LOG_ADD(LOG_FLOAT, toferrorR, &errorR)
 /**
 * @brief Threshold for state R update
 */
  LOG_ADD(LOG_FLOAT, tofthresR, &thresholdR)  
 /**
 * @brief Detection factor df parameter
 */
  LOG_ADD(LOG_FLOAT, tofdf, &detectionFactor)  

LOG_GROUP_STOP(kalman)

PARAM_GROUP_START(kalman)
/**
  * @brief Measured distance sensor reading from flow deck for state F estimation
  */
  PARAM_ADD_CORE(PARAM_FLOAT, measuredDistanceF, &measuredDistanceF)
  /**
  * @brief Predicted distance from flow deck for state F estimation
  */
  PARAM_ADD_CORE(PARAM_FLOAT, predictedDistanceF, &predictedDistanceF)
  /**
  * @brief Error between measured and prediction values for state F estimation
  */
  PARAM_ADD_CORE(PARAM_FLOAT, errorF, &errorF)
  /**
  * @brief Threshold for state F update
  */
  PARAM_ADD_CORE(PARAM_FLOAT, thresholdF, &thresholdF)

  /**
  * @brief Measured distance sensor reading from flow deck for state R estimation
  */
  PARAM_ADD_CORE(PARAM_FLOAT, measuredDistanceR, &measuredDistanceR)
  /**
  * @brief Predicted distance from flow deck for state R estimation
  */
  PARAM_ADD_CORE(PARAM_FLOAT, predictedDistanceR, &predictedDistanceR)
  /**
  * @brief Error between measured and prediction values for state R estimation
  */
  PARAM_ADD_CORE(PARAM_FLOAT, errorR, &errorR)
  /**
  * @brief Threshold for state R update
  */
  PARAM_ADD_CORE(PARAM_FLOAT, thresholdR, &thresholdR)
  /**
  * @brief Detection factor parameter
  */
  PARAM_ADD_CORE(PARAM_FLOAT, detectionfactorFR, &detectionFactor)
  
PARAM_GROUP_STOP(kalman)