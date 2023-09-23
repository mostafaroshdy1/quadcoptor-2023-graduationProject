//
// Created by Fares on 10/19/2022.
//

#ifndef PID_BALANCE_ARDUINO_PIDMANAGER_H
#define PID_BALANCE_ARDUINO_PIDMANAGER_H

#include "PIDAxis.h"

class PIDManager {
  private:
    float minLimit, maxLimit;
  public:
    PIDAxis roll, pitch, yaw, altitudeApprox;
    float totalPIDValue;


    void initializePID(PIDAxis roll, PIDAxis pitch, PIDAxis yaw, PIDAxis altitudeApprox, float *rollConstants, float *pitchConstants,
                       float *yawConstants, float *altitudeConstants, float minLimit, float maxLimit);

    /*
       For test
    */

    float calculateRollPIDValueWithoutLPF(float setPoint, float processPoint, double sampleTime);

    float calculatePitchPIDValueWithoutLPF(float setPoint, float processPoint, double sampleTime);

    float calculateYawPIDValueWithoutLPF(float setPoint, float processPoint, double sampleTime);

    /*----------------------------------------------------*/

    float calculateRollPitchPIDValueWithoutLPF(PIDAxis roll, PIDAxis pitch);

    float calculateRollYawPIDValueWithoutLPF(PIDAxis roll, PIDAxis yaw);

    float calculatePitchYawPIDValueWithoutLPF(PIDAxis pitch, PIDAxis yaw);

    float calculateBMPPIDValue(float setPoint, float processPoint, double sampleTime);



    PIDManager();

    ~PIDManager();
};


#endif //PID_BALANCE_ARDUINO_PIDMANAGER_H
