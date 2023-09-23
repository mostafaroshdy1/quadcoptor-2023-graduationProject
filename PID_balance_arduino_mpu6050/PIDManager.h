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
    PIDAxis roll, pitch, yaw;
    float totalPIDValue;


    void initializePID(PIDAxis roll, PIDAxis pitch, PIDAxis yaw,float *rollConstants, float *pitchConstants,
                               float *yawConstants, float minLimit, float maxLimit);

    /*
     * For test
     */
    float calculateRollPIDValueWithLPF(float setPoint, float processPoint, double sampleTime);

    float calculatePitchPIDValueWithLPF(float setPoint, float processPoint, double sampleTime);

    float calculateYawPIDValueWithLPF(float setPoint, float processPoint, double sampleTime);

    float calculateRollPIDValueWithoutLPF(float setPoint, float processPoint, double sampleTime);

    float calculatePitchPIDValueWithoutLPF(float setPoint, float processPoint, double sampleTime);

    float calculateYawPIDValueWithoutLPF(float setPoint, float processPoint, double sampleTime);

    /*----------------------------------------------------*/

    float calculateRollPitchPIDValueWithoutLPF();

    float calculateRollYawPIDValueWithoutLPF();

    float calculatePitchYawPIDValueWithoutLPF();



    PIDManager();

    ~PIDManager();
};


#endif //PID_BALANCE_ARDUINO_PIDMANAGER_H
