//
// Created by Fares on 10/19/2022.
//

#include "PIDManager.h"

PIDManager::PIDManager() {}

PIDManager::~PIDManager() {}

void PIDManager::initializePID(PIDAxis roll, PIDAxis pitch, PIDAxis yaw,PIDAxis altitudeApprox, float *rollConstants, float *pitchConstants,
                               float *yawConstants,float *altitudeConstants, float minLimit, float maxLimit) {
    this->roll = roll;
    this->pitch = pitch;
    this->yaw = yaw;
    this->altitudeApprox=altitudeApprox;
    this->minLimit = minLimit;
    this->maxLimit = maxLimit;

    this->roll.initializePIDAxis(rollConstants[0], rollConstants[1], rollConstants[2]);
    this->pitch.initializePIDAxis(pitchConstants[0], pitchConstants[1], pitchConstants[2]);
    this->yaw.initializePIDAxis(yawConstants[0], yawConstants[1], yawConstants[2]);
    this->altitudeApprox.initializePIDAxis(altitudeConstants[0], altitudeConstants[1], altitudeConstants[2]);
}


float PIDManager::calculateRollPIDValueWithoutLPF(
        float setPoint,
        float processPoint,
        double sampleTime) {
    return roll.calculatePIDValueWithoutLPF(
            setPoint,
            processPoint,
            minLimit, maxLimit,
            sampleTime
    );
}

float PIDManager::calculatePitchPIDValueWithoutLPF(
        float setPoint,
        float processPoint,
        double sampleTime) {
    return pitch.calculatePIDValueWithoutLPF(
            setPoint,
            processPoint,
            minLimit, maxLimit,
            sampleTime
    );
}

float PIDManager::calculateYawPIDValueWithoutLPF(
        float setPoint,
        float processPoint,
        double sampleTime) {
    return yaw.calculatePIDValueWithoutLPF(
            setPoint,
            processPoint,
            minLimit, maxLimit,
            sampleTime
    );
}

float PIDManager::calculateBMPPIDValue(float setPoint, float processPoint, double sampleTime) {
    return altitudeApprox.calculatePIDValueWithoutLPF(setPoint, processPoint, minLimit, maxLimit, sampleTime);
}


