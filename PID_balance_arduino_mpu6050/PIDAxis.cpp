//
// Created by Fares on 10/20/2022.
//

#include "PIDAxis.h"
#include "Arduino.h" //To allow Serial.print() in cpp file


PIDAxis::PIDAxis() {
    
}

PIDAxis::~PIDAxis() {}

void PIDAxis::initializeParameters(
        float setPoint,
        float processPoint,
        float tau
) {
    this->setPoint = setPoint;
    this->processPoint = processPoint;
    this->tau = tau;
}

void PIDAxis::initializePIDAxis(float k_p, float k_i, float k_d) {
    proportional = 0;
    integral = 0;
    derivative = 0;
    pidValue = 0;
    kp = k_p;
    ki = k_i;
    kd = k_d;
//    Serial.println((String) kp + " " + (String) ki + " " + (String) kd);
}

float PIDAxis::calculatePIDValueWithLPF(
        float setPoint,
        float processPoint,
        float minLimit,
        float maxLimit,
        double sampleTime,
        float tau
) {
    initializeParameters(setPoint, processPoint, tau);

    error = this->setPoint - this->processPoint;

    proportional = kp * error;

    integral += 0.5 * ki * sampleTime * (error + previousError);

    derivative = -(2.0 * kd * (this->processPoint - this->previousProcessPoint) +
                   (2.0 * this->tau - sampleTime) * derivative)
                 / (2.0 * this->tau + sampleTime);

    pidValue = proportional + integral + derivative;

    if (pidValue < minLimit)
        pidValue = minLimit;

    if (pidValue > maxLimit)
        pidValue = maxLimit;


    previousError = error;
    previousProcessPoint = this->processPoint;

    return pidValue;
}

float PIDAxis::calculatePIDValueWithoutLPF(
        float setPoint,
        float processPoint,
        float minLimit,
        float maxLimit,
        double sampleTime
) {
    initializeParameters(setPoint, processPoint, 0);

    error = this->setPoint - this->processPoint;

    proportional = kp * error;

    integral += 0.5 * ki * sampleTime * (error + previousError);

    derivative = kd * ((error - previousError) / sampleTime);

    pidValue = proportional + integral + derivative;

//    Serial.print((String) kp + " " + (String) ki + " " + (String) kd + "  ");
//    Serial.println("calculated -> p = " + (String) proportional + ", i = " + (String) integral + ", d = " +
//                   (String) derivative + ", pid = " + (String) pidValue);
    if (pidValue < minLimit)
        pidValue = minLimit;

    if (pidValue > maxLimit)
        pidValue = maxLimit;


    previousError = error;
    previousProcessPoint = this->processPoint; //Not used here

    return pidValue;
}
