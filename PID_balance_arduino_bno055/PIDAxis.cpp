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
  setPIDConstants(k_p, k_i, k_d);
  //    Serial.println((String) kp + " " + (String) ki + " " + (String) kd);
}

void PIDAxis::setPIDConstants(float k_p, float k_i, float k_d) {
  kp = k_p;
  ki = k_i;
  kd = k_d;
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

  //Remove sampleTime from calculation
  if (error > -3 && error < 3) {
    integral += ki * sampleTime * error;
  }


  //Remove sampleTime from calculation

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
