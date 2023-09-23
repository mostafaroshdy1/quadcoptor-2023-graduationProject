//
// Created by Fares on 10/20/2022.
//

#ifndef PID_BALANCE_ARDUINO_PIDAXIS_H
#define PID_BALANCE_ARDUINO_PIDAXIS_H


class PIDAxis {
private:
    void initializeParameters(
            float setPoint,
            float processPoint,
            float tau
    );

public:
    PIDAxis();


    ~PIDAxis();

    float setPoint, processPoint, previousProcessPoint;
    float error, previousError;
    float proportional, integral, derivative, pidValue;
    float tau = 0.02;
    float kp, ki, kd;
//    double sampleTime;


    void initializePIDAxis(float k_p, float k_i, float k_d);

    float calculatePIDValueWithLPF(
            float setPoint,
            float processPoint,
            float minLimit,
            float maxLimit,
            double sampleTime,
            float tau
    );

    float calculatePIDValueWithoutLPF(
            float setPoint,
            float processPoint,
            float minLimit,
            float maxLimit,
            double sampleTime
    );
};


#endif //PID_BALANCE_ARDUINO_PIDAXIS_H
