
#include "PIDManager.h"
#include "IMUManager.h"
#include <Servo.h>

/*---------------ESCs---------------*/
Servo esc_front_left, esc_front_right, esc_rear_left, esc_rear_right;
int FR = 6;
int FL = 10;
int RR = 5;
int RL = 9;
float pwmFR, pwmFL, pwmRR, pwmRL;
/*----------------------------------*/



IMUManager imuManager;

PIDManager pidManager;
/////////////////PID CONSTANTS/////////////////
float kp = 2; //3.55
float ki = 0.008; //0.005
float kd = 0; //2.05
float rollConstants[3] = {kp, ki, kd};
float pitchConstants[3] = {kp, ki, kd};
float yawConstants[3] = {kp, ki, kd};
///////////////////////////////////////////////
PIDAxis roll, pitch, yaw;


float throttle = 1000; //initial value of throttle to the motors
float minPWM = 1000;
float maxPWM = 1400;
float minPIDValue = -150;
float maxPIDValue = 150;
float desired_angle = 0; //This is the angle in which we whant the
//balance to stay steady




float *imuError;
float *Acc_Angles;
float *Gyro_Angles;
float *Total_Angles;


float serialReceivingInput;

void setup() {
    Serial.begin(9600);

    //  time = millis();


    setupESCs();


    imuError = imuManager.calculateIMUError();
    pidManager.initializePID(roll, pitch, yaw, rollConstants, pitchConstants, yawConstants, minPIDValue,
                             maxPIDValue);


}//end of setup void

void loop() {


    initializeIMU();

    pidManager.calculateRollPIDValueWithoutLPF(
            desired_angle,
            Total_Angles[0],
            imuManager.elapsedTime
    );
    pidManager.calculatePitchPIDValueWithoutLPF(
            desired_angle,
            Total_Angles[1],
            imuManager.elapsedTime
    );
    pidManager.calculateYawPIDValueWithoutLPF(
            desired_angle,
            Total_Angles[2],
            imuManager.elapsedTime
    );
    //  if (abs(Total_Angles[2]) > 180) {
    //
    //    pidManager.totalPIDValue = -1 * pidManager.totalPIDValue;
    //
    //  }
    Serial.print(pidManager.pitch.pidValue);


    limitPWM();

    Serial.print(" ");
    Serial.print(pwmFR);
    Serial.print(" ");
    Serial.print(pwmRR);
    Serial.print(" ");
    Serial.print(pwmFL);
    Serial.print(" ");
    Serial.print(pwmRL);
    Serial.print(" ");
    Serial.print(pidManager.roll.error);
    Serial.print(" ");
    Serial.print(Total_Angles[0]);
    Serial.print(" ");
    Serial.print(pidManager.pitch.error);
    Serial.print(" ");
    Serial.print(Total_Angles[1]);
    Serial.print(" ");
    Serial.print(pidManager.yaw.error);
    Serial.print(" ");
    Serial.print(Total_Angles[2]);
    Serial.println();

    pitchTest();


    if (pidManager.pitch.error < 6 && pidManager.pitch.error > -6)
        hover();

    if (Serial.available() > 0) {
        serialReceivingInput = Serial.parseFloat();
        Serial.println("Desired angle: " + (String) desired_angle);


    }
    if (serialReceivingInput != 0)
        desired_angle = serialReceivingInput;


}//end of loop void


void setupESCs() {
    esc_front_right.attach(FR, minPWM, maxPWM);
    esc_front_left.attach(FL, minPWM, maxPWM);
    esc_rear_right.attach(RR, minPWM, maxPWM);
    esc_rear_left.attach(RL, minPWM, maxPWM);

    esc_front_right.writeMicroseconds(throttle);
    esc_front_left.writeMicroseconds(throttle);
    esc_rear_right.writeMicroseconds(throttle);
    esc_rear_left.writeMicroseconds(throttle);

    delay(2000);
    throttle = 1100;
}

void updateESCs() {

    esc_front_right.writeMicroseconds(pwmFR);
    esc_front_left.writeMicroseconds(pwmFL);
    esc_rear_right.writeMicroseconds(pwmRR);
    esc_rear_left.writeMicroseconds(pwmRL);
}


void hover() {

    esc_front_right.writeMicroseconds(0);
    esc_front_left.writeMicroseconds(0);
    esc_rear_right.writeMicroseconds(0);
    esc_rear_left.writeMicroseconds(0);
}

void limitPWM() {
    if (pwmFR < minPWM) {
        pwmFR = minPWM;
    }
    if (pwmFR > maxPWM) {
        pwmFR = maxPWM;
    }
    if (pwmFL < minPWM) {
        pwmFL = minPWM;
    }
    if (pwmFL > maxPWM) {
        pwmFL = maxPWM;
    }
    if (pwmRR < minPWM) {
        pwmRR = minPWM;
    }
    if (pwmRR > maxPWM) {
        pwmRR = maxPWM;
    }
    if (pwmRL < minPWM) {
        pwmRL = minPWM;
    }
    if (pwmRL > maxPWM) {
        pwmRL = maxPWM;
    }
}

void rollTest() {
    pwmFR = throttle - pidManager.roll.pidValue;
    pwmRR = throttle - pidManager.roll.pidValue;
    pwmFL = throttle + pidManager.roll.pidValue;
    pwmRL = throttle + pidManager.roll.pidValue;
    updateESCs();
}


void pitchTest() {
    pwmFR = throttle - pidManager.pitch.pidValue;
    pwmFL = throttle - pidManager.pitch.pidValue;
    pwmRR = throttle + pidManager.pitch.pidValue;
    pwmRL = throttle + pidManager.pitch.pidValue;
    updateESCs();
}

void yawTest() {
    pwmFR = throttle - pidManager.pitch.pidValue;
    pwmFL = throttle + pidManager.pitch.pidValue;
    pwmRR = throttle + pidManager.pitch.pidValue;
    pwmRL = throttle - pidManager.pitch.pidValue;
    updateESCs();
}

void initializeIMU() {
    Acc_Angles = imuManager.readAccelerometerData(imuError);
    Gyro_Angles = imuManager.readGyroData(imuError);
    Total_Angles = imuManager.getTotalAnglesWithComplementaryFilter(Gyro_Angles, Acc_Angles);
}

//Call in setup
void calibrateThrottle(float baseAltitude, float altitude) {
    float accuracy = 0.25;
    for (int i = 1040; i < 1500; ++i) {
        if (altitude > baseAltitude + acuuracy){
            throttle = i;
            break;
        }
    }

}