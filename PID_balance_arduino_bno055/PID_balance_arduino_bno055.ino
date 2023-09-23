
#include "PIDManager.h"
#include "Stabilizer.h"
#include <Servo.h>
//#include "Math.h"

/*---------------ESCs---------------*/
Servo esc_front_left, esc_front_right, esc_rear_left, esc_rear_right;
int FR = 6;
int FL = 5;
int RR = 3;
int RL = 9;
float pwmFR, pwmFL, pwmRR, pwmRL;
/*----------------------------------*/

#define process 'A' // it can be either r for Roll or p for pitch or y for yaw or a for Altituide or c for combined pitch and roll or A for All   
#define sensor 55 // 55 for redaing bno only , 390 for redaing bmp only 500 for both 


Stabilizer stabilizer;

PIDManager pidManager;
/////////////////PID CONSTANTS///////////////// oscillating then oscillation decreased 0.74027 0.092087 0.12817
float kp = 0.74027;//0.007518;//0.8;//0.06385; //1.3
float ki = 0.092087;//0.000020566;//0.82;//1.2111; //0
float kd = 0.33;//0.66814;//0.000057;//0.00081136; //0.8
float rollConstants[3] = {kp, ki, kd};//kp=1.3 , ki=0.06 , kd=0.09
float pitchConstants[3] = {kp, ki, kd};
float yawConstants[3] = {0.07756, 0.5, 0.001};
float altitudeConstants[3] = {kp, ki, kd};
///////////////////////////////////////////////
PIDAxis roll, pitch, yaw, altitude;


float throttle = 1300; //initial value of throttle to the motors
float minPWM = 1150;
float maxPWM = 1400;
float minPIDValue = -200;
float maxPIDValue = 200;
float roll_desired_angle = 0;
float pitch_desired_angle = 0;
float yaw_desired_angle = 0;
float desired_altitude = 2.5; //This is the height in which we want the
//balance to stay steady


double time, timePrev, elapsedTime;
double responseTime, rTimePrev,rTime;


float serialReceivingInput;

void setup() {
  Serial.begin(9600);

  time = millis();
  rTime = millis();


  setupESCs();
#if sensor==55
  stabilizer.setupBNO();
#elif sensor==390
  stabilizer.setupBMPSPI();
  stabilizer.setBaseline();
#else
  stabilizer.setupBNO();
  stabilizer.setupBMPSPI();
  stabilizer.setBaseline();
#endif
  pidManager.initializePID(roll, pitch, yaw, altitude, rollConstants, pitchConstants, yawConstants, altitudeConstants, minPIDValue,
                           maxPIDValue);


}//end of setup void

void loop() {
  pidManager.roll.setPIDConstants(kp, ki, kd);
  pidManager.pitch.setPIDConstants(kp, ki, kd);

                          // the previous time is stored before the actual time read
  


#if sensor==55
  stabilizer.readIMUValues();
  time = millis();                        // actual time read
  elapsedTime = (time - timePrev) / 1000; //divide by 1000 in order to obtain seconds
  
  rTimePrev = rTime;
  pidManager.calculateRollPIDValueWithoutLPF(
    roll_desired_angle,
    stabilizer.roll,
    elapsedTime
  );
  pidManager.calculatePitchPIDValueWithoutLPF(
    pitch_desired_angle,
    stabilizer.pitch,
    elapsedTime
  );
  pidManager.calculateYawPIDValueWithoutLPF(
    yaw_desired_angle,
    stabilizer.yaw,
    elapsedTime
  );
timePrev = time;
#elif sensor==390
  if (! stabilizer.bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  stabilizer.readAltitude();
  rTimePrev = rTime;
  pidManager.calculateBMPPIDValue(
    desired_altitude,
    stabilizer.altitude,
    elapsedTime
  );
#else
  stabilizer.readIMUValues();
  stabilizer.readAltitude();
  rTimePrev = rTime;
  pidManager.calculateRollPIDValueWithoutLPF(
    roll_desired_angle,
    stabilizer.roll,
    elapsedTime
  );
  pidManager.calculatePitchPIDValueWithoutLPF(
    pitch_desired_angle,
    stabilizer.pitch,
    elapsedTime
  );
  pidManager.calculateYawPIDValueWithoutLPF(
    yaw_desired_angle,
    stabilizer.yaw,
    elapsedTime
  );
  if (! stabilizer.bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }
  
  pidManager.calculateBMPPIDValue(
    desired_altitude,
    stabilizer.altitude,
    elapsedTime
  );
#endif


  limitPWM();

#if process=='r'
  printRoll();
  rollTest();

#elif process=='p'
  printPitch();
  pitchTest();

#elif process=='y'
  printYAW();
  yawTest();

#elif process=='a'
//  printHeight();
  pidPressureTest();

#elif process=='c'
  //  printcombined();
  rollPitchTest();

#elif process=='A'
  //  printAll();
  //  Serial.println("Kd = " + (String)kd + " |Roll angle = " + (String) stabilizer.roll + " |pwm Right = " + (String)pwmRR + " |pwm Left = " + (String)pwmFL + " |pid value = " + (String) pidManager.roll.pidValue);
  pidAllTest();
  //  delay(100);

rTime = millis();
responseTime = (rTime - rTimePrev) / 1000;

  //not yet implemented the pid for pitch , roll , altituide and yaw all together
  /* #elif process=='A':
     printPitch();
     rollPitchTest();
  */

#endif



  //  if (Serial.available() > 0) {
  //    serialReceivingInput = Serial.parseFloat();
  //    Serial.println("Desired angle: " + (String) desired_angle);
  //
  //  }
  //  if (serialReceivingInput != 0)
  //    desired_angle = serialReceivingInput;

  //  if (pidManager.roll.error > 8 || pidManager.roll.error < -8) {
  //
  //    kd += 0.0002;
  //  }

  sendDataToExcel();
}//end of loop void


void setupESCs() {
  esc_front_right.attach(FR, minPWM, maxPWM);
  esc_front_left.attach(FL, minPWM, maxPWM);
  esc_rear_right.attach(RR, minPWM, maxPWM);
  esc_rear_left.attach(RL, minPWM, maxPWM);

  esc_front_right.writeMicroseconds(1000);
  esc_front_left.writeMicroseconds(1000);
  esc_rear_right.writeMicroseconds(1000);
  esc_rear_left.writeMicroseconds(1000);

  delay(2000);


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
  //    if (pidManager.roll.error < 3 && pidManager.roll.error > -3)
  //        hover();
}


void pitchTest() {
  pwmFR = throttle - pidManager.pitch.pidValue;
  pwmFL = throttle - pidManager.pitch.pidValue;
  pwmRR = throttle + pidManager.pitch.pidValue;
  pwmRL = throttle + pidManager.pitch.pidValue;
  updateESCs();
  //    if (pidManager.pitch.error < 3 && pidManager.pitch.error > -3)
  //        hover();

}

void yawTest() {
  pwmFR = throttle - pidManager.pitch.pidValue;
  pwmFL = throttle + pidManager.pitch.pidValue;
  pwmRR = throttle + pidManager.pitch.pidValue;
  pwmRL = throttle - pidManager.pitch.pidValue;
  updateESCs();
  //    if (pidManager.yaw.error < 3 && pidManager.yaw.error > -3)
  //        hover();
}

void pidPressureTest() {
  //pid signs not tested yet
  pwmFR = throttle - pidManager.altitudeApprox.pidValue ;
  pwmRR = throttle - pidManager.altitudeApprox.pidValue ;
  pwmFL = throttle + pidManager.altitudeApprox.pidValue ;
  pwmRL = throttle + pidManager.altitudeApprox.pidValue ;
  updateESCs();
}
void rollPitchTest() {
  pwmRL = throttle + pidManager.roll.pidValue + pidManager.pitch.pidValue;
  pwmFL = throttle + pidManager.roll.pidValue - pidManager.pitch.pidValue;
  pwmRR = throttle - pidManager.roll.pidValue + pidManager.pitch.pidValue;
  pwmFR = throttle - pidManager.roll.pidValue - pidManager.pitch.pidValue;
  updateESCs();
}
void pidAllTest() {
  pwmRL = throttle + pidManager.roll.pidValue + pidManager.pitch.pidValue ;//+ pidManager.yaw.pidValue;//+ pidManager.altitudeApprox.pidValue;
  pwmFL = throttle + pidManager.roll.pidValue - pidManager.pitch.pidValue ;//- pidManager.yaw.pidValue;//+ pidManager.altitudeApprox.pidValue;//- pidManager.yaw.pidValue ;
  pwmRR = throttle - pidManager.roll.pidValue + pidManager.pitch.pidValue ;//- pidManager.yaw.pidValue;//+ pidManager.altitudeApprox.pidValue;//- pidManager.yaw.pidValue ;
  pwmFR = throttle - pidManager.roll.pidValue - pidManager.pitch.pidValue ;//+ pidManager.yaw.pidValue;//+ pidManager.altitudeApprox.pidValue;//+ pidManager.yaw.pidValue ;
  updateESCs();
}

void printRoll()
{
  //Serial.print(" pidValue:");
  //Serial.print(pidManager.roll.pidValue);
  Serial.print("pwmFR:");
  Serial.print(pwmFR);
  Serial.print(" pwmRR:");
  Serial.print(pwmRR);
  Serial.print(" pwmFL:");
  Serial.print(pwmFL);
  Serial.print(" pwmRL:");
  Serial.print(pwmRL);
  Serial.print(" Roll_Error:");
  Serial.print(pidManager.roll.error);
  Serial.print(" Roll_Angle:");
  Serial.print(stabilizer.roll);
  Serial.println();
}
void printPitch()
{
  //Serial.print(" pidValue:");
  //Serial.print(pidManager.pitch.pidValue);
  Serial.print("pwmFR:");
  Serial.print(pwmFR);
  Serial.print(" pwmFL:");
  Serial.print(pwmFL);
  Serial.print(" pwmRR:");
  Serial.print(pwmRR);
  Serial.print(" pwmRL:");
  Serial.print(pwmRL);
  Serial.print(" Pitch_Error:");
  Serial.print(pidManager.pitch.error);
  Serial.print(" Pitch_Angle:");
  Serial.print(stabilizer.pitch);
  Serial.println();
}
void printYAW()
{
  //Serial.print(" pidValue:");
  //Serial.print(pidManager.yaw.pidValue);
  Serial.print("pwmFR:");
  Serial.print(pwmFR);
  Serial.print(" pwmRL:");
  Serial.print(pwmRL);
  Serial.print(" pwmFL:");
  Serial.print(pwmFL);
  Serial.print(" pwmRR:");
  Serial.print(pwmRR);
  Serial.print(" YAW_Error:");
  Serial.print(pidManager.yaw.error);
  Serial.print(" YAW_Angle:");
  Serial.print(stabilizer.yaw);
  Serial.println();
}
void printHeight() {
//  Serial.print(" Altituide_Error:");
  Serial.print(pidManager.altitudeApprox.error);
  Serial.print(" ");
  Serial.print(stabilizer.altitude);
  Serial.println();
}
void printcombined() {
  Serial.print("pwmFR:");
  Serial.print(pwmFR);
  Serial.print(" pwmFL:");
  Serial.print(pwmFL);
  Serial.print(" pwmRR:");
  Serial.print(pwmRR);
  Serial.print(" pwmRL:");
  Serial.print(pwmRL);
  Serial.print(" Pitch_Error:");
  Serial.print(pidManager.pitch.error);
  Serial.print(" Pitch_Angle:");
  Serial.print(stabilizer.pitch);
  Serial.print(" Roll_Error:");
  Serial.print(pidManager.roll.error);
  Serial.print(" Roll_Angle:");
  Serial.print(" ");
  Serial.print(stabilizer.roll);
  printPIDValuesAll();
  Serial.println();
}

void printAll() {
  //Serial.print(" pidValue:");
  //Serial.print(pidManager.yaw.pidValue);
  Serial.print("pwmFR:");
  Serial.print(pwmFR);
  Serial.print(" pwmRL:");
  Serial.print(pwmRL);
  Serial.print(" pwmFL:");
  Serial.print(pwmFL);
  Serial.print(" pwmRR:");
  Serial.print(pwmRR);
  Serial.print(" Roll_Error:");
  Serial.print(pidManager.roll.error);
  Serial.print(" Roll_Angle:");
  Serial.print(stabilizer.roll);
  Serial.print(" Pitch_Error:");
  Serial.print(pidManager.pitch.error);
  Serial.print(" Pitch_Angle:");
  Serial.print(stabilizer.pitch);
  Serial.print(" YAW_Error:");
  Serial.print(pidManager.yaw.error);
  Serial.print(" YAW_Angle:");
  Serial.print(stabilizer.yaw);
  printPIDValuesAll();
  //    Serial.print(" Altituide_Error:");
  //  Serial.print(pidManager.altitudeApprox.error);
  //  Serial.print(" Altituide:");
  //  Serial.print(stabilizer.altitude);
  Serial.println();
}

void printPWMForResponseVisualization() {
  Serial.print("pwmFR:");
  Serial.print(pwmFR);
  Serial.print(" pwmRL:");
  Serial.print(pwmRL);
  Serial.print(" pwmFL:");
  Serial.print(pwmFL);
  Serial.print(" pwmRR:");
  Serial.print(pwmRR);
}

void printPIDValuesAll() {
  Serial.print("roll pid: ");
  Serial.print(pidManager.roll.pidValue);
  Serial.print(" pitch pid: ");
  Serial.print(pidManager.pitch.pidValue);
  Serial.print(" yaw pid: ");
  Serial.print(pidManager.yaw.pidValue);
}

void printPIDParts() {
  Serial.print("Roll -> ");
  Serial.print("pid value: ");
  Serial.print(pidManager.roll.pidValue);
  Serial.print(" | proportional: ");
  Serial.print(pidManager.roll.proportional);
  Serial.print(" | integral: ");
  Serial.print(pidManager.roll.integral);
  Serial.print(" | derivative: ");
  Serial.print(pidManager.roll.derivative);
  Serial.println();

}

void sendDataToExcel() {
  Serial.print(pwmFR);
  Serial.print(",");
  Serial.print(pwmFL);
  Serial.print(",");
  Serial.print(pwmRR);
  Serial.print(",");
  Serial.print(pwmRL);
  Serial.print(",");
//    Serial.print(stabilizer.yaw);
//  Serial.print(stabilizer.altitude);
//  Serial.print(",");
//  Serial.print(pidManager.altitudeApprox.error);
  //  Serial.print(",");
  //  Serial.print(pidManager.pitch.error);
//    Serial.print(",");
    Serial.print(stabilizer.pitch);
    Serial.print(",");
  //  Serial.print(pidManager.roll.error);
  //  Serial.print(" ");
    Serial.print(stabilizer.roll);
  //  Serial.print(",");
  //  Serial.print(pidManager.roll.pidValue);
  //  Serial.print(",");
  //  Serial.print(pidManager.pitch.pidValue);
  Serial.println();
}
//Call in setup
//void calibrateThrottle(float baseAltitude, float altitude) {
//    float accuracy = 0.25;
//    for (int i = 1040; i < 1500; ++i) {
//        if (altitude > baseAltitude + acuuracy){
//            throttle = i;
//            break;
//        }
//    }
//
//}
