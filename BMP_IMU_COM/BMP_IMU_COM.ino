//#include "BMP180.h"
#include "../PID_balance_arduino/IMU6050.h"
#include "../PID_balance_arduino/z_I2Cdev.h"

char received;
float P;

float *ypr; // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

//BMP180 bmp;
IMU6050 imu;
void setup()
{
  Serial.begin(57600);

  Fastwire::setup(100, true);
  //bmp.init_SENSOR();
  //delay(1000);
  imu.initialize();
  delay(1000);

}

void loop()
{

  //check if serial is avalible
//  if (Serial.available()) {
//    received = Serial.read();
//    if (received == 'p') {
//      P = bmp.get_height();
//      Serial.print("$DS");
//      Serial.print(P);
//      Serial.print("$\n");
//
//    }
//
//    else if (received == 'i') {
      ypr = imu.get_yaw_pitch_roll();
      Serial.print("Yaw: ");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print(" |Pitch:");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("  |Roll:");
      Serial.print(ypr[2] * 180 / M_PI);
      Serial.print("\n");
    //}

 // }



}
