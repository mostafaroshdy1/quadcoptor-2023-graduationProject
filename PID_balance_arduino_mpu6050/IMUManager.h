//
// Created by Fares on 10/23/2022.
//

#ifndef GRADUATION_PROJECT_2023_DRONE_IMUMANAGER_H
#define GRADUATION_PROJECT_2023_DRONE_IMUMANAGER_H

#include <Wire.h>

class IMUManager {
  private:
    //Variables for gyro
    //Variables for time control
    int gyro_error = 0;                       //We use this variable to only calculate once the gyro data error
    int16_t Gyr_rawX, Gyr_rawY, Gyr_rawZ;     //Here we store the raw data read
    float Gyro_angle_x, Gyro_angle_y, Gyro_angle_z;        //Here we store the angle value obtained with Gyro data
    float Gyro_raw_error_x, Gyro_raw_error_y, Gyro_raw_error_z; //Here we store the initial gyro data error

    //Variables for acc
    int acc_error = 0;                       //We use this variable to only calculate once the Acc data error
    float rad_to_deg = 180 / 3.141592654;    //This value is for passing from radians to degrees values
    float Acc_rawX, Acc_rawY, Acc_rawZ;    //Here we store the raw data read
    float Acc_angle_x, Acc_angle_y;          //Here we store the angle value obtained with Acc data
    float Acc_angle_error_x, Acc_angle_error_y; //Here we store the initial Acc data error

    

  public:
    float elapsedTime, time, timePrev;
    // Arrays and ptr
    float totalIMUError[5] = {0};         //Store All error values for both acc and gyro
    float Acceleration_angle[2] = {0};     //Store the roll and pitch of ACC
    float Gyro_angle[3] = {0};    //Store the roll , pitch and yaw of GYRO
    float Total_angle[3]; //Store filtered data
    float *calculateIMUError();
    float *readAccelerometerData(float *acc_error);

    float *readGyroData(float *gyro_error);

    float *getTotalAnglesWithComplementaryFilter(float *gyroAngles, float *accelerometerAngles);
};


#endif //GRADUATION_PROJECT_2023_DRONE_IMUMANAGER_H
