//
// Created by Fares on 10/23/2022.
//

#ifndef GRADUATION_PROJECT_2023_DRONE_STABILIZER_H
#define GRADUATION_PROJECT_2023_DRONE_STABILIZER_H

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "Adafruit_BMP3XX.h"

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

class Stabilizer {

private:
    imu::Vector<3> eulerDegrees;
    float baselineSum = 0;

public:
    Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28, &Wire);
    Adafruit_BMP3XX bmp;
    float roll, pitch, yaw;
    float baseline;
    bool isBMPBaselineSet = false;
    float altitude;
    void setupBNO();
    void setupBMPI2C();
    void setupBMPSPI();
    void setBaseline();
    void readAltitude();
    void readIMUValues();
};


#endif //GRADUATION_PROJECT_2023_DRONE_STABILIZER_H
