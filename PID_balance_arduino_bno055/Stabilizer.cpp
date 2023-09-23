//
// Created by Fares on 10/23/2022.
//

#include "Stabilizer.h"
#include <Wire.h>
#include "Arduino.h" //To allow Serial.print() in cpp file


#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

void Stabilizer::setupBNO() {
  Serial.begin(9600);

  /* Initialise the sensor */
  if (!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while (1);
  }
  delay(1000);

  bno.setExtCrystalUse(true);
}


void Stabilizer::readIMUValues() {
  eulerDegrees = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  roll = eulerDegrees.z() - 9.28;
  pitch = eulerDegrees.y() -4.6;
  yaw = eulerDegrees.x();
  if(yaw > 180){
    yaw -= 360;
  }
}

void Stabilizer::setupBMPI2C() {
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire

    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

}
void Stabilizer::setupBMPSPI() {

  if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

}
void Stabilizer::setBaseline() {

  if (!isBMPBaselineSet)
  {
    int iterations = 100;
    int skipIterations = 5;
    for (int i = 0; i < iterations; i++) {
      float baselineReading = bmp.readPressure() / 100;
      if (i < skipIterations)
      {
        baselineReading = 0;
        continue;
      }

      baselineSum += baselineReading;

    }
    baseline = baselineSum / (iterations - skipIterations);
    isBMPBaselineSet = true;
  }

}

void Stabilizer::readAltitude() {
  altitude = bmp.readAltitude(baseline);
}
