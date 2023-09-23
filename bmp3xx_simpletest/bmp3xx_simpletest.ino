/***************************************************************************
  This is a library for the BMP3XX temperature & pressure sensor

  Designed specifically to work with the Adafruit BMP388 Breakout
  ----> http://www.adafruit.com/products/3966

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"

#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BMP3XX bmp;
float baseline;
bool referenceSet = false;
void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Adafruit BMP388 / BMP390 test");

  //  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
  //if (! bmp.begin_SPI(BMP_CS)) {  // hardware SPI mode
  if (! bmp.begin_SPI(BMP_CS, BMP_SCK, BMP_MISO, BMP_MOSI)) {  // software SPI mode
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
  bmp.setOutputDataRate(BMP3_ODR_50_HZ);

  baseline = bmp.readPressure() / 100;
  //  Serial.println("Baseline: "+(String) baseline);
  //  baseline = bmp.readPressure() / 100;
}
float pressureAvg = 0 ;
void loop() {
  if (! bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
    return;
  }

  if (referenceSet == false)
  {
    //    for (int i= 0; i < 5; i++) {
    //      bmp.readPressure() / 100;
    //      referenceSet = true;
    //    }
    for (int i = 0; i < 100; i++) {
      baseline = bmp.readPressure() / 100;
      if (i < 5)
      {
        baseline = 0;
        continue;
      }

      pressureAvg += baseline;

    }
    pressureAvg /= 95;
  }
  referenceSet = true;
  //
  //  Serial.print("Temperature = ");
  //  Serial.print(bmp.temperature);
  //  Serial.println(" *C");
  //
  //  Serial.print("Pressure = ");
  //  Serial.print(bmp.pressure / 100.0);
  //  Serial.println(" hPa");

  //  Serial.print("Approx. Altitude = ");
  float avg = 0;
  int iterations = 1;
  for (int i = 0; i < iterations; i++) {
    avg += bmp.readAltitude(pressureAvg);
  }

  avg /= iterations;
  Serial.print(avg);
  //  Serial.println(" m");

  Serial.println();
  //  delay(2000);
}
