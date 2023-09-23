#include <Adafruit_BMP085.h>

/***************************************************
  This is an example for the BMP085 Barometric Pressure & Temp Sensor

  Designed specifically to work with the Adafruit BMP085 Breakout
  ----> https://www.adafruit.com/products/391

  These pressure and temperature sensors use I2C to communicate, 2 pins
  are required to interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/

// Connect VCC of the BMP085 sensor to 3.3V (NOT 5.0V!)
// Connect GND to Ground
// Connect SCL to i2c clock - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 5
// Connect SDA to i2c data - on '168/'328 Arduino Uno/Duemilanove/etc thats Analog 4
// EOC is not used, it signifies an end of conversion
// XCLR is a reset pin, also not used here

Adafruit_BMP085 bmp;
double baseline;
double error, desiredValue;
void setup() {
  Serial.begin(9600);
  desiredValue = 2;
  if (!bmp.begin()) {
    Serial.println("Could not find a valid BMP085 sensor, check wiring!");
    while (1) {}
  } else {
    baseline = bmp.readPressure();
    error = calculateAvgAltitude();
    Serial.println("Average error: " + (String)calculateAvgAltitude());
  }


}

void loop() {
  //    Serial.print("Temperature = ");
  //    Serial.print(bmp.readTemperature());
  //    Serial.println(" *C");
  //
  //    Serial.print("Pressure = ");
  //    Serial.print(bmp.readPressure());
  //    Serial.println(" Pa");
  //
  //    Serial.print("Altitude = ");
  //    Serial.print(bmp.readAltitude());
  //    Serial.println(" meters");
  //
  //    Serial.print("Pressure at sealevel (calculated) = ");
  //    Serial.print(bmp.readSealevelPressure());
  //    Serial.println(" Pa");
  double adjustedAltitude = reachToDesiredValue(desiredValue, bmp.readAltitude(baseline), error, 1);
  Serial.print("Real altitude = ");
  Serial.print(adjustedAltitude);
  Serial.println(" meters");
  //
  //  Serial.println();
  //    delay(500);
}
double calculateAvgAltitude() {
  double readings = 0;
  double readingAvg = 0;
  for (int i = 0; i < 100; i++) {
    double reading = bmp.readAltitude(baseline);
    readings += bmp.readAltitude(baseline);
  }
  readingAvg = readings / 100;
  return readingAvg;
}

double reachToDesiredValue(double desired, double reading, double error, double accuracy) {
  double adjustedValue;
  if (reading >= desired - accuracy && reading <= desired + accuracy) {
    if (reading < desired) {
      if (error > 0) {
        adjustedValue = reading - error;
      } else {
        adjustedValue = reading + error;
      }
    } else {
      if (error > 0) {
        adjustedValue = reading + error;
      } else {
        adjustedValue = reading - error;
      }
    }
  }else{
    adjustedValue = reading;
  }
  return adjustedValue;

}
