#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"

#define SENSOR1_CS 10
MagneticSensorAS5048A sensor1(SENSOR1_CS);


void setup() {
  Serial.begin(115200);
  sensor1.init();
}

void loop(){
  sensor1.update();
  Serial.println(sensor1.getAngle());
}