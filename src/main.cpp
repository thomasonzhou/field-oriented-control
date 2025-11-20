#include "Arduino.h"
#include "Wire.h"
#include "SPI.h"
#include "SimpleFOC.h"
#include "SimpleFOCDrivers.h"
#include "encoders/as5048a/MagneticSensorAS5048A.h"

constexpr int SENSOR1_CS = 10;
MagneticSensorAS5048A sensor1(SENSOR1_CS);

BLDCDriver3PWM driver1 = BLDCDriver3PWM(9, 5, 6, 8);

constexpr int HT2205_POLE_PAIRS = 7;
BLDCMotor motor1 = BLDCMotor(HT2205_POLE_PAIRS);

Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&motor1.target, cmd); }
void doLimit(char* cmd) { command.scalar(&motor1.voltage_limit, cmd); }

void setup() {
  Serial.begin(115200);
  sensor1.init();

  SimpleFOCDebug::enable(&Serial);
  
  driver1.voltage_power_supply = 12;
  driver1.voltage_limit = 12;

  if (!driver1.init()){
    Serial.println("Driver init failed");
    return;
  }

  motor1.linkDriver(&driver1);
  motor1.controller = MotionControlType::velocity_openloop;

  if (!motor1.init()){
    Serial.println("Motor init failed");
    return;
  }

  motor1.target = 2.0f * M_PI;

  command.add('T', doTarget, "target velocity");
  command.add('L', doLimit, "voltage limit");
}

void loop(){
  sensor1.update();
  // Serial.println(sensor1.getAngle());
  // driver1.setPwm(3,6,5);
  motor1.move();
  command.run();
}