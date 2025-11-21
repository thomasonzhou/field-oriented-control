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
void doMotor(char* cmd) {command.motor(&motor1, cmd);}

void setup() {
  Serial.begin(115200);
  // comment out for improved performance
  // SimpleFOCDebug::enable(&Serial);
  // motor1.useMonitoring(Serial);

  sensor1.init();
  motor1.linkSensor(&sensor1);
  
  driver1.voltage_power_supply = 12;
  driver1.voltage_limit = 12;
  if (!driver1.init()){
    Serial.println("Driver failed");
    return;
  }
  motor1.linkDriver(&driver1);

  motor1.torque_controller = TorqueControlType::voltage;
  motor1.controller = MotionControlType::torque;

  motor1.voltage_sensor_align = 12.0f;
  if (!motor1.init()){
    Serial.println("Motor failed");
    return;
  }


  if(!motor1.initFOC()){
    Serial.println("FOC failed");
    return;
  }

  motor1.target = 6.0; // Nm  

  command.add('M', doMotor, "Motor");
  Serial.println(F("Motor ready."));
  Serial.println(F("Set the target with command M:"));
}

void loop(){
  motor1.loopFOC();
  motor1.move();
  command.run();
}