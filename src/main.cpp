#include <SimpleFOC.h>

// for PWM magnetic sensor
// constexpr int ENCODER1_PWM_PIN = 2;
// constexpr int MIN_ENCODER1_PWM_PULSE_US = 12;
// constexpr int MAX_ENCODER1_PWM_PULSE_US = 998;
// MagneticSensorPWM sensor1 = MagneticSensorPWM(
//     ENCODER1_PWM_PIN,
//     MIN_ENCODER1_PWM_PULSE_US,
//     MAX_ENCODER1_PWM_PULSE_US);
// void non_blocking_encoder1_PWM_handle()
// {
//   sensor1.handlePWM();
// }

// for AS5048A SPI
constexpr int ENC1_CS_PIN = 10;
constexpr int ENC1_SCK_PIN = 13;
constexpr int ENC1_MOSI_PIN = 11;
constexpr int ENC1_MISO_PIN = 12;
constexpr int ENC1_BIT_RES = 14;
constexpr int ENC1_ANGLE_READ_REGISTER = 0x3FFF;
MagneticSensorSPI sensor1 = MagneticSensorSPI(ENC1_CS_PIN, ENC1_BIT_RES, ENC1_ANGLE_READ_REGISTER);

// for 3 phase PWM control
constexpr int DRIVER_PWM_PIN_N1 = 9;
constexpr int DRIVER_PWM_PIN_N2 = 6;
constexpr int DRIVER_PWM_PIN_N3 = 5;
constexpr int DRIVER_PWM_PIN_EN = 4;
BLDCDriver3PWM driver1 = BLDCDriver3PWM(
  DRIVER_PWM_PIN_N1,
  DRIVER_PWM_PIN_N2,
  DRIVER_PWM_PIN_N3,
  DRIVER_PWM_PIN_EN
);

// for gimbal motor
constexpr int MOTOR_POLE_PAIRS = 7;
BLDCMotor motor1 = BLDCMotor(MOTOR_POLE_PAIRS);
constexpr float MOTOR_VOLTAGE = 12.0f;
constexpr float MOTOR_MAX_RPM = 792.0f;

// control
// Commander command = Commander(Serial);
// void run_motor(char* cmd){
//   command.motor(&motor1, cmd);
// }

void setup()
{
  sensor1.init();
  // sensor1.enableInterrupt(non_blocking_encoder1_PWM_handle);
  motor1.linkSensor(&sensor1);

  driver1.voltage_power_supply = MOTOR_VOLTAGE;
  driver1.init();

  motor1.linkDriver(&driver1);
  // motor1.velocity_limit = MOTOR_MAX_RPM * M_PI / 60.0;
  motor1.controller = MotionControlType::velocity;
  motor1.voltage_limit = MOTOR_VOLTAGE;
  motor1.voltage_sensor_align = MOTOR_VOLTAGE;
  motor1.useMonitoring(Serial);
  Serial.begin(115200);
  
  motor1.init();
  motor1.initFOC();

  // command.add('A', run_motor, "motor");
  _delay_ms(1000);
}

void loop()
{
  motor1.loopFOC();
  // Serial.println(sensor1.getAngle() * 180.0/M_PI);

  motor1.move(2.0);
  motor1.monitor();

  // command.run();
}
