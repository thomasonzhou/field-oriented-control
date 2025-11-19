#include <SimpleFOC.h>

// for PWM magnetic sensor
constexpr int ENCODER1_PWM_PIN = 2;
constexpr int MIN_ENCODER1_PWM_PULSE_US = 12;
constexpr int MAX_ENCODER1_PWM_PULSE_US = 998;

MagneticSensorPWM sensor1 = MagneticSensorPWM(
    ENCODER1_PWM_PIN,
    MIN_ENCODER1_PWM_PULSE_US,
    MAX_ENCODER1_PWM_PULSE_US);

void non_blocking_encoder1_PWM_handle()
{
  sensor1.handlePWM();
}

void setup()
{
  Serial.begin(115200);
  sensor1.init();
  sensor1.enableInterrupt(non_blocking_encoder1_PWM_handle);
}

void loop()
{
  sensor1.update();

  Serial.print("pos: ");
  Serial.print(sensor1.getAngle());
  Serial.print(", vel: ");
  Serial.print(sensor1.getVelocity());
  Serial.println();
}
