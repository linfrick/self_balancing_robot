#include <AFMotor.h>

AF_DCMotor motor_a(4);
AF_DCMotor motor_b(3);

void setup() {
  //Set initial speed of the motor & stop
  motor_a.setSpeed(200);
  motor_a.run(RELEASE);
  motor_b.setSpeed(200);
  motor_b.run(RELEASE);
}

void loop() {
  uint8_t i;

  // Turn on motor
  motor_a.run(FORWARD);
  motor_b.run(FORWARD);
  
  

}
