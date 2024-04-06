#include "Wire.h"
#include <AFMotor.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle -2.5

AF_DCMotor motor_left(4);
AF_DCMotor motor_right(3);


MPU6050 mpu;

int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    motor_left.setSpeed(leftMotorSpeed);
    motor_left.run(FORWARD);
  }
  else {
    motor_left.setSpeed(255 + leftMotorSpeed);
    motor_left.run(BACKWARD);
  }
  if(rightMotorSpeed >= 0) {
    motor_right.setSpeed(rightMotorSpeed);
    motor_right.run(FORWARD);
  }
  else {
    motor_right.setSpeed(255 + rightMotorSpeed);
    motor_right.run(BACKWARD);
  }
}

void init_PID() {  
  // initialize Timer1
  cli();          // disable global interrupts
  TCCR1A = 0;     // set entire TCCR1A register to 0
  TCCR1B = 0;     // same for TCCR1B    
  // set compare match register to set sample time 5ms
  OCR1A = 9999;    
  // turn on CTC mode
  TCCR1B |= (1 << WGM12);
  // Set CS11 bit for prescaling by 8
  TCCR1B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK1 |= (1 << OCIE1A);
  sei();          // enable global interrupts
}

void setup() {
  Serial.begin(9600);
  // set the status LED to output mode 
  pinMode(13, OUTPUT);
  // release motors
  motor_left.run(RELEASE);
  motor_right.run(RELEASE);
  // initialize the MPU6050 and set offset values
  mpu.initialize();
  mpu.setYAccelOffset(1593);
  mpu.setZAccelOffset(963);
  mpu.setXGyroOffset(40);
  // initialize PID sampling loop
  init_PID();
}

void loop() {
  // read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();  
  gyroX = mpu.getRotationX();
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);
  //Serial.print("Angolo misurato: "); //con la stringa non lo plotta
  Serial.println(currentAngle);
}
// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accY, accZ)*RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate*sampleTime;  
  currentAngle = 0.9934*(prevAngle + gyroAngle) + 0.0066*(accAngle);
  
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;  
  errorSum = constrain(errorSum, -300, 300);
  //calculate output from P, I and D values
  motorPower = Kp*(error) + Ki*(errorSum)*sampleTime - Kd*(currentAngle-prevAngle)/sampleTime;
  prevAngle = currentAngle;
  // toggle the led on pin13 every second
  count++;
  if(count == 200)  {
    count = 0;
    digitalWrite(13, !digitalRead(13));
  }
}