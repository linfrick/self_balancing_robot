#include "Wire.h"
#include <AFMotor.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

#define Kp  40
#define Kd  0.05
#define Ki  40
#define sampleTime  0.005
#define targetAngle 0

AF_DCMotor motor_left(4);
AF_DCMotor motor_right(3);


MPU6050 mpu;

int16_t accX, accZ, gyroY;
volatile int motorPower;
volatile float accAngle, gyroAngle, gyroRate, currentAngle, prevAngle=0, error, prevError=0, errorSum=0;
volatile byte count=0;

void setMotors(int leftMotorSpeed, int rightMotorSpeed) {
  if(leftMotorSpeed >= 0) {
    motor_left.setSpeed(leftMotorSpeed);
    motor_left.run(FORWARD);
  }
  else {
    motor_left.setSpeed(-leftMotorSpeed);
    motor_left.run(BACKWARD);
  }
  if(rightMotorSpeed >= 0) {
    motor_right.setSpeed(rightMotorSpeed);
    motor_right.run(FORWARD);
  }
  else {
    motor_right.setSpeed(-rightMotorSpeed);
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
  mpu.setXAccelOffset(-147);
  mpu.setZAccelOffset(5345);
  mpu.setYGyroOffset(-32);
  // initialize PID sampling loop
  init_PID();
}

void loop() {
  // read acceleration and gyroscope values
  accX = mpu.getAccelerationX();
  accZ = mpu.getAccelerationZ();  
  gyroY = mpu.getRotationY();
  // set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  setMotors(motorPower, motorPower);

  Serial.println(motorPower);
  /*
  Serial.print(prevAngle + gyroAngle);
  Serial.print(" - ");
  Serial.println(accAngle);
  
  //Serial.print("Angolo misurato: "); //con la stringa non lo plotta
  //Serial.print("currentAngle ");
  //Serial.println(currentAngle);
  Serial.print(" gyroY ");
  Serial.print(gyroY);
  Serial.print(" gyroZ ");
  Serial.print(mpu.getRotationZ());
  Serial.print(" gyroX ");
  Serial.println(mpu.getRotationX());
  */
}

float in_min = -32768;
float in_max = 32767;
float out_min = -250;
float out_max = 250;
float k1 = (out_max - out_min) / (in_max - in_min);
float k2 = out_min - (in_min * k1);

float fmap(float x)
{
  return x*k1 + k2;
}

// The ISR will be called every 5 milliseconds
ISR(TIMER1_COMPA_vect)
{
  // calculate the angle of inclination
  accAngle = atan2(accX, accZ)*RAD_TO_DEG;
  gyroRate = fmap(gyroY)- 1.03;
  gyroAngle = gyroRate*sampleTime;  
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
