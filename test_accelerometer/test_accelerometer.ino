#include "I2Cdev.h"
#include "MPU6050.h"
#include "math.h"

MPU6050 mpu;

int16_t accX, accY, accZ, gyroX, gyroY, gyroZ;
float accAngle, gyroAngle, gyroRate;
void setup() {  
  mpu.initialize();
  Serial.begin(9600);
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

void loop() {  
  accZ = mpu.getAccelerationZ();
  accX = mpu.getAccelerationX();
  accY = mpu.getAccelerationY();
  gyroX = mpu.getRotationX();
  gyroY = mpu.getRotationY();
  gyroZ = mpu.getRotationZ();
  
  accAngle = atan2(accX, accZ)*RAD_TO_DEG;
  gyroRate = fmap(gyroY) - 1.03;
  gyroAngle = gyroAngle + (float)gyroRate*0.005;
  
  if(isnan(gyroAngle));
  else
    Serial.println(accAngle);
}
