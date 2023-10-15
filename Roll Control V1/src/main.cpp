#include <Arduino.h>
#include <Adafruit_MPU6050.h>

Adafruit_MPU6050 mpu;

float XAccel; float YAccel; float ZAccel;
float XGyro; float YGyro; float ZGyro;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG); //expected range
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ); //filter
}

void loop() {
  // put your main code here, to run repeatedly:
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  ZGyro = g.gyro.z;
  Serial.print(" GZ: ");
  Serial.print(ZGyro);
  Serial.println("");
}