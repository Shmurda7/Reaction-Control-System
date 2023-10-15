#include <Arduino.h>
#include <MPU9250.h>

MPU9250 imu(Wire, 0x68);

//define some variables
float XAccel; float YAccel; float ZAccel;
float XGyro; float YGyro; float ZGyro;
float XMag; float YMag; float ZMag;

int solenoidPin_1 = 13;

//Math Portion
float Roll_Old; float Pitch_Old; // for filtering
float Roll; float Pitch; float Yaw;

// Time portion
float dt {0.0f}; // Lam 
float millisOld {0.0f}; // Lam

float currTime {0.0f}; // -Lam
float prevTime {0.0f}; // -Lam


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(solenoidPin_1, OUTPUT); //Sets the pin as an output

  imu.begin();
  // setting the accelerometer full scale range to +/-8G 
  imu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  imu.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // setting DLPF bandwidth to 20 Hz
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  imu.setSrd(19);

}

void loop() {
  //Time Step
  /*currTime = millis();
  dt = (currTime - prevTime)/1000;
  Serial.print("Time: ");
  Serial.print(currTime/1000, 3);
  Serial.print("s ");
  Serial.print("\t");*/

  // Reads the Data
  imu.readSensor();
  //Acceleration (m/s)
  XAccel = imu.getAccelX_mss();
  YAccel = imu.getAccelY_mss();
  ZAccel = imu.getAccelZ_mss();
  //Gyroscope (rot/s)
  XGyro = imu.getGyroX_rads();
  YGyro = imu.getGyroY_rads();
  ZGyro = imu.getGyroZ_rads();
  //Magnometer ()
  XMag = imu.getMagX_uT();
  YMag = imu.getMagY_uT();
  ZMag = imu.getMagZ_uT();
/*
  // display the data
  //Serial.print(" AX: ");
  Serial.print(XAccel,3);
  Serial.print("\t");
  //Serial.print(" AY: ");
  Serial.print(YAccel,3);
  Serial.print("\t");
  //Serial.print(" AZ: ");
  Serial.print(ZAccel,3);
  Serial.print("\t");
  //Serial.print(" GX: ");
  Serial.print(XGyro,3);
  Serial.print("\t");
  //Serial.print(" GY: ");
  Serial.print(YGyro,3);
  Serial.print("\t");
  //Serial.print(" GZ: ");
  Serial.print(ZGyro,3);
  Serial.print("\t");
  //Serial.print(" MX: ");
  Serial.print(XMag,3);
  Serial.print("\t");
  //Serial.print(" MY: ");
  Serial.print(YMag,3);
  Serial.print("\t");
  //Serial.print(" MZ: ");
  Serial.print(ZMag,3);
  Serial.print("\t");
*/

  Roll_Old = Roll + ((XGyro*(180/(3.1416))) * dt);
  Pitch_Old = Pitch + ((YGyro*(180/(3.1416))) * dt);
  Yaw = Yaw + ((ZGyro*(180/(3.1416))) * dt);
    //Complementary filter, not for Yaw b/c doesn't have one
  Roll = 0.96 * Roll_Old + 0.04 * XAccel;
  Pitch = 0.96 * Pitch_Old + 0.04 * YAccel;

  //Serial.print(" Roll: ");
  Serial.print(Roll,3); 
  Serial.print("\t");
  //Serial.print(" Pitch: ");
  Serial.print(Pitch,3);
  Serial.print("\t");
  //Serial.print(" Yaw: ");
  Serial.print(Yaw,3); 
  Serial.print("\t");

  /*  //Turn on LED if moved from original position
  if (abs(ZGyro) > 0.2 ){
    digitalWrite(solenoidPin_1, HIGH);
    Serial.print(" Z On");
    } else {                
      digitalWrite(solenoidPin_1, LOW);  
      Serial.print(" Z Off"); 
    }*/

  Serial.println();

  prevTime = currTime;
  delay(5);
}