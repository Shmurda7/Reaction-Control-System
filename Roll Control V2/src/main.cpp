#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_BMP085.h>
#include <vector> // -Lam

Adafruit_MPU6050 mpu;
Adafruit_BMP085 bmp;
//define some variables
float XAccel; float YAccel; float ZAccel;
float XGyro; float YGyro; float ZGyro;
float Temp;
float Pres;

int solenoidPin_1 = 13;
int solenoidPin_2 = 14;
int solenoidPin_3 = 15;

//Math Portion
float Roll_Old; float Pitch_Old; // for filtering
float Roll; float Pitch; float Yaw;

// Time portion
float dt {0.0f}; // Lam 
float millisOld {0.0f}; // Lam

float currTime {0.0f}; // -Lam
float prevTime {0.0f}; // -Lam
//#define SAMPLES_PER_SECOND 10 //- Lam

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  mpu.begin();
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG); //affects the refrenced pie chart
  mpu.setFilterBandwidth(MPU6050_BAND_94_HZ);
  pinMode(solenoidPin_1, OUTPUT); //Sets the pin as an output
  pinMode(solenoidPin_2, OUTPUT);
  pinMode(solenoidPin_3, OUTPUT);
  //mpu::Vector<3> gyr = MPU6050_GYRO_CONFIG; //Yo I don't think the mpu class has any vector attribute. I think I know what you mean so Imma just use the vector header -Lam
  //std::vector<int> gyr {MPU6050_GYRO_CONFIG}; //looks like MPU6050_GYRO_CONFIG is typedef for an int
  bmp.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  currTime = millis(); // - Lam
  //if (currTime - prevTime >= 1000.00f/SAMPLES_PER_SECOND)
  //{
  dt = (currTime - prevTime)/1000.00f;
  sensors_event_t a, g, temp; //can possibly have in set up --If you have these in set up, they'll run out of scope in the setup function and be destroyed and you won't be able to use
                                // there in the loop function anymore. make em global variables - Lam
                                // BTW initializing stuff in the loop function is bad practice since you are creating and destroying objects for very loop.  Make them global like I said - Lam                        
  Serial.print("Time: ");
  Serial.print(currTime/1000.0f);
  Serial.print("s ");
  bmp.readPressure();  
  mpu.getEvent(&a, &g, &temp);

  XAccel = a.acceleration.x;
  YAccel = a.acceleration.y;
  ZAccel = a.acceleration.z;
  Serial.print(" AX: ");
  Serial.print(XAccel);
  Serial.print(" AY: ");
  Serial.print(YAccel);
  Serial.print(" AZ: ");
  Serial.print(ZAccel/9.81);
  //collecting data from the gyroscope w/ accounted error
  XGyro = g.gyro.x ; //howtomechantronics mpu 6050 code
  YGyro = g.gyro.y;
  ZGyro = g.gyro.z; 
  Serial.print(" GX: ");
  Serial.print(XGyro);
  Serial.print(" GY: ");
  Serial.print(YGyro);
  Serial.print(" GZ: ");
  Serial.print(ZGyro); // to fix Z drift apparently you need a 9 axis IMU with an mangnometer
  //https://robotics.stackexchange.com/questions/16757/what-is-the-algorithm-to-get-position-linear-displacement-and-linear-velocity
  //understanding IMU
  //https://lastminuteengineers.com/mpu6050-accel-gyro-arduino-tutorial/


  //Positioning Portion
  Roll_Old = Roll + ((XGyro*(180/(3.1416))) * dt);
  Pitch_Old = Pitch + ((YGyro*(180/(3.1416))) * dt);
  Yaw = (Yaw + ((ZGyro*(180/(3.1416))) * dt)) - 0.1;
  //Complementary filter, not for Yaw b/c doesn't have one
  Roll = 0.96 * Roll_Old + 0.04 * XAccel;
  Pitch = 0.96 * Pitch_Old + 0.04 * YAccel;
  Serial.print(" Yaw: ");
  Serial.print(Yaw);  

  //Turn on LED if moved from original position
  if (abs(ZGyro) > 0.02 ){
    digitalWrite(solenoidPin_1, HIGH);
    Serial.print(" Z On");
    } else {                
      digitalWrite(solenoidPin_1, LOW);  
      Serial.print(" Z Off"); 
    }
  
  Serial.println();
  prevTime = currTime;
}