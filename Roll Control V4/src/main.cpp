#include <Arduino.h>
#include <MPU9250.h>

#include <SPI.h>
#include<SD.h>

#include "SdFat.h"
#include "FreeStack.h"

/*
#include <UI/electricui.h>
#include <UI/eui_macro.h>
#include <UI/eui_types.h>
#include <UI/eui_utilities.h>*/


MPU9250 imu(Wire, 0x68);

//define some variables
float XAccel; float YAccel; float ZAccel;
float XGyro; float YGyro; float ZGyro;
float XMag; float YMag; float ZMag;

int solenoidPin_1 = 13; int solenoidPin_2 = 14;

//Math Portion
float Roll_Angle; float Pitch_Angle; float Yaw_Angle;
float Roll_Old; float Pitch_Old; // for filtering
float Roll; float Pitch; float Yaw;
float moment_inertia; float angular_moment;
float torque; float mass_flow;
float mass = 10; float radius = 5;
float thrust = 145.6; float isp = 55.51;
float Vtwo; float density = 1.87;
float fire;

// Time portion
float dt {0.0f};
float millisOld {0.0f};

float currTime {0.0f}; 
float prevTime {0.0f}; 

// File Setup
const int chipSelectPin = 48;
File myFile;

/*
// User configurable settings
uint8_t   blink_enable = 1;
uint8_t   led_state    = 0;
uint16_t  glow_time    = 200;

uint32_t  led_timer  = 0;   // track when the light turned on or off

eui_interface_t serial_comms = EUI_INTERFACE( &serial_write ); 
eui_message_t tracked_vars[] =
{
  EUI_UINT8(  "led_blink",  blink_enable ),
  EUI_UINT8(  "led_state",  led_state ),
  EUI_UINT8(  "XAccel",  XAccel ),
  EUI_UINT8(  "YAccel",  YAccel ),
  EUI_UINT8(  "ZAccel",  ZAccel ),
  EUI_UINT8(  "XGyro",  XGyro ),
  EUI_UINT8(  "YGyro",  YGyro ),
  EUI_UINT8(  "ZGyro",  ZGyro ),
  EUI_UINT8(  "XMag",  XMag ),
  EUI_UINT8(  "YMag",  YMag ),
  EUI_UINT8(  "ZMag",  ZMag ),
  EUI_UINT8(  "Pitch",  Pitch ),
  EUI_UINT8(  "Roll",  Roll ),
  EUI_UINT8(  "Yaw",  Yaw ),
  EUI_UINT16( "glow_time",   glow_time ),
};

*/

void setup() {
  
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(solenoidPin_1, OUTPUT); //Sets the pin as an output
  pinMode(solenoidPin_2, OUTPUT);
/*
    // Provide the library with the interface we just setup
  eui_setup_interface( &serial_comms );
    // Provide the tracked variables to the library
  EUI_TRACK( tracked_vars);

  // Provide a identifier to make this board easy to find in the UI
  eui_setup_identifier( "hello", 5 );

  led_timer = millis();
*/
  imu.begin();
  // setting the accelerometer full scale range to +/-8G 
  imu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  imu.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // setting DLPF bandwidth to 20 Hz
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  imu.setSrd(19);
  
  SD.begin(chipSelectPin);
  if (!SD.begin(chipSelectPin)) {
    Serial.println("SD card initialization failed!");
    while(1);
  }
  Serial.println("SD card initialized.");

   // Open a file for writing
  myFile = SD.open("datalog.txt", FILE_WRITE);
  if (!myFile) {
    Serial.println("Error opening datalog.txt");
    return;
  }

  myFile.close();
}


void loop() {
  //Time Step
  currTime = millis();
  dt = (currTime - prevTime)/1000;

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

  Roll_Angle = Roll_Angle + ((XGyro*(180/(3.1416))) * dt);
  Pitch_Angle = Pitch_Angle + ((YGyro*(180/(3.1416))) * dt);
  Yaw_Angle = Yaw_Angle + ((ZGyro*(180/(3.1416))) * dt);




  Roll_Old = Roll + ((XGyro*(180/(3.1416))) * dt);
  Pitch_Old = Pitch + ((YGyro*(180/(3.1416))) * dt);
    //Complementary filter, not for Yaw b/c doesn't have one
  Roll = 0.96 * Roll_Old + 0.04 * XAccel;
  Pitch = 0.96 * Pitch_Old + 0.04 * YAccel;

  // Open the file in append mode
  myFile = SD.open("datalog.txt", FILE_WRITE);
  if (myFile) {
  Serial.print("Time: ");
  Serial.print(currTime/1000, 3);
  Serial.print("s ");
  Serial.print("\t");
    // Write your data to the file
  Serial.print(" Roll_Angle: ");
  Serial.print(Roll_Angle,3); 
  Serial.print("\t");
  Serial.print(" Pitch_Angle: ");
  Serial.print(Pitch_Angle,3);
  Serial.print("\t");
  Serial.print(" Yaw_Angle: ");
  Serial.print(Yaw_Angle,3); 
  Serial.print("\t");

  Serial.print(" Roll: ");
  Serial.print(Roll,3); 
  Serial.print("\t");
  Serial.print(" Pitch: ");
  Serial.print(Pitch,3);
  Serial.print("\t");
  Serial.print(" Yaw: ");
  Serial.print(ZGyro,3); 
  Serial.print("\t");

  myFile.close();


  //Section for Rotation
  //need to be able to tell code, to know what it's original position was
  //to determine how far it moves from that original position
  if (abs(Yaw) > Yaw) {
    
  }
  Serial.println();


  prevTime = currTime;
  delay(5);
}
}

/*
void serial_write( uint8_t *data, uint16_t len )
{
  Serial.write( data, len ); //output on the main serial port
}
*/