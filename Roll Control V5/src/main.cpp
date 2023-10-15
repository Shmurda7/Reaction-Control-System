#include <Arduino.h>
#include <MPU9250.h>
#include <SPI.h>
#include <SD.h>
#include <TinyGPS++.h>
#include <Adafruit_BMP085.h>
#include <HardwareSerial.h>
#include <SoftwareSerial.h>
#include <SimpleKalmanFilter.h>

//Sensor Defining
MPU9250 imu(Wire, 0x68);
Adafruit_BMP085 bmp;
TinyGPSPlus gps;
static const uint32_t BuadRate = 9600;

//try doing the #dfine & DEBUG stuff from Ralph S Bacon
//varbiales for the code
double xgyro, ygyro, zgyro; 

double pitch_angle, roll_angle, yaw_angle, roll, pitch;
double old_pitch_angle, old_roll_angle, old_yaw_angle; //deg
double pitch_speed, roll_speed, yaw_speed;
double old_pitch_speed, old_roll_speed, old_yaw_speed; //deg/s

double xaccel, yaccel, zaccel; //m/s

double xmag, ymag, zmag; //uT

double pres, temp, altitude, sealvlpres;

double lat, lng, sat;

// Time Portion
double dt = 0, old_millis = 0, curr_time = 0, prev_time = 0;

//defining the visual staters & comms ////////MAKE SURE TO GIVE PINS/////////
uint8_t push_button, passive_buzzer;
uint8_t loud_buzzer;
uint8_t red_led, white_led, yellow_led;
uint8_t solenoid_uno = 15, solenoid_dos = 14, solenoid_tres = 13, solenoid_quart = 41;
static const uint8_t rxPin = 0, txPin= 1; //Pins used for Communication
SoftwareSerial ss(rxPin, txPin);      //RX & TX in Serial

//PID Gains
double kP = 1;
double kI = 0.1;
double kD = 0.01;

//error
double cumulative_error = 0; //for I term
double previous_error = 0; // for D term

//File & Flash Setup
File myFile;

//functions
double pid(double yaw_angle) ;
void update_control(double new_yaw_angle, double old_yaw_angle, double dt);
void datalogger();

void setup() {
  //Serial for baudrate
  Serial.begin(BuadRate);

  //Pin out for States
  pinMode(red_led, OUTPUT);
  pinMode(white_led, OUTPUT);
  pinMode(yellow_led, OUTPUT);
  pinMode(push_button, INPUT);
  pinMode(passive_buzzer, OUTPUT);
  pinMode(loud_buzzer, OUTPUT);
  pinMode(solenoid_uno, OUTPUT);
  pinMode(solenoid_dos, OUTPUT);
  pinMode(solenoid_tres, OUTPUT);
  pinMode(solenoid_quart, OUTPUT);

  //Starting State
  digitalWrite(passive_buzzer, LOW);
  digitalWrite(loud_buzzer, LOW);
  digitalWrite(red_led, LOW);
  digitalWrite(white_led, LOW);
  digitalWrite(yellow_led, LOW);

  if (!imu.begin()) {
    Serial.println("MPU9250 initialization failed!");
    while(1);
  }
  // setting the accelerometer full scale range to +/-8G 
  imu.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  imu.setGyroRange(MPU9250::GYRO_RANGE_250DPS);
  // setting DLPF bandwidth to 20 Hz
  imu.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  imu.setSrd(19);
/*
    //Data Logging Initilization
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("SD card initialization failed!");
    while(1);
  }
  Serial.println("SD card initialized.");
  //Open the file
    myFile = SD.open("F_DATA.txt", FILE_WRITE);
  if (myFile) {
    myFile.println("Time, XAcc, YAcc, ZAcc, Roll_Speed, Pitch_Speed, Yaw_Speed, Roll_Angle, Pitch_Angle, Yaw_Angle, XMag, YMag, ZMag, Alt, SealvlPres, Pres, Temp, Sat, Long, Lat");
    //myFile.close();
  } else {
    Serial.println("Error opening data file!");
  }
*/
  if (!bmp.begin()) {
    Serial.println("BMP180 initialization failed!");
    while(1);
  }
  //Starting GPS
  ss.begin(BuadRate);
}

void loop() {
  curr_time = millis();
  dt = (curr_time - prev_time)/1000;

  // Reads the Data
  imu.readSensor();
  //Acceleration (m/s)
  xaccel = imu.getAccelX_mss();
  yaccel = imu.getAccelY_mss();
  zaccel = imu.getAccelZ_mss();
  //Gyroscope (deg/s)
  roll_speed = imu.getGyroX_rads();
  pitch_speed = imu.getGyroY_rads();
  yaw_speed = imu.getGyroZ_rads();
  //Magnometer (uT)
  xmag = imu.getMagX_uT();
  ymag = imu.getMagY_uT();
  zmag = imu.getMagZ_uT();

  altitude = bmp.readAltitude(); //m
  sealvlpres = bmp.readSealevelPressure(); //Pa
  pres = bmp.readPressure(); //Pa
  temp = bmp.readTemperature(); //C

  lat = gps.location.lat();
  lng = gps.location.lng(); 

  while (ss.available()){
    gps.encode(ss.read());
    sat = gps.satellites.value();
  }

          //Angle Math
  roll_angle = roll_angle + ((roll_speed*(180/(3.1416))) * dt);
  pitch_angle = pitch_angle + ((pitch_speed*(180/(3.1416))) * dt);
  yaw_angle = yaw_angle + ((yaw_speed*(180/(3.1416))) * dt);

  old_roll_speed = roll + ((roll_speed*(180/(3.1416))) * dt);
  old_pitch_speed = pitch + ((pitch_speed*(180/(3.1416))) * dt);

  //Complementary filter, not for Yaw b/c doesn't have one
  roll = 0.96 * old_roll_speed + 0.04 * xaccel;
  pitch = 0.96 * old_pitch_speed + 0.04 * yaccel;

  if (yaw_angle != 0){
    double new_yaw_angle = pid(yaw_angle);
    update_control(new_yaw_angle, old_yaw_angle, dt);
    old_yaw_angle = new_yaw_angle;
  }

  datalogger();

  prev_time = curr_time;
}

void datalogger(){
  myFile = SD.open("F_DATA.txt", FILE_WRITE);
  if (myFile) {
    myFile.print(curr_time/1000, 3);
    //Reads MPU9250 data
    myFile.print(",");
    myFile.print(xaccel);
    myFile.print(",");
    myFile.print(yaccel);
    myFile.print(",");
    myFile.print(zaccel);
    myFile.print(",");
    myFile.print(roll_speed);
    myFile.print(",");
    myFile.print(pitch_speed);
    myFile.print(",");
    myFile.print(yaw_speed);
    myFile.print(",");
    myFile.print(roll_angle);
    myFile.print(",");
    myFile.print(pitch_angle);
    myFile.print(",");
    myFile.print(yaw_angle);
    myFile.print(",");
    myFile.print(xmag);
    myFile.print(",");
    myFile.print(ymag);
    myFile.print(",");
    myFile.print(zmag);

    //Reads BMP180 data
    myFile.print(",");
    myFile.print(altitude);
    myFile.print(",");
    myFile.print(sealvlpres);
    myFile.print(",");
    myFile.print(pres);
    myFile.print(",");
    myFile.print(temp);
    myFile.print(",");
    myFile.print(sat);
    myFile.print(",");
    myFile.print(lat);
    myFile.print(",");
    myFile.print(lng);
    myFile.println();
    myFile.close();


  }
}

void update_control(double new_yaw_angle, double old_yaw_angle, double dt){

  if (yaw_angle > old_yaw_angle) {
    for (double pos = old_yaw_angle; pos <= new_yaw_angle; pos += 1){
      digitalWrite(solenoid_uno, HIGH);
      digitalWrite(solenoid_dos, HIGH);
      digitalWrite(solenoid_tres, LOW);
      digitalWrite(solenoid_quart, LOW);
    }
  }
  if (yaw_angle < old_yaw_angle) {
    for (double pos = old_yaw_angle; pos >= new_yaw_angle; pos -= 1){
      digitalWrite(solenoid_tres, HIGH);
      digitalWrite(solenoid_quart, HIGH);
      digitalWrite(solenoid_uno, LOW);
      digitalWrite(solenoid_dos, LOW);
    }
  }
}

double pid(double yaw_angle) {
  
  double setpoint = 0;
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print(",");

  Serial.print(" Angle: ");
  Serial.print(yaw_angle);
  Serial.print(",");

  Serial.print(" Speed: ");
  Serial.print(yaw_speed);
  Serial.print(",");

  double error = setpoint-yaw_angle;
  Serial.print(" Error: ");
  Serial.print(error);
  Serial.print(",");

  double p_value = error*kP;
  double i_value = cumulative_error*kI;
  double d_value = (error-previous_error)*kD;

  double pid_value = p_value+i_value+d_value;

  Serial.print(" pid_value: ");
  Serial.println(pid_value);

  cumulative_error += error;
  previous_error = error;

  double new_yaw_angle = map(pid_value, -360, 360, 0, 360);

  if (new_yaw_angle > 360){
    new_yaw_angle = 360;
  }

  if (new_yaw_angle < 0){
    new_yaw_angle = 0;
  }

  return pid_value;
}