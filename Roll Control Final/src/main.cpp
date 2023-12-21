#include <Arduino.h>
#include <MPU9250.h>
#include <math.h>
#include <EEPROM.h>

//Sensor Defining
MPU9250 IMU(Wire, 0x68);
int status;

////////// Variables //////////
// Accelerometer
double xaccel = 0, yaccel = 0, zaccel = 0;
// Gyroscope
double roll_rate = 0, pitch_rate = 0, yaw_rate = 0;
double roll_angle = 0, pitch_angle = 0, yaw_angle = 0;
// Magnometer
double xmag = 0, ymag = 0, zmag = 0;
// Temperature
double temp;

// Phils lab
int gravity  = 9.81;

//time
double dt = 0, old_millis = 0, curr_time = 0, prev_time = 0;

/////////////// Filters ///////////////////
//Low Pass Filter
int lpfGyr = 0.7;
int  lpfAcc = 0.9;
int  lpfMag = 0.4;

//Complimentary Filter Setup
double alpha = 0.93f;

double accel_roll_angle = 0, accel_pitch_angle = 0, accel_yaw_angle;
double mag_roll_angle = 0, mag_pitch_angle = 0, mag_yaw_angle = 0;
double comp_roll_angle = 0, comp_pitch_angle = 0, comp_yaw_angle = 0;

//bias
double bias_roll_rate, bias_pitch_rate, bias_yaw_rate;

///////////////// PID Controller /////////////////////
//Solenoid Initialization
#define SOLENOID_OFF LOW
#define SOLENOID_ON HIGH

uint8_t solenoid_uno = 21, solenoid_dos = 16, solenoid_tres = 15, solenoid_quart = 41;
double old_yaw_angle;

//PID Gain
int kP = 1;
int kI = 0.1;
int kD = 0.01;

//error
double cumulative_error = 0; //for I term
double previous_error = 0; // for D term

//flash memory
int address = 0;

double pid(double comp_yaw_angle) ;
void update_control(double new_yaw_angle, double old_yaw_angle, double dt);
void datalogger();
void writeDoubleToEEPROM(int address, double value);
double readDoubleFromEEPROM(int address);


void setup() {
  Serial.begin(115200);

  pinMode(solenoid_uno, OUTPUT);
  pinMode(solenoid_dos, OUTPUT);
  pinMode(solenoid_tres, OUTPUT);
  pinMode(solenoid_quart, OUTPUT);

  //while(!Serial) {}

  // start communication with IMU 
  status = IMU.begin();
  /*
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
  */
  // setting the accelerometer full scale range to +/-8G 
  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
  // setting the gyroscope full scale range to +/-500 deg/s
  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
  // setting DLPF bandwidth to 20 Hz
  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
  // setting SRD to 19 for a 50 Hz update rate
  IMU.setSrd(50);
}

void loop() {
  /////////  Time Portion  //////////
  curr_time = millis();
  dt = (curr_time - prev_time)/1000;

  /////////  IMU Reading Portion  //////////

  /////////  IMU Reading Portion  //////////

  //Bias Reading
  bias_roll_rate = IMU.getGyroBiasX_rads();
  bias_pitch_rate = IMU.getGyroBiasY_rads();
  bias_yaw_rate = IMU.getGyroBiasZ_rads();


  //Measurement Reading & Low Pass Filter Implementation
  IMU.readSensor();
  xaccel = lpfAcc*xaccel + (1 - lpfAcc)*IMU.getAccelX_mss();
  yaccel = lpfAcc*yaccel + (1 - lpfAcc)*IMU.getAccelY_mss();
  yaccel = lpfAcc*zaccel + (1 - lpfAcc)*IMU.getAccelY_mss();
  roll_rate = lpfGyr*roll_rate + (1 - lpfGyr)*IMU.getGyroX_rads();
  pitch_rate = lpfGyr*pitch_rate + (1 - lpfGyr)*IMU.getGyroY_rads();
  yaw_rate = lpfGyr*yaw_rate + (1 - lpfGyr)*IMU.getGyroZ_rads();
  xmag = lpfMag*xmag + (1 - lpfMag)*IMU.getMagX_uT();
  ymag = lpfMag*zmag + (1 - lpfMag)*IMU.getMagY_uT();
  zmag = lpfMag*zmag + (1 - lpfMag)*IMU.getMagZ_uT();
  temp = IMU.getTemperature_C();



  /////////  Math Portion  //////////
  //Accelerometer
  //phi_hat = atanf(yaccel/zaccel) * RAD_TO_DEG;
  //theta_hat = asinf(xaccel/ gravity) * RAD_TO_DEG;

  accel_roll_angle = atan2(-xaccel, sqrt(pow(yaccel,2) + pow(zaccel,2)))*RAD_TO_DEG;
  accel_pitch_angle = atan2(yaccel, sqrt(pow(xaccel,2) + pow(zaccel,2)))*RAD_TO_DEG;
  accel_yaw_angle = atan2((sqrt(pow(xaccel,2)+pow(yaccel,2))),zaccel)*RAD_TO_DEG; 
  mag_roll_angle = atan2(ymag,zmag);
  mag_pitch_angle = atan2(xmag,zmag);
  mag_yaw_angle = atan2(ymag,xmag);

  roll_angle += ((roll_rate * RAD_TO_DEG) * dt);
  pitch_angle += ((pitch_rate * RAD_TO_DEG) * dt);
  yaw_angle += ((yaw_rate * RAD_TO_DEG) * dt);

  //Phils Comp Filter
  /*
  ////////// Body Transformation ///////////
  
  //body rates to euler rates
  float phi_dot = 1*roll_rate + sinf(roll_angle*DEG_TO_RAD)*tanf(pitch_angle*DEG_TO_RAD)*pitch_rate + cosf(roll_angle*DEG_TO_RAD)*tanf(pitch_angle*DEG_TO_RAD)*yaw_rate;
  float theta_dot = cosf(roll_angle*DEG_TO_RAD)*pitch_rate - sinf(roll_angle*DEG_TO_RAD)*yaw_rate;
  float gamma_dot = sinf(roll_angle*DEG_TO_RAD)*(1/cosf(pitch_angle*DEG_TO_RAD))*pitch_rate + cosf(roll_angle*DEG_TO_RAD)*(1/cosf(pitch_angle*DEG_TO_RAD))*yaw_rate;
  comp_roll_angle = (alpha*accel_roll_angle + (1 - alpha)*(roll_rate + dt*phi_dot))*RAD_TO_DEG;
  comp_pitch_angle = (alpha*accel_pitch_angle + (1 - alpha)*(pitch_rate + dt*theta_dot))*RAD_TO_DEG;
  comp_yaw_angle = (alpha*accel_yaw_angle + (1 - alpha)*(yaw_rate + dt*gamma_dot))*RAD_TO_DEG;
  */
  //Complimentary Filter w/ accel
  /*
  comp_roll_angle = alpha * (roll_angle + roll_rate * dt) + (1 - alpha) * accel_roll_angle;
  comp_pitch_angle = alpha * (pitch_angle + pitch_rate * dt) + (1 - alpha) * accel_pitch_angle;
  comp_yaw_angle = alpha * (yaw_angle + yaw_rate * dt) + (1 - alpha) * accel_yaw_angle; 
  */
  //Complimentary Filter w/ mag
  comp_roll_angle = alpha * (roll_angle + roll_rate * dt) + (1 - alpha) * mag_roll_angle;
  comp_pitch_angle = alpha * (pitch_angle + pitch_rate * dt) + (1 - alpha) * mag_pitch_angle;
  comp_yaw_angle = alpha * (yaw_angle + yaw_rate * dt) + (1 - alpha) * mag_yaw_angle; 

  double new_yaw_angle = pid(comp_yaw_angle);
    //////// PID Controller ////////
  if (comp_yaw_angle <= 5){
    update_control(new_yaw_angle, old_yaw_angle, dt);
    old_yaw_angle = new_yaw_angle;
  }

  /////////  Printing Portion  //////////
  
  Serial.print("Time_Step: ");
  Serial.print(curr_time/1000, 3);
  Serial.print("\t");
  /*
  //Complimentary Filter
  
  Serial.print(" Roll_Angle (C): ");
  Serial.print(comp_roll_angle,6);
  Serial.print("\t");
  Serial.print(" Pitch_Angle (C): ");
  Serial.print(comp_pitch_angle,6);
  Serial.print("\t");
  Serial.print(" Yaw_Angle (C): ");
  Serial.print(comp_yaw_angle,6); 
  //Serial.print("\t"); 
  Serial.println();
  */
  /*
  Serial.print("Yaw_Angle_Bias: ");
  Serial.println(bias_yaw_rate);
  */
  /*
  //Normal
  Serial.print(" Roll_Angle (N): ");
  Serial.print(roll_angle,6);
  Serial.print("\t");
  Serial.print(" Pitch_Angle (N): ");
  Serial.print(pitch_angle,6);
  Serial.print("\t");
  Serial.print(" Yaw_Angle (N): ");
  Serial.print(yaw_angle,6);
  Serial.print("\t");*/
  /*
  Serial.print("\t");
  Serial.print(" Roll_Rate: ");
  Serial.print(roll_rate,6);
  Serial.print("\t");
  Serial.print(" Pitch_Rate: ");
  Serial.print(pitch_rate,6);
  Serial.print("\t");
  Serial.print(" Yaw_Rate: ");
  Serial.println(yaw_rate,6);
  Serial.print("\t");
  Serial.print(" Temp: ");
  Serial.println(int_temp,6);
  */
  /*
  // Write the value to EEPROM
  writeDoubleToEEPROM(address, comp_yaw_angle);
  
  // Read the value from EEPROM
  double valueRead = readDoubleFromEEPROM(address);
  Serial.println(valueRead); 
 */
  prev_time = curr_time;
}
/*
void writeDoubleToEEPROM(int address, double value) {
  byte* p = (byte*)(void*)&value;
  for (unsigned int i = 0; i < sizeof(value); i++) {
    EEPROM.write(address + i, *p++);
  }
}

double readDoubleFromEEPROM(int address) {
  double value;
  byte* p = (byte*)(void*)&value;
  for (unsigned int i = 0; i < sizeof(value); i++) {
    *p++ = EEPROM.read(address + i);
  }
  return value;
}
*/
void update_control(double new_yaw_angle, double old_yaw_angle, double dt){
  if (comp_yaw_angle > old_yaw_angle) {
    for (double pos = old_yaw_angle; pos <= new_yaw_angle; pos += 1){
      digitalWrite(solenoid_uno, HIGH);
      digitalWrite(solenoid_dos, HIGH);
      digitalWrite(solenoid_tres, LOW);
      digitalWrite(solenoid_quart, LOW);
    }
  }
  if (comp_yaw_angle < old_yaw_angle) {
    for (double pos = old_yaw_angle; pos >= new_yaw_angle; pos -= 1){
      digitalWrite(solenoid_tres, HIGH);
      digitalWrite(solenoid_quart, HIGH);
      digitalWrite(solenoid_uno, LOW);
      digitalWrite(solenoid_dos, LOW);
    }
  }
}



double pid(double comp_yaw_angle) {
  
  double setpoint = 0;
  double error = setpoint-comp_yaw_angle;
  double p_value = error*kP;
  double i_value = cumulative_error*kI;
  double d_value = (error-previous_error)*kD;

  double pid_value = p_value+i_value+d_value;
  /*
  Serial.print("Setpoint: ");
  Serial.print(setpoint);
  Serial.print("\t");
  */
  Serial.print(" Angle: ");
  Serial.print(comp_yaw_angle);
  Serial.print("\t");
  
  Serial.print(" Speed: ");
  Serial.print(roll_rate);
  Serial.print("\t");
  
  Serial.print(" Error: ");
  Serial.print(error);
  Serial.print("\t");

  Serial.print(" pid_value: ");
  Serial.print(pid_value);
  Serial.println();
  
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

/*
void writeToEEPROM(int address, int value) {
  EEPROM.write(address, comp_yaw_angle);
}
*/