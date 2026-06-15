/*AVIONICS ROLL CONTROL*/

/*----------Libraries used----------*/
#include <Arduino.h>               /*Needed for core functions and voids cycles*/
#include <Wire.h>                  /*Needed for I2C communication*/
#include <SD.h>                    /*Needed for the SD logging*/
#include <SPI.h>                   /*Needed for the SD logging*/
#include <Adafruit_Sensor.h>       /*Needed for Adafruit Libraries*/
#include <Adafruit_BMP280.h>       /*Needed for the BMP 280*/
#include <QMC5883L.h>              /*Needed for the magnetometer*/
#include <esp_now.h>               /*Needed for the onboard wifi communication protocol*/
#include <WiFi.h>                  /*Needed for the onboard wifi communication protocol*/
#include <esp_wifi.h>              /*Needed for the onboard wifi communication protocol*/
#include <HardwareSerial.h>        /*Needed for gps communication*/
#include <Servo.h>                 /*Needed for the servo control*/
#include <TinyGPS++.h>             /*Needed for GPS*/

/*----------Voltage Section---------- */
#define BAT_PIN 34                 /*ADC input pin*/
float v_raw = 0.0;                 /*Voltage raw measurement*/
float v_out = 0.0;                 /*Voltage OUT (resistor out)*/
float v_bat = 0.0;                 /*Voltage of the Battery*/

/*----------RF section----------*/
HardwareSerial GPS(1);             /*Use UART1 (Serial1 alternative)*/ 
#define TX_PIN 17                  /*Chooses TX pin for UART communication*/
#define RX_PIN 16                  /*Chooses TX pin for UART communication*/

/*----------Pyro + MPU + BUZZER pins----------*/
#define PY_1 12                    /*Pyro Charge 1 Pin*/
#define PY_2 14                    /*Pyro Charge 2 Pin*/
#define BUZ 32                     /*Buzzer Pin (PWM - DIGITAL)*/
#define MPU_ADDR 0x68              /*MPU address I2C*/
#define BMP_ADDR 0x76              /*BMP address I2C*/
#define MAG_ADDR 0x0D              /*MAG address I2C*/

/*----------Servo Pins and values PWM----------*/
#define S1 26                      /*Servo physical pin (PWM - DIGITAL)*/
#define SERVO_MIN 1000             /*Servo MIN pwm value [us]*/
#define SERVO_MAX 2000             /*Servo MAX pwm value [us]*/
#define SERVO_CENTER 1500          /*Servo reference pwm value (center)*/

/*----------PID controller values, setpoints, K + output----------*/
float r_set = 0.0;                 /*Roll setpoint [°]*/
float error = 0.0;                 /*Pid signed error from setpoint [°]*/
float prevError = 0.0;             /*Pid previous iteration error for derivative calculations [°]*/
float KP = 3.3;                   /*Proportional gain PID controller*/
float KI = 1.32;                  /*Integral gain PID controller*/
float KD = 0.43;                  /*Derivative gain PID controller*/
float integral = 0.0;              /*Integral value [°s]*/
float derivative = 0.0;            /*Derivative value*/
float output = 0.0;                /*Output value from pid*/
int S1_pos_pwm = 0;                /*Actual servo command pwm [us]*/
float S1_pos = 0.0;                /*Real servo command [°]*/
float r_unwrapped = 0.0;           /*Unwrapped roll value [°] (0+-infinity)*/

/*----------SD values and Pin----------*/
const int SD_CS = 5;               /*Chip_Select_Pin for SD (DIGITAL)*/
#define BUFFER_SIZE 512            /*Buffer size in bits for SD printing (execution is faster if buffered)*/
char sdBuffer[BUFFER_SIZE];        /*Actual buffer allocation in character format */
size_t sdIndex = 0;                /*SD index for buffered writing*/

/*----------Objects----------*/
Adafruit_BMP280 bmp;               /*Barometer object*/
QMC5883L mag;                      /*Magnetometer object*/
File data;                         /*Data file object for SD*/
Servo servo;                       /*Servo object for Servo*/
TinyGPSPlus gps;                   /*Object for GPS*/

/*----------Strings----------*/                                  
String cmd_ser;                    /*String for incoming commands via USB (serial) for testing*/
String cmd_wifi;                   /*String for incoming commands via WiFi*/

/*-----------------------------------------------------------------------------------------------------------------------------*/
unsigned long fq_fixed = 20;       /*IMPORTANT !!!! fixed frequency for the code FIRST STARTUP*/

float p0 = 101120;                 /*IMPORTANT !!!! variable to set correct p0 for altitude calculations*/

float num_cal = 500.0;             /*IMPORTANT !!!! variable to set number of iteration for calibrating MPU and BMP*/

float ts_apogee = 8.5;             /*IMPORTANT !!!! variable to set deployment charge delay from launch in s T_apogee + 2s*/

float az_threshold = 20.0;         /*IMPORTANT !!!! variable to set Z acceleration threshold for starting the timer*/

float al_threshold = 4.0;          /*IMPORTANT !!!! variable to set altitude threshold for deploying the parachute safely*/

float min_apogee_threshold = 10.0; /*IMPORTANT !!!! variable to set altitude threshold above which you can deploy parachute*/

bool ser_flag = true;              /*IMPORTANT !!!! variable to select if serial transmission () is ACTIVE or NOT*/ 

bool sd_flag = true;              /*IMPORTANT !!!! variable to select if SD logging is ACTIVE or NOT*/

/*==============================================================================================================================================*/
//kf - LOWER = MORE SMOOTHING
float KF_AL = 0.075;               /*EMA filter coefficient for altitude calculation [0.075] */

float KF_VEL = 0.04;               /*EMA filter coefficient for velocity calculation [0.05] */

float KF_GPS = 0.5;                /*EMA filter coefficient for GPS X Y calculations [0.3]*/

float KF_TP = 0.075;               /*EMA filter coefficient for TEMP calculations [0.075]*/

float KF_ACC = 0.30;               /*EMA filter coefficient for MPU data to feed in the complementary [0.075]*/

float KF_MAG = 0.15;               /*EMA filter coefficient for MAG data to feed in the complementary [0.5]*/

//KF - HIGHER = MORE GYRO
float KF_ATT_XY = 0.5;             /*COMP filter coefficient to fuse gyro and mpu data [0.93]*/     

float KF_ATT_Z = 0.5;              /*COMP filter coefficient to fuse gyro and mag data [0.93]*/

/*-----------------------------------------------------------------------------------------------------------------------------*/

unsigned long dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;    /*Fixed working cycle interval [us]; calculating T = 1/f: 100Hz -> 10 ms = 10000 us; floating point continuously is slower and less deterministic, switching to integer math; rounded_result = (A + B/2) / B*/

bool mpu_fake = true;                                              /*Needed for MPU6050 clones with different I2C chip addresses from 0x68*/

int om = 1;                                                        /*State machine operative mode: 0=low_power ; 1=idle ; 2=armed ; 3=Powered_Ascent ; 4=Coast ; 5=Parachute_Event ; 6=Stable_Descent ; 7=Landed*/
int sd = 0;                                                        /*SD working or not*/
float fq = 0.0;                                                    /*Operating frequency of main loop [Hz]*/

unsigned long time_now = 0, time_last = 0, dt_logged = 0;          /*Current time [us] ; Previous time [us] ; Delta time between cycles [us]*/

float ts = 0.0, ts_now = 0.0, ts_last = 0.0;                       /*timestamp from PWR_ON [s]; never gets reset; can be logged ; current timestamp [s] ; previous timestamp [s]*/
float dt = 0.0;                                                    /*Delta time between cycles [s]*/
float ts_tof = 0.0, ts_tof_0 = 0.0;                                /*Timestamp from LAUNCH [s] ; delta time from PWR_ON to LAUNCH [s]*/

float ax = 0.0, ay = 0.0, az = 0.0;                                /*X Y Z accelerations [m/s^2] */
float gx = 0.0, gy = 0.0, gz = 0.0;                                /*Wx Wy Wz angular velocities [°/sec] */
float tp = 0.0;                                                    /*Temperature [°C] */
float p = 0.0, y = 0.0, r = 0.0;                                   /*Pitch Yaw Roll [°] */
float vz_r = 0.0;                                                  /*Raw Z velocity [m/s] */
float vz = 0.0;                                                    /*Filtered Z velocity [m/s] */

float al_r = 0.0;                                                  /*Raw altitude [m] */
float al = 0.0;                                                    /*Filtered altitude [m] */
float al_last = 0.0;                                               /*Previous iteration altitude (filtered) [m] */
float al_max = 0.0;                                                /*Maximum altitude (filered) [m] */

float vx = 0.0;
float vy = 0.0; 
float x_pos_r = 0.0;
float x_pos = 0.0;
float y_pos_r = 0.0;
float y_pos = 0.0;

float lat = 0.0;
float lon = 0.0;
float heading = 0.0;
float gps_al = 0.0;
int sat = 0;
float gps_speed = 0.0;

/*----------Auxiliary_Variables----------*/
float g = 9.80665;                                                 /*Gravity constant [m/s^2] */
uint32_t packet_id = 0;                                             /*Telemetry packedID*/

int16_t ax_raw = 0, ay_raw = 0, az_raw = 0;                        /*Raw accelerometer data pulled from i2c channel (16bit fixed size integer format)*/

int16_t gx_raw = 0, gy_raw = 0, gz_raw = 0;                        /*Raw gyro data pulled from i2c channel (16bit fixed size integer format)*/
int16_t tp_raw = 0;                                                /*Raw temperature data pulled from i2c channel mpu6050 fixed size integer format*/

float sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0;                    /*Sum variable for initial calibration*/
float sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;                    /*Sum variable for initial calibration*/
float ax_ema = 0.0, ay_ema = 0.0, az_ema = 0.0;

float sum_tp = 0.0;                                                /*Sum variable for initial calibration*/
float ax_avg = 0.0, ay_avg = 0.0, az_avg = 0.0;                    /*Averaged value for initial calibration*/
float gx_avg = 0.0, gy_avg = 0.0, gz_avg = 0.0;                    /*Averaged value for initial calibration*/
float tp_avg = 0.0;                                                /*Averaged value for initial calibration*/
float a_scale = 0.0;                                               /*Acceleration scaling factor (supposed same for every axis)*/
float gx_bias = 0.0, gy_bias = 0.0, gz_bias = 0.0;                 /*Bias gyro factor (axis dependent)*/

float sum_al = 0.0;                                                /*Sum variable for initial calibration*/
float al_avg = 0.0;                                                /*Averaged value for initial calibration*/
float al_bias = 0.0;                                               /*Bias barometer factor*/

int16_t mag_x_raw = 0, mag_y_raw = 0, mag_z_raw = 0;               /*Raw magnetometer data pulled from i2c channel (16bit fixed size integer)*/
float mag_x = 0.0, mag_y = 0.0, mag_z = 0.0;
float mag_x_ema = 0.0, mag_y_ema = 0.0, mag_z_ema = 0.0;
float sum_mag_x = 0.0, sum_mag_y = 0.0, sum_mag_z = 0.0;
float avg_mag_x = 0.0, avg_mag_y = 0.0, avg_mag_z = 0.0;
float m_scale = 0.0;
float mag_bias = 0.0;
float magRoll_raw = 0.0; //pre remapping
float magRoll_prev = 0.0;
float magRoll_offset = 0.0;
float delta = 0.0;
float magRoll_true = 0.0;
float magRoll_mapped = 0.0;
float cal_mag_z_comp = 0.0, cal_mag_x_comp = 0.0;
float mag_x_comp = 0.0, mag_y_comp = 0.0, mag_z_comp = 0.0;

float pitch_rad = 0.0, yaw_rad = 0.0, roll_rad = 0.0;
float cal_pitch = 0.0, cal_yaw = 0.0;

float gyroPitch = 0.0;
float gyroYaw = 0.0;
float gyroRoll = 0.0;
float accelPitch = 0.0, accelYaw = 0.0, magRoll = 0.0;             /*First input of the Complementary Filter (pitch and yaw computed from accelerometer ; roll computed from magnetometer)*/

float pitchErrorCov = 1.0, yawErrorCov = 1.0, rollErrorCov = 1.0;  /*Error covariance value for Simplified Kalman Filter*/              
float processNoise = 0.001, measurementNoise = 0.03;               /*Noise of the process value for Simplified Kalman Filter*/
float processNoise_mag = 0.0005, measurementNoise_mag = 0.04;
float Kp = 0.0, Ky = 0.0, Kr = 0.0;                                /*Kalman filter coefficients for sensor fusion (exponenatial moving average filter)*/

uint8_t gsAddress[] = {0xF8, 0xB3, 0xB7, 0x49, 0x44, 0x14};        /*Ground station MAC address !!! do not change*/

const double MAX_DEFLECTION_RAD = 0.698;                                     /*40 deg in rad, max tab deflection for aero limits*/


double servo_scale = (double)(SERVO_MAX - SERVO_CENTER) / MAX_DEFLECTION_RAD;         /*Calculate actual servo scaling factor*/

bool buzzerActive = false;
uint8_t currentBeep = 0;
uint8_t totalBeeps = 0;
bool buzzerState = false; // HIGH or LOW
unsigned long lastChange = 0;
uint16_t onTime = 0;
uint16_t offTime = 0;

bool pyroSequenceActive = false;
unsigned long pyroDelayTimer = 0;

/*----------Send_Data_to_GS----------*/
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {} /*Callback function used by ESP-NOW, right now does nothing, can be used for debugging*/

/*----------Receive_Data_from_GS----------*/
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) { /*Callback function used by ESP-NOW, reads data and trims string to match size of command*/
    cmd_wifi = String((const char*)incomingData, len);                           /*Construct string with explicit length*/
    cmd_wifi.trim();                                                             /*Remove leading/trailing whitespace/newlines*/
}


/*----------I2C_Reset----------*/
void I2C_Reset() {
  pinMode(22, INPUT_PULLUP);          /*SDA pin set as input pullup for i2c communication*/
  pinMode(21, INPUT_PULLUP);          /*SCL pin set as input pullup for i2c communication*/

  for (int i = 0; i < 9; i++) {       /*Clock SCL manually to release any stuck slaves*/
    pinMode(21, OUTPUT);              /*Set SCL to digital output*/
    digitalWrite(21, LOW);            /*Set SCL to LOW*/
    delayMicroseconds(5);             /*Blocking delay (fine in the setup) to ensure correct release*/
    digitalWrite(21, HIGH);           /*Set SCL to HIGH*/
    delayMicroseconds(5);             /*Blocking delay (fine in the setup) to ensure correct release*/
  }

  pinMode(22, OUTPUT);                /*Set SDA to digital output*/
  digitalWrite(22, LOW);              /*Set SDA to LOW*/
  delayMicroseconds(5);               /*Blocking delay (fine in the setup) to ensure correct release*/
  digitalWrite(21, HIGH);             /*Set SCL to HIGH*/
  delayMicroseconds(5);               /*Blocking delay (fine in the setup) to ensure correct release*/
  digitalWrite(22, HIGH);             /*Set SDA to HIGH - this finishes the STOP condition to reset the I2C bus*/
}

/*----------Task_Declaration----------*/
void core0Task(void *pvParameters); // forward declaration


/*=============================================================================================================================================================*/

/*----------Robust_Startup----------*/
void setup() {
  GPS.begin(9600, SERIAL_8N1, 16, 17);                        /*Begin telemetry transmission on TX and RX pins at 9600 baud rate*/
  Serial.flush();                                              /*Flushes any unwanted serial data*/
  delay(10);                                                   /*Blocking delay (fine in the setup) to ensure correct flush*/
  Serial.println("STARTUP_SEQUENCE...");                       /*After initialization print the startup sequence begin on the monitor*/
  delay(10);                                                  /*Blocking delay (fine in the setup) to ensure correct startup*/   

  digitalWrite(BUZ, HIGH); delay(50); digitalWrite(BUZ, LOW); /*Sound confirmation for correct Startup and initializing Setup and Calibration*/

  I2C_Reset();                                                 /*Restart (recover) I2C bus first*/
  delay(10);                                                   /*Blocking delay (fine in the setup) to ensure correct reset*/ 
  Wire.begin(21, 22);                                          /*Starts I2C communication*/
  delay(10);                                                   /*Blocking delay (fine in the setup) to ensure correct BUS startup*/
  Wire.setClock(400000);                                       /*I2C clock speed set as highest for maximum performance*/
  delay(10);                                                   /*Blocking delay (fine in the setup) to ensure correct startup and stabilization before next part*/

  /*----------I2C_Check----------*/
  byte deviceCount = 0;                                        /*Variable to count # of I2C devices*/
  Serial.begin(256000);                                        /*Begin serial communication at 256000 baud rate*/
  Serial.println("STARTUP_SEQUENCE...");                       /*After initialization print the startup sequence begin on the monitor*/
  for (uint8_t addr = 1; addr < 127; addr++) {                 /*Checks every I2C address for sensors*/
    Wire.beginTransmission(addr);                              /*Initialize I2C transmission to the specific address*/
    if (Wire.endTransmission() == 0) {                         /*If transmission is not ended*/
      Serial.print("Found device at 0x");                      /*Print the I2C memory address of the device as hexadecimal*/
      if (addr < 16) Serial.print("0");                         
      Serial.println(addr, HEX);                             
      deviceCount++;                                           /*Continue onto the next addresses*/
      delay(5);                                                /*Wait 5ms to ensure more robust address counting (blocking delay is fine in the setup)*/
    }
  }
  if (deviceCount != 3) {                                          /*Blocking condition if a number of sensors != 3 even if there is mpu and barometer (essential sensors)*/
    Serial.println("NOT ENOUGH SENSORS FOUND!");                   /*Print error message*/
    delay(200);                                                    /*Wait 200ms (blocking delay is fine in the startup)*/
    digitalWrite(BUZ, HIGH); delay(1000); digitalWrite(BUZ, LOW);  /*Buzzer tone to indicate wrong number of sensors found*/
    while(true) delay(1000);                                       /*Stops activity and prevents accidental startup*/

  } else {
    Serial.print("TOTAL SENSORS FOUND: ");                                      /*Prints the number of devices found for debugging*/
    Serial.println(deviceCount); 
    for (int i = 0; i < deviceCount; i++) {                                     /*For clause that repeats for each connected sensor*/
       digitalWrite(BUZ, HIGH); delay(25); digitalWrite(BUZ, LOW); delay(50);   /*Buzzer tone to indicate correct number of sensor found (each beep represents a sensor)*/
    }
    delay(10);                                                                 /*Wait 200ms to ensure more robust startup (blocking delay is fine in the setup)*/                         
  }

  /*----------Initialize_MPU_6050 and calibrate gyro + bias----------*/
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6B); Wire.write(0x00); Wire.endTransmission();  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1A); Wire.write(0x03); Wire.endTransmission();  // DLPF config
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x1C); Wire.write(0x18); Wire.endTransmission();  // Accel ±16g
  Wire.beginTransmission(MPU_ADDR); Wire.write(0x6C); Wire.write(0x00); Wire.endTransmission();  // Enable all axes
  
  for (int i = 0; i < num_cal; i++) {             /*Sensor calibration until i=numcal*/
    Wire.beginTransmission(MPU_ADDR);             /*Begin transmission on the correct address for the MPU*/
    Wire.write(0x41); 
    Wire.endTransmission(false);                  /*Continue transmitting*/
    Wire.requestFrom(MPU_ADDR, 8, true);         /*Request data*/

    tp_raw = Wire.read() << 8 | Wire.read();
    gx_raw = Wire.read() << 8 | Wire.read();
    gy_raw = Wire.read() << 8 | Wire.read();
    gz_raw = Wire.read() << 8 | Wire.read();

    sum_gx += gx_raw/131.0f; sum_gy += gy_raw/131.0f;  sum_gz += gz_raw/131.0f;   /*Start summing scaled data to calibrate gyro*/
    sum_tp += tp_raw;                                                             /*Start summing data to calibrate thermometer*/
    delay(10);                                                                    /*Wait 10ms to ensure no duplicated data is pulled*/
  }

  gx_bias = sum_gx / num_cal;   
  gy_bias = sum_gy / num_cal;  
  gz_bias = sum_gz / num_cal;   /*Calculate average for each gyro axis*/

  tp_avg = (float)sum_tp / num_cal;                                               /*Calculate average temperature*/
  delay(10);                                                                      /*Wait 50ms to ensure correct startup and for debugging purposes*/
  Serial.println("MPU_6050_ON...");                                               /*Send confirmation of the calibration*/

  /*----------Initialize_BMP_280 and calibrate altitude bias at startup----------*/
  if (bmp.begin(BMP_ADDR)) {                      /*Start I2C communication at correct address*/
    bmp.setSampling(                              /*Set sampling info*/
    Adafruit_BMP280::MODE_NORMAL,                 /*Normal Mode = Continuos periodic measurements*/
    Adafruit_BMP280::SAMPLING_X1,                 /*Lowest oversampling setting*/
    Adafruit_BMP280::SAMPLING_X1,                 /*Lowest oversampling setting*/
    Adafruit_BMP280::FILTER_OFF);                 /*Remove internal filtering to apply custom filter for better sensor precision-responsiveness combination*/
    for (int i = 0; i < num_cal; i++) {           /*Sensor calibration until i=numcal*/
      sum_al += (1.0f - pow(bmp.readPressure() / p0, 0.1903f)) * 44330.0f;     /*Pressure is read in Pa, summing altitude for calibration*/
      delay(10);                   /*Wait 10ms to ensure no duplicated data is pulled*/
    }
    al_bias = (float)(sum_al / num_cal);                 /*Calculated altitude bias*/
    delay(10);                                    /*Wait 200ms to ensure correct startup and for debugging purposes*/
    Serial.println("BMP_280_ON...");              /*Send confirmation of the calibration*/
  }

  /*----------Initialize_MAG and calibrate heading bias at startup (true North vs Roll = 0°)----------*/
  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x0B); // reset register
  Wire.write(0x01);
  Wire.endTransmission();

  Wire.beginTransmission(MAG_ADDR);
  Wire.write(0x09); // control register
  Wire.write(0x1D); // continuous mode, 200Hz, 8G range, 512 OSR
  Wire.endTransmission();
  Serial.println("MAG_QMC5883L_ON...");        /*Send confirmation of the calibration*/

  /*----------Initialize_GPS----------*/
  Serial.println("UBLOX_6M_ON...");

  /*----------Initialize_MOSFETS_&_BUZZER----------*/
  pinMode(PY_1, OUTPUT);          /*Set Pyro pin to output*/
  pinMode(PY_2, OUTPUT);          /*Set Pyro pin to output*/
  pinMode(BUZ, OUTPUT);           /*Set buzzer pin to output*/
  Serial.println("PYRO_ON...");   /*Send confirmation of the calibration*/
  Serial.println("BUZZER_ON..."); /*Send confirmation of the calibration*/

  /*----------Initialize_SD----------*/
  if (sd_flag == true) {
    if (SD.begin(SD_CS, SPI, 40000000)){    /*Set SPI clock speed to maximum and checks if SD is mounted correctly*/                                    
      Serial.println("SD_MOUNT_ON...");     /*Send confirmation*/
      sd = 1;                               /*SD parameter for debugging*/
    } else {
      Serial.println("SD_ERR...");          /*Send error message */
      sd = 0;                               /*Do not block the launch, SD is non flight critical, just flag it to false*/
    }
  }
 
  if (sd_flag == true && sd == 1){                                   /*if the flag is true and SD is truly mounted continue with SD initialization*/
    data = SD.open("/data.txt", FILE_WRITE);                         /*Open data.txt file and go in write mode*/
    if (data) {                                                      /*If the command is successful then write*/
      data.println("om,fq,ts,ax,ay,az,gx,gy,gz,p,y,r,al,vz,S1_pos,tp,x_pos,y_pos,sat,v_bat,hdg"); /*Print header in the txt file*/
      Serial.println("FILE_ON...");                                  /*Send Serial confirmation of SD activation*/
      sd = 1;                                                        /*Set SD variable to ON*/
    } else {
      Serial.println("FILE_ERR...");                                 /*Send error message for debugging*/
      sd = 0;
    }
  }

  /*----------Initialize_WiFi----------*/
  WiFi.mode(WIFI_STA);                               /*Set Wifi mode to wifi station*/
  delay(200);                                        /*Give WiFi stack time to initialize properly !!! very important*/

  esp_wifi_set_promiscuous(true);                    /*Set promiscuous mode to ensure correct channel change*/
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);    /*Channel set as SAME CHANNEL as GS*/
  esp_wifi_set_promiscuous(false);                   /*Set promiscuous mode false to ensure no additional channel change is made*/
  delay(200);                                        /*Stabilization time before reading mac address !!! very important*/

  if (esp_now_init() != ESP_OK) {                    /*If Initialization is not OK*/
    Serial.println("WIFI_ROCKET_ERR...");            /*Error sent if initialization of espnow is not possible*/
    while(true) delay(1000);                         /*Stop setup here -> Wifi telemetry is flight critical for arming the rocket at a safe distance*/
  }

  Serial.print("Rocket MAC: "); Serial.println(WiFi.macAddress());     /*Rocket MAC address is printed for debug and configuration purposes*/
   
  esp_now_peer_info_t peerInfo = {};                                   /*Create peerInfo array*/
  memcpy(peerInfo.peer_addr, gsAddress, 6);                            /*6 bytes MAC address*/
  peerInfo.channel = 1;                                                /*ame as GS channel*/
  peerInfo.encrypt = false;                                            /*Do not encrypt*/

  esp_now_register_send_cb(onDataSent);       /*???*/
  esp_now_register_recv_cb(onDataRecv);       /*???*/

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {                         /*If peer is not added send GS error to communicate link error*/
    Serial.println("WIFI_CONNECTION_GS_ERR...");                       /*Send error via serial for debugging purposes*/
    while(true) delay(1000);                                           /*Stop setup here -> Wifi telemetry is flight critical for arming the rocket at a safe distance*/ 
  } 
  else {
     Serial.println("WIFI_ON...");                                     /*If link is active send confirmation via serial*/
  }

  /*----------Headers----------*/
  if(ser_flag == true) {                                            /*Send headers via serial*/  
    Serial.print("om"); Serial.print("\t");                                           
    Serial.print("fq"); Serial.print("\t");   
    Serial.print("sd"); Serial.print("\t");                      
    Serial.print("ts "); Serial.print("\t");                      
    Serial.print("ax"); Serial.print("\t");                         
    Serial.print("ay"); Serial.print("\t");                         
    Serial.print("az"); Serial.print("\t");                         
    Serial.print("gx"); Serial.print("\t");                         
    Serial.print("gy"); Serial.print("\t");                         
    Serial.print("gz"); Serial.print("\t");                                                
    Serial.print("p"); Serial.print("\t");                         
    Serial.print("y"); Serial.print("\t");                         
    Serial.print("r"); Serial.print("\t");                                                                       
    Serial.print("al"); Serial.print("\t");
    Serial.print("vz"); Serial.print("\t");
    Serial.print("S1_pos"); Serial.print("\t");
    Serial.print("tp"); Serial.print("\t");
    Serial.print("x_pos   "); Serial.print("\t");
    Serial.print("y_pos   "); Serial.print("\t");
    Serial.print("sat"); Serial.print("\t");
    Serial.print("v_bat"); Serial.print("\t");
    Serial.print("hdg"); Serial.println("\t");
  }
  
  /*----------Send_Headers_via_WiFi----------*/
  // not done for GS purposes (easier decoding script)

  /*----------Initialize_and_test_Servos----------*/
  servo.attach(S1);                                           /*Attach servo to Pin*/

  digitalWrite(BUZ, HIGH); delay(200); digitalWrite(BUZ, LOW);  /*Send buzzer confirmation of correct startup of servo*/
  delay(200);
  digitalWrite(BUZ, HIGH); delay(200); digitalWrite(BUZ, LOW);

  // NOW start the task - SD already initialized and ready
  xTaskCreatePinnedToCore(
    core0Task,
    "SD_WRITE",
    4096,
    NULL,
    1,
    NULL,
    0
  );
}

/*===================================================================================================================================================================*/
typedef struct __attribute__((packed)) {     /*Struct to store different types of data together ; packing removes padding bytes*/
    uint32_t packet_id;
    uint8_t om;
    float fq;
    uint8_t sd;
    float ts;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float p;
    float y;
    float r;
    float al;
    float vz;
    float S1_pos;
    float tp;
    float x_pos;
    float y_pos;
    uint8_t sat;
    float v_bat;
    float heading;
} TelemetryPacket; /*This closes the struct and creates the type name*/

/*----------Struct for BEEPER----------*/
struct BeepPattern {
  uint8_t beeps;          // number of beeps
  uint16_t onTime;        // ms HIGH
  uint16_t offTime;       // ms between beeps
};
BeepPattern SINGLE = {1, 150, 200};
BeepPattern DOUBLE = {2, 150, 200};
BeepPattern TRIPLE = {3, 150, 200};

/*----------Struct for PYRO----------*/
struct PyroChannel {
  bool active = false;
  bool fired = false;
  unsigned long startTime = 0;
  uint16_t duration = 1000;       // pyro duration is 1000ms
};
PyroChannel PY1;
PyroChannel PY2;

/*----------BEEPING START FUNCTION----------*/
void startBeep(BeepPattern pattern) {
  buzzerActive = true;

  currentBeep = 0;
  totalBeeps = pattern.beeps;

  onTime = pattern.onTime;
  offTime = pattern.offTime;

  buzzerState = false;
  lastChange = millis();
}

/*----------BEEPING FUNCTION----------*/
void updateBuzzer() {
  if (!buzzerActive) return;
  unsigned long now = millis();

  if (!buzzerState) {
    // LOW state handling (pause between beeps)
    if (now - lastChange >= offTime) {

      if (currentBeep >= totalBeeps) {
        buzzerActive = false;
        digitalWrite(BUZ, LOW);
        return;
      }

      digitalWrite(BUZ, HIGH);
      buzzerState = true;
      lastChange = now;
    }
  }
  else {
    // HIGH state handling
    if (now - lastChange >= onTime) {
      digitalWrite(BUZ, LOW);
      buzzerState = false;
      currentBeep++;
      lastChange = now;
    }
  }
}

/*----------FIRE----------*/
void firePY1() {
  if (PY1.fired) return;           // prevent double-firing
  PY1.fired = true;
  PY1.active = true;
  PY1.startTime = millis();
  digitalWrite(PY_1, HIGH);
  Serial.println("<PYRO_1> ON!");

  pyroSequenceActive = true;
  pyroDelayTimer = millis();
}

void firePY2() {
  if (PY2.fired) return;           // prevent double-firing
  PY2.fired = true;
  PY2.active = true;
  PY2.startTime = millis();
  digitalWrite(PY_2, HIGH);
  Serial.println("<PYRO_2> ON!");
}

void updatePyros() {
  unsigned long now = millis();

  // PY1 shutdown
  if (PY1.active && (now - PY1.startTime >= PY1.duration)) {
    digitalWrite(PY_1, LOW);
    Serial.println("<PYRO_1> OFF...");
    PY1.active = false;
  }

  // PY2 starts 1s after PY1 ignition
  if (pyroSequenceActive && !PY2.fired && (now - pyroDelayTimer >= 1000)) {
    firePY2();
    pyroSequenceActive = false;
  }

  // PY2 shutdown
  if (PY2.active && (now - PY2.startTime >= PY2.duration)) {
    digitalWrite(PY_2, LOW);
    Serial.println("<PYRO_2> OFF...");
    PY2.active = false;
  }
}

/*=========================CORE 1 - CONTROL LOOP (loop() always runs on Core 1)===============================*/
void loop() {
  /*----------TIME_LOG_SECTION----------*/
  time_now = micros();                       /*Log time at beginning of loop*/

  while (GPS.available()) {                  /*Always read GPS*/
    gps.encode(GPS.read());
  }

  if (time_now - time_last >= dt_fixed) {    /*Loop at preset frequency = calculated before*/
    dt_logged = time_now - time_last;        /*Effective dt logged in us , could differ from fixed dt because of non deterministic behaviour*/
    dt = float(dt_logged) / 1e6;             /*Dt converted from int to float, then from us to s*/
    fq = 1 / dt;                             /*Frequency calculated from dt_logged = true working frequency*/
    ts += dt;                                /*Time stamp updated*/

    /*----------Voltage_data----------*/
    v_raw = analogRead(BAT_PIN);                          /*Esp32 ADC reading*/
    v_out = (v_raw / 4095.0f) * 3.3f * (133.0f / 33.0f);  /*ESP32 ADC scaling (R1+R2)/R2*/
    v_bat = v_bat + 0.1 * (v_out - v_bat);                /*EMA filtering to reduce noise*/

    /*Keep servo at 0° - when control active this gets overridden*/
    if (om == 0 || om == 1) {
      r_set = 0.0;
      S1_pos_pwm = constrain(SERVO_CENTER + (int)(output * servo_scale), SERVO_MIN, SERVO_MAX); /*Convert to servo command*/
      servo.writeMicroseconds(S1_pos_pwm);                                                      /*Write the position = move the servo*/
      S1_pos = float(S1_pos_pwm - SERVO_CENTER) / servo_scale;
    }
    

    /*----------MPU_6050_data----------*/
    Wire.beginTransmission(MPU_ADDR);   /*Same code as the setup, pull data directly from the MPU memory address via I2C*/
    Wire.write(0x3B);  
    Wire.endTransmission(false); 
    Wire.requestFrom(MPU_ADDR, 14, true);

    if (Wire.available() == 14) {
      ax_raw = Wire.read() << 8 | Wire.read();
      ay_raw = Wire.read() << 8 | Wire.read();
      az_raw = Wire.read() << 8 | Wire.read();
      tp_raw = Wire.read() << 8 | Wire.read();
      gx_raw = Wire.read() << 8 | Wire.read();
      gy_raw = Wire.read() << 8 | Wire.read();
      gz_raw = Wire.read() << 8 | Wire.read();
    }  
    /*---ACC---*/
    //Rocket X = - Sensor Z
    //Rocket Y = - Sensor Y
    //Rocket Z = - Sensor X

    // Step 1: Conversion from RAW to FLOAT + MAPPING 
    ax = -az_raw / 2048.0f; 
    ay = -ay_raw / 2048.0f; 
    az = -ax_raw / 2048.0f; 
    
    // Step 2: rescaling axes - individual + and - (assumed no bias during scaling)
    //X
    if (ax > 0) {
      ax = ax * g * 0.99407; // X+ scaling DO NOT TOUCH
    }
    if (ax < 0) {
      ax = ax * g * 0.96851; // X- scaling DO NOT TOUCH
    }

    //Y
    if (ay > 0) {
      ay = ay * g * 1.0031; // Y+ scaling DO NOT TOUCH
    }
     if (ay < 0) {
       ay = ay * g * 0.99407; // Y- scaling scaling DO NOT TOUCH
    }

    //Z
    if (az > 0) {
      az = az * g * 1.0384; // Z+ scaling DO NOT TOUCH
    }
    if (az < 0) {
      az = az * g * 0.94619; // Z- scaling DO NOT TOUCH
    }

    ax_ema = ax_ema + KF_ACC * (ax - ax_ema); // exponential moving average before kalman filtering
    ay_ema = ay_ema + KF_ACC * (ay - ay_ema);
    az_ema = az_ema + KF_ACC * (az - az_ema);
      
    /*---GYRO---*/
    //Rocket X = - Sensor Y
    //Rocket Y = Sensor Z
    //Rocket Z = - Sensor X

    gx = -(gy_raw / 131.0 - gy_bias); 
    gy =  (gz_raw / 131.0 - gz_bias); 
    gz = -(gx_raw / 131.0 - gx_bias);    
    
    /*---TEMP---*/
    tp = tp + KF_TP * ((tp_raw/ 340.0f + 36.53f) - tp);    /*EMA filter for temperature + conversion from raw units*/

    /*----------BMP_280_data----------*/
    al_r = (1.0f - pow(bmp.readPressure() / p0, 0.1903f)) * 44330.0f - al_bias;   /*Raw altitude is computed with the power law (!!! p0 needs to be set accordingly) and bias is removed */
    al = al + KF_AL * (al_r - al);                                                /*Filtered altitude is computed with the exponential filter using the coefficient set previously*/
    vz_r = (al - al_last) / dt;                                                   /*Raw Z velocity is computed as a simple numeric derivation using two successive altitude iterations*/
    vz = vz + KF_VEL * (vz_r - vz);                                               /*Filtered Z velocity is computed with the exponental filter using the coefficient set previously*/

    /*----------MAG_data----------*/
    Wire.beginTransmission(MAG_ADDR);
    Wire.write(0x00); 
    Wire.endTransmission();
    Wire.requestFrom(MAG_ADDR, 6);

    if (Wire.available() == 6) {
      mag_x_raw = (Wire.read() | Wire.read() << 8);
      mag_y_raw = (Wire.read() | Wire.read() << 8);
      mag_z_raw = (Wire.read() | Wire.read() << 8);

      mag_x = mag_x_raw + 151.0f;
      mag_y = mag_y_raw;
      mag_z = (mag_z_raw + 172.0f) * 1.068282f;

      heading = -atan2(mag_z, mag_x) * 180.0f / M_PI;
    }
      
    /*----------ATTITUDE_CALCULATION_SECTION----------*/
    /*---ROCKET---(made FACING THE ROCKET AVIONICS ESP32)*/
    // VERSION 1.2 06/06/2026

    // Rocket X → downrange
    // Rocket Y → left
    // Rocket Z → up (toward nosecone)

    /*----------ATTITUDE_CALCULATION_SECTION----------*/
    accelPitch = atan2(ay_ema, sqrt(ax_ema * ax_ema + az_ema * az_ema)) * 180.0 / PI; /*Kalman Pitch input 1 -> Acc Trigonometry rescaled to degrees */
    p += gx * dt;                                                                      /*Kalman Pitch input 2 -> Gyro Integration calculated on TRUE dt between iterations */
    pitchErrorCov += processNoise;                                                     /*Covariance is computed from the process noise*/
    Kp = pitchErrorCov / (pitchErrorCov + measurementNoise);                           /*Kalman constant Pitch calculation computed from error covariance and measurement noise*/
    p += Kp * (accelPitch - p);                                                        /*Pitch computed via recursion from accelerometer and guro data*/
    pitchErrorCov *= (1 - Kp);                                                         /*Covariance multiplied updated*/
    
    accelYaw = atan2(-ax_ema, sqrt(ay_ema * ay_ema + az_ema * az_ema)) * 180.0 / PI;   /*Kalman Yaw -> Acc Trigonometry*/
    y += gy * dt;                                                                      /*Kalman Yaw -> Gyro Integration*/
    yawErrorCov += processNoise;                                                       /*Covariance is computed from the process noise*/
    Ky = yawErrorCov / (yawErrorCov + measurementNoise);                               /*Kalman constant Yaw calculation*/
    y += Ky * (accelYaw - y);                                                          /*Yaw computed via recursion from accelerometer and gyro data*/
    yawErrorCov *= (1 - Ky);                                                           /*Covariance multiplied updated*/   
       
    r += gz * dt;
    r_unwrapped += gz * dt;

    if (r > 180.0f)  r -= 360.0f;
    if (r < -180.0f) r += 360.0f;
                          
    /*----------Position_Calculation----------*/                   
    lat = gps.location.lat();
    lon = gps.location.lng();
    sat = gps.satellites.value();
      
    x_pos = lon;
    y_pos = lat;
    /*==========ALGORITHMS_SECTION==========*/


    /*__________Launch_Detection_Algorithm__________________________________________________________________________________________________________________________*/
    if (abs(az) > az_threshold && om == 2) {                                      /*20 m/s^2 acceleration to initialize timer & only valid if operative mode is 1=armed*/
      om = 3;                                                                    /*Change operative mode of state machine (timer is started)*/

      Serial.println("LAUNCH_DETECTED...");                                      /*Print confirmation to serial for debug purposes*/        
      ts_tof_0 = ts;                                                             /*Timestamp time of flight 0 is the true t=0 at liftoff*/
    }
    ts_tof = ts - ts_tof_0;                                                      /*Time stamp for time of flight is updated*/
    /*______________________________________________________________________________________________________________________________________________________________*/



    /*__________Burnout_Detection_Algorithm_________________________________________________________________________________________________________________________*/
    if (az < 0 && om == 3){                                               /*negative acceleration to check for burnout & only valid if operative mode is 3=powered_ascent*/
      om = 4;

      Serial.println("BURNOUT_DETECTED...");                              /*Print confirmation to serial for debug purposes*/ 
    }
    /*______________________________________________________________________________________________________________________________________________________________*/



    /*__________Apogee_Detection_Algorithm__________________________________________________________________________________________________________________________*/
    if (al > al_max) {                          /*al = main altitude variable, costantly changing, threshold if altitude > max altitude*/
      al_max = al;                              /*if altitude > max altitude set new max altitude to current altitude*/
    }
    /*______________________________________________________________________________________________________________________________________________________________*/



    /*__________Parachute_Deployment_Algorithm______________________________________________________________________________________________________________________*/
    if (al_max - al > al_threshold && (om == 2 || om == 3 || om == 4) && al > min_apogee_threshold){          /*If signed difference between maximum altitude and current altitude is greater than the threshold and operative mode is either 2 or 3 or 4 and the current altitude is above a certain threshold (removes pad noise accidental firing)*/
      om = 5;                                                                                                 /*Set operative mode to 5=parachute*/

      servo.writeMicroseconds(SERVO_CENTER);                                                                  /*Center servo*/
      Serial.println("<PYROS> DEPLOYED BY BAROMETER...");                                                    /*Serial log for testing*/

      firePY1();                                                         /*Fires FIRST deployment charge ; Keep mosfet on 1s for full burn ; Deactivates mosfet after firing*/
      firePY2();                                                         /*Fires SECOND deployment charge ; Keep mosfet on 1s for full burn ; Deactivates mosfet after firing*/                                                                              
    }

    if (ts_tof > ts_apogee &&  (om == 3 || om == 4)){                     /*Having the operative mode 2 in the if prevents a double firing (unnecessary), if any of the methods fail the backup method will sense that om is still == 3 or 4 and proceed to fire the charge*/
      om = 5;                                                             /*Change operative mode to parachute*/

      servo.writeMicroseconds(SERVO_CENTER);                              /*Center servo*/
      Serial.println("<PYROS> DEPLOYED BY TIMER...");                    
                           
      firePY1();                                                          /*Fires FIRST deployment charge ; Keep mosfet on 1s for full burn ; Deactivates mosfet after */
      firePY2();                                                          /*Fires SECOND deployment charge ; Keep mosfet on 1s for full burn ; Deactivates mosfet after firing*/   
    }
    /*______________________________________________________________________________________________________________________________________________________________*/



    /*__________Stable_Descent_Detection_Algorithm_______________________________________________________________________________________________________________________*/
    if (om == 5 && abs(vz) < 12) {  /*If parachute has fired and descent velocity is less than 12 m/s threshold*/
      om = 6;

      fq_fixed = 20; dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;       /*Update working frequency -> less data file */
      Serial.println("STABLE_DESCENT...");                                   /*Serial log for testing*/  
    }
    /*______________________________________________________________________________________________________________________________________________________________*/

    /*__________Touchdown_Detection_Algorithm_______________________________________________________________________________________________________________________*/

    if ((om == 5 || om == 6) && ((gx+gy+gz)<9)) {            /*If the parachute has fired and angular rate is less than 9 degrees per second !!! not valid for water touchdown*/
      om = 7;                                                /*Set operative mode to landed*/

      fq_fixed = 5; dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;   /*Update working frequency -> less data file */
      Serial.println("TOUCHDOWN...");                                   /*Serial log for testing*/                   
    }
    /*______________________________________________________________________________________________________________________________________________________________*/


    /*==========ACTIVE_CONTROL_SECTION==============================================================================================================================*/
    //    if ((om == 3 || om == 4 || om == 5 || om == 6) && (ts_tof - ts_tof_0 < 24.0)) {                  /*Only active if liftoff is detected and for a limited time period*/

    if (om == 2) {                   // test /*Only active if liftoff is detected and for a limited time period*/
      r_set = 0;                                                                                     /*First setpoint active in the beginning and after last setpoint*/                                                 
      if (ts_tof > 0.0 && ts_tof <= 4) {                                                             /*Setpoint scheduler roll program +90° -90°*/
        r_set = 0.0;
      } 
      else if (ts_tof > 4.0 && ts_tof <= 10.0) {
        r_set = 90.0*PI/180 * (ts_tof - 4.0) / 6.0;   /*1.5s costant slope rise to +90 degrees*/
      } 
      else if (ts_tof > 10.0 && ts_tof <= 12.0) {
        r_set = 90.0*PI/180;                          /*Fixed setpoint at 90 degrees*/
      } 
      else if (ts_tof > 12.0 && ts_tof <= 18.0) {
        r_set = 90.0*PI/180 - (90.0*PI/180) * (ts_tof - 12.0) / 6.0; /*1.5s constant slope descent to +0 degrees*/
      } 
      else if (ts_tof > 18.0 && ts_tof <= 24.0) { /*safety mode 2 , OFF at t+24s*/
        r_set = 0.0;
      }

      error = - (r_set - r_unwrapped)*PI/180;                                                     /*actual error calculation in radians (needed to import correct gains from simulink)*/
      derivative = 0.368 * derivative + 0.632 * (gz);                               /*Simple finite difference derivative + Low pass filter for the derivative tau = 1/N ; alpha * d + (1-alpha) d(d)/dt ; alpha = e^-Ndt (dt=100Hz=0.01s)*/
      prevError = error;                                                            /*Recursive statement for derivative*/

      output = KP * error + KI * integral + KD * derivative;                        /*Output calculation, uses true integral from previous calculation*/

      
      bool saturated = (output >= MAX_DEFLECTION_RAD && error > 0) || (output <= -MAX_DEFLECTION_RAD && error < 0); /*Saturated condition check with directionality (if error and u unsat have different signs this means the controller is trying to push in the correct direction -> apply only integral contraint)*/
      if (!saturated) {
        integral += error * dt;                                  /*Anti windup integral*/
        integral = constrain(integral, -1500, 1500);             /*Constrained value always*/
      }

      output = constrain(output, -MAX_DEFLECTION_RAD, MAX_DEFLECTION_RAD);                  /*Saturate output to physical tab limits*/

      S1_pos_pwm = constrain(SERVO_CENTER + (int)(output * servo_scale), SERVO_MIN, SERVO_MAX); /*Convert to servo command*/
      servo.writeMicroseconds(S1_pos_pwm);                                                      /*Write the position = move the servo*/
      S1_pos = float(S1_pos_pwm - SERVO_CENTER) / servo_scale;
    }

    /*==========Serial_Printing====================================================================================================================================*/
    if (ser_flag == true) {
      Serial.print(om); Serial.print("\t");                                 // 1 Op Mode for state machine status
      Serial.print(fq); Serial.print("\t");                                 // 2 Frequency Hz for correct loop speed check
      Serial.print(sd); Serial.print("\t");                                 // 3 Sd Mode for logging check (not needed in SD logging file)
      Serial.print(ts); Serial.print("\t");                                 // 4 Ts 
      Serial.print(ax); Serial.print("\t");                                 // 5 X-axis accelerometer data
      Serial.print(ay); Serial.print("\t");                                 // 6 Y-axis accelerometer data
      Serial.print(az); Serial.print("\t");                                 // 7 Z-axis accelerometer data
      Serial.print(gx); Serial.print("\t");                                 // 8 X-axis gyroscope data
      Serial.print(gy); Serial.print("\t");                                 // 9 Y-axis gyroscope data
      Serial.print(gz); Serial.print("\t");                                 // 10 Z-axis gyroscope data
      Serial.print(p); Serial.print("\t");                                  // 11 Pitch angle
      Serial.print(y); Serial.print("\t");                                  // 12 Yaw angle
      Serial.print(r); Serial.print("\t");                                  // 13 Roll angle
      Serial.print(al); Serial.print("\t");                                 // 14 Altitude
      Serial.print(vz); Serial.print("\t");                                 // 15 Vz
      Serial.print(S1_pos); Serial.print("\t");                             // 16 S1_pos
      Serial.print(tp); Serial.print("\t");                                 // 17 Temperature
      Serial.print(x_pos,7); Serial.print("\t");                            // 18 Longitude
      Serial.print(y_pos,7); Serial.print("\t");                            // 19 Latitude
      Serial.print(sat); Serial.print("\t");                                // 20 Sats
      Serial.print(v_bat); Serial.print("\t");                              // 21 Battery Voltage
      Serial.print(heading); Serial.println("\t");                          // 22 Heading
    }

    /*==========Telemetry_Data===================================================================================================================================*/
    TelemetryPacket pkt;                     /*Assign variables to telemetry packets*/
    pkt.packet_id = packet_id+1;             // packet ID
    pkt.om = om;                             // 1 Op Mode for state machine status
    pkt.fq = fq;                             // 2 Frequency Hz for correct loop speed check
    pkt.sd = sd;                             // 3 Sd Mode for logging check (not needed in SD logging file)
    pkt.ts = ts;                             // 4 Timestamp for time variable
    pkt.ax = ax;                             // 5 X-axis accelerometer data
    pkt.ay = ay;                             // 6 Y-axis accelerometer data
    pkt.az = az;                             // 7 Z-axis accelerometer data
    pkt.gx = gx;                             // 8 X-axis gyroscope data
    pkt.gy = gy;                             // 9 Y-axis gyroscope data
    pkt.gz = gz;                             // 10 Z-axis gyroscope data
    pkt.p = p;                               // 11 Pitch angle
    pkt.y = y;                               // 12 Yaw angle
    pkt.r = r;                               // 13 Roll angle
    pkt.al = al;                             // 14 Altitude
    pkt.vz = vz;                             // 15 Vz
    pkt.S1_pos = S1_pos;                     // 16 S1_pos
    pkt.tp = tp;                             // 17 Temperature
    pkt.x_pos = x_pos;                       // 18 X position
    pkt.y_pos = y_pos;                       // 19 Y position
    pkt.sat = sat;                           // 20 Sat number
    pkt.v_bat = v_bat;                       // 21 Battery voltage
    pkt.heading = heading;                           // 22 heading

    esp_now_send(gsAddress, (uint8_t*)&pkt, sizeof(pkt));  /*Send telemetry packet*/
      

    /*==========Remote_Commands=================================================================================================================================*/
    if (Serial.available()) {                                /*Check if serial connection is available*/
      cmd_ser = Serial.readStringUntil('\n');                /*Read serial command*/
      cmd_ser.trim();                                        /*Trim serial command to remove extra white spaces*/
    }

    /*--------0--Idle_Command----------*/
    if (om == 0 && (cmd_wifi == "IDL" || cmd_ser == "IDL")) {                     /*Manual Idle from Low Power mode*/
      om = 1;                                                                     /*Set operative mode to 1=idle*/                                                            
      cmd_wifi = ""; cmd_ser = "";                                                /*Resets the string for incoming commands ; Resets the string for incoming commands*/                                 
      fq_fixed = 20; dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;            /*Frequency set to 20Hz as standard for idle mode ; Set dt_fixed accordingly*/   
      Serial.println("SYSTEM_IDLE...");                                           /*Print to SERIAL and FILE the external command*/
    }

    /*--------1--LowPower_Command----------*/
    if (om == 1 && (cmd_wifi == "LPO" || cmd_ser == "LPO")) {                     /*Manual Low Power form Idle mode*/
      om = 0;                                                                     /*Set operative mode to 1=idle*/
      cmd_wifi = ""; cmd_ser = "";                                                /*Resets the string for incoming commands ; Resets the string for incoming commands*/                                 
      fq_fixed = 5; dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;             /*Frequency set to 5Hz as standard for low_power mode ; Set dt_fixed accordingly*/   
      Serial.println("SYSTEM_LOW-POWER...");                                      /*Print to SERIAL and FILE the external command*/
    }

    /*--------2--Reboot_Command----------*/
    if ((om == 0 || om == 1 || om == 7) && (cmd_wifi == "REB" || cmd_ser == "REB")) {    /*If reboot command is detected !!! ONLY possible to reboot if in LOW POWER ; IDLE ; LANDED */
      om = 1;                                                                            /*Go back to IDLE MODE*/
      cmd_wifi = ""; cmd_ser = "";                                                       /*Resets the string for incoming commands ; Resets the string for incoming commands*/                                   
      Serial.println("SYSTEM_REBOOTING...");                                             /*Print to SERIAL and FILE the external command*/
      ESP.restart();                                                                     /*Restart the esp32 lastly*/
    }

    /*--------3--Manual_Arming----------*/
    if (om == 1 && (cmd_wifi == "ARM" || cmd_ser == "ARM")) {               /*Manual arming of the systems only possible if we are in 1=idle mode ; no need for arming if already armed*/
      om = 2;                                                               /*Set operative mode to 2=armed*/
      cmd_wifi = ""; cmd_ser = "";                                          /*Resets the string for new incoming commands ; Resets the string for new incoming commands*/
      fq_fixed = 100; dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;     /*Set frequency to normal frequency ; Set dt fixed accordingly*/
      Serial.println("SYSTEM_ARMED...");                                    /*Print to SERIAL and FILE the external command*/
      startBeep(DOUBLE);                                                    /*2Buzzer TONE*/ 
    }

    /*--------4--Manual_Disarming----------*/
    if (om == 2 && (cmd_wifi == "DRM" || cmd_ser == "DRM")) {                   /*Manual disarming of the systems only possible if we are in 2=armed_mode ; no need for disarming if already disarmed*/
      om = 1;                                                                   /*Set operative mode to 1=idle*/
      cmd_wifi = ""; cmd_ser = "";                                              /*Resets the string for new incoming commands ; Resets the string for new incoming commands*/
      fq_fixed = 20; dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;          /*Set frequency to normal frequency ; Set dt fixed accordingly*/                                                                        
      Serial.println("SYSTEM_DISARMED...");                                     /*Print to SERIAL and FILE the external command*/
      startBeep(SINGLE);                                                        /*1Buzzer TONE*/                                                                                      
    }

    /*-------5--Manual_Parachute----------*/
    if (om == 2 && (cmd_ser == "PAR" || cmd_wifi == "PAR")) {                   /*If operative mode is 2=armed and either from wifi or usb serial command parachute is received !!! This means that if BOTH the MPU and BMP fail to detect apogee it's manually possible to deploy it*/
      om = 5;                                                                   /*Set operative mode to 5=parachute*/
      cmd_wifi = ""; cmd_ser = "";                                              /*Resets the string for new incoming commands ; Resets the string for new incoming commands*/
      fq_fixed = 100; dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;         /*Set frequency to normal frequency ; Set dt fixed accordingly*/
      
      firePY1();                                                             /*Deploy first pyro charge, same sequence as automatic parachute*/
      Serial.println("<PYRO_1> DEPLOYED MANUALLY...");                       /*Print to SERIAL and FILE the external command*/
      firePY2();                                                             /*Deploy second pyro charge, same sequence as automatic parachute*/
      Serial.println("<PYRO_2> DEPLOYED MANUALLY...");                       /*Print to SERIAL and FILE the external command*/

    }

  updateBuzzer();
  updatePyros();

  time_last = time_now;    /*recursive statement for time stamp*/
  al_last = al;            /*recursive statement for altitude*/
  }
}


/*=====================================CORE 0 TASK - SD writing=============================================*/
void core0Task(void *pvParameters) {
  while (true) {
    // everything that runs on core 0

    // Map operative mode to delay interval
    int interval_ms;
    switch (om) {
      case 0:  interval_ms = 200;  break; // 5Hz  low power
      case 1:  interval_ms = 50;   break; // 20Hz  idle
      case 2:  interval_ms = 10;   break; // 100Hz armed
      case 3:  interval_ms = 10;   break; // 100Hz powered_ascent
      case 4:  interval_ms = 10;   break; // 100Hz  coast
      case 5:  interval_ms = 10;   break; // 100Hz  parachute
      case 6:  interval_ms = 50;   break; // 20Hz  stable_descent
      case 7:  interval_ms = 200;  break; // 5Hz  landed
    }

    /*==========File_Write=========================================================================================================================================*/
      if (sd && sd_flag) {                                                         /*If SD is ON (flag == 1) and SD writing is enabled*/
      char line[400];                                                              /*Make a char array for better write speed*/
      int len = snprintf(line, sizeof(line),                                       /*Calculate char array length*/
        "%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.7f,%.7f,%d,%.3f,%.3f\n",
        om, fq, ts, ax, ay, az, gx, gy, gz, p, y, r, al, vz, S1_pos, tp, x_pos, y_pos, sat, v_bat, heading);

        for (int i = 0; i < len; i++) {              /*Print individual characters to the SD*/
          sdBuffer[sdIndex++] = line[i];
          if (sdIndex >= BUFFER_SIZE) {
            data.write((uint8_t*)sdBuffer, sdIndex); /*Write only one time, file not closed but data flushed to ensure no data is lost in case of accident power off*/
            data.flush();
            sdIndex = 0;
          }
        }
      }
    vTaskDelay(interval_ms / portTICK_PERIOD_MS); // 100Hz
  }
}

