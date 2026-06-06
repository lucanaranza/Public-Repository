/*MPU 6050 script C++*/

// must output ONLY raw data int16t format


#include "mpu6050.h"

static int16_t ax_raw, ay_raw, az_raw;
static int16_t gx_raw, gy_raw, gz_raw;               // Raw gyro data I2C integer format
int16_t tp_raw = 0;                                  // Raw temp data I2C integer format

float sum_ax = 0.0, sum_ay = 0.0, sum_az = 0.0;      // Sum variable for initial calibration
float sum_gx = 0.0, sum_gy = 0.0, sum_gz = 0.0;      // Sum variable for initial calibration
float sum_tp = 0.0;                                  // Sum variable for initial calibration
float ax_avg = 0.0, ay_avg = 0.0, az_avg = 0.0;      // Averaged value for initial calibration
float gx_avg = 0.0, gy_avg = 0.0, gz_avg = 0.0;      // Averaged value for initial calibration
float tp_avg = 0.0;                                  // Averaged value for initial calibration

float a_scale = 0.0;                                 // Acceleration SCALING factor (assumed to be the same for every axis)
float gx_bias = 0.0, gy_bias = 0.0, gz_bias = 0.0;   // Gyroscope BIAS factor (axis dependent) (offset from true value)

/*========== power management, gyro range, accel range, etc... ==========*/ 
void mpu_init() { 
  Wire.beginTransmission((uint8_t)MPU);  // This is hardware init
  Wire.write(0x6B);                      // direct register write
  Wire.write(0x00);                      // direct register write
  Wire.endTransmission();                // End transmission and go to next instruction
 
  Wire.beginTransmission((uint8_t)MPU);  // This configures DLPF
  Wire.write(0x1A);                      // direct register write
  Wire.write(0x03);                      // direct register write
  Wire.endTransmission();                // End transmission and go to next instruction

  Wire.beginTransmission((uint8_t)MPU);  // Initialization ???
  Wire.write(0x6C);                      // direct register write
  Wire.write(0x00);                      // direct register write
  Wire.endTransmission();                // End transmission and go to next instruction
}

/*========== MPU read raw data ==========*/ 
void mpu_read(MPUData &out) {   // I2C read here
  Wire.beginTransmission((uint8_t)MPU);        // Begin transmission on the correct address for the MPU*/
  Wire.write(0x3B);                            // Direct register write
  Wire.endTransmission(false);                 // Continue transmitting
  Wire.requestFrom((uint8_t)MPU, 14, true);    // Request data

  ax_raw = Wire.read() << 8 | Wire.read();     // Read data directly from memory 
  ay_raw = Wire.read() << 8 | Wire.read();
  az_raw = Wire.read() << 8 | Wire.read();
  tp_raw = Wire.read() << 8 | Wire.read();
  gx_raw = Wire.read() << 8 | Wire.read();
  gy_raw = Wire.read() << 8 | Wire.read();
  gz_raw = Wire.read() << 8 | Wire.read();
}