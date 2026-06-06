/*MPU Processing script*/

// gives out correct physical quantities inertial values (not g corrected)

#include <Arduino.h>                 // Needed for delay functionss
#include "mpu6050_processing.h"      // Needed for header instructions
#include "drivers/01-MPU/mpu6050.h"  // Needed for raw data pulling from I2C
#include "config/constants.h"        // Needed for g constant for calibration

static IMUData imu = {};

static float gx_bias = 0.0f;
static float gy_bias = 0.0f;
static float gz_bias = 0.0f;

static float a_scale = 0.0f;

void imu_calibrate() {
  for (int i = 0; i < num_cal; i++) {
    sum_ax += ax_raw;       // Start summing data to calibrate accelerometer
    sum_ay += ay_raw; 
    sum_az += az_raw;   

    sum_gx += gx_raw / 131.0f; // Start summing scaled data to calibrate gyro
    sum_gy += gy_raw / 131.0f;  
    sum_gz += gz_raw / 131.0f; 

    sum_tp += tp_raw;         // Start summing data to calibrate thermometer

    delayMicroseconds(10000); // Small delay to wait TEMPORARY!!!!
  }

  ax_avg = sum_ax / num_cal; // Calculate average for each acceleration axis
  ay_avg = sum_ay / num_cal;  
  az_avg = sum_az / num_cal;    

  a_scale = g / sqrt(ax_avg*ax_avg + ay_avg*ay_avg + az_avg*az_avg);  // From the averaged accelerations calculate scaling factor !!! rocket must remain still

  gx_bias = sum_gx / num_cal; // Calculate average for each gyro axis
  gy_bias = sum_gy / num_cal; 
  gz_bias = sum_gz / num_cal; 

  tp_avg = sum_tp / num_cal;  // Calculate average temperature   
  Serial.println("MPU_6050_CALIBRATED...");
  delay(200);

}

void imu_process() {
ax = -ay_raw * a_scale;  // Accel (scale + axis transform)
ay =  az_raw * a_scale;
az = -ax_raw * a_scale;

gx = -gy_raw / 131.0f + gy_bias;  // Gyro (scale + bias correction + axis transform)
gy =  gz_raw / 131.0f - gz_bias;
gz = -gx_raw / 131.0f + gx_bias;

tp = tp_raw / 100.0f;  // Temp
}

