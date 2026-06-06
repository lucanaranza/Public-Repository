/*MPU 6050 header*/

// "the contract"
// raw data structure
// function prototypes
// no logic
// must output ONLY raw data:

#pragma once
#include <Arduino.h>      // Needed for core functions and voids cycles
#include <Wire.h>         // Needed for I2C communication

struct MPUData {
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temp;
};


void mpu_init();
void mpu_read(MPUData &out);
void mpu_calibrate();

struct MPUConfig {  /*Needed for MPU6050 clones with different I2C chip addresses from 0x68*/
    bool MPU_fake; 
};