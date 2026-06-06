/*Header file for MPU6050 processing*/

// "contract" of the cpp file 

#pragma once

struct MPUData {
    float ax, ay, az;
    float gx, gy, gz;
    float tp;
};

void MPU_calibrate() {
  for (int i = 0; i < num_cal; i++)
};

void MPU_process();

const MPUData& MPU_getData();