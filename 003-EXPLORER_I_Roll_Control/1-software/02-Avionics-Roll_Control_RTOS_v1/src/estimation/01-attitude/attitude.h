/*Header file for Attitude*/

// This file is the interface "contract" of the attitude system, it defines:
// what the module provides
// what other modules are allowed to use
// data types shared across the system
// It does not contain logic.

#pragma once
#include <Arduino.h>

/*Quaternion definition*/
struct Quaternion {  /*Quaternion struct W=module X Y Z*/
    float w, x, y, z // module, X comp, Y comp, Z comp
};

/*Function declaration attitude_initialization*/
void attitude_init(); 

/*Function declaration attitude_update*/
void attitude_update(float ax, float ay, float az, // XYZ accelerations from ACCELEROMETER (true) [m/s^2]
                     float gx, float gy, float gz, // XYZ angular velocities from GYROSCOPE [°/s]
                     float mx, float my, float mz, // XYZ magnetometer readings from MAGNETOMETER [uT]
                     float dt);                    // dt time interval between calculations (TRUE) [s]
                     
/*Function declaration attitude_quaternion_getter*/
Quaternion attitude_getQuaternion();