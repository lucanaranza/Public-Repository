/*Header file for Navigation*/

//This file is the interface "contract" of the navigation system, it defines:
//what the module provides
//what other modules are allowed to use
//data types shared across the system
//It does not contain logic.

#pragma once          // header safety
#include <Arduino.h>

/*State vector definition*/
struct NavState { //12 components struct
    float x, y, z;    // POSITION (navigation frame) [m]
    float vx, vy, vz; // VELOCITIES (navigation frame) [m/s]

    float al;         // ALTITUDE (mission-relevant) from barometer (filtered) [m]
    float al_last;    // previous ALTITUDE (mission-relevant) from barometer (filtered) [m]
    float al_max;     // maximum ALTITUDE (mission-relevant) from barometer (filtered) [m]

    float lat, lon;   // GPS latitude and longitude [decimal degrees N-E]
    float hdg;        // HEADING north-referenced [°N]
};

/*Function declaration navigation_initialization*/
void navigation_init();

/*Function declaration navigation_update*/
void navigation_update(float dt);

/*Function declaration navigation-state_getter*/
const NavState& navigation_getState(); // better for RTOS (no copy overhead)