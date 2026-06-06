/*GPS driver*/

// must output ONLY raw data:
#pragma once
#include "shared_data.h"

struct GPSData {
    float lat;
    float lon;
    float alt;
    float speed;
    float course;
    bool fix;
    uint8_t sats;
};

void gps_init();
void gps_update(GPSData &gps);