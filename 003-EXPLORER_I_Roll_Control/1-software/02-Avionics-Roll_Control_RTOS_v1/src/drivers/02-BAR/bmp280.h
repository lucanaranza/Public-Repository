/*BMP 280 module*/

// must output ONLY raw data:
#pragma once
#include "shared_data.h"

void bmp_init(float p0);
float bmp_read_altitude();