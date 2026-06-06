/*SD logger module*/

// logging only
// the "contract"

#pragma once
#include "shared_data.h"

bool sd_init(int cs_pin);
void sd_write(const SensorData &s, const FlightData &f);