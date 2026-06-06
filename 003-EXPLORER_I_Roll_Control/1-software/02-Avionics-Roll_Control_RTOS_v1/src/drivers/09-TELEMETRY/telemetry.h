/*Telemetry file Header*/

// communication only
// the "contract"
#pragma once
#include "shared_data.h"

void telemetry_init();
void telemetry_send(const SensorData &s, const FlightData &f);