/*-----IMPORTANT!!!-Flight_configuration_parameters-----*/

// pure configuration only (no behavior)
// So it is correct that it contains:
// thresholds
// enums
// static tables

#pragma once
#include <stdint.h>

constexpr float TS_APOGEE      = 8.5f;  /*variable to set deployment charge delay from launch detection [s] (roughly equal to expected apogee time + deltaT detection + deltaT first charge OFF*/
constexpr float AZ_THRESHOLD   = 20.0f; /*variable to set true Z acceleration threshold [m/s^2] for starting the timer (launch detection) (roughly equal to worst min acceleration at liftoff seen by the rocket but > 5 sigma for filtered Z acc values)*/
constexpr float AL_THRESHOLD   = 4.0f;  /*variable to set altitude threshold [m] for deploying the parachute when descending (roughly equal to 3 sigma for filtered data values)*/
constexpr float MIN_APOGEE_ALT = 10.0f; /*variable to set altitude threshold [m] above which you can safely deploy parachute (roughly equal to minimum altitude needed for the parachute to eject and inflate = deltaT detection + deltaT pyro + deltaT opening + deltaT terminal_velocity)*/
constexpr float MAX_RECOVERY_VZ = 12.0f;/*variable to set maximum velocity threshold [m/s] under which the parachute is deployed*/
enum FlightMode {
    LOW_POWER = 0,
    IDLE      = 1,
    ARMED     = 2,
    ASCENT    = 3,
    COAST     = 4,
    PARACHUTE_EVENT = 5,
    STABLE_DESCENT = 6,
    LANDED    = 7
};

struct FlightProfile {
    uint16_t FQ_SENSOR;     /*Sensors data pulling frequency [Hz]*/
    uint16_t FQ_CONTROL;    /*Control System frequency [Hz] -> !lower than physical MG90s servo limits*/
    uint16_t FQ_SD;         /*SD logging frequency [Hz]*/
    uint16_t FQ_TELEMETRY;  /*Telemetry frequency [Hz]*/
};

/*Flight profiles of STATE MACHINE operating frequencies, sensor-control-SD-telemetry*/
static constexpr FlightProfile FLIGHT_PROFILES[] = {
    /* LOW_POWER */ {10, 100, 10, 10},
    /* IDLE      */ {20, 100, 20, 20},
    /* ARMED     */ {200, 100, 100, 20},
    /* ASCENT    */ {200, 100, 100, 20},
    /* COAST     */ {200, 100, 100, 20},
    /* PARACHUTE */ {200, 100, 100, 20},
    /* STABLE DESCENT*/ {200, 100, 100, 20},
    /* LANDED    */ {10, 100, 10, 10}
};