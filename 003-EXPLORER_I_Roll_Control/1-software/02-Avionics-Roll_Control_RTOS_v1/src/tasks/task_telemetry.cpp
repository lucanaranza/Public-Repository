/*-----Task_for_Telemetry-----*/
#include <Arduino.h>

unsigned long lastTelemetry = 0;   /*Last telemetry time [us]*/
String cmd_ser;                    /*String for incoming commands via USB (serial) for testing*/