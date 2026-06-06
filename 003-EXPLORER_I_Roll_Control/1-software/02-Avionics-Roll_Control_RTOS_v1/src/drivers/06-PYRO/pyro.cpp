/*Pyro file C++*/

// “execute command”

#include <Arduino.h>               /*Needed for core functions: pinMode(), micros(), digitalWrite()*/

pinMode(PY_1, OUTPUT);             /*Set Pyro pin to output*/
pinMode(PY_2, OUTPUT);             /*Set Pyro pin to output*/
Serial.println("PYRO_ON...");      /*Send confirmation of the calibration*/