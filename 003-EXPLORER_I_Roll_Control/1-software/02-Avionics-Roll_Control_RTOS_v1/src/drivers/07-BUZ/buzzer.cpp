/*Buzzer file C++*/

// “execute command”

#include <Arduino.h>               /*Needed for core functions: pinMode(), micros(), digitalWrite()*/

pinMode(BUZ, OUTPUT);              /*Set buzzer pin to output*/
Serial.println("BUZZER_ON...");    /*Send confirmation of the calibration*/