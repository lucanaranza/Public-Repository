/*-----BMP280_script_C++_-----*/

// must output ONLY raw data:

/*----------Libraries used----------*/
#include <Adafruit_Sensor.h>       /*Needed for the BMP 280*/
#include <Adafruit_BMP280.h>       /*Needed for the BMP 280*/
#include <Arduino.h>               /*Needed for core functions and voids cycles*/

Adafruit_BMP280 bmp;               /*Barometer object*/

float al_r;                        /*Raw altitude [m] */
