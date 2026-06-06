/*I2C Utils file C++*/

#include <Arduino.h>      // Needed for delay, Digitalwrite etc
#include "config/pins.h"  // Needed for PINs allocation

/*----------I2C_Reset----------*/
void I2C_Reset() {
  pinMode(I2C_SCL, INPUT_PULLUP);  // SCL pin set as input pullup for i2c communication
  pinMode(I2C_SDA, INPUT_PULLUP);  // SDA pin set as input pullup for i2c communication

  for (int i = 0; i < 9; i++) {    // Clock SCL manually to release any stuck slaves
    pinMode(I2C_SCL, OUTPUT);      // Set SCL to digital output
    digitalWrite(I2C_SCL, LOW);    // Set SCL to LOW
    delayMicroseconds(5);          // Blocking delay (fine in the setup) to ensure correct release
    digitalWrite(I2C_SCL, HIGH);   // Set SCL to HIGH
    delayMicroseconds(5);          // Blocking delay (fine in the setup) to ensure correct release
  }

  pinMode(I2C_SDA, OUTPUT);        // Set SDA to digital output
  digitalWrite(I2C_SDA, LOW);      // Set SDA to LOW
  delayMicroseconds(5);            // Blocking delay (fine in the setup) to ensure correct release
  digitalWrite(I2C_SCL, HIGH);     // Set SCL to HIGH
  delayMicroseconds(5);            // Blocking delay (fine in the setup) to ensure correct release
  digitalWrite(I2C_SDA, HIGH);     // Set SDA to HIGH - this finishes the STOP condition to reset the I2C bus
}

/*----------I2C_Check----------*/
  byte deviceCount = 0;                                        /*Variable to count # of I2C devices*/
  for (uint8_t addr = 1; addr < 127; addr++) {                 /*Checks every I2C address for sensors*/
    Wire.beginTransmission(addr);                              /*Initialize I2C transmission to the specific address*/
    if (Wire.endTransmission() == 0) {                         /*If transmission is not ended*/
      Serial.print("Found device at 0x");                      /*Print the I2C memory address of the device as hexadecimal*/
      if (addr < 16) Serial.print("0");                         
      Serial.println(addr, HEX);                             
      deviceCount++;                                           /*Continue onto the next addresses*/
      delay(5);                                                /*Wait 5ms to ensure more robust address counting (blocking delay is fine in the setup)*/
    }
  }
  if (deviceCount != 3) {                                          /*Blocking condition if a number of sensors != 3 even if there is mpu and barometer (essential sensors)*/
    Serial.println("NOT ENOUGH SENSORS FOUND!");                   /*Print error message*/
    delay(200);                                                    /*Wait 200ms (blocking delay is fine in the startup)*/
    digitalWrite(BUZ, HIGH); delay(1000); digitalWrite(BUZ, LOW);  /*Buzzer tone to indicate wrong number of sensors found*/
    while(true) delay(1000);                                       /*Stops activity and prevents accidental startup*/

  } else {
    Serial.print("TOTAL SENSORS FOUND: ");                                      /*Prints the number of devices found for debugging*/
    Serial.println(deviceCount); 
    for (int i = 0; i < deviceCount; i++) {                                     /*For clause that repeats for each connected sensor*/
       digitalWrite(BUZ, HIGH); delay(100); digitalWrite(BUZ, LOW); delay(200); /*Buzzer tone to indicate correct number of sensor found (each beep represents a sensor)*/
    }
    delay(200);                                                                 /*Wait 200ms to ensure more robust startup (blocking delay is fine in the setup)*/                         
  }