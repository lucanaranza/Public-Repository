/*SD logger file C++*/

// logging only

#include <Arduino.h>        // Needed for core functions and voids cycles
#include <SD.h>             // Needed for the SD logging
#include <SPI.h>            // Needed for the SD logging


char sdBuffer[BUFFER_SIZE];        /*Actual buffer allocation in character format */
size_t sdIndex = 0;                /*Character writing index*/
bool sd_flag = false;              /*checks if SD is operative or not*/

File data;                         /*Data file object for SD*/

  /*----------Initialize_SD----------*/
  if(SD.begin(SD_CS, SPI, 40000000)){     /*Set SPI clock speed to maximum and checks if SD is mounted correctly*/
    Serial.println("SD_MOUNT_ON...");     /*Send confirmation*/
    sd_flag = true;                       /*Boolean flag for debugging*/
  } else {
    Serial.println("SD_ERR...");          /*Send error message */
    sd_flag = false;                      /*Do not block the launch, SD is non flight critical, just flag it to false*/
  }
  
  if (sd_flag){                                                      /*if the flag is true continue with SD initialization*/
    data = SD.open("/data.txt", FILE_WRITE);                         /*Open data.txt file and go in write mode*/
    if (data) {                                                      /*If the command is successful then write*/
      data.println("om,fq,ts,ax,ay,az,gx,gy,gz,p,y,r,al,vz,S1_pos"); /*Print header in the txt file*/
      Serial.println("FILE_ON...");                                  /*Send Serial confirmation of SD activation*/
      sd = 1;                                                        /*Set SD variable to ON*/
    } else {
      Serial.println("FILE_ERR...");                                 /*Send error message for debugging*/
    }
  }

/*----------File_Write----------*/
if (sd_flag) {                                                               /*If SD is ON (flag == 1)*/
  char line[132];                                                              /*Make a char array for better write speed*/
  int len = snprintf(line, sizeof(line),                                       /*Calculate char array length*/
  "%d,%.3f,%4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d\n",
  om, fq, ts, ax, ay, az, gx, gy, gz, p, y, r, al, vz, S1_pos
  );
  for (int i = 0; i < len; i++) {              /*Print individual characters to the SD*/
    sdBuffer[sdIndex++] = line[i];
      if (sdIndex >= BUFFER_SIZE) {
        data.write((uint8_t*)sdBuffer, sdIndex); /*Write only one time, file not closed but data flushed to ensure no data is lost in case of accident power off*/
        data.flush();
        sdIndex = 0;
      }
    }
}