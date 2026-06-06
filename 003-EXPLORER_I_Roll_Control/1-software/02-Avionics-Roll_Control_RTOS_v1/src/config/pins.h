/*-----Configuration_script_PINS*/

#pragma once

/*SDA SCL PINS*/
#define I2C_SCL = 21
#define I2C_SDA = 22

/*Pyro system*/
#define PY_1 12        /*Pyro_Charge_1_Pin*/
#define PY_2 14        /*Pyro_Charge_2_Pin*/

/*Buzzer*/
#define BUZ 32         /*Buzzer_Pin (PWM - DIGITAL)*/

/*IMU*/
#define MPU_ADDR 0x68  /*MPU_address_I2C*/

/*Servo*/
#define SERVO_PIN 26   /*Servo physical pin (PWM - DIGITAL)*/

/*SD*/
const int SD_CS = 5;   /*Chip_Select_Pin for SD (DIGITAL)*/