/*-----Servo_Control_script_C++_------*/

// only “execute command”

#include <Arduino.h>               /*Needed for core functions: pinMode(), micros(), digitalWrite()*/
#include <Servo.h>                 /*Needed for the servo control*/

Servo servo;                       /*Servo object for Servo*/

  /*----------Initialize_and_test_Servos----------*/
  servo.attach(S1);                                           /*Attach servo to Pin*/
  servo.writeMicroseconds(SERVO_CENTER);                      /*Test servo with predefined sequence 0 ; -45 ; 0 ; +45*/
  delay(150);
  servo.writeMicroseconds(1125);
  delay(150);
  servo.writeMicroseconds(SERVO_CENTER);
  delay(150);
  servo.writeMicroseconds(1875);
  delay(150);
  servo.writeMicroseconds(SERVO_CENTER);
  delay(300);

  // TRANSFER TO BUZZER ROUTINE LATER
   digitalWrite(BUZ, HIGH); delay(200); digitalWrite(BUZ, LOW);  /*Send buzzer confirmation of correct startup of servo*/
  delay(200);
  digitalWrite(BUZ, HIGH); delay(200); digitalWrite(BUZ, LOW);