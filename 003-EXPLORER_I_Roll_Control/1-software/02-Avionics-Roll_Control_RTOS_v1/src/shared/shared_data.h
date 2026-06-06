/*Central data structure*/
#pragma once
#include <Arduino.h>

/*-----Raw_Measurements-----*/
struct SensorData {     
    float ax, ay, az;   /*X Y Z Accelerations IMU [m/s^2]*/
    float gx, gy, gz;   /*X Y Z Angular velocities IMU [°/s]*/
    float mx, my, mz;   /*X Y Z Magnetometer data [uT]*/
    float tp;           /*Temperature [°C]*/

    float al;           /*Altitude [m]*/
    float vz;           /*Z Velocity [m/s]*/
};

/*-----Navigation_Data_(estimated state)-----*/
struct NavData {
    float q[4];         /*Quaternion Vector (attitude)*/
    float vx, vy, vz;   /*X Y Z Velocities*/
    float px, py, pz;   /*X Y Z Position*/
};

/*-----Flight_Data-----*/
struct FlightData {
    //Guards = “what condition is true?”
    // States = “what phase am I in?”
    int om;      /*Operative mode [0-1-2-3-4-5] -> STATES: 0-low_power ; 1-idle ; 2-armed ; 3-powered ascent ; 4-coast ; 5-parachute_descent ; 6-landed*/
                 /* EVENTS: (transition triggers) -> 0-1 manual_ON ; 1-2 manual_ARM ; 2-3 automatic_detection(accelerometer) ; 3-4 automatic_detection(accelerometer) ; 4-5 PYRO event ; 5-6 automatic_detection*/
    float ts;    /*Timestamp [s]*/
    float fq;    /*Frequency [Hz]*/

    int s1_pos, s2_pos, s3_pos, s4_pos; /*Servo 1-4 positions PWM [us]*/

    bool pyro_armed;            /*Boolean variable for pyro deployer armed*/
    bool parachute_deployed;    /*Boolean variable for parachute deployed*/
};

/*-----Control_Data-----*/
struct ControlData {
    float r_set;   /*Roll setpoint [°]*/
};