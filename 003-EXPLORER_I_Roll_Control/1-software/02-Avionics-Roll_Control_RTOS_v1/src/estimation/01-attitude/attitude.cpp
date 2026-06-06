/*Attitude variables calculations*/

// Actual implementation of the filter, contains:
// EKF  math
// internal state
// cached outputs (pitch,yaw,roll, quaternion, hdg)

#include "attitude.h"

static float p = 0.0f, y = 0.0f, r = 0.0f;                         /*cached float number values Pitch Yaw Roll [°] (outputs)*/
static Quaternion q = {};                                          /*cached quaternion struct float number values W X Y Z (internal EKF state)*/
float pitchErrorCov = 1.0, yawErrorCov = 1.0, rollErrorCov = 1.0;  /*Error covariance value for Simplified Kalman Filter*/              
float processNoise = 0.001, measurementNoise = 0.03;               /*Noise of the process value for Simplified Kalman Filter*/

float accelPitch = 0.0, accelYaw = 0.0, magRoll = 0.0;             /*First input of the Kalman filter (pitch and yaw computed from accelerometer ; roll computed from magnetometer)*/
float Kp = 0.0, Ky = 0.0, Kr = 0.0;                                /*Kalman filter coefficients for sensor fusion (exponenatial moving average filter)*/


/*1-Initialization*/
void attitude_init() {
    q = {1.0f, 0.0f, 0.0f, 0.0f};
    p = 0.0f;
    y = 0.0f;
    r = 0.0f;
}

/*2-Getter*/
Quaternion attitude_getQuaternion() {
    return q;
}

/*3-Update*/
void attitude_update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt)
{
  accelPitch = atan2(ax, sqrt(ay * ay + az * az)) * 180.0 / PI;  // 1) Pitch input 1 -> Acc Trigonometry
  p += gx * dt;                                                  // 2) Pitch input 2 -> Gyro Integration calculated on TRUE dt between iterations 
  pitchErrorCov += processNoise;                                 // 3) Covariance is computed from the process noise
  Kp = pitchErrorCov / (pitchErrorCov + measurementNoise);       // 4) Constant Pitch calculation computed from error covariance and measurement noise
  p += Kp * (accelPitch - p);                                    // 5) Pitch computed via recursion from accelerometer and gyro data
  pitchErrorCov *= (1.0f - Kp);                                  // 6) Covariance multiplied updated

  accelYaw = atan2(ay, sqrt(ay * ay + az * az)) * 180.0 / PI;    // 1) Yaw input 1 -> Acc Trigonometry
  y += gy * dt;                                                  // 2) Yaw input 2 -> Gyro Integration
  yawErrorCov += processNoise;                                   // 3) Covariance is computed from the process noise
  Ky = yawErrorCov / (yawErrorCov + measurementNoise);           // 4) Constant Yaw calculation
  y += Ky * (accelYaw - y);                                      // 5) Yaw computed via recursion from accelerometer and gyro data
  yawErrorCov *= (1.0f - Ky);                                    // 6) Covariance multiplied updated
    
  r += gz * dt;                                                  // 1) Roll input 1 -> Gyro Integration
  mx_h = mx * cosf(p*PI/180.0f) + mz * sinf(pitch_rad);          
  my_h = my;
  magRoll = atan2f(-my_h, mx_h) * 180.0f / PI - magRollRef;      // 2) Roll input 2 -> Magn trigonometry
  rollErrorCov += processNoise;                                  // 3) Covariance is computed from the process noise
  Kr = rollErrorCov / (rollErrorCov + measurementNoise);         // 4) Constant Roll calculation
  r += Kr * (magRoll - r);                                       // 5) Roll computed via recursion from accelerometer and gyro data
  rollErrorCov *= (1.0f - Kr);                                   // 6) Covariance multiplied updated                                                               
  if (r >  180.0f) r -= 360.0f;                                  // 7) roll needs to stay between -180° and +180°, (PID controller safety and overall data)
  if (r < -180.0f) r += 360.0f;
}