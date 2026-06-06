/*-----Configuration_Settings-----*/

constexpr bool SERIAL_ENABLED = true;   // variable to select if serial transmission is ACTIVE or NOT

constexpr int num_cal_MPU = 500;   // Number of iteration for calibrating MPU
constexpr int num_cal_BMP = 400;   // Number of iteration for calibrating BMP

#define BUFFER_SIZE 512            // Buffer size in bits for SD printing (execution is faster if buffered)

constexpr float KF_BAR_AL = 0.075;  // low pass filter coefficient for altitude calculation []
constexpr float KF_BAR_VZ = 0.05;   // low pass filter coefficient for Z velocity calculation []

constexpr float KF_GPS_XY = 0.075;  // low pass filter coefficient for GPS lat,lon calculations []
constexpr float KF_GPS_Z  = 0.05;   // low pass filter coefficient for GPS altitude calculation []
constexpr float KF_GPS_VXY = 0.05;  // low pass filter coefficient for
constexpr float KF_GPS_VZ = 0.05;   // low pass filter coefficient for GPS Z velocity calculation []

constexpr float KF_MPU_VXY = 0.05;  // low pass filter coefficient for

constexpr float KF_XY = 0.05;  // complementary filter coefficient for GPS and IMU X,Y position correction []
constexpr float KF_Z  = 0.02;  // complementary filter coefficient for GPS and BARO Z position correction []
constexpr float KF_VXY = 0.05; // complementary filter coefficient for 
constexpr float KF_VZ = 0.02;  // complementary filter coefficient for 

uint8_t gsAddress[] = {0xF8, 0xB3, 0xB7, 0x49, 0x44, 0x14};  // Ground station MAC address !!! do not change
