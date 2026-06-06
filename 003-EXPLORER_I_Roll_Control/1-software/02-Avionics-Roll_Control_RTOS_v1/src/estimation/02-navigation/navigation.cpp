/*Navigation variables calculations*/

// owns the following:
// position XYZ 
// velocities XYZ
// altitude (now, previous, max)
// GPS lat longitude
// derived heading

#include "navigation.h"
#include "config/settings.h"
#include "config/constants.h"

static NavState nav = {};

/*Navigation update pipeline*/
// 1. Read sensors (raw data)
// 2. Measurement filtering
// 3. State prediction (kinematics)
// 4. Corrections (GPS / baro / etc.) there is ONE navigation state, corrected over time, naturally recursive comp filter, avoids duplicated estimators
// 5. Derived outputs (apogee, etc.)

// Position  BARO+GPS -> primary for Z:barometer, correction for Z:GPS, primary for X,Y:GPS
// Velocity derivation BARO+GPS and integration IMU ->  primary for Z:barometer, correction for Z:IMU+GPS (slight,optional), primary for X,Y:GPS, correction for X,Y:IMU (slight)

/*Estimation Layers*/
// A - Measurement Filtering (BARO smoothing, GPS smoothing, baro&GPS bias removal)
// B - Kinematic Prediction (integrate velocity, propagate position)
// C - Correction Layer (IMU+GPS correction)

/*Variable initialization*/
static float al_bar_raw = 0.0f, al_bar_corrected = 0.0f;

static float z_gps_raw = 0.0f, z_gps_corrected = 0.0f, z_gps = 0.0f, z_gps_last = 0.0f;

static float vz_bar_raw = 0.0f, vz_bar = 0.0f;

static float vz_gps_raw = 0.0f, vz_gps = 0.0f;

static float lat_gps_raw = 0.0f, lon_gps_raw = 0.0f;
static float dlat = 0.0f, dlon = 0.0f;

static float x_gps = 0.0f, y_gps = 0.0f;
static float x_gps_last = 0.0f, y_gps_last = 0.0f;

static float vx_mpu_raw = 0.0f, vy_mpu_raw = 0.0f;
static float vx_mpu = 0.0f, vy_mpu = 0.0f;

static float vx_gps_raw = 0.0f, vy_gps_raw = 0.0f;
static float vx_gps = 0.0f, vy_gps = 0.0f;

static float x_imu = 0.0f;
static float y_imu = 0.0f;

void navigation_update(float dt, float pr, float gps_lat, float gps_lon, float z) {

    /*==================== 1) BAROMETER: Altitude - layer A ====================*/
    al_bar_raw = 44330.0f * (1.0f - pow(nav.pr / pr_0, 0.1903f));         // get altitude_raw from pressure read from the bmp, use p0 for scaling (works well up to 11km)
    al_bar_corrected = al_bar_raw - nav.al_bar_offset;                    // remove offset from the raw measurement
    nav.al = KF_BAR_AL * nav.al + (1.0f - KF_BAR_AL) * al_bar_corrected;  // simple low-pass filter for correct altitude measurement

    /*==================== 2) GPS: Altitude - layer A ====================*/
    z_gps_raw = gps_z;                                                  // raw gps Z position WGS84 ellipsoid based
    z_gps_corrected = z_gps_raw - nav.z_gps_offset;                     // remove offset from the raw measurement
    z_gps = KF_GPS_Z * z_gps + (1.0f - KF_GPS_Z) * z_gps_corrected;     // simple low pass filter for correct GPS altitude measurement

    /*==================== 3) SENSOR FUSION: Altitude from BAROMETER and GPS - Layer C ====================*/
    nav.z = nav.al;                                 // initialize Z position to be the same as barometric altitude
    nav.z += KF_Z * (z_gps - nav.z)                 // complementary filter for correct Z position 

    /*==================== 4) BAROMETER: Z Velocity - layer A+B ====================*/
    vz_bar_raw = (nav.al - nav.al_last) / dt;                      // simple finite difference derivation from barometer correct altitude
    vz_bar = KF_BAR_VZ * vz_bar + (1.0f - KF_BAR_VZ) * vz_bar_raw; // low pass filter for correct barometer Z velocity measurement

    /*==================== 5) GPS: Z Velocity - layer A+B ====================*/
    vz_gps_raw = (z_gps - nav.z_gps_last) / dt;                        // simple finite difference derivative
    vz_gps = KF_GPS_VZ * vz_gps + (1.0f - KF_GPS_VZ) * vz_gps_raw;     // low pass filter for correct GPS Z velocity measurement

    /*==================== 6) SENSOR FUSION: Z Velocity from BAROMETER and GPS - Layer C ====================*/
    nav.vz = vz_bar;                                    // initialize Z velocity to be the same as barometer velocity
    nav.vz += KF_VZ * (vz_gps - nav.vz);                // complementary filter for correct Z velocity

    /*==================== 7) GPS: LAT,LON + X,Y Position - layer A ====================*/
    lat_gps_raw = 0;                                          // raw gps latitude  
    lon_gps_raw = 0;                                          // raw gps longitude
    nav.lat = nav.lat + KF_GPS_XY * (lat_gps_raw - nav.lat);  // simple exponential filter for correct latitude
    nav.lon = nav.lon + KF_GPS_XY * (lon_gps_raw - nav.lon);  // simple exponential filter for correct longitude

    dlat = nav.lat - nav.lat_offset;                         // delta latitude is calculated with respect to launch initial position
    dlon = nav.lon - nav.lon_offset;                         // delta longitude is calculated with respect to launch initial position
    x_gps = dlon * r_earth * cos(nav.lat_offset*PI/180.0f);  // spherical coordinates from longitude to x position (dependance on cosine of latitude)
    y_gps = dlat * r_earth;                                  // spherical coordinates from latitude to y position
                                       
    /*==================== 8) IMU: X,Y Velocities - layer A ====================*/
    vx_mpu_raw += nav.ax * dt;                                           // integrate X acceleration to get X velocity
    vy_mpu_raw += nav.ay * dt;                                           // integrate Y acceleration to get Y velocity
    vx_mpu = KF_MPU_VXY * vx_mpu + (1.0f - KF_MPU_VXY) * vx_mpu_raw;     // low pass filter for correct MPU X velocity
    vy_mpu = KF_MPU_VXY * vy_mpu + (1.0f - KF_MPU_VXY) * vy_mpu_raw;     // low pass filter for correct MPU Y velocity

    /*==================== 9) GPS: X,Y Velocities - layer A ====================*/
    vx_gps_raw = (x_gps - x_gps_last) / dt;                              // simple finite difference derivative X velocity calculation
    vy_gps_raw = (y_gps - y_gps_last) / dt;                              // simple finite difference derivative Y velocity calculation
    vx_gps = KF_GPS_VXY * vx_gps + (1.0f - KF_GPS_VXY) * vx_gps_raw;     // low pass filter for correct GPS X velocity
    vy_gps = KF_GPS_VXY * vy_gps + (1.0f - KF_GPS_VXY) * vy_gps_raw;     // low pass filter for correct GPS Y velocity

    /*==================== 10) SENSOR FUSION: X,Y Velocities from IMU and GPS - Layer C ====================*/
    nav.vx += KF_VXY * (vx_gps - vx_mpu); // complementary filter
    nav.vy += KF_VXY * (vy_gps - vy_mpu); // complementary filter

    /*==================== 11) MPU X,Y Positions - Layer A ====================*/
    x_imu += vx_mpu * dt;    // integrate X velocity to get X position (IMU)
    y_imu += vy_mpu * dt;    // integrate Y velocity to get Y position (IMU)

    /*==================== 12) SENSOR FUSION: X,Y Positions from previous Layer B + GPS Correction ====================*/
    nav.x += KF_XY * (x_gps - nav.x); // complementary filter 
    nav.y += KF_XY * (y_gps - nav.y); // complementary filter

    /*----- Apogee tracking -----*/
    if (nav.al > nav.al_max) {  // al = main altitude variable, costantly changing, threshold if altitude > max altitude
        nav.al_max = nav.al;    // if altitude > max altitude set new max altitude to current altitude
    }

    /*----- Update Z altitude state (barometer) -----*/
    nav.al_last = nav.al;       // update Z altitude barometer
    z_gps_last = z_gps;         // update Z gps
    x_gps_last = x_gps;         // update X gps
    y_gps_last = y_gps;         // update Y gps
}