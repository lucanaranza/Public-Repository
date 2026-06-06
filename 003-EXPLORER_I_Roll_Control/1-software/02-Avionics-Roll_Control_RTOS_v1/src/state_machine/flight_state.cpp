/*Flight state - state machine*/

// Responsible for:
// mode transitions only

float ts = 0.0, ts_now = 0.0, ts_last = 0.0;    // timestamp from PWR_ON [s]; never gets reset; can be logged ; current timestamp [s] ; previous timestamp [s]
float dt = 0.0;                                 // Delta time between cycles [s]
float ts_tof = 0.0, ts_tof_0 = 0.0;             // Timestamp from LAUNCH [s] ; delta time from PWR_ON to LAUNCH [s]

#include "flight_state.h"
#include "config/flight_config.h"

FlightMode mode = LOW_POWER;

static float ts_launch = 0.0f;

void flight_update(const NavState& nav, float ts)
{
  switch (mode){
    case ARMED:
      if (fabs(nav.az) > AZ_THRESHOLD){
        mode = ASCENT;
        ts_launch = ts;
      }
      break;
      
      case ASCENT:
        if (vz < vz_max){
          mode = COAST;
        }
        break;

        default:
            break;
    }
}

      ///*----------Launch_Detection_Algorithm----------*/
      //if (abs(az) > az_threshold && om == 1)                      /*20 m/s^2 acceleration to initialize timer & only valid if operative mode is 1 = armed*/
      //{ 
      //  Serial.println("LAUNCH_DETECTED...");                     /*Print confirmation to serial for debug purposes*/
      //  data.println("LAUNCH_DETECTED...");
      //  om = 2;                                                   /*Change operative mode of state machine (timer is started)*/
      //  ts_tof_0 = ts;                                            /*Timestamp time of flight 0 is the true t=0 at liftoff*/
      //}
      //ts_tof = ts - ts_tof_0;                                     /*Time stamp for time of flight is updated*/

       //     if (om == 3 && ((gx+gy+gz)<9)) {                           /*If the parachute has fired and angular rate is less than 9 degrees per second !!! not valid for water touchdown*/
       // om = 4;                                                  /*Set operative mode to landed*/

       // Serial.println("TOUCHDOWN_CONFIRMED...");                /*Serial log for testing*/
       // data.println("TOUCHDOWN_CONFIMED...");                   /*Data log for analysis*/
      //}
      //if (om == 4) {
      //  fq_fixed = 10;                                           /*Update working frequency -> less data file */
       // dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;        /*Update dt_fixed */
      //}