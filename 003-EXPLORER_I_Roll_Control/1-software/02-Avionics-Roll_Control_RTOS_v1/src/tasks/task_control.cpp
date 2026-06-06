/*-----Task_for_ACTIVE_CONTROL_SYSTEM-----*/

float error = 0.0;                 /*Pid signed error from setpoint*/
float prevError = 0.0;             /*Pid previous iteration error for derivative calculations*/
float integral = 0.0;              /*Integral value*/
float derivative = 0.0;            /*Derivative value*/
float output = 0.0;                /*Output value from pid*/

      /*----------ACTIVE_CONTROL_SECTION----------*/
      if (om == 2) {                                    /*Only active if liftoff is detected*/
        r_set = 0;                                      /*First setpoint active in the beginning and after last setpoint*/                                                 
        if (ts_tof > 0.0 && ts_tof <= 1.5) {            /*Setpoint scheduler roll program +90° -90°*/
          r_set = 0.0;
        } 
        else if (ts_tof > 1.5 && ts_tof <= 3.0) {
          r_set = 90.0*PI/180 * (ts_tof - 1.5) / 1.5;   /*1.5s costant slope rise to +90 degrees*/
        } 
        else if (ts_tof > 3.0 && ts_tof <= 4.0) {
          r_set = 90.0*PI/180;                          /*Fixed setpoint at 90 degrees*/
        } 
        else if (ts_tof > 4.0 && ts_tof <= 5.5) {
          r_set = 90.0*PI/180 - (90.0*PI/180) * (ts_tof - 4.0) / 1.5; /*1.5s constant slope descent to +0 degrees*/
        } 
        else if (ts_tof > 5.5) {
          r_set = 0.0;
        }

      
        error = r_set - r*PI/180;                                                     /*actual error calculation in radians (needed to import correct gains from simulink)*/
        derivative = 0.368 * derivative + 0.632 * ((error - prevError) / dt);         /*Simple finite difference derivative + Low pass filter for the derivative tau = 1/N ; alpha * d + (1-alpha) d(d)/dt ; alpha = e^-Ndt (dt=100Hz=0.01s)*/
        prevError = error;                                                            /*Recursive statement for derivative*/

        output = KP * error + KI * integral + KD * derivative;                        /*Output calculation, uses true integral from previous calculation*/

        const double MAX_DEFLECTION_RAD = 0.4363;                                     /*25 deg in rad, max tab deflection for aero limits*/

        bool saturated = (output >= MAX_DEFLECTION_RAD && error > 0) || (output <= -MAX_DEFLECTION_RAD && error < 0); /*Saturated condition check with directionality (if error and u unsat have different signs this means the controller is trying to push in the correct direction -> apply only integral contraint)*/
        if (!saturated) {
         integral += error * dt;                                  /*Anti windup integral*/
         integral = constrain(integral, -1500, 1500);             /*Constrained value always*/
        }

        output = constrain(output, -MAX_DEFLECTION_RAD, MAX_DEFLECTION_RAD);                  /*Saturate output to physical tab limits*/

        double servo_scale = (double)(SERVO_MAX - SERVO_CENTER) / MAX_DEFLECTION_RAD;         /*Calculate actual servo scaling factor*/
        S1_pos = constrain(SERVO_CENTER + (int)(output * servo_scale), SERVO_MIN, SERVO_MAX); /*Convert to servo command*/
        servo.writeMicroseconds(S1_pos);                                                      /*Write the position = move the servo*/
      }