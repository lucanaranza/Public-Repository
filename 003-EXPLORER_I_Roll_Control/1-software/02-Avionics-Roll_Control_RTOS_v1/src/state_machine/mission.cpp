/*Mission file C++*/    

// contains MISSION LOGIC
// when to switch from idle to arm
// when to switch from arm to powered ascent (launch )
// when to switch from la
      
      
      /*----------Parachute_Deployment_Algorithm----------*/
      if (al_max - al > al_threshold && (om == 1 || om == 2) && al > min_apogee_threshold){  /*If signed difference between maximum altitude and current altitude is greater than the threshold and operative mode is either 1 or 2 and the current altitude is above a certain threshold (removes pad noise accidental firing)*/
        om = 3;                                                                              /*Set operative mode to 3 = parachute*/
        servo.writeMicroseconds(SERVO_CENTER);                                               /*Center servo*/

        Serial.println("<PYRO_1> DEPLOYED BY BAROMETER...");     /*Serial log for testing*/
        data.println("<PYRO_1> DEPLOYED BY BAROMETER...");       /*Data log for analysis*/

        digitalWrite(PY_1, HIGH);                                /*Fires first deployment charge*/
        delay(1000);                                             /*Keep mosfet on 1s for full burn*/
        digitalWrite(PY_1, LOW);                                 /*Deactivates mosfet after firing*/

        fq_fixed = 20;                                           /*Return back to normal logging frequency -> less data size*/ 
        dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;        /*Calculation to update dt_fixed*/
      }
      if (ts_tof > ts_apogee &&  om == 2){                       /*Having the operative mode 2 in the if prevents a double firing (unnecessary), if any of the methods fail the backup method will sense that om is still == 2 and proceed to fire the charge*/
        om = 3;                                                  /*Change operative mode to parachute*/
        servo.writeMicroseconds(SERVO_CENTER);                   /*Center servo*/

        Serial.println("<PYRO_1> DEPLOYED BY TIMER...");         /*Serial log for testing*/
        data.println("<PYRO_1> DEPLOYED BY TIMER...");           /*Data log for analysis*/

        digitalWrite(PY_1, HIGH);                                /*Fires first deployment charge*/
        delay(1000);                                             /*Keep mosfet on 1s for full burn*/
        digitalWrite(PY_1, LOW);                                 /*Deactivates mosfet after*/

        fq_fixed = 20;                                           /*Return back to normal logging frequency -> less data size*/ 
        dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;        /*Calculation to update dt_fixed*/
      }