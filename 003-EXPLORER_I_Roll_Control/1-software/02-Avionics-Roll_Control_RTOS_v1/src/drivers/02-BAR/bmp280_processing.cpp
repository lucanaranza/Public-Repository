/*BMP280 Processing script*/

float sum_al = 0.0;           // Sum variable for initial calibration
float al_avg = 0.0;           // Averaged value for initial calibration
float al_bias = 0.0;          // Bias barometer factor


  /*----------Initialize_BMP_280 and calibrate altitude bias at startup----------*/
  if (bmp.begin(0x76)) {                          // Start I2C communication at correct address
    bmp.setSampling(                              // Set sampling info
    Adafruit_BMP280::MODE_NORMAL,                 // Normal Mode = Continuos periodic measurements
    Adafruit_BMP280::SAMPLING_X1,                 // Lowest oversampling setting 
    Adafruit_BMP280::SAMPLING_X1,                 // Lowest oversampling setting
    Adafruit_BMP280::FILTER_OFF);                 // Remove internal filtering to apply custom filter for better sensor precision-responsiveness combination
    for (int i = 0; i < num_cal; i++) {           // Sensor calibration until i=numcal
      sum_al += bmp.readAltitude(p0 / 100.0);     // Pressure is read in hPa, summing altitude for calibration
      delayMicroseconds(10000);                   // Wait 10ms to ensure no duplicated data is pulled
    }
    al_bias = (sum_al / num_cal);                 // Calculated altitude bias
    delay(200);                                   // Wait 200ms to ensure correct startup and for debugging purposes
    Serial.println("BMP_280_ON...");              // Send confirmation of the calibration
  }
