/*Telemetry file C++*/

// communication only

#include <esp_now.h>       // Needed for the onboard wifi communication protocol
#include <WiFi.h>          // Needed for the onboard wifi communication protocol
#include <esp_wifi.h>      // Needed for the onboard wifi communication protocol

String cmd_wifi;           // String for incoming commands via WiFi

/*----------Robust_Startup----------*/
  Telemetry.begin(115200, SERIAL_8N1, TX_PIN, RX_PIN);         /*Begin telemetry transmission on TX and RX pins at 115200 baud rate*/
  Serial.flush();                                              /*Flushes any unwanted serial data*/
  delay(200);                                                  /*Blocking delay (fine in the setup) to ensure correct flush*/
  Serial.begin(500000);                                        /*Begin serial communication at 500000 baud rate*/
  Serial.println("STARTUP_SEQUENCE...");                       /*After initialization print the startup sequence begin on the monitor*/
  delay(200);                                                  /*Blocking delay (fine in the setup) to ensure correct startup*/   

  digitalWrite(BUZ, HIGH); delay(200); digitalWrite(BUZ, LOW); /*Sound confirmation for correct Startup and initializing Setup and Calibration*/

  I2C_Reset();                                                 /*Restart (recover) I2C bus first*/
  delay(200);                                                  /*Blocking delay (fine in the setup) to ensure correct reset*/ 
  Wire.begin(21, 22);                                          /*Starts I2C communication*/
  delay(200);                                                  /*Blocking delay (fine in the setup) to ensure correct BUS startup*/
  Wire.setClock(800000);                                       /*I2C clock speed set as highest for maximum performance*/
  delay(200);                                                  /*Blocking delay (fine in the setup) to ensure correct startup and stabilization before next part*/



  /*----------Send_Data_to_GS----------*/
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {} // Callback function used by ESP-NOW, right now does nothing, can be used for debugging*/

/*----------Receive_Data_from_GS----------*/
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) { // Callback function used by ESP-NOW, reads data and trims string to match size of command*/
    cmd_wifi = String((const char*)incomingData, len);                           // Construct string with explicit length*/
    cmd_wifi.trim();                                                             // Remove leading/trailing whitespace/newlines*/
}


  /*----------Initialize_WiFi----------*/
  WiFi.mode(WIFI_STA);                               /*Set Wifi mode to wifi station*/
  delay(200);                                        /*Give WiFi stack time to initialize properly !!! very important*/

  esp_wifi_set_promiscuous(true);                    /*Set promiscuous mode to ensure correct channel change*/
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);    /*Channel set as SAME CHANNEL as GS*/
  esp_wifi_set_promiscuous(false);                   /*Set promiscuous mode false to ensure no additional channel change is made*/
  delay(200);                                        /*Stabilization time before reading mac address !!! very important*/

  if (esp_now_init() != ESP_OK) {                    /*If Initialization is not OK*/
    Serial.println("WIFI_ROCKET_ERR...");            /*Error sent if initialization of espnow is not possible*/
    while(true) delay(1000);                         /*Stop setup here -> Wifi telemetry is flight critical for arming the rocket at a safe distance*/
  }

  Serial.print("Rocket MAC: "); Serial.println(WiFi.macAddress());     /*Rocket MAC address is printed for debug and configuration purposes*/
   
  esp_now_peer_info_t peerInfo = {};                                   /*Create peerInfo array*/
  memcpy(peerInfo.peer_addr, gsAddress, 6);                            /*6 bytes MAC address*/
  peerInfo.channel = 1;                                                /*ame as GS channel*/
  peerInfo.encrypt = false;                                            /*Do not encrypt*/

  esp_now_register_send_cb(onDataSent);       /*???*/
  esp_now_register_recv_cb(onDataRecv);       /*???*/

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {                         /*If peer is not added send GS error to communicate link error*/
    Serial.println("WIFI_CONNECTION_GS_ERR...");                       /*Send error via serial for debugging purposes*/
    while(true) delay(1000);                                           /*Stop setup here -> Wifi telemetry is flight critical for arming the rocket at a safe distance*/ 
  } 
  else {
     Serial.println("WIFI_ON...");                                     /*If link is active send confirmation via serial*/
  }

  /*----------Headers----------*/
  Serial.print("om"); Serial.print("\t");                              /*Send headers via serial*/               
  Serial.print("fq"); Serial.print("\t");   
  Serial.print("sd"); Serial.print("\t");                      
  Serial.print("ts"); Serial.print("\t");                      
  Serial.print("ax"); Serial.print("\t");                         
  Serial.print("ay"); Serial.print("\t");                         
  Serial.print("az"); Serial.print("\t");                         
  Serial.print("gx"); Serial.print("\t");                         
  Serial.print("gy"); Serial.print("\t");                         
  Serial.print("gz"); Serial.print("\t");                                                
  Serial.print("p"); Serial.print("\t");                         
  Serial.print("y"); Serial.print("\t");                         
  Serial.print("r"); Serial.print("\t");                                                                       
  Serial.print("al"); Serial.print("\t");
  Serial.print("vz"); Serial.print("\t");
  Serial.print("S1_pos"); Serial.println("\t");

  /*----------Send_Headers_via_WiFi----------*/
  // not done for GS purposes (easier decoding script)

  



     /*----------Serial_Printing----------*/
      if (ser == true) {
        Serial.print(om); Serial.print("\t");                                 // 1 Op Mode for state machine status
        Serial.print(fq); Serial.print("\t");                                 // 2 Frequency Hz for correct loop speed check
        Serial.print(sd); Serial.print("\t");                                 // 3 Sd Mode for logging check (not needed in SD logging file)
        Serial.print(ts, 4); Serial.print("\t");                              // 4 Timestamp for time variable
        Serial.print(ax); Serial.print("\t");                                 // 5 X-axis accelerometer data
        Serial.print(ay); Serial.print("\t");                                 // 6 Y-axis accelerometer data
        Serial.print(az); Serial.print("\t");                                 // 7 Z-axis accelerometer data
        Serial.print(gx); Serial.print("\t");                                 // 8 X-axis gyroscope data
        Serial.print(gy); Serial.print("\t");                                 // 9 Y-axis gyroscope data
        Serial.print(gz); Serial.print("\t");                                 // 10 Z-axis gyroscope data
        Serial.print(p); Serial.print("\t");                                  // 11 Pitch angle
        Serial.print(y); Serial.print("\t");                                  // 12 Yaw angle
        Serial.print(r); Serial.print("\t");                                  // 13 Roll angle
        Serial.print(al); Serial.print("\t");                                 // 14 Altitude
        Serial.print(vz); Serial.print("\t");                                 // 15 Vz
        Serial.print(float(S1_pos-1500)*90.00/1000.00); Serial.println("\t"); // 16 S1_pos
      }


      /*----------Remote_Commands----------*/
      if (Serial.available()) {                                /*Check if serial connection is available*/
        cmd_ser = Serial.readStringUntil('\n');                /*Read serial command*/
        cmd_ser.trim();                                        /*Trim serial command to remove extra white spaces*/
      }

      if (cmd_wifi == "REB" || cmd_ser == "REB") {             /*If reboot command is detected*/
        cmd_wifi = "";                                         /*Resets the string for incoming commands*/
        cmd_ser = "";                                          /*Resets the string for incoming commands*/

        fq_fixed = 100;                                        /*Frequency set to 20Hz as standard*/
        dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;      /*Set dt_fixed accordingly*/

        Serial.println("SYSTEM_REBOOTING...");
        data.println("SYSTEM_REBOOTING...");

        ESP.restart();                                         /*Restart the esp32 lastly*/         
      }

      if (om == 0 && (cmd_wifi == "ARM" || cmd_ser == "ARM")) {   /*Manual arming of the systems only possible if we are in system boot mode ; no need for arming if already disarmed*/
        om = 1;                                                   /*Set operative mode to 1 = armed*/
        fq_fixed = 100;                                           /*Set frequency to normal frequency*/
        dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;         /*Set dt fixed accordingly*/

        Serial.println("SYSTEM_ARMED...");
        data.println("SYSTEM_ARMED...");

        digitalWrite(BUZ, HIGH); delay(200); digitalWrite(BUZ, LOW);

        cmd_wifi = "";                                            /*Resets the string for new incoming commands*/
        cmd_ser = "";                                             /*Resets the string for new incoming commands*/
      }

      if (om == 1 && (cmd_ser == "PAR" || cmd_wifi == "PAR")) {   /*If operative mode is 1=armed and either from wifi or usb serial command parachute is received*/
        om = 3;                                                   /*Set operative mode to 3=parachute*/
        Serial.println("<PYRO_1> DEPLOYED MANUALLY...");
        digitalWrite(PY_1, HIGH);                                 /*Deploy first pyro charge*/
        delay(1000);
        digitalWrite(PY_1, LOW);                                  /*Deploy second pyro charge*/

        digitalWrite(BUZ, HIGH); delay(200); digitalWrite(BUZ, LOW); delay(200);
        digitalWrite(BUZ, HIGH); delay(200); digitalWrite(BUZ, LOW); delay(200);
        digitalWrite(BUZ, HIGH); delay(200); digitalWrite(BUZ, LOW); delay(200);
        digitalWrite(BUZ, HIGH); delay(200); digitalWrite(BUZ, LOW); delay(200);

        fq_fixed = 100;                                       /*Frequency set to 100Hz as standard*/
        dt_fixed = (1000000UL + fq_fixed / 2) / fq_fixed;     /*dt fixed computed accordingly*/

        cmd_wifi = "";                                        /*Resets the string for new incoming commands*/
        cmd_ser = "";                                         /*Resets the string*/
      }

      /*TELEMETRY PACKET*/

      typedef struct __attribute__((packed)) {     /*Struct to store different types of data together ; packing removes padding bytes*/
    uint32_t packet_id;
    uint8_t om;
    float fq;
    uint8_t sd;
    float ts;
    float ax;
    float ay;
    float az;
    float gx;
    float gy;
    float gz;
    float p;
    float y;
    float r;
    float al;
    float vz;
    uint8_t S1_pos;
} TelemetryPacket;                           /*This closes the struct and creates the type name*/

            /*----------Telemetry_Data----------*/
      TelemetryPacket pkt;                     /*Assign variables to telemetry packets*/
      pkt.packet_id++;
      pkt.om = om;
      pkt.fq = fq;
      pkt.sd = sd;
      pkt.ts = ts;
      pkt.ax = ax;
      pkt.ay = ay;
      pkt.az = az;
      pkt.gx = gx;
      pkt.gy = gy;
      pkt.gz = gz;
      pkt.p = p;
      pkt.y = y;
      pkt.r = r;
      pkt.al = al;
      pkt.vz = vz;
      pkt.S1_pos = S1_pos;


      // Telemetry sending

            unsigned long now = micros();                            /*Calculates elapsed time now */
      if (now - lastTelemetry >= telemetryInterval) {          /*Telemetry interval different from other working frequencies*/
        lastTelemetry = now;                                   /*Recursive statement for telemetry*/
        esp_now_send(gsAddress, (uint8_t*)&pkt, sizeof(pkt));  /*Send telemetry packet*/
      }