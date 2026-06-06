/*ESP32 RECEIVE GROUND STATION CODE*/
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Replace with your rocket ESP32 MAC address
uint8_t rocketAddress[] = {0x30, 0xAE, 0xA4, 0xC9, 0xB2, 0xD8};

char data_telemetry[20];  // WiFi packet size (more than enough)
char command[5];          // size of packet to send ("ARM" "RES" "PAR")s

void setup() {
  /*Robust Startup*/
  delay(200);
  Serial.begin(115200); 
  delay(200);
  Serial.println("STARTUP_SEQUENCE_GROUND-STATION...");
  delay(200);
  WiFi.mode(WIFI_STA);
  /*Initialize WiFi connection*/
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("WIFI_ERR");
    return;
  }
  sp_now_peer_info_t peerInfo = {};                //Add receiver as a peer
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("WIFI_ERR");
    return;
  }
  Serial.println("WIFI_ON");
}
void loop() {
  //check for serial command
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');

    if (cmd.length() > 0) {
      snprintf(command, sizeof(command), cmd);
      if (ok) {
        Serial.print("COMMAND SENT: ");
        Serial.println(cmd);
      } else {
        Serial.println("FAILED TO SEND COMMAND !");
      }
    }
  }

  //read telemetry continuosly
  msg[strlen(msg)] = '\0';  // null-terminate just in case
  memset(data_telemetry, 0, sizeof(data_telemetry));
    radio.read(&data_telemetry, sizeof(data_telemetry));
    String data_telemetry_string = String(data_telemetry);
    data_telemetry_string.trim();
    Serial.print("Telemetry: ");
    Serial.println(data_telemetry_string);
}

