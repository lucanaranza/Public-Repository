/*GS ROLL CONTROL 1 SERVO*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

/* ================= CONFIG ================= */

// Rocket MAC (CHANGE THIS)
uint8_t rocketAddress[6] = {0xC0, 0x5D, 0x89, 0xB1, 0xAB, 0x20};
#define ESPNOW_CHANNEL 1
#define CMD_MAX_LEN 5

/* ================= DATA ================= */

typedef struct __attribute__((packed)) {
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
} TelemetryPacket;

TelemetryPacket rxTelemetry;
char commandBuffer[CMD_MAX_LEN];

/* ================= FLAGS ================= */

volatile bool canSend = true;  // Flag to track if last command was sent

/* ================= CALLBACKS ================= */

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("CMD SEND: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");

    // Allow next command to be sent
    canSend = true;
}

void onDataRecv(const uint8_t *mac,
                const uint8_t *incomingData,
                int len) {

    if (len != sizeof(TelemetryPacket)) {
        Serial.println("RX SIZE ERROR");
        return;
    }

    memcpy(&rxTelemetry, incomingData, sizeof(rxTelemetry));

    // Print telemetry nicely
    // Using tab-separated values
    Serial.printf("%u\t%u\t%.2f\t%u\t%.3f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%.2f\t%u\n",
      rxTelemetry.packet_id,
      rxTelemetry.om,
      rxTelemetry.fq,
      rxTelemetry.sd,
      rxTelemetry.ts,
      rxTelemetry.ax, rxTelemetry.ay, rxTelemetry.az,
      rxTelemetry.gx, rxTelemetry.gy, rxTelemetry.gz,
      rxTelemetry.p, rxTelemetry.y, rxTelemetry.r,
      rxTelemetry.al,
      rxTelemetry.vz,
      rxTelemetry.S1_pos
    );

}

/* ================= SETUP ================= */

void setup() {
    Serial.begin(500000);
    delay(300);

    WiFi.mode(WIFI_STA);
    delay(200);

    // Force channel for ESP-NOW
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    delay(200);

    // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW INIT FAIL");
        while (true);
    }

    Serial.print("GS MAC: ");
    Serial.println(WiFi.macAddress());

    // Add rocket as peer
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, rocketAddress, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("PEER ADD FAIL");
        while (true);
    }

    Serial.println("GROUND STATION READY");
}

/* ================= LOOP ================= */

void loop() {
    // Only read/send if last command was confirmed
    if (Serial.available() && canSend) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.length() > 0) {
            // Send command over ESP-NOW
            if (esp_now_send(rocketAddress,
                             (uint8_t*)cmd.c_str(),
                             cmd.length() + 1) == ESP_OK) {
                Serial.print("COMMAND QUEUED: ");
                Serial.println(cmd);
                canSend = false;  // Wait for onDataSent callback
            } else {
                Serial.print("COMMAND SEND FAIL (queue full?): ");
                Serial.println(cmd);
            }
        }
    }
}
