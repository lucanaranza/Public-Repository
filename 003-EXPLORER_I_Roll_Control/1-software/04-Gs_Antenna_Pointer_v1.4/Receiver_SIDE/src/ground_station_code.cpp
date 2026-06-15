/*ESP32 WIFI / ANTENNA POINTER*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>
#include <Servo.h>   /*Servo control library*/

/* ================= CONFIG ================= */
uint8_t rocketAddress[6] = {0xC0, 0x5D, 0x89, 0xB1, 0xAB, 0x20};    // Rocket MAC (DON'T CHANGE THIS !!!)
uint32_t lastPacketTime = 0;
#define ESPNOW_CHANNEL 1
#define CMD_MAX_LEN 4
Servo azServo;
Servo elServo;
#define AZ_SERVO_PIN 4
#define EL_SERVO_PIN 5
/* ================= SERVO LIMITS ================= */
float constrainAz(float az)
{
    return constrain(az, 0.0f, 180.0f);
}
float constrainEl(float el)
{
    return constrain(el, 0.0f, 90.0f);
}
/* ================= SMOOTH MOTION ================= */
float currentAz = 0.0;
float currentEl = 0.0;
float targetAz = 0.0;
float targetEl = 0.0;
const float SERVO_STEP_DEG = 1.5;   /* CHANGED: 1.5° per 20ms */
const uint32_t SERVO_UPDATE_MS = 20;
uint32_t lastServoUpdate = 0;
/* ================= MODE SYSTEM ================= */
enum Mode {
    MANUAL_ALL = 0,
    AUTO_EL_MANUAL_AZ = 1
};
Mode mode = MANUAL_ALL;
/* ================= GEOMETRY INPUTS ================= */
float distance_m = 10.0;
float gs_height_m = 1.0;
float manual_az = 90.0;
float manual_el = 0.0;
/* ================= TELEMETRY ================= */
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
    float S1_pos;
    float tp;
    float x_pos;
    float y_pos;
    uint8_t sat;
    float v_bat;
    float heading;
} TelemetryPacket;

TelemetryPacket rxTelemetry;
/* ================= FLAGS ================= */
volatile bool canSend = true;

/* ================= FUNCTION PROTOTYPE ================= */
void updateServoSmooth();

/* ================= UTIL ================= */
float computeElevation(float distance, float rocketAlt, float gsAlt)
{
    float h = rocketAlt + 0.3f;
    return atan(h / distance) * 180.0 / PI;
}

/* ================= CALLBACKS ================= */
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status)
{
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "CMD OK" : "CMD FAIL");
    canSend = true;
}

void onDataRecv(const uint8_t *mac,
                const uint8_t *incomingData,
                int len)
{
    if (len != sizeof(TelemetryPacket)) {
        Serial.println("RX SIZE ERROR");
        return;
    }
    memcpy(&rxTelemetry, incomingData, sizeof(rxTelemetry));

    float az = manual_az;
    float el = manual_el;

    if (mode == AUTO_EL_MANUAL_AZ) {
        el = computeElevation(distance_m, rxTelemetry.al, gs_height_m);
    }

    targetAz = constrainAz(az);
    targetEl = constrainEl(el);

    lastPacketTime = millis();

    // ---------------------------------------------
    // FULL STRUCTURED OUTPUT (NEW FORMAT)
    // ---------------------------------------------
    Serial.printf(
        "%.2f,%.2f,%.2f,%d,"    // az el dist mode
        "%lu,%u,%.2f,%u,%.3f,"  // packetid om fq sd ts
        "%.3f,%.3f,%.3f,"       //ax ay az 
        "%.3f,%.3f,%.3f,"       //gx gy gz
        "%.2f,%.2f,%.2f,"       // p y r
        "%.2f,%.2f,%.3f,"       // al vz s1pos
        "%.3f,%.7f,%.7f,"       // tp xpos ypos 
        "%d,%.3f,%.3f\n",       // sat vbat hdg

        // GS tracking info
        targetAz,                // AZ
        targetEl,                // EL
        distance_m,              // DIST
        (int)mode,               // MODE

        // telemetry packet (same order as struct)
        rxTelemetry.packet_id,   // packet_id
        rxTelemetry.om,          // om
        rxTelemetry.fq,          // fq
        rxTelemetry.sd,          // sd
        rxTelemetry.ts,          // ts

        rxTelemetry.ax,          // ax
        rxTelemetry.ay,          // ay
        rxTelemetry.az,          // az

        rxTelemetry.gx,          // gx
        rxTelemetry.gy,          // gy
        rxTelemetry.gz,          // gz

        rxTelemetry.p,           // p
        rxTelemetry.y,           // y
        rxTelemetry.r,           // r

        rxTelemetry.al,          // al
        rxTelemetry.vz,          // vz
        rxTelemetry.S1_pos,      // S1_pos

        rxTelemetry.tp,           // tp
        rxTelemetry.x_pos,        // X pos
        rxTelemetry.y_pos,        // Y pos
        rxTelemetry.sat,          // sat
        rxTelemetry.v_bat,        // v_bat
        rxTelemetry.heading       // hdg
    );
}

/* ================= SETUP ================= */

void setup()
{
    Serial.begin(500000);
    delay(300);
    Serial.println("GS Initializing...");

    WiFi.mode(WIFI_STA);

    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(ESPNOW_CHANNEL, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW INIT FAIL");
        while (true);
    }

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, rocketAddress, 6);
    peerInfo.channel = ESPNOW_CHANNEL;
    peerInfo.encrypt = false;

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("PEER FAIL");
        while (true);
    }

    Serial.println("GS READY!");

    azServo.attach(AZ_SERVO_PIN);
    elServo.attach(EL_SERVO_PIN);

    delay(500);

    azServo.write(90);
    elServo.write(0);

    currentAz = 90;
    currentEl = 0;
    targetAz = 90;
    targetEl = 0;
    Serial.println("SERVO READY!");
}

/* ================= LOOP ================= */

void loop()
{
    updateServoSmooth();

    if (Serial.available() && canSend)
    {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd == "MAN") {
            mode = MANUAL_ALL;
            Serial.println("MODE MANUAL");
            return;
        }

        if (cmd == "AUTO") {
            mode = AUTO_EL_MANUAL_AZ;
            Serial.println("MODE AUTO EL / MAN AZ");
            return;
        }

        if (cmd.startsWith("AZ")) {
            manual_az = cmd.substring(2).toFloat();

            if (mode == MANUAL_ALL)
                targetAz = constrainAz(manual_az);

            Serial.print("AZ SET ");
            Serial.println(manual_az);
            return;
        }

        if (cmd.startsWith("EL")) {
            manual_el = cmd.substring(2).toFloat();

            if (mode == MANUAL_ALL)
                targetEl = constrainEl(manual_el);

            Serial.print("EL SET ");
            Serial.println(manual_el);
            return;
        }

        if (cmd.startsWith("DIST")) {
            distance_m = cmd.substring(4).toFloat();
            Serial.print("DIST SET");
            Serial.println(distance_m);
            return;
        }

        esp_now_send(rocketAddress,
                     (uint8_t*)cmd.c_str(),
                     cmd.length() + 1);

        canSend = false;
    }
}

/* ================= SERVO SMOOTH CONTROL ================= */

void updateServoSmooth()
{
    if (millis() - lastServoUpdate < SERVO_UPDATE_MS)
        return;

    lastServoUpdate = millis();

    // AZ
    if (currentAz < targetAz)
        currentAz = min(currentAz + SERVO_STEP_DEG, targetAz);
    else if (currentAz > targetAz)
        currentAz = max(currentAz - SERVO_STEP_DEG, targetAz);

    // EL
    if (currentEl < targetEl)
        currentEl = min(currentEl + SERVO_STEP_DEG, targetEl);
    else if (currentEl > targetEl)
        currentEl = max(currentEl - SERVO_STEP_DEG, targetEl);

    azServo.write((int)currentAz);
    elServo.write((int)currentEl);
}