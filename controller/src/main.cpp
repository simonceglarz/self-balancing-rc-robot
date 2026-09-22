#include <Arduino.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <WiFi.h>

// =================== Pin map ===================
#define VRX_PIN 34   // ADC1 — input-only pin, fine for analog read
#define VRY_PIN 35   // ADC1 — input-only pin, fine for analog read
#define SW_PIN  32   // digital, has internal pull-up available

// =================== Robot's ESP-NOW address ===================
// Fill this in with the ROBOT's AP MAC address, not its station MAC —
// the robot runs WiFi.softAP(), so the address you want is what
// WiFi.softAPmacAddress() prints on ITS serial monitor at boot
// (main.cpp now prints this — look for "Robot AP MAC:" in its Serial
// output). Format: 6 bytes, comma-separated, 0x-prefixed.
uint8_t robotMac[] = {0xB0, 0xCB, 0xD8, 0xCD, 0x8B, 0xA1};

// Must match the robot's AP channel (see WIFI_CHANNEL in main.cpp).
// ESP-NOW packets are only received if sender and receiver agree on
// the channel — this is the single most common reason ESP-NOW "just
// doesn't work" on a first attempt.
const int WIFI_CHANNEL = 1;

// =================== Joystick calibration ===================
int centerX = 2048, centerY = 2048;   // overwritten by auto-calibration in setup()
const int DEADZONE = 150;             // ADC counts around center treated as exactly 0 —
                                       // cheap joysticks don't rest dead-center or return
                                       // to the exact same spot every time
const int ADC_MAX = 4095;             // ESP32 ADC is 12-bit

// Packet layout MUST match the robot's JoyPacket struct exactly —
// ESP-NOW just copies raw bytes, there's no schema negotiation.
typedef struct {
    float x;   // -1 (left) .. +1 (right)
    float y;   // -1 (back) .. +1 (forward)
} JoyPacket;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Optional — uncomment while debugging link issues, noisy otherwise
    // Serial.println(status == ESP_NOW_SEND_SUCCESS ? "sent OK" : "send FAILED");
}

// Maps a raw ADC reading to -1..1, treating DEADZONE counts around the
// calibrated center as exactly 0, then scaling the remaining travel on
// each side independently (cheap joysticks are rarely symmetric around
// center).
float normalize(int raw, int center) {
    int delta = raw - center;
    if (abs(delta) < DEADZONE) return 0.0;
    if (delta > 0) return (float)(delta - DEADZONE) / (float)(ADC_MAX - center - DEADZONE);
    else           return (float)(delta + DEADZONE) / (float)(center - DEADZONE);
}

void setup() {
    Serial.begin(115200);
    pinMode(SW_PIN, INPUT_PULLUP);

    WiFi.mode(WIFI_STA);
    // Lock to the robot's channel BEFORE esp_now_init() — WiFi.mode()
    // alone doesn't guarantee a channel if this board isn't also
    // joining an AP.
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW init failed");
        while (1) delay(1000);
    }
    esp_now_register_send_cb(onDataSent);

    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, robotMac, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer — check robotMac[] is correct");
        while (1) delay(1000);
    }

    // Auto-center calibration: assumes the stick is at rest (untouched)
    // for this half-second at power-on. Re-flash or power-cycle if you
    // accidentally touched it during this window and centering feels off.
    Serial.println("Calibrating center — don't touch the stick...");
    long sumX = 0, sumY = 0;
    const int CAL_SAMPLES = 50;
    for (int i = 0; i < CAL_SAMPLES; i++) {
        sumX += analogRead(VRX_PIN);
        sumY += analogRead(VRY_PIN);
        delay(10);
    }
    centerX = sumX / CAL_SAMPLES;
    centerY = sumY / CAL_SAMPLES;
    Serial.print("Center: "); Serial.print(centerX); Serial.print(", "); Serial.println(centerY);
    Serial.println("Ready.");
}

void loop() {
    int rawX = analogRead(VRX_PIN);
    int rawY = analogRead(VRY_PIN);

    JoyPacket packet;
    packet.x = constrain(normalize(rawX, centerX), -1.0, 1.0);
    packet.y = constrain(normalize(rawY, centerY), -1.0, 1.0);

    esp_now_send(robotMac, (uint8_t *)&packet, sizeof(packet));

    delay(50);   // ~20Hz — matches the webpage joystick's own send throttle
}
