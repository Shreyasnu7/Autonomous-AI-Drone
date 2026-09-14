/**
 * ULTRA DRONE - ESP32 SENSOR HUB (UDP WIFI EDITION)
 * ------------------------------
 * Role: 
 *  1. Read 4x VL53L1X ToF Sensors (Omnidirectional)
 *  2. Read MPU6050 Gyro
 *  3. Control Gimbal Servos & LEDs
 *  4. Send JSON Telemetry to Radxa via WiFi UDP
 *  5. Receive LED/Gimbal Commands from Radxa via WiFi UDP
 * 
 * WIRING (Verified):
 *  - WiFi: 2.4GHz (Seema)
 *  - Radxa IP: 192.168.0.11:8888
 *  - TOF XSHUT: D23, D26, D15, D5
 *  - GIMBAL: D25, D32
 *  - LED: D33
 *  - I2C: D21 (SDA), D22 (SCL)
 */

#include <Wire.h>
#include <VL53L1X.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_NeoPixel.h>
#include <ArduinoJson.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// --- WIFI CONFIGURATION (Multi-network auto-fallback) ---
// ESP32 tries each WiFi network in order until one connects
// Network 1: Cubie's own DroneAP hotspot (primary for drone flight)
// Network 2: Phone hotspot (for development/testing)
// Network 3: 4G dongle WiFi (alternative)

struct WiFiConfig {
    const char* ssid;
    const char* password;
    IPAddress radxa_ip;
};

WiFiConfig networks[] = {
    {"Excitel_2.4G_188909681", "8003390975", IPAddress(192, 168, 1, 14)}, // Home WiFi - PRIMARY (Radxa can ONLY use this; ESP joins here too)
    {"DroneAP",     "dronepass123",  IPAddress(10, 42, 0, 1)},      // Cubie's own AP (future flight)
    {"S21 ultra",   "22219413",      IPAddress(10, 195, 147, 217)}, // Phone hotspot (Radxa can't see it - last resort)
    {"4G-UFI-224D", "1234567890",    IPAddress(192, 168, 100, 98)}  // 4G dongle WiFi
};
const int NUM_NETWORKS = 4;

WiFiUDP udp;
IPAddress radxa_ip(10, 42, 0, 1);  // Will be updated when connected
uint16_t radxa_port = 8888;

// --- PIN CONFIGURATION ---
#define LED_PIN 33
#define NUM_LEDS 32
#define LEDS_PER_ARM 8
#define BRIGHTNESS 70   // Reduced from 150: halves LED current draw (single power module)

#define PIN_GIM_PITCH 25
#define PIN_GIM_YAW 32
#define PIN_ONBOARD 2

// TOF SENSORS
#define PIN_XSHUT1 23
#define PIN_XSHUT2 26
#define PIN_XSHUT3 15
#define PIN_XSHUT4 5

// PCA BUFFER CONFIG
#define PCA_ADDR 0x70
#define SHARED_PCA_CHANNEL 2 

// --- GLOBAL OBJECTS ---
Adafruit_NeoPixel strip(NUM_LEDS, LED_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_MPU6050 mpu;
VL53L1X tof1, tof2, tof3, tof4;

// --- STATE VARIABLES ---
bool bus_alive = false;
bool mpu_active = false;
bool t1=false, t2=false, t3=false, t4=false; // Init success flags
int init_stage = 0;
unsigned long init_timer = 0;
bool is_armed = false;
unsigned long last_wifi_check = 0;
int connected_network_idx = 0; // Which network[] entry we're on

float g_ax, g_ay, g_az;
float g_gx, g_gy, g_gz;
int CurrentPitch = 90;
int CurrentYaw = 90;
int dist1=-1, dist2=-1, dist3=-1, dist4=-1;

// --- LIVE I2C SCANNER (diagnostic: shows what's actually on the bus) ---
String i2c_found = "scanning";
unsigned long last_i2c_scan = 0;
void scanI2C() {
    String found = "";
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            char buf[8]; sprintf(buf, "0x%02X ", addr);
            found += buf;
        }
    }
    i2c_found = (found.length() > 0) ? found : "NONE";
}

// --- SERVO LOGIC REMOVED (backup: esp32_firmware_BACKUP_with_servos_2026-06-21.ino) ---
bool stabilize_active = false;
bool manual_led_override = false;
const float PITCH_GAIN = 1.2; 
const float YAW_GAIN = 1.0;
const int PITCH_DIR = -1; 
const int YAW_DIR = 1;

// --- LED DEFINE ---
uint32_t C_RED, C_GREEN, C_BLUE, C_WHITE, C_ORANGE, C_CYAN, C_PURPLE, C_GOLD, C_BLACK;

enum FlightMode {
    M_BOOT = 0, 
    M_IDLE = 1,          // Gaming RGB (Rainbow Cylon/Breathe)
    M_ARMED = 2,         // Solid/Pulse Gold/White (Ready)
    M_ASCEND = 10,       // Audi Wipe Up
    M_DESCEND = 11,      // Audi Wipe Down
    M_FWD = 12,          // Front Arms White, Rear Red
    M_BACK = 13,         // Rear Arms Flash Red
    M_LEFT = 14,         // Left Side wipe
    M_RIGHT = 15,        // Right Side wipe
    M_HOVER = 16,        // Pulse White/Blue
    M_RTH = 20,          // Return to Home (Strobe Purple)
    M_AI_MODE = 22,      // Matrix Rain (Green)
    M_BATTERY_LOW = 90,  // Flash Red Critical
    M_ERROR = 99,
    M_DEMO_CYCLE = 100
};

FlightMode currentMode = M_BOOT;
unsigned long led_timer = 0;
int led_tick = 0;

// --- HELPER: SELECT PCA CHANNEL ---
void selectChannel(uint8_t ch) {
  Wire.beginTransmission(PCA_ADDR);
  Wire.write(1 << ch); 
  Wire.endTransmission();
}

// --- HELPER: INIT SINGLE TOF (POLOLU) ---
bool initTOF(VL53L1X &sensor, int pin, uint8_t new_addr) {
    selectChannel(SHARED_PCA_CHANNEL); // Ensure Channel is Open
    digitalWrite(pin, HIGH);
    delay(150); 
    sensor.setTimeout(500);
    
    // Init at default 0x29
    if (!sensor.init()) { 
        Serial.printf("❌ TOF Pin %d: FAILED INIT\n", pin);
        return false;
    }
    
    sensor.setAddress(new_addr);
    Serial.printf("✅ TOF Pin %d: FOUND -> 0x%02X\n", pin, new_addr);
    
    // START RANGING
    sensor.setDistanceMode(VL53L1X::Long);
    delay(50);
   sensor.setMeasurementTimingBudget(100000); // 100ms for stability
    delay(100);
    sensor.startContinuous(100);
    delay(150);
    return true;
}

// --- HELPERS (LED) ---
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  if(WheelPos < 170) { WheelPos -= 85; return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3); }
  WheelPos -= 170; return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
uint32_t Color(uint8_t r, uint8_t g, uint8_t b) { return strip.Color(r,g,b); }
uint32_t Fade(uint32_t c1, uint32_t c2, int speed) {
   float factor = (sin(millis() / (float)speed) + 1.0) / 2.0; 
   uint8_t r = (uint8_t)((c1 >> 16 & 0xFF) * factor + (c2 >> 16 & 0xFF) * (1-factor));
   uint8_t g = (uint8_t)((c1 >> 8 & 0xFF) * factor + (c2 >> 8 & 0xFF) * (1-factor));
   uint8_t b = (uint8_t)((c1 & 0xFF) * factor + (c2 & 0xFF) * (1-factor));
   return strip.Color(r,g,b);
}

void setArm(int arm, uint32_t c) {
    int start = arm * LEDS_PER_ARM;
    for(int i=0; i<LEDS_PER_ARM; i++) strip.setPixelColor(start+i, c);
}

void runAnimation(int arm, int type, uint32_t c1, uint32_t c2, int speed_delay) {
    int start = arm * LEDS_PER_ARM;
    switch(type) {
        case 0: setArm(arm, 0); break; // OFF
        case 1: setArm(arm, c1); break; // SOLID
        case 2: // SCANNER (Cylon)
             {
                 int pos = (millis() / speed_delay) % (LEDS_PER_ARM*2 - 2);
                 if (pos >= LEDS_PER_ARM) pos = (LEDS_PER_ARM*2 - 2) - pos;
                 strip.fill(c2, start, LEDS_PER_ARM);
                 strip.setPixelColor(start + pos, c1);
             } break;
        case 4: // MATRIX RAIN
             {
                 int key = (millis() / 50 + arm*3) % 20;
                 if (key < LEDS_PER_ARM) strip.setPixelColor(start + key, c1);
                 // Decay
                 for(int i=0; i<LEDS_PER_ARM; i++) {
                     uint32_t pixel = strip.getPixelColor(start+i);
                     uint8_t r=(pixel>>16), g=(pixel>>8), b=pixel;
                     if(r>10)r-=10; if(g>10)g-=10; if(b>10)b-=10;
                     strip.setPixelColor(start+i, r, g, b);
                 }
             } break;
        case 5: { // CHASE UP
                 int offset = (millis() / speed_delay) % LEDS_PER_ARM;
                 for(int i=0; i<LEDS_PER_ARM; i++) {
                      int dist = (i + offset) % LEDS_PER_ARM;
                      if(dist < 3) strip.setPixelColor(start + i, c1);
                      else strip.setPixelColor(start + i, c2);
                 }
             } break;
        case 6: { // CHASE DOWN
                 int offset = (millis() / speed_delay) % LEDS_PER_ARM;
                 for(int i=0; i<LEDS_PER_ARM; i++) {
                      int dist = ((LEDS_PER_ARM - 1 - i) + offset) % LEDS_PER_ARM;
                      if(dist < 3) strip.setPixelColor(start + i, c1);
                      else strip.setPixelColor(start + i, c2);
                 }
             } break;
        case 8: { // AUDI WIPE
                 int step = (millis() / speed_delay) % (LEDS_PER_ARM + 6); 
                 strip.fill(c2, start, LEDS_PER_ARM);
                 if(step < LEDS_PER_ARM) for(int i=0; i<=step; i++) strip.setPixelColor(start+i, c1);
             } break;
    }
}

// --- LED LOGIC ---
void runLEDs() {
    if (millis() - led_timer < 20) return;
    led_timer = millis();
    led_tick++;

    // AUTO-TRANSITION: Boot -> Idle after 2 seconds
    if (currentMode == M_BOOT && millis() > 2000) currentMode = M_IDLE;

    // 1. AUTO-FLIGHT INDICATORS (Only if ARMED and not manually overridden)
    if (is_armed && !manual_led_override && mpu_active) {
         if (g_az < 8.0) currentMode = M_DESCEND;
         else if (g_az > 12.0) currentMode = M_ASCEND;
         else if (g_ax > 3.0) currentMode = M_BACK;
         else if (g_ax < -3.0) currentMode = M_FWD;
         else if (g_ay > 3.0) currentMode = M_LEFT;
         else if (g_ay < -3.0) currentMode = M_RIGHT;
         else currentMode = M_HOVER;
    }
    
    // 2. BUS FAILURE OVERRIDE
    if (!bus_alive && !manual_led_override) currentMode = M_ERROR;

    // 3. RENDER ANIMATIONS
    switch(currentMode) {
        case M_BOOT:      
            for(int i=0; i<4; i++) runAnimation(i, 6, C_BLUE, 0, 40); 
            break;
        case M_IDLE: // "Gaming RGB"
            { 
               uint16_t j = (millis()/20)%256; 
               for(int i=0; i<32; i++) strip.setPixelColor(i, Wheel((i*2+j)&255)); 
            } 
            break;
        case M_ARMED: // "Cool Lighting"
            { 
               uint32_t c = Fade(C_GOLD, C_WHITE, 1000); 
               for(int i=0; i<32; i++) strip.setPixelColor(i, c); 
            } 
            break;
        case M_ASCEND: for(int i=0; i<4; i++) runAnimation(i, 5, C_CYAN, 0, 30); break;
        case M_DESCEND: for(int i=0; i<4; i++) runAnimation(i, 6, C_PURPLE, 0, 30); break;
        case M_FWD: 
             for(int i=0; i<4; i++) {
                 if(i==0 || i==1) setArm(i, C_WHITE); 
                 else runAnimation(i, 6, C_RED, 0, 40); 
             }
             break;
        case M_BACK: 
             for(int i=0; i<4; i++) {
                 if(i==2 || i==3) setArm(i, C_RED); 
                 else runAnimation(i, 5, C_WHITE, 0, 40); 
             }
             break;
        case M_LEFT: 
             runAnimation(0, 8, C_ORANGE, 0, 30); runAnimation(2, 8, C_ORANGE, 0, 30);
             setArm(1, C_BLACK); setArm(3, C_BLACK);
             break;
        case M_RIGHT: 
             runAnimation(1, 8, C_ORANGE, 0, 30); runAnimation(3, 8, C_ORANGE, 0, 30);
             setArm(0, C_BLACK); setArm(2, C_BLACK);
             break;
        case M_HOVER: for(int i=0; i<4; i++) runAnimation(i, 1, C_BLUE, Color(0,0,50), 1000); break;
        case M_RTH: 
             {
                 bool phase = (millis()/100)%2;
                 setArm(0, phase?C_WHITE:0); setArm(1, phase?0:C_WHITE);
                 setArm(2, phase?C_WHITE:0); setArm(3, phase?0:C_WHITE);
             }
             break;
        case M_AI_MODE: for(int i=0; i<4; i++) runAnimation(i, 4, C_GREEN, 0, 0); break;
        case M_BATTERY_LOW: 
             { bool p=(millis()/150)%2; if(p) strip.fill(C_RED); else strip.fill(0); }
             break;
        case M_ERROR:
             { bool p=(millis()/250)%2; for(int i=0; i<4; i++) runAnimation(i, 1, p?C_RED:0, 0, 0); } 
             break;
        case M_DEMO_CYCLE: 
             {
                 uint32_t t = millis() / 5000;
                 int mode = t % 5;
                 if(mode==0) { // Gaming
                     uint16_t j = (millis()/15)%256; for(int i=0; i<32; i++) strip.setPixelColor(i, Wheel((i*3+j)&255));
                 } else if(mode==1) { // Matrix
                     for(int i=0; i<4; i++) runAnimation(i, 4, C_GREEN, 0, 0);
                 } else if(mode==2) { // Police
                     bool phase = (millis()/100)%2;
                     setArm(0, phase?C_RED:C_BLUE); setArm(2, phase?C_RED:C_BLUE);
                     setArm(1, phase?C_BLUE:C_RED); setArm(3, phase?C_BLUE:C_RED);
                 } else { // Audi
                     for(int i=0; i<4; i++) runAnimation(i, 8, C_ORANGE, 0, 40);
                 }
             }
             break;
        default: for(int i=0; i<4; i++) runAnimation(i, 1, C_WHITE, 0, 1000); break;
    }
    strip.show();
}

// ===== SERVO HELPERS REMOVED (gimbal servos disabled for now) =====

// --- SENSOR INIT (reusable: called at boot AND for live self-healing) ---
// Returns true if the PCA9548A mux is found and sensors initialized.
bool initSensors() {
    Wire.beginTransmission(PCA_ADDR);
    if (Wire.endTransmission() != 0) {
        bus_alive = false;
        return false;            // PCA not on the bus yet — nothing to init
    }
    Serial.println("PCA9548A FOUND");
    bus_alive = true;

    t1 = initTOF(tof1, PIN_XSHUT1, 0x30);
    t2 = initTOF(tof2, PIN_XSHUT2, 0x31);
    t3 = initTOF(tof3, PIN_XSHUT3, 0x32);
    t4 = initTOF(tof4, PIN_XSHUT4, 0x33);

    selectChannel(SHARED_PCA_CHANNEL); delay(10);
    if (mpu.begin()) {
        Serial.println("MPU6050 OK");
        mpu_active = true;
        mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
        mpu.setGyroRange(MPU6050_RANGE_500_DEG);
        mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    } else {
        Serial.println("MPU6050 NOT FOUND");
        mpu_active = false;
    }
    return true;
}

// --- SETUP ---
void setup() {
    // Full speed from the start — instant boot (motors are the only heavy load,
    // and they're handled separately; LEDs + sensors start immediately).
    pinMode(PIN_XSHUT1, OUTPUT); digitalWrite(PIN_XSHUT1, LOW);
    pinMode(PIN_XSHUT2, OUTPUT); digitalWrite(PIN_XSHUT2, LOW);
    pinMode(PIN_XSHUT3, OUTPUT); digitalWrite(PIN_XSHUT3, LOW);
    pinMode(PIN_XSHUT4, OUTPUT); digitalWrite(PIN_XSHUT4, LOW);
    pinMode(PIN_ONBOARD, OUTPUT); digitalWrite(PIN_ONBOARD, HIGH);

    Serial.begin(115200);
    Serial2.begin(115200);

    // LEDs ON instantly
    strip.begin();
    strip.setBrightness(BRIGHTNESS);
    strip.fill(0); strip.show();

    // Enable ESP32 internal pull-ups on SDA/SCL — rescues a dead I2C bus if the
    // PCA/sensor board has no external pull-up resistors (common cause of [NONE]).
    pinMode(21, INPUT_PULLUP);
    pinMode(22, INPUT_PULLUP);
    Wire.begin(21, 22);
    Wire.setClock(100000);   // 100kHz — safest/most tolerant of weak pull-ups & long wires
    delay(50);

    Serial.println("\n--- System Boot (PCA9548A Mode) ---");

    // 1. INIT COLORS
    C_RED = strip.Color(255, 0, 0); C_GREEN = strip.Color(0, 255, 0); C_BLUE = strip.Color(0, 0, 255);
    C_WHITE = strip.Color(255, 255, 255); C_ORANGE = strip.Color(255, 100, 0); C_CYAN = strip.Color(0, 255, 255);
    C_PURPLE = strip.Color(200, 0, 255); C_GOLD = strip.Color(255, 200, 0); C_BLACK = 0;

    // LEDs ON instantly (all arms blue) — before WiFi so there's no wait
    for (int arm = 0; arm < 4; arm++) setArm(arm, C_BLUE);
    strip.show();

    // 2. CHECK PCA + sensors (will auto-retry in loop() if not found now)
    Serial.println("Checking sensor bus...");
    if (!initSensors()) Serial.println("PCA9548A NOT FOUND (will keep retrying)");

    // --- WIFI CONNECTION ---
    WiFi.mode(WIFI_STA);

    bool wifi_connected = false;
    for (int n = 0; n < NUM_NETWORKS && !wifi_connected; n++) {
        Serial.print("Trying WiFi: ");
        Serial.println(networks[n].ssid);
        WiFi.begin(networks[n].ssid, networks[n].password);

        int attempts = 0;
        while (WiFi.status() != WL_CONNECTED && attempts < 20) {
            delay(500);
            Serial.print(".");
            attempts++;
        }

        if (WiFi.status() == WL_CONNECTED) {
            radxa_ip = networks[n].radxa_ip;
            connected_network_idx = n;
            Serial.print("\nConnected to: ");
            Serial.println(networks[n].ssid);
            Serial.print("   ESP32 IP: ");
            Serial.println(WiFi.localIP());
            wifi_connected = true;
        } else {
            Serial.println(" Failed");
            WiFi.disconnect();
        }
    }

    if (!wifi_connected) {
        Serial.println("No WiFi! Retrying first network...");
        WiFi.begin(networks[0].ssid, networks[0].password);
        int t = 0;
        while (WiFi.status() != WL_CONNECTED && t < 40) { delay(1000); t++; }
        radxa_ip = networks[0].radxa_ip;
    }

    udp.begin(radxa_port);
    Serial.println("UDP Ready on port 8888");

    // ===== Servos removed — no gimbal init =====
    CurrentPitch = 90; CurrentYaw = 90;

    Serial.println("\n--- SETUP COMPLETE: Starting Loop ---\n");
}

const int CL_MAX = 256;
char cl_buffer[CL_MAX];
int cl_index = 0;

void checkCommand(Stream &s) {
    while (s.available()) {
        char c = s.read();
        if (cl_index >= CL_MAX - 1) cl_index = 0;
        
        if (c == '\n' || c == '\r') {
            if (cl_index > 0) {
                cl_buffer[cl_index] = '\0';
                String input = String(cl_buffer);
                cl_index = 0;
                StaticJsonDocument<512> doc;
                DeserializationError error = deserializeJson(doc, input);
                if (!error) {
                    // PROCESS NORMAL COMMANDS (From Radxa)
                    if (doc.containsKey("armed")) {
                        is_armed = doc["armed"];
                        manual_led_override = false;
                        if(is_armed) currentMode = M_ARMED;
                        else currentMode = M_IDLE;
                    }
                    if (doc.containsKey("gim")) {
                        // servos removed — just track commanded angles for telemetry
                        int p = doc["gim"][0]; int y = doc["gim"][1];
                        CurrentPitch = map(p, -90, 90, 0, 180);
                        CurrentYaw = map(y, -90, 90, 0, 180);
                    }
                    if (doc.containsKey("stab")) stabilize_active = doc["stab"];
                    if (doc.containsKey("mode")) {
                         int m = doc["mode"];
                         if(m >= 0 && m <= 100) { 
                            currentMode = (FlightMode)m;
                            manual_led_override = true;
                         }
                    }
                }
            }
        } else {
            cl_buffer[cl_index++] = c;
        }
    }
}

unsigned long last_telem = 0;

void loop() {
    runLEDs();
    checkCommand(Serial);

    // SELF-HEALING CHECKER: every 2s while the bus is down, scan I2C and try to
    // (re)initialize sensors. The moment the PCA wiring is good, sensors come up
    // live and the red error LEDs clear themselves — no reboot needed.
    if (!bus_alive && millis() - last_i2c_scan > 2000) {
        last_i2c_scan = millis();
        scanI2C();          // diagnostic: list what's actually on the bus
        if (initSensors()) {
            Serial.println("✅ Sensor bus RECOVERED — sensors online!");
            currentMode = M_IDLE;   // leave error state
        }
    }

    // WiFi reconnection check (every 5 seconds)
    if (millis() - last_wifi_check > 5000) {
        last_wifi_check = millis();
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("⚠️ WiFi lost! Reconnecting...");
            WiFi.disconnect();
            delay(100);
            // Try all networks in order
            for (int n = 0; n < NUM_NETWORKS; n++) {
                WiFi.begin(networks[n].ssid, networks[n].password);
                int attempts = 0;
                while (WiFi.status() != WL_CONNECTED && attempts < 10) {
                    delay(250);
                    attempts++;
                }
                if (WiFi.status() == WL_CONNECTED) {
                    radxa_ip = networks[n].radxa_ip;
                    connected_network_idx = n;
                    Serial.print("✅ Reconnected to: "); Serial.println(networks[n].ssid);
                    break;
                }
                WiFi.disconnect();
            }
        }
    }

    // MA-19 FIX: Also check for commands arriving via UDP (bridge sends via WiFi UDP)
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
        char udpBuf[256];
        int len = udp.read(udpBuf, sizeof(udpBuf) - 1);
        if (len > 0) {
            udpBuf[len] = '\0';
            // Parse the UDP command the same way as Serial commands
            String udpCmd = String(udpBuf);
            udpCmd.trim();
            if (udpCmd.startsWith("{")) {
                StaticJsonDocument<256> cmdDoc;
                DeserializationError udpErr = deserializeJson(cmdDoc, udpCmd);
                if (!udpErr) {
                    // Armed state: {"armed": true/false}
                    if (cmdDoc.containsKey("armed")) {
                        is_armed = cmdDoc["armed"].as<bool>();
                        manual_led_override = false;
                        currentMode = is_armed ? M_ARMED : M_IDLE;
                    }
                    // Gimbal command: {"gim": [pitch, yaw]}
                    if (cmdDoc.containsKey("gim")) {
                        // servos removed — track commanded angles only
                        int p = cmdDoc["gim"][0];
                        int y = cmdDoc["gim"][1];
                        CurrentPitch = constrain(map(p, -90, 90, 0, 180), 10, 170);
                        CurrentYaw = constrain(map(y, -90, 90, 45, 135), 45, 135);
                        stabilize_active = false;
                    }
                    // LED command: {"led": "RED"}
                    if (cmdDoc.containsKey("led")) {
                        String color = cmdDoc["led"].as<String>();
                        manual_led_override = true;
                        // Map color string to LED mode
                        if (color == "RED") currentMode = M_BATTERY_LOW;      // Flash red
                        else if (color == "BLUE") currentMode = M_HOVER;      // Pulse blue
                        else if (color == "GREEN") currentMode = M_AI_MODE;   // Matrix green
                        else if (color == "OFF") { currentMode = M_IDLE; manual_led_override = false; }
                    }
                    // Mode command: {"mode": 16}
                    if (cmdDoc.containsKey("mode")) {
                        currentMode = (FlightMode)cmdDoc["mode"].as<int>();
                    }
                    // Stabilization toggle: {"stab": true/false}
                    if (cmdDoc.containsKey("stab")) {
                        stabilize_active = cmdDoc["stab"].as<bool>();
                    }
                }
            }
        }
    }

    // === SENSOR READING LOOP ===
    selectChannel(SHARED_PCA_CHANNEL); 
    
    if(mpu_active) {
         sensors_event_t a, g, temp;
         if(mpu.getEvent(&a, &g, &temp)) {
             g_ax = a.acceleration.x; g_ay = a.acceleration.y; g_az = a.acceleration.z;
             g_gx = g.gyro.x; g_gy = g.gyro.y; g_gz = g.gyro.z;
             
             if (stabilize_active) {
                 // servos removed — compute stabilization angles for telemetry only
                 float pitch_deg = (atan2(g_ax, g_az) * 180.0) / PI;
                 int s_pitch = 90 + (int)(pitch_deg * PITCH_GAIN * PITCH_DIR);
                 int s_yaw = 90 - (int)(g_gz * 10.0 * YAW_GAIN * YAW_DIR);
                 CurrentPitch = constrain(s_pitch, 10, 170);
                 CurrentYaw = constrain(s_yaw, 45, 135);
            }
         }
    }

    if(t1 && tof1.dataReady()) { dist1 = tof1.read(false); }
    if(t2 && tof2.dataReady()) { dist2 = tof2.read(false); }
    if(t3 && tof3.dataReady()) { dist3 = tof3.read(false); }
    if(t4 && tof4.dataReady()) { dist4 = tof4.read(false); }

    // TELEMETRY (UDP + Serial Debug)
    if (millis() - last_telem > 100) { // 10Hz Update
        StaticJsonDocument<1024> doc;
        doc["t1"] = dist1; doc["t2"] = dist2; 
        doc["t3"] = dist3; doc["t4"] = dist4;
        doc["ax"] = g_ax; doc["ay"] = g_ay; doc["az"] = g_az;
        doc["gx"] = g_gx; doc["gy"] = g_gy; doc["gz"] = g_gz;
        doc["gp"] = CurrentPitch; doc["cy"] = CurrentYaw;  // MA-20 FIX: was "gy" which overwrote gyro Y
        doc["lm"] = (int)currentMode;

        // DIAGNOSTIC FIELDS (so we can debug through the Radxa without USB)
        doc["pca"]  = bus_alive ? 1 : 0;     // I2C mux / sensor bus alive?
        doc["i2c"]  = i2c_found;              // live list of I2C addresses found on the bus
        doc["mpu"]  = mpu_active ? 1 : 0;     // MPU6050 found?
        doc["s1"]   = t1 ? 1 : 0;             // ToF sensors that initialized
        doc["s2"]   = t2 ? 1 : 0;
        doc["s3"]   = t3 ? 1 : 0;
        doc["s4"]   = t4 ? 1 : 0;
        doc["rssi"] = WiFi.RSSI();            // WiFi signal strength (dBm)
        doc["ip"]   = WiFi.localIP().toString();
        doc["up"]   = (unsigned long)(millis() / 1000); // uptime sec (resets to 0 = ESP rebooted!)

        String json_str;
        serializeJson(doc, json_str);

        // WIRE LINK (most reliable): stream over UART to the Radxa (/dev/ttyAS1 once
        // the wire is on pin 18). "ESPJSON:" prefix lets the Radxa pick it out of logs.
        Serial.print("ESPJSON:");
        Serial.println(json_str);

        // WiFi UDP — unicast to known Radxa IP AND broadcast (reaches Radxa on ANY IP)
        udp.beginPacket(radxa_ip, radxa_port);
        udp.print(json_str);
        udp.endPacket();

        udp.beginPacket(IPAddress(255,255,255,255), radxa_port);
        udp.print(json_str);
        udp.endPacket();

        last_telem = millis();
    }
}
