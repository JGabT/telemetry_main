#include <Arduino.h>
#include <FS.h>
#include <LittleFS.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <TinyGPS++.h>
#include <NimBLEDevice.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <Preferences.h>

// =============================================================================
// HARDWARE PIN DEFINITIONS & CONSTANTS
// =============================================================================

#define GPS_RX_PIN 16
#define GPS_TX_PIN 17

#define I2C_SDA 21
#define I2C_SCL 22
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

// Onboard BOOT button on most ESP32 devkits. Active LOW.
// Change this if the pit button is wired to a different GPIO.
#define PIT_BUTTON_PIN 0

#define BMS_MAC_ADDRESS "d2:1a:01:01:c9:a7"
#define SERVICE_UUID_MAIN "0000fff0-0000-1000-8000-00805f9b34fb"
#define CHAR_UUID_TX "0000fff1-0000-1000-8000-00805f9b34fb"
#define CHAR_UUID_RX "0000fff2-0000-1000-8000-00805f9b34fb"

#define SPEED_DEADBAND 0.8f
#define BURN_TO_COAST_THRES 22.5f
#define COAST_TO_BURN_THRES 21.5f
#define COAST_TO_CUT_THRES 30.5f
#define CUT_TO_COAST_THRES 29.5f

// Flush every 2 seconds at 5 Hz. Small enough to survive a sudden reset,
// large enough to avoid hammering the flash.
#define RAM_BUFFER_SIZE 10

#define RUN_SCHEMA_VERSION 2
#define BUTTON_DEBOUNCE_MS 40
#define BUTTON_LONG_PRESS_MS 2000

// =============================================================================
// DATA STRUCTURES & GLOBAL STATES
// =============================================================================

enum DriveState {
  STATE_STOP,
  STATE_BURN,
  STATE_COAST,
  STATE_CUTOFF
};

struct DalyTelemetry {
  float totalVoltage = 0.0f;
  float current = 0.0f;
  float soc = 0.0f;
  uint16_t maxCellMv = 0;
  uint8_t maxCellNo = 0;
  uint16_t minCellMv = 0;
  uint8_t minCellNo = 0;
  float maxTemp = 0.0f;
  float minTemp = 0.0f;
  uint8_t bmsState = 0;
  bool chargeMos = false;
  bool dischargeMos = false;
  uint32_t remCapMah = 0;
  uint8_t cellCount = 12;
  uint16_t cycles = 0;
  uint16_t cellMv[48] = { 0 };
  bool cellBalancing[48] = { false };
} bmsData;

struct BMSDeviceInfo {
  NimBLEAddress address;
  char name[32] = "";
  int rssi;
  bool found = false;
} bmsDevInfo;

struct LogRecord {
  unsigned long recordNum;
  unsigned long timestampMs;
  char dateStr[11];
  char timeStr[12];
  bool hasFix;
  float lat;
  float lon;
  float clampedSpeed;
  float rawSpeed;
  int sats;
  DriveState state;
  bool pit;
  bool bmsConnected;
  float voltage;
  float current;
  float soc;
  uint16_t maxCellMv;
  uint16_t minCellMv;
  uint32_t remCapMah;
};

// Drivers
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);
TinyGPSCustom gnggaSats(gps, "GNGGA", 7);
AsyncWebServer server(80);
Preferences prefs;

// General states
bool fsMounted = false;
bool oledReady = false;
bool prefsReady = false;
unsigned long recordsLogged = 0;
char currentLogPath[32] = "";
DriveState currentDriveState = STATE_STOP;

// Recording / pit states
bool recordingActive = false;
bool resumedRun = false;
bool pitMode = false;
float lastSpeedKmh = 0.0f;
volatile bool requestTogglePit = false;
volatile bool requestNewRun = false;

// Wi-Fi States
bool wifiActive = false;
unsigned long parkedStartTime = 0;

// RAM Telemetry Log Buffer
LogRecord ramLogBuffer[RAM_BUFFER_SIZE];
size_t ramBufferIndex = 0;

// BLE Handles
static NimBLEClient* pClient = nullptr;
static NimBLERemoteCharacteristic* pTxCharacteristic = nullptr;
static NimBLERemoteCharacteristic* pRxCharacteristic = nullptr;
static volatile bool bleConnected = false;
static volatile bool doConnect = false;

// Thread-safe ring buffer
static uint8_t bleRxRing[1024];
static volatile size_t ringHead = 0;
static volatile size_t ringTail = 0;
static portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;

// Frame assembly
static uint8_t frameAssembleBuf[13];
static size_t frameAssembleLen = 0;
static bool frameBaseDetected = false;
static int frameBase = 0;

// =============================================================================
// HARDWARE INITIALIZATION & SELF-HEALING DISPLAY RECOVERY
// =============================================================================

void initOLEDHardware() {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);  // Standard Mode 100kHz for maximum noise rejection

  if (display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR) || display.begin(SSD1306_SWITCHCAPVCC, 0x3D)) {
    oledReady = true;
    display.ssd1306_command(SSD1306_SETCONTRAST);
    display.ssd1306_command(0xFF);
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
  } else {
    oledReady = false;
  }
}

void safeRenderDisplay() {
  if (!oledReady) {
    initOLEDHardware();
    return;
  }

  // Render current frame buffer to display
  display.display();

  // Test I2C communication integrity to catch hardware freezes caused by EMI
  Wire.beginTransmission(OLED_ADDR);
  uint8_t i2cErr = Wire.endTransmission();

  // Re-initialize display hardware and reset I2C peripheral if a bus error occurs
  if (i2cErr != 0) {
    Wire.end();
    delay(10);
    initOLEDHardware();
  }
}

// =============================================================================
// PERSISTENT RUN STATE (RESTART FAILSAFE)
// =============================================================================

void savePitState() {
  if (prefsReady) prefs.putBool("pit", pitMode);
}

void saveRunState() {
  if (!prefsReady) return;
  prefs.putUShort("schema", RUN_SCHEMA_VERSION);
  prefs.putBool("active", recordingActive);
  prefs.putString("path", currentLogPath);
  prefs.putULong("records", recordsLogged);
  prefs.putBool("pit", pitMode);
}

void saveRecordCount() {
  if (prefsReady) prefs.putULong("records", recordsLogged);
}

void clearActiveRunState() {
  currentLogPath[0] = '\0';
  recordsLogged = 0;
  recordingActive = false;
  resumedRun = false;

  if (prefsReady) {
    prefs.putBool("active", false);
    prefs.putString("path", "");
    prefs.putULong("records", 0);
  }
}

void ensureFileEndsWithNewline(const char* path) {
  File f = LittleFS.open(path, "r");
  if (!f) return;

  size_t sz = f.size();
  if (sz == 0) {
    f.close();
    return;
  }

  f.seek(sz - 1);
  int last = f.read();
  f.close();

  if (last != '\n') {
    File a = LittleFS.open(path, FILE_APPEND);
    if (a) {
      a.write('\n');
      a.close();
    }
  }
}

unsigned long findLastRecordNumber(const char* path) {
  File f = LittleFS.open(path, "r");
  if (!f) return 0;

  unsigned long last = 0;
  while (f.available()) {
    String line = f.readStringUntil('\n');
    int comma = line.indexOf(',');
    if (comma <= 0) continue;

    char numBuf[16];
    int n = comma < (int)sizeof(numBuf) - 1 ? comma : (int)sizeof(numBuf) - 1;
    memcpy(numBuf, line.c_str(), n);
    numBuf[n] = '\0';

    unsigned long v = strtoul(numBuf, nullptr, 10);
    if (v > last) last = v;
  }
  f.close();
  return last;
}

void restoreRunState() {
  if (!prefsReady) return;
  if (!prefs.getBool("active", false)) return;

  String path = prefs.getString("path", "");
  if (path.length() == 0 || path.length() >= sizeof(currentLogPath)) {
    clearActiveRunState();
    return;
  }

  if (!LittleFS.exists(path)) {
    clearActiveRunState();
    return;
  }

  strncpy(currentLogPath, path.c_str(), sizeof(currentLogPath) - 1);
  currentLogPath[sizeof(currentLogPath) - 1] = '\0';

  // Only resume files written by this firmware/schema.
  File f = LittleFS.open(currentLogPath, "r");
  if (f) {
    String header = f.readStringUntil('\n');
    f.close();
    if (header.indexOf("Pit") < 0) {
      Serial.println("[FS] Previous run has an older schema, not resuming.");
      clearActiveRunState();
      return;
    }
  }

  ensureFileEndsWithNewline(currentLogPath);

  unsigned long lastInFile = findLastRecordNumber(currentLogPath);
  unsigned long storedCount = prefs.getULong("records", 0);
  recordsLogged = (lastInFile > storedCount) ? lastInFile : storedCount;

  recordingActive = true;
  resumedRun = true;

  Serial.printf("[FS] Resuming run %s at record %lu\n", currentLogPath, recordsLogged);
}

// =============================================================================
// WEB SERVER INITIALIZATION WITH PAST RUNS FILE MANAGER
// =============================================================================

void setupWebEndpoints() {
  server.on("/", HTTP_GET, [](AsyncWebServerRequest* request) {
    String html = "<!DOCTYPE html><html><head><meta name='viewport' content='width=device-width, initial-scale=1.0'>";
    html += "<title>Cardinal One Telemetry</title>";
    html += "<style>body{font-family:Arial,sans-serif;background:#121212;color:#fff;padding:20px;text-align:center;}";
    html += ".card{background:#1e1e1e;border-radius:8px;padding:15px;margin:10px auto;max-width:500px;box-shadow:0 4px 6px rgba(0,0,0,0.3);}";
    html += "a.btn{background:#b30000;color:#fff;padding:10px 18px;text-decoration:none;border-radius:4px;display:inline-block;margin:5px;}";
    html += "a.alt{background:#005f99;}";
    html += "ul{list-style:none;padding:0;}li{margin:8px 0;background:#2a2a2a;padding:10px;border-radius:4px;display:flex;justify-content:space-between;align-items:center;}";
    html += "</style></head><body>";
    html += "<h2>Cardinal One Telemetry</h2>";
    html += "<div class='card'>";
    html += "<h3>Mode: <span style='color:";
    html += pitMode ? "#ffaa00" : "#00ffcc";
    html += ";'>";
    html += pitMode ? "PIT" : "TRACK";
    html += "</span></h3>";
    html += "<p>Recording: <b>";
    html += recordingActive ? "ACTIVE" : "WAITING FOR GPS FIX";
    html += "</b></p>";
    html += "<h3>Active Run: <span style='color:#00ffcc;'>";
    html += currentLogPath[0] ? String(currentLogPath) : "none";
    html += "</span></h3>";
    if (currentLogPath[0]) {
      html += "<a href='/download?file=" + String(currentLogPath).substring(1) + "' class='btn'>Download Active Run CSV</a>";
    }
    html += "<a href='/togglepit' class='btn alt'>Toggle PIT / TRACK</a>";
    html += "<a href='/newrun' class='btn'>Start New Run</a>";
    html += "</div>";
    html += "<div class='card'><h3>Stored Runs (LittleFS)</h3><ul>";

    File root = LittleFS.open("/");
    if (root) {
      File file = root.openNextFile();
      while (file) {
        String fileName = String(file.name());
        if (fileName.startsWith("/")) {
          fileName = fileName.substring(1);
        }
        if (fileName.endsWith(".csv")) {
          html += "<li><span>" + fileName + " (" + String(file.size() / 1024) + " KB)</span>";
          html += "<a href='/download?file=" + fileName + "' class='btn'>CSV</a></li>";
        }
        file = root.openNextFile();
      }
      root.close();
    }

    html += "</ul></div></body></html>";
    request->send(200, "text/html", html);
  });

  server.on("/download", HTTP_GET, [](AsyncWebServerRequest* request) {
    if (request->hasParam("file")) {
      String fileName = "/" + request->getParam("file")->value();
      if (LittleFS.exists(fileName)) {
        request->send(LittleFS, fileName, "text/csv", true);
        return;
      }
    }
    request->send(404, "text/plain", "Log file not found.");
  });

  // State changes are flagged here and applied safely from the main loop.
  server.on("/togglepit", HTTP_GET, [](AsyncWebServerRequest* request) {
    requestTogglePit = true;
    request->redirect("/");
  });

  server.on("/newrun", HTTP_GET, [](AsyncWebServerRequest* request) {
    requestNewRun = true;
    request->redirect("/");
  });
}

void startWiFiAP() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Cardinal-Telemetry", "mapua1234");
  wifiActive = true;

  // Start web server AFTER Wi-Fi stack mbox handles are allocated
  server.begin();

  Serial.println("\n[WIFI] Vehicle parked. Access Point Started.");
  Serial.println("[WIFI] Connect to 'Cardinal-Telemetry' (Pass: mapua1234)");
  Serial.println("[WIFI] Open http://192.168.4.1 in your browser.");
}

void stopWiFiAP() {
  server.end();
  WiFi.softAPdisconnect(true);
  WiFi.mode(WIFI_OFF);
  wifiActive = false;

  Serial.println("\n[WIFI] Vehicle moving. Access Point Stopped.");
}

// =============================================================================
// GNSS INITIALIZATION & HELPERS
// =============================================================================

void sendUBXCommand(const uint8_t* msg, uint16_t len) {
  uint8_t ck_a = 0, ck_b = 0;
  for (uint16_t i = 2; i < len; i++) {
    ck_a += msg[i];
    ck_b += ck_a;
  }
  gpsSerial.write(msg, len);
  gpsSerial.write(ck_a);
  gpsSerial.write(ck_b);
}

void forceNMEAOutput115200() {
  uint8_t cfg_prt[] = {
    0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00,
    0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00,
    0x02, 0x00, 0x00, 0x00, 0x00, 0x00
  };
  sendUBXCommand(cfg_prt, sizeof(cfg_prt));
}

void setGPSRate10Hz() {
  uint8_t cfg_rate[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0x64, 0x00,
    0x01, 0x00, 0x01, 0x00
  };
  sendUBXCommand(cfg_rate, sizeof(cfg_rate));
}

void initGNSS() {
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  forceNMEAOutput115200();
  delay(100);
  gpsSerial.end();

  gpsSerial.begin(38400, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  forceNMEAOutput115200();
  delay(100);
  gpsSerial.end();

  gpsSerial.begin(115200, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(100);
  forceNMEAOutput115200();
  delay(150);

  setGPSRate10Hz();
  delay(100);
}

int getSatCount() {
  if (gnggaSats.isValid() && atoi(gnggaSats.value()) > 0) {
    return atoi(gnggaSats.value());
  }
  return gps.satellites.value();
}

bool hasValidFix() {
  return gps.location.isValid() && gps.location.age() < 2000 && getSatCount() >= 4;
}

void getPhilippineTime(int& year, int& month, int& day, int& hour, int& minute, int& second) {
  year = gps.date.year();
  month = gps.date.month();
  day = gps.date.day();
  hour = gps.time.hour() + 8;
  minute = gps.time.minute();
  second = gps.time.second();

  if (hour >= 24) {
    hour -= 24;
    day++;
    const int daysInMonth[] = { 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31 };
    int maxDays = daysInMonth[(month >= 1 && month <= 12) ? month - 1 : 0];

    if (month == 2 && ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))) {
      maxDays = 29;
    }

    if (day > maxDays) {
      day = 1;
      month++;
      if (month > 12) {
        month = 1;
        year++;
      }
    }
  }
}

void updateDriveStrategy(float speed) {
  if (speed <= 0.0f) {
    currentDriveState = STATE_STOP;
    return;
  }

  switch (currentDriveState) {
    case STATE_STOP:
      currentDriveState = STATE_BURN;
      break;
    case STATE_BURN:
      if (speed >= BURN_TO_COAST_THRES) {
        currentDriveState = STATE_COAST;
      }
      break;
    case STATE_COAST:
      if (speed < COAST_TO_BURN_THRES) {
        currentDriveState = STATE_BURN;
      } else if (speed >= COAST_TO_CUT_THRES) {
        currentDriveState = STATE_CUTOFF;
      }
      break;
    case STATE_CUTOFF:
      if (speed < CUT_TO_COAST_THRES) {
        currentDriveState = STATE_COAST;
      }
      break;
  }
}

// =============================================================================
// DALY BMS PROTOCOL & STREAM PROCESSING
// =============================================================================

bool verifyChecksum(const uint8_t* pkt, size_t len) {
  if (len < 2) return false;
  uint8_t sum = 0;
  for (size_t i = 0; i < len - 1; ++i) sum += pkt[i];
  return (sum == pkt[len - 1]);
}

void parseDalyFrame(const uint8_t* frame) {
  if (frame[0] != 0xA5 || !verifyChecksum(frame, 13)) return;

  uint8_t cmd = frame[2];
  const uint8_t* payload = frame + 4;

  switch (cmd) {
    case 0x90:
      {
        uint16_t total_raw = (payload[0] << 8) | payload[1];
        uint16_t current_raw = (payload[4] << 8) | payload[5];
        uint16_t soc_raw = (payload[6] << 8) | payload[7];

        bmsData.totalVoltage = total_raw / 10.0f;
        bmsData.current = ((int)current_raw - 30000) / 10.0f;
        bmsData.soc = soc_raw / 10.0f;
        break;
      }
    case 0x91:
      {
        bmsData.maxCellMv = (payload[0] << 8) | payload[1];
        bmsData.maxCellNo = payload[2];
        bmsData.minCellMv = (payload[3] << 8) | payload[4];
        bmsData.minCellNo = payload[5];
        break;
      }
    case 0x92:
      {
        bmsData.maxTemp = payload[0] - 40.0f;
        bmsData.minTemp = payload[2] - 40.0f;
        break;
      }
    case 0x93:
      {
        bmsData.bmsState = payload[0];
        bmsData.chargeMos = (payload[1] == 1);
        bmsData.dischargeMos = (payload[2] == 1);
        bmsData.remCapMah = ((uint32_t)payload[4] << 24) | ((uint32_t)payload[5] << 16) | ((uint32_t)payload[6] << 8) | payload[7];
        break;
      }
    case 0x94:
      {
        if (payload[0] >= 1 && payload[0] <= 48) {
          bmsData.cellCount = payload[0];
        }
        bmsData.cycles = (payload[5] << 8) | payload[6];
        break;
      }
    case 0x95:
      {
        uint8_t rawFrameNo = payload[0];
        if (!frameBaseDetected) {
          frameBase = (rawFrameNo == 0) ? 0 : 1;
          frameBaseDetected = true;
        }

        int idx = (int)rawFrameNo - frameBase;
        if (idx >= 0 && idx < 16) {
          for (int i = 0; i < 3; ++i) {
            int cellIndex = idx * 3 + i;
            if (cellIndex < bmsData.cellCount && cellIndex < 48) {
              int p = 1 + i * 2;
              uint16_t mv = (payload[p] << 8) | payload[p + 1];
              bmsData.cellMv[cellIndex] = (mv == 0xFFFF) ? 0 : mv;
            }
          }
        }
        break;
      }
    case 0x97:
      {
        int count = bmsData.cellCount > 48 ? 48 : bmsData.cellCount;
        for (int i = 0; i < count; ++i) {
          int byteIndex = i / 8;
          int bitIndex = i % 8;
          bmsData.cellBalancing[i] = (payload[byteIndex] >> bitIndex) & 0x01;
        }
        break;
      }
  }
}

void processBleRingBuffer() {
  while (true) {
    uint8_t b = 0;
    bool hasByte = false;

    portENTER_CRITICAL(&ringMux);
    if (ringHead != ringTail) {
      b = bleRxRing[ringTail];
      ringTail = (ringTail + 1) % sizeof(bleRxRing);
      hasByte = true;
    }
    portEXIT_CRITICAL(&ringMux);

    if (!hasByte) break;

    if (frameAssembleLen == 0) {
      if (b == 0xA5) {
        frameAssembleBuf[frameAssembleLen++] = b;
      }
    } else {
      frameAssembleBuf[frameAssembleLen++] = b;
      if (frameAssembleLen == 13) {
        parseDalyFrame(frameAssembleBuf);
        frameAssembleLen = 0;
      }
    }
  }
}

static void notifyCB(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  if (length == 0 || pData == nullptr) return;

  portENTER_CRITICAL(&ringMux);
  for (size_t i = 0; i < length; ++i) {
    size_t nextHead = (ringHead + 1) % sizeof(bleRxRing);
    if (nextHead != ringTail) {
      bleRxRing[ringHead] = pData[i];
      ringHead = nextHead;
    }
  }
  portEXIT_CRITICAL(&ringMux);
}

// =============================================================================
// BLE CALLBACKS & CONNECTION HANDLERS (NimBLE v2.0 Thread-Safe)
// =============================================================================

class ScanCallbacks : public NimBLEScanCallbacks {
  void onDiscovered(const NimBLEAdvertisedDevice* dev) override {
    if (dev == nullptr) return;

    if (dev->getAddress().toString() == std::string(BMS_MAC_ADDRESS)) {
      bmsDevInfo.address = dev->getAddress();

      std::string devName = dev->getName();
      if (!devName.empty()) {
        strncpy(bmsDevInfo.name, devName.c_str(), sizeof(bmsDevInfo.name) - 1);
        bmsDevInfo.name[sizeof(bmsDevInfo.name) - 1] = '\0';
      }

      bmsDevInfo.rssi = dev->getRSSI();
      bmsDevInfo.found = true;
      doConnect = true;
    }
  }
};
static ScanCallbacks scanCallbacks;

class ClientCallbacks : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* client) override {
    bleConnected = true;
    frameBaseDetected = false;
  }

  void onDisconnect(NimBLEClient* client, int reason) override {
    bleConnected = false;
    pRxCharacteristic = nullptr;
    pTxCharacteristic = nullptr;
  }
};
static ClientCallbacks clientCallbacks;

void sendDalyRequest(uint8_t cmd) {
  if (!pRxCharacteristic || !bleConnected || !pClient || !pClient->isConnected()) return;

  uint8_t pkt[13];
  pkt[0] = 0xA5;
  pkt[1] = 0x40;
  pkt[2] = cmd;
  pkt[3] = 0x08;
  memset(&pkt[4], 0x00, 8);

  uint8_t sum = 0;
  for (int i = 0; i < 12; ++i) sum += pkt[i];
  pkt[12] = sum;

  if (pRxCharacteristic->canWrite()) {
    pRxCharacteristic->writeValue(pkt, 13, false);
  }
}

bool discoverAndSubscribe() {
  if (!pClient || !pClient->isConnected()) return false;

  NimBLERemoteService* svc = pClient->getService(SERVICE_UUID_MAIN);
  if (!svc) return false;

  pTxCharacteristic = svc->getCharacteristic(CHAR_UUID_TX);
  pRxCharacteristic = svc->getCharacteristic(CHAR_UUID_RX);

  if (!pTxCharacteristic || !pRxCharacteristic) return false;

  if (pTxCharacteristic->canNotify()) {
    return pTxCharacteristic->subscribe(true, notifyCB, false);
  }
  return false;
}

bool connectToBMS() {
  if (!bmsDevInfo.found) return false;

  if (pClient != nullptr) {
    if (pClient->isConnected()) return true;
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
  }

  pClient = NimBLEDevice::createClient();
  if (!pClient) return false;

  pClient->setClientCallbacks(&clientCallbacks, false);
  pClient->setConnectTimeout(5000);

  if (!pClient->connect(bmsDevInfo.address)) {
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
    return false;
  }

  if (!discoverAndSubscribe()) {
    pClient->disconnect();
    NimBLEDevice::deleteClient(pClient);
    pClient = nullptr;
    return false;
  }
  return true;
}

void startScan(uint32_t durationMs) {
  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&scanCallbacks, false);
  scan->setActiveScan(true);
  scan->setInterval(80);
  scan->setWindow(60);
  scan->setMaxResults(0);
  scan->start(durationMs, false);
}

// =============================================================================
// STORAGE & DISPLAY RENDERING
// =============================================================================

void checkFlashCapacityAndClean() {
  size_t totalBytes = LittleFS.totalBytes();
  size_t usedBytes = LittleFS.usedBytes();

  if (totalBytes > 0 && ((float)usedBytes / totalBytes) > 0.85f) {
    char oldestPath[32] = "";
    int oldestRun = 1000000;

    File root = LittleFS.open("/");
    if (root) {
      File file = root.openNextFile();
      while (file) {
        String path = String(file.path());
        int runNo = 0;
        if (path.startsWith("/run_") && path.endsWith(".csv")) {
          runNo = path.substring(5, path.length() - 4).toInt();
          if (runNo > 0 && runNo < oldestRun && path != String(currentLogPath)) {
            oldestRun = runNo;
            strncpy(oldestPath, path.c_str(), sizeof(oldestPath) - 1);
            oldestPath[sizeof(oldestPath) - 1] = '\0';
          }
        }
        file = root.openNextFile();
      }
      root.close();
    }

    if (oldestPath[0] != '\0') {
      Serial.printf("[FS] Low space, removing oldest run %s\n", oldestPath);
      LittleFS.remove(oldestPath);
    }
  }
}

bool createNewRunFile() {
  checkFlashCapacityAndClean();

  int sessionIdx = 1;
  while (sessionIdx <= 9999) {
    snprintf(currentLogPath, sizeof(currentLogPath), "/run_%04d.csv", sessionIdx);
    if (!LittleFS.exists(currentLogPath)) break;
    sessionIdx++;
  }

  if (sessionIdx > 9999) {
    currentLogPath[0] = '\0';
    return false;
  }

  File f = LittleFS.open(currentLogPath, FILE_WRITE);
  if (!f) {
    currentLogPath[0] = '\0';
    return false;
  }

  f.println("Record,Millis,PST_Date,PST_Time,Fix,Lat,Lon,Speed_kmh,Raw_Speed_kmh,Sats,State,"
            "Pit,BMS_Connected,Total_V,Current_A,SOC_Pct,Max_Cell_mV,Min_Cell_mV,RemCap_mAh");
  f.close();

  recordsLogged = 0;
  recordingActive = true;
  resumedRun = false;
  saveRunState();

  Serial.printf("[FS] New run started: %s\n", currentLogPath);
  return true;
}

bool flushRamBufferToFlash() {
  if (ramBufferIndex == 0) return true;
  if (!fsMounted || currentLogPath[0] == '\0') return false;

  File logFile = LittleFS.open(currentLogPath, FILE_APPEND);
  if (!logFile) return false;

  for (size_t i = 0; i < ramBufferIndex; i++) {
    LogRecord& rec = ramLogBuffer[i];
    logFile.printf("%lu,%lu,%s,%s,%s,",
                   rec.recordNum, rec.timestampMs, rec.dateStr, rec.timeStr, rec.hasFix ? "3D" : "NONE");

    if (rec.hasFix) {
      logFile.printf("%.6f,%.6f,%.2f,%.2f,%d,", rec.lat, rec.lon, rec.clampedSpeed, rec.rawSpeed, rec.sats);
    } else {
      logFile.printf(",,0.00,%.2f,%d,", rec.rawSpeed, rec.sats);
    }

    switch (rec.state) {
      case STATE_STOP: logFile.print("STOP,"); break;
      case STATE_BURN: logFile.print("BURN,"); break;
      case STATE_COAST: logFile.print("COAST,"); break;
      case STATE_CUTOFF: logFile.print("CUTOFF,"); break;
    }

    logFile.printf("%d,%d,%.2f,%.2f,%.1f,%u,%u,%lu\n",
                   rec.pit ? 1 : 0,
                   rec.bmsConnected ? 1 : 0,
                   rec.voltage, rec.current, rec.soc,
                   rec.maxCellMv, rec.minCellMv, rec.remCapMah);
  }

  logFile.close();
  ramBufferIndex = 0;
  saveRecordCount();
  return true;
}

bool startNewRun() {
  if (!fsMounted) return false;
  if (!flushRamBufferToFlash()) return false;
  return createNewRunFile();
}

void togglePitMode() {
  pitMode = !pitMode;
  savePitState();

  // Make the pit transition durable immediately.
  flushRamBufferToFlash();

  Serial.printf("[PIT] Mode: %s\n", pitMode ? "PIT" : "TRACK");
}

void renderDashboard(float displaySpeed, int sats, bool hasFix) {
  if (!oledReady) return;

  display.clearDisplay();

  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 2);
  display.printf("G:%02d", sats);

  display.setCursor(34, 2);
  display.print(hasFix ? "3D" : "NO");

  display.setCursor(52, 2);
  display.print(fsMounted ? "FS:OK" : "FS:--");

  display.setCursor(90, 2);
  display.print(wifiActive ? "WIFI" : (bleConnected ? "BMS" : "--"));

  display.setCursor(120, 2);
  display.print(recordingActive ? "*" : " ");

  display.drawLine(0, 12, 127, 12, SSD1306_WHITE);

  display.setTextSize(3);
  display.setCursor(2, 18);
  if (displaySpeed < 10.0f) {
    display.printf(" %3.1f", displaySpeed);
  } else {
    display.printf("%4.1f", displaySpeed);
  }

  display.setTextSize(1);
  display.setCursor(84, 18);
  display.print("km/h");

  display.setCursor(84, 30);
  switch (currentDriveState) {
    case STATE_STOP: display.print("STOP  "); break;
    case STATE_BURN: display.print("BURN>>"); break;
    case STATE_COAST: display.print("COAST "); break;
    case STATE_CUTOFF: display.print("CUTOFF"); break;
  }

  display.drawLine(0, 42, 127, 42, SSD1306_WHITE);
  display.setCursor(0, 45);
  if (bleConnected) {
    display.printf("%4.1fV %4.1fA %3.0f%%", bmsData.totalVoltage, bmsData.current, bmsData.soc);
  } else {
    display.print("BMS DISCONNECTED");
  }

  display.setCursor(0, 55);
  if (gps.time.isValid()) {
    int hr = (gps.time.hour() + 8) % 24;
    display.printf("%02d:%02d:%02d", hr, gps.time.minute(), gps.time.second());
  } else {
    display.print("--:--:--");
  }

  display.setCursor(52, 55);
  display.print(pitMode ? "PIT" : "TRK");

  display.setCursor(76, 55);
  display.printf("#%lu", recordsLogged);

  // Self-healing frame commit with I2C health check and peripheral reset
  safeRenderDisplay();
}

// =============================================================================
// PIT BUTTON & PENDING WEB ACTIONS
// =============================================================================

void updatePitButton(float speed) {
  static bool rawState = HIGH;
  static bool stableState = HIGH;
  static unsigned long lastChange = 0;
  static unsigned long pressStart = 0;
  static bool longPressHandled = false;

  bool raw = digitalRead(PIT_BUTTON_PIN);
  unsigned long now = millis();

  if (raw != rawState) {
    rawState = raw;
    lastChange = now;
  }

  if (now - lastChange >= BUTTON_DEBOUNCE_MS) {
    if (stableState != rawState) {
      stableState = rawState;

      if (stableState == LOW) {
        pressStart = now;
        longPressHandled = false;
      } else {
        // Short press: toggle pit / track.
        if (!longPressHandled && (now - pressStart) >= 50) {
          togglePitMode();
        }
      }
    }
  }

  // Long press while stopped: end this run and start a fresh one.
  if (stableState == LOW && !longPressHandled && (now - pressStart) >= BUTTON_LONG_PRESS_MS) {
    longPressHandled = true;
    if (speed < 1.0f) {
      Serial.println("[RUN] Long press detected, starting a new run.");
      startNewRun();
    } else {
      Serial.println("[RUN] Long press ignored while moving.");
    }
  }
}

void handlePendingActions() {
  if (requestTogglePit) {
    requestTogglePit = false;
    togglePitMode();
  }

  if (requestNewRun) {
    requestNewRun = false;
    startNewRun();
  }
}

// =============================================================================
// ARDUINO SETUP & MAIN LOOP
// =============================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // 1. Pit button
  pinMode(PIT_BUTTON_PIN, INPUT_PULLUP);

  // 2. OLED Display Setup with EMI-resilient 100kHz I2C Clock
  initOLEDHardware();

  if (oledReady) {
    display.setTextSize(2);
    display.setCursor(16, 16);
    display.print("CARDINAL");
    display.setCursor(44, 36);
    display.print("ONE");
    display.display();
  }

  delay(1500);

  // 3. Persistent run state (used for the restart failsafe)
  prefsReady = prefs.begin("telemetry", false);
  if (prefsReady) {
    uint16_t schema = prefs.getUShort("schema", 0);
    if (schema != RUN_SCHEMA_VERSION) {
      // Do not resume a file written by an incompatible firmware.
      prefs.clear();
      prefs.putUShort("schema", RUN_SCHEMA_VERSION);
    }
    pitMode = prefs.getBool("pit", false);
  }

  // 4. Internal Flash LittleFS Setup (retry a few times before giving up)
  for (int attempt = 0; attempt < 3 && !fsMounted; attempt++) {
    if (LittleFS.begin(true)) {
      fsMounted = true;
    } else {
      delay(500);
    }
  }

  if (fsMounted) {
    restoreRunState();
  }

  // 5. Register web server route handlers (server starts lazily when parked)
  setupWebEndpoints();

  // 6. GNSS Receiver Setup
  initGNSS();

  // 7. NimBLE BLE Setup
  NimBLEDevice::init("ESP32-Telemetry-Host");
  startScan(8000);
}

void loop() {
  // 0. Handle button and web-requested state changes in the main task
  handlePendingActions();

  // 1. Read GNSS stream
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // 2. Process incoming BLE packets safely on Core 1
  processBleRingBuffer();

  // 3. Asynchronous Connection Dispatcher (Safely stops scan in main task context)
  if (doConnect && !bleConnected) {
    doConnect = false;
    if (NimBLEDevice::getScan()->isScanning()) {
      NimBLEDevice::getScan()->stop();
    }
    connectToBMS();
  }

  // 4. Strict, Non-overlapping Sequential BMS Query Engine
  static unsigned long lastBmsSend = 0;
  static uint8_t queryStep = 0;

  if (bleConnected && pClient && pClient->isConnected()) {
    unsigned long now = millis();
    if (now - lastBmsSend >= 120) {
      lastBmsSend = now;
      switch (queryStep) {
        case 0: sendDalyRequest(0x90); break;
        case 1: sendDalyRequest(0x90); break;
        case 2: sendDalyRequest(0x91); break;
        case 3: sendDalyRequest(0x90); break;
        case 4: sendDalyRequest(0x92); break;
        case 5: sendDalyRequest(0x90); break;
        case 6: sendDalyRequest(0x93); break;
        case 7: sendDalyRequest(0x90); break;
        case 8: sendDalyRequest(0x94); break;
      }
      queryStep = (queryStep + 1) % 9;
    }
  } else if (!NimBLEDevice::getScan()->isScanning() && !bleConnected) {
    static unsigned long lastBleRetry = 0;
    if (millis() - lastBleRetry >= 6000) {
      lastBleRetry = millis();
      startScan(4000);
    }
  }

  // 5. Main Telemetry Tick (5 Hz / 200 ms)
  static unsigned long lastTelemetryTick = 0;
  if (millis() - lastTelemetryTick >= 200) {
    lastTelemetryTick = millis();

    bool hasFix = hasValidFix();
    float rawSpeed = hasFix ? gps.speed.kmph() : 0.0f;
    float clampedSpeed = (rawSpeed < SPEED_DEADBAND) ? 0.0f : rawSpeed;
    lastSpeedKmh = clampedSpeed;

    updateDriveStrategy(clampedSpeed);
    int sats = getSatCount();

    updatePitButton(clampedSpeed);

    // Automatic recording: wait for a real fix, then create the run file.
    // If a previous run was restored at boot, recording is already active.
    if (fsMounted && !recordingActive && hasFix) {
      startNewRun();
    }

    renderDashboard(clampedSpeed, sats, hasFix);

    // Wi-Fi Access Point Logic (pit mode opens it much sooner)
    if (clampedSpeed == 0.0f) {
      if (parkedStartTime == 0) {
        parkedStartTime = millis();
      } else {
        unsigned long parkedThreshold = pitMode ? 5000UL : 30000UL;
        if (!wifiActive && (millis() - parkedStartTime >= parkedThreshold)) {
          startWiFiAP();
        }
      }
    } else {
      parkedStartTime = 0;
      if (wifiActive) {
        stopWiFiAP();
      }
    }

    if (fsMounted && recordingActive && ramBufferIndex < RAM_BUFFER_SIZE) {
      recordsLogged++;
      LogRecord& rec = ramLogBuffer[ramBufferIndex];
      rec.recordNum = recordsLogged;
      rec.timestampMs = millis();
      rec.hasFix = hasFix;
      rec.lat = hasFix ? gps.location.lat() : 0.0f;
      rec.lon = hasFix ? gps.location.lng() : 0.0f;
      rec.clampedSpeed = clampedSpeed;
      rec.rawSpeed = rawSpeed;
      rec.sats = sats;
      rec.state = currentDriveState;
      rec.pit = pitMode;
      rec.bmsConnected = bleConnected;
      rec.voltage = bmsData.totalVoltage;
      rec.current = bmsData.current;
      rec.soc = bmsData.soc;
      rec.maxCellMv = bmsData.maxCellMv;
      rec.minCellMv = bmsData.minCellMv;
      rec.remCapMah = bmsData.remCapMah;

      if (gps.date.isValid() && gps.time.isValid()) {
        int yr, mo, dy, hr, mn, sc;
        getPhilippineTime(yr, mo, dy, hr, mn, sc);
        snprintf(rec.dateStr, sizeof(rec.dateStr), "%04d-%02d-%02d", yr, mo, dy);
        snprintf(rec.timeStr, sizeof(rec.timeStr), "%02d:%02d:%02d.%02d", hr, mn, sc, gps.time.centisecond());
      } else {
        snprintf(rec.dateStr, sizeof(rec.dateStr), "NO_DATE");
        snprintf(rec.timeStr, sizeof(rec.timeStr), "NO_TIME");
      }

      ramBufferIndex++;

      if (ramBufferIndex >= RAM_BUFFER_SIZE) {
        flushRamBufferToFlash();
      }
    } else if (fsMounted && ramBufferIndex >= RAM_BUFFER_SIZE) {
      // Previous flush failed. Keep retrying so buffered records are not lost.
      flushRamBufferToFlash();
    }
  }

  // 6. Serial Diagnostics at 1 Hz
  static unsigned long lastSerialOutput = 0;
  static bool blinkState = false;
  if (millis() - lastSerialOutput >= 1000) {
    lastSerialOutput = millis();
    blinkState = !blinkState;
    Serial.printf("\rRun:%s rec:%lu %s %s %s %c",
                  currentLogPath[0] ? currentLogPath : "(waiting)",
                  recordsLogged,
                  pitMode ? "PIT" : "TRACK",
                  recordingActive ? "REC" : "IDLE",
                  resumedRun ? "RESUMED" : "",
                  blinkState ? '.' : ' ');
  }
}
