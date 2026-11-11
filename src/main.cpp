#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#include "secrets.h"

// ---------- SETTINGS ----------
#define DEV "must1"                        // device identifier

#define RS485_RX 16
#define RS485_TX 17
#define DE_RE 4

const uint8_t  SLAVE_ID    = 4;
const uint32_t BAUD        = 19200;
const uint16_t REG_STATE   = 25201;        // CHECK: 4 = grid, 5 = battery
const uint32_t DEBOUNCE_MS = 15000;
const uint32_t POLL_MS     = 3000;

struct Range { uint16_t start; uint16_t count; };
Range ranges[] = { {20000, 50}, {25200, 80} };
const int NR = sizeof(ranges) / sizeof(ranges[0]);
const int MAXREG = 130;                    // count summ
const int CHUNK = 10;

// ---------- OBJECTS ----------
WiFiClientSecure tgClient;
UniversalTelegramBot bot(BOT_TOKEN, tgClient);
WiFiClient mqttNet;
PubSubClient mqtt(mqttNet);
ModbusMaster node;

uint16_t addrs[MAXREG];
uint16_t vals[MAXREG];
bool     okReg[MAXREG];
bool     discovered[MAXREG];
int      nTotal = 0;
bool     firstScanDone = false;

enum Mode { UNKNOWN, GRID, BATTERY };
Mode confirmed = UNKNOWN, candidate = UNKNOWN;
uint32_t candidateSince = 0;
String pending = "";

void preTx()  { digitalWrite(DE_RE, HIGH); }
void postTx() { digitalWrite(DE_RE, LOW);  }

void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) delay(200);
}

void notify(const String& msg) { pending = msg; }

void flushPending() {
  if (pending.length() == 0) return;
  ensureWifi();
  if (WiFi.status() == WL_CONNECTED && bot.sendMessage(CHAT_ID, pending, "")) pending = "";
}

// ---------- MODBUS ----------
void scanRegisters() {
  int idx = 0;
  for (int r = 0; r < NR; r++) {
    for (uint16_t off = 0; off < ranges[r].count; off += CHUNK) {
      uint16_t a = ranges[r].start + off;
      uint16_t n = min<uint16_t>(CHUNK, ranges[r].count - off);
      uint8_t res = node.readHoldingRegisters(a, n);
      if (res == node.ku8MBSuccess) {
        for (uint16_t i = 0; i < n; i++) {
          addrs[idx + i] = a + i;
          vals[idx + i] = node.getResponseBuffer(i);
          okReg[idx + i] = true;
        }
      } else if (!firstScanDone) {
        // First pass: trying every registry to find wich one works
        for (uint16_t i = 0; i < n; i++) {
          addrs[idx + i] = a + i;
          uint8_t r1 = node.readHoldingRegisters(a + i, 1);
          okReg[idx + i] = (r1 == node.ku8MBSuccess);
          if (okReg[idx + i]) vals[idx + i] = node.getResponseBuffer(0);
          delay(20);
        }
      } else {
        for (uint16_t i = 0; i < n; i++) { addrs[idx + i] = a + i; okReg[idx + i] = false; }
      }
      idx += n;
      delay(20);
    }
  }
  nTotal = idx;
  firstScanDone = true;
}

bool getReg(uint16_t addr, uint16_t &v) {
  for (int i = 0; i < nTotal; i++)
    if (addrs[i] == addr && okReg[i]) { v = vals[i]; return true; }
  return false;
}

// ---------- MQTT ----------
void publishDiscovery() {
  char topic[96], payload[512];
  const char* dev =
    "\"dev\":{\"ids\":[\"" DEV "\"],\"name\":\"Must PV18-6048 ECO\","
    "\"mf\":\"Must\",\"mdl\":\"PV18-6048 ECO\"}";

  // Power mode
  snprintf(payload, sizeof(payload),
    "{\"name\":\"Режим\",\"uniq_id\":\"" DEV "_mode\",\"stat_t\":\"" DEV "/mode\","
    "\"avty_t\":\"" DEV "/status\",%s}", dev);
  mqtt.publish("homeassistant/sensor/" DEV "_mode/config", payload, true);

  // Every working registry
  for (int i = 0; i < nTotal; i++) {
    if (!okReg[i]) continue;
    snprintf(topic, sizeof(topic), "homeassistant/sensor/" DEV "_r%u/config", addrs[i]);
    snprintf(payload, sizeof(payload),
      "{\"name\":\"Reg %u\",\"uniq_id\":\"" DEV "_r%u\",\"stat_t\":\"" DEV "/reg/%u\","
      "\"avty_t\":\"" DEV "/status\",\"stat_cla\":\"measurement\",%s}",
      addrs[i], addrs[i], addrs[i], dev);
    mqtt.publish(topic, payload, true);
    discovered[i] = true;
    mqtt.loop();
  }
}

void mqttConnect() {
  static uint32_t last = 0;
  if (mqtt.connected() || WiFi.status() != WL_CONNECTED) return;
  if (millis() - last < 5000) return;
  last = millis();
  if (mqtt.connect(DEV, MQTT_USER, MQTT_PASS, DEV "/status", 0, true, "offline")) {
    mqtt.publish(DEV "/status", "online", true);
    if (firstScanDone) publishDiscovery();
  }
}

void publishValues() {
  if (!mqtt.connected()) return;
  char topic[48], val[12];
  for (int i = 0; i < nTotal; i++) {
    if (!okReg[i]) continue;
    snprintf(topic, sizeof(topic), DEV "/reg/%u", addrs[i]);
    snprintf(val, sizeof(val), "%u", vals[i]);
    mqtt.publish(topic, val);
  }
  const char* m = confirmed == GRID ? "grid" : confirmed == BATTERY ? "battery" : "unknown";
  mqtt.publish(DEV "/mode", m, true);
}

// ---------- Mode + Telegram ----------
void updateMode() {
  uint16_t raw;
  if (!getReg(REG_STATE, raw)) return;
  Mode m = (raw == 4) ? GRID : (raw == 5) ? BATTERY : confirmed;
  if (m == UNKNOWN) return;
  if (m != candidate) { candidate = m; candidateSince = millis(); }
  if (candidate != confirmed && millis() - candidateSince > DEBOUNCE_MS) {
    if (confirmed != UNKNOWN) {
      notify(candidate == BATTERY
             ? "⚡ Світло зникло, інвертор працює від батареї"
             : "💡 Світло з'явилось, інвертор перейшов на мережу");
    }
    confirmed = candidate;
  }
}

void setup() {
  Serial2.begin(BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW);
  Serial2.begin(BAUD, SERIAL_8N1, RX2, TX2);
  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTx);
  node.postTransmission(postTx);
  tgClient.setInsecure();
  tgClient.setTimeout(5);   // секунди, замість довгого очікування
  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(768);
  ensureWifi();
  if (WiFi.status() == WL_CONNECTED) {
    bool ok = bot.sendMessage(CHAT_ID, "✅ ESP32 запущено, стежу за інвертором", "");
    Serial.printf("Telegram старт: %s\n", ok ? "надіслано" : "ПОМИЛКА");
  }
}

void loop() {
  static uint32_t lastPoll = 0;
  mqttConnect();
  mqtt.loop();

  if (millis() - lastPoll > POLL_MS) {
    lastPoll = millis();
    scanRegisters();
    int good = 0;
    for (int i = 0; i < nTotal; i++) good += okReg[i];
    Serial.printf("регістрів відповіло: %d з %d\n", good, nTotal);
    for (int i = 0; i < nTotal; i++)
      if (okReg[i]) Serial.printf("%u=%u\n", addrs[i], vals[i]);

    updateMode();
    if (mqtt.connected()) {
      bool needDisc = false;
      for (int i = 0; i < nTotal; i++) if (okReg[i] && !discovered[i]) needDisc = true;
      if (needDisc) publishDiscovery();
      publishValues();
    }
  }
  flushPending();
  delay(20);
}