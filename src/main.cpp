// Must PV18 ECO -> Telegram + Home Assistant (MQTT) через ESP32 + MAX485
// Протокол (за ESPHome-конфігом для PV18): Modbus RTU, 19200 8N1, адреса 4, holding-регістри 25200+
// Потрібен include/secrets.h з: WIFI_SSID, WIFI_PASS, BOT_TOKEN, CHAT_ID,
//                               MQTT_HOST, MQTT_PORT, MQTT_USER, MQTT_PASS

#include <Arduino.h>
#include <stdarg.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ModbusMaster.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include "secrets.h"

#define DEV "must1"                  // префікс топіків: must1/...

// Дублює вивід у Serial і в TCP-порт 23. Останні 2 КБ зберігає, щоб показати їх при підключенні.
class TeeLog : public Print {
  WiFiServer server{23};
  WiFiClient client;
  char ring[2048];
  size_t head = 0;
  bool full = false;
 public:
  void beginServer() { server.begin(); }
  void handle() {
    if (server.hasClient()) {
      if (client) client.stop();
      client = server.accept();
      if (full) client.write((const uint8_t*)ring + head, sizeof(ring) - head);
      client.write((const uint8_t*)ring, head);
    }
  }
  size_t write(uint8_t b) override {
    Serial.write(b);
    ring[head++] = (char)b;
    if (head >= sizeof(ring)) { head = 0; full = true; }
    if (client && client.connected()) client.write(b);
    return 1;
  }
};
TeeLog Log;

// ---------- RS485 ----------
#define RS485_RX 16                  // G16 <- RO
#define RS485_TX 17                  // G17 -> DI
#define DE_RE    4                   // G4  -> DE + RE (скручені разом)

const uint8_t  SLAVE_ID    = 4;
const uint32_t BAUD        = 19200;
const uint16_t REG_GRID_V  = 25207;  // напруга мережі, множник 0.1 В
const uint16_t GRID_OK_RAW = 1500;   // понад 150.0 В = мережа є
const uint32_t DEBOUNCE_MS = 15000;  // стан має триматись 15 с
const uint32_t POLL_MS     = 3000;
const uint32_t INTER_MS    = 200;    // пауза між Modbus-запитами (як в ESPHome-конфігу)

#define DIAG_SWEEP 1                 // 1 = коли інвертор мовчить, раз на 5 хв перебирати швидкість/адресу

struct Range { uint16_t start; uint16_t count; };
Range ranges[] = { {15200, 25}, {20000, 50}, {25200, 80} };
const int NR = sizeof(ranges) / sizeof(ranges[0]);
const int MAXREG = 160;              // >= сума count (155)
const int CHUNK = 10;

// Відомі регістри (з ESPHome-конфігу). Решта з'являться як "Reg NNNNN" із сирим значенням.
struct Known { uint16_t addr; const char* name; const char* unit; float scale; bool sgn; };
const Known KNOWN[] = {
  {25201, "Стан інвертора",         "",   1.0f,  false},
  {25205, "Напруга батареї",        "V",  0.1f,  false},
  {25206, "Вихідна напруга",        "V",  0.1f,  false},
  {25207, "Напруга мережі",         "V",  0.1f,  false},
  {25208, "Напруга шини",           "V",  0.1f,  false},
  {25210, "Струм інвертора",        "A",  0.1f,  false},
  {25211, "Струм мережі",           "A",  0.1f,  false},
  {25212, "Струм навантаження",     "A",  0.1f,  false},
  {25213, "Потужність інвертора",   "W",  1.0f,  true},
  {25214, "Потужність мережі",      "W",  1.0f,  true},
  {25215, "Потужність навантаження","W",  1.0f,  false},
  {25216, "Навантаження",           "%",  1.0f,  false},
  {25226, "Частота мережі",         "Hz", 0.01f, false},
  {25273, "Потужність батареї",     "W",  1.0f,  true},
  {25274, "Струм батареї",          "A",  1.0f,  true},
  {15205, "Напруга PV",             "V",  0.1f,  false},
  {15207, "Струм PV",               "A",  0.1f,  false},
  {15208, "Потужність PV",          "W",  1.0f,  false},
};
const int NKNOWN = sizeof(KNOWN) / sizeof(KNOWN[0]);

const Known* findKnown(uint16_t addr) {
  for (int i = 0; i < NKNOWN; i++) if (KNOWN[i].addr == addr) return &KNOWN[i];
  return nullptr;
}

// ---------- об'єкти ----------
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
bool     firstScanDone = false;      // true після першого повного скану з відповіддю інвертора
bool     otaStarted = false;
int      roHigh = -1;                // тест стану лінії RO перед запуском UART

enum Mode { UNKNOWN, GRID, BATTERY };
Mode confirmed = UNKNOWN, candidate = UNKNOWN;
uint32_t candidateSince = 0;
String pending = "";

// ---------- допоміжне ----------
void logf(const char* fmt, ...) {
  char buf[200];
  va_list a;
  va_start(a, fmt);
  vsnprintf(buf, sizeof(buf), fmt, a);
  va_end(a);
  Log.println(buf);
  if (mqtt.connected()) mqtt.publish(DEV "/log", buf);
}

void service() {                     // викликати в довгих операціях
  mqtt.loop();
  ArduinoOTA.handle();
  Log.handle();
}

void pause(uint32_t ms) {
  uint32_t t = millis();
  while (millis() - t < ms) { service(); delay(5); }
}

void preTx()  { digitalWrite(DE_RE, HIGH); }
void postTx() { digitalWrite(DE_RE, LOW);  }

void ensureWifi() {
  if (WiFi.status() == WL_CONNECTED) return;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 10000) delay(200);
}

void startOtaIfReady() {
  if (otaStarted || WiFi.status() != WL_CONNECTED) return;
  ArduinoOTA.setHostname("must-bridge");
  ArduinoOTA.begin();
  Log.beginServer();
  otaStarted = true;
  Log.println("[OTA] готово");
}

// ---------- Telegram ----------
void notify(const String& msg) { pending = msg; }

void flushPending() {
  static uint32_t lastTry = 0;
  if (pending.length() == 0) return;
  if (millis() - lastTry < 10000) return;
  lastTry = millis();
  ensureWifi();
  if (WiFi.status() != WL_CONNECTED) {
    Log.println("[TG] немає Wi-Fi");
    return;
  }
  bool ok = bot.sendMessage(CHAT_ID, pending, "");
  Log.printf("[TG] %s\n", ok ? "надіслано" : "ПОМИЛКА");
  if (ok) pending = "";
}

// ---------- діагностика: сирий запит і перебір ----------
#if DIAG_SWEEP
uint16_t mbCrc(const uint8_t* d, int n) {
  uint16_t c = 0xFFFF;
  for (int i = 0; i < n; i++) {
    c ^= d[i];
    for (int j = 0; j < 8; j++) c = (c & 1) ? (c >> 1) ^ 0xA001 : c >> 1;
  }
  return c;
}

// Шле запит і слухає 400 мс. Шумові потоки не друкує, лише короткі або схожі на відповідь Modbus.
void rawProbe(uint32_t baud, uint8_t id, uint16_t reg) {
  uint8_t f[8] = {id, 0x03, (uint8_t)(reg >> 8), (uint8_t)reg, 0x00, 0x01, 0, 0};
  uint16_t c = mbCrc(f, 6);
  f[6] = c & 0xFF;
  f[7] = c >> 8;
  Serial2.updateBaudRate(baud);
  while (Serial2.available()) Serial2.read();
  digitalWrite(DE_RE, HIGH);
  Serial2.write(f, 8);
  Serial2.flush();
  digitalWrite(DE_RE, LOW);

  uint8_t rx[64];
  int total = 0, n = 0;
  uint32_t t = millis();
  while (millis() - t < 400) {
    while (Serial2.available()) {
      int b = Serial2.read();
      total++;
      if (n < (int)sizeof(rx)) rx[n++] = (uint8_t)b;
    }
    delay(1);
  }
  bool looksModbus = (n >= 5 && rx[0] == id && rx[1] == 0x03);
  if (looksModbus || total < 20) {
    char buf[100];
    int k = 0;
    for (int i = 0; i < n && k < 90; i++) k += snprintf(buf + k, sizeof(buf) - k, "%02X ", rx[i]);
    buf[k] = 0;
    logf("[RAW] baud=%u id=%u -> %d байт: %s%s", (unsigned)baud, id, total, buf,
         looksModbus ? " <== СХОЖЕ НА MODBUS!" : "");
  }
}

void sweep() {
  const uint32_t bauds[] = {19200, 9600, 4800, 38400};
  for (uint32_t b : bauds) {
    for (uint8_t id = 1; id <= 8; id++) {
      rawProbe(b, id, REG_GRID_V);
      service();
    }
  }
  Serial2.updateBaudRate(BAUD);
  logf("[RAW] перебір завершено (4 швидкості x 8 адрес)");
}
#endif

// ---------- Modbus ----------
void scanRegisters() {
  // Швидка перевірка: якщо інвертор мовчить, не скануємо всі регістри
  uint8_t p1 = node.readHoldingRegisters(REG_GRID_V, 1);
  service();
  uint8_t p2 = (p1 == node.ku8MBSuccess) ? p1 : node.readHoldingRegisters(25201, 1);
  service();
  if (p1 != node.ku8MBSuccess && p2 != node.ku8MBSuccess) {
    static uint32_t lastLog = 0;
    if (lastLog == 0 || millis() - lastLog > 30000) {
      lastLog = millis() ? millis() : 1;
      logf("[Modbus] немає відповіді (код %u, %u; 226=таймаут, 224=сміття на лінії)", p1, p2);
    }
#if DIAG_SWEEP
    static uint32_t lastSweep = 0;
    if (lastSweep == 0 || millis() - lastSweep > 300000) {
      lastSweep = millis() ? millis() : 1;
      sweep();
    }
#endif
    nTotal = 0;
    return;
  }

  int idx = 0;
  for (int r = 0; r < NR; r++) {
    for (uint16_t off = 0; off < ranges[r].count; off += CHUNK) {
      uint16_t a = ranges[r].start + off;
      uint16_t n = min<uint16_t>(CHUNK, ranges[r].count - off);
      pause(INTER_MS);
      uint8_t res = node.readHoldingRegisters(a, n);
      if (res == node.ku8MBSuccess) {
        for (uint16_t i = 0; i < n; i++) {
          addrs[idx + i] = a + i;
          vals[idx + i] = node.getResponseBuffer(i);
          okReg[idx + i] = true;
        }
      } else if (!firstScanDone) {
        // перший прохід: блок не прочитався, пробуємо кожен регістр окремо
        for (uint16_t i = 0; i < n; i++) {
          addrs[idx + i] = a + i;
          pause(INTER_MS);
          uint8_t r1 = node.readHoldingRegisters(a + i, 1);
          okReg[idx + i] = (r1 == node.ku8MBSuccess);
          if (okReg[idx + i]) vals[idx + i] = node.getResponseBuffer(0);
        }
      } else {
        for (uint16_t i = 0; i < n; i++) { addrs[idx + i] = a + i; okReg[idx + i] = false; }
      }
      idx += n;
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

// ---------- MQTT / Home Assistant ----------
void publishDiscovery() {
  char topic[96], payload[700], tpl[160];
  const char* dev =
    "\"dev\":{\"ids\":[\"" DEV "\"],\"name\":\"Must PV18-6048 ECO\","
    "\"mf\":\"Must\",\"mdl\":\"PV18-6048 ECO\"}";

  // режим живлення (з'являється в HA одразу, навіть без відповіді інвертора)
  snprintf(payload, sizeof(payload),
    "{\"name\":\"Режим\",\"uniq_id\":\"" DEV "_mode\",\"stat_t\":\"" DEV "/mode\",%s}", dev);
  mqtt.publish("homeassistant/sensor/" DEV "_mode/config", payload, true);

  for (int i = 0; i < nTotal; i++) {
    if (!okReg[i]) continue;
    snprintf(topic, sizeof(topic), "homeassistant/sensor/" DEV "_r%u/config", addrs[i]);
    const Known* k = findKnown(addrs[i]);
    if (k) {
      snprintf(tpl, sizeof(tpl), "{%% set v = value|int(0) %%}{{ (%s * %g) | round(2) }}",
               k->sgn ? "((v-65536) if v>32767 else v)" : "v", k->scale);
      snprintf(payload, sizeof(payload),
        "{\"name\":\"%s\",\"uniq_id\":\"" DEV "_r%u\",\"stat_t\":\"" DEV "/reg/%u\","
        "\"avty_t\":\"" DEV "/status\",\"stat_cla\":\"measurement\","
        "\"val_tpl\":\"%s\"%s%s%s,%s}",
        k->name, addrs[i], addrs[i], tpl,
        k->unit[0] ? ",\"unit_of_meas\":\"" : "", k->unit, k->unit[0] ? "\"" : "", dev);
    } else {
      snprintf(payload, sizeof(payload),
        "{\"name\":\"Reg %u\",\"uniq_id\":\"" DEV "_r%u\",\"stat_t\":\"" DEV "/reg/%u\","
        "\"avty_t\":\"" DEV "/status\",\"stat_cla\":\"measurement\",%s}",
        addrs[i], addrs[i], addrs[i], dev);
    }
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
    Log.println("[MQTT] підключено");
    mqtt.publish(DEV "/status", "online", true);
    publishDiscovery();
    logf("[BOOT] reset=%d rssi=%d", (int)esp_reset_reason(), WiFi.RSSI());
    logf("[RO] лінія приймача: високий рівень %d з 500 (без шуму має бути 500)", roHigh);
  } else {
    Log.printf("[MQTT] помилка, state=%d\n", mqtt.state());
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

// ---------- режим (за напругою мережі) + Telegram ----------
void updateMode() {
  uint16_t gv;
  if (!getReg(REG_GRID_V, gv)) return;
  Mode m = (gv > GRID_OK_RAW) ? GRID : BATTERY;   // BATTERY = мережі немає
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

// ---------- setup / loop ----------
void setup() {
  Serial.begin(115200);
  delay(1500);
  Log.printf("\nПричина скидання: %d (1=живлення, 3=програмне, 4=паніка, 6-7=watchdog, 9=brownout)\n",
                (int)esp_reset_reason());

  pinMode(DE_RE, OUTPUT);
  digitalWrite(DE_RE, LOW);          // режим приймання

  // Тест лінії RO до запуску UART: без шуму на лінії має бути 500 з 500
  pinMode(RS485_RX, INPUT);
  int hi = 0;
  for (int i = 0; i < 500; i++) { hi += digitalRead(RS485_RX); delay(1); }
  roHigh = hi;
  Log.printf("[RO] високий рівень %d з 500\n", roHigh);

  Serial2.begin(BAUD, SERIAL_8N1, RS485_RX, RS485_TX);
  node.begin(SLAVE_ID, Serial2);
  node.preTransmission(preTx);
  node.postTransmission(postTx);

  tgClient.setInsecure();
  tgClient.setTimeout(5);            // секунди; щоб заблокований Telegram не вішав усе

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setBufferSize(1024);
  mqtt.setKeepAlive(60);

  ensureWifi();
  Log.printf("[WiFi] %s\n", WiFi.status() == WL_CONNECTED
                ? WiFi.localIP().toString().c_str() : "НЕ підключено");
  startOtaIfReady();

  if (WiFi.status() == WL_CONNECTED) {
    String s = "✅ ESP32 запущено\nIP: " + WiFi.localIP().toString() +
               " (" + String(WiFi.RSSI()) + " дБм)";
    bool ok = bot.sendMessage(CHAT_ID, s, "");
    Log.printf("[TG] старт: %s\n", ok ? "надіслано" : "ПОМИЛКА");
    if (!ok) notify(s);
  }
}

void loop() {
  static uint32_t lastPoll = 0;
  static uint32_t lastWifiCheck = 0;

  if (millis() - lastWifiCheck > 30000) {
    lastWifiCheck = millis();
    ensureWifi();
    startOtaIfReady();
  }

  mqttConnect();
  service();

  if (millis() - lastPoll > POLL_MS) {
    scanRegisters();
    lastPoll = millis();

    int good = 0;
    for (int i = 0; i < nTotal; i++) good += okReg[i];
    Log.printf("регістрів відповіло: %d з %d\n", good, nTotal);

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