#include <WiFi.h>
#include <PubSubClient.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <DNSServer.h>

// Конфигурация
#define RELAY_PIN_COUNT 8
const uint8_t relayPins[RELAY_PIN_COUNT] = {13, 15, 2, 4, 16, 17, 5, 18};
bool globalActiveHigh = true; // Глобальный активный уровень (HIGH/LOW)

// Настройки MQTT
#define MQTT_CLIENT_ID "ESP32_Relay_Controller"
#define MQTT_TOPIC_PREFIX "home/relays/"

// Настройки точки доступа
#define AP_SSID "RelayController"
#define AP_PASS "config1234"

// Глобальные переменные
bool relayStates[RELAY_PIN_COUNT] = {false};
bool lastStates[RELAY_PIN_COUNT] = {false};
bool firstPublish = true;
bool isMyMessage = false;

// Параметры подключения
String ssid = "";
String password = "";
String mqtt_server = "mqtt.eclipseprojects.io";
String mqtt_user = "";
String mqtt_password = "";

WiFiClient wifiClient;
PubSubClient client(wifiClient);
AsyncWebServer server(80);
DNSServer dns;
bool shouldReboot = false;
unsigned long lastMqttReconnect = 0;

void setup() {
  // Инициализация пинов реле
  for (uint8_t i = 0; i < RELAY_PIN_COUNT; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], globalActiveHigh ? LOW : HIGH);
  }

  // Инициализация файловой системы
  if (!LittleFS.begin()) {
    return;
  }

  loadConfiguration();  // Загрузка настроек сети и MQTT
  loadRelayStates();    // Загрузка состояний реле

  // Настройка WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  // Настройка MQTT
  client.setServer(mqtt_server.c_str(), 1883);
  client.setCallback(mqttCallback);

  // Маршруты веб-сервера
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->redirect("/control");
  });

  server.on("/config", HTTP_GET, handleConfig);
  server.on("/control", HTTP_GET, handleControl);
  server.on("/save", HTTP_POST, handleSaveSettings);
  server.on("/relay", HTTP_GET, handleRelayRequest);
  server.on("/relaystate", HTTP_GET, handleRelayStateRequest);
  server.onNotFound(handleNotFound);

  dns.start(53, "*", IPAddress(192, 168, 4, 1));
  server.begin();
}

void loop() {
  handleWiFiConnection();
  handleMqttConnection();
  handleDNSServer();
  
  if (shouldReboot) {
    delay(1000);
    ESP.restart();
  }
}

// Обработка подключения к WiFi
void handleWiFiConnection() {
  static unsigned long lastCheck = 0;
  if (millis() - lastCheck < 10000) return;
  lastCheck = millis();

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
    if (WiFi.waitForConnectResult() != WL_CONNECTED) {
      startAPMode();
    }
  }
}

// Переключение в режим точки доступа
void startAPMode() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASS);
}

// Обработка подключения к MQTT
void handleMqttConnection() {
  if (!client.connected() && millis() - lastMqttReconnect > 5000) {
    mqttReconnect();
    lastMqttReconnect = millis();
  }
  client.loop();
}

// Переподключение к MQTT
void mqttReconnect() {
  if (client.connect(MQTT_CLIENT_ID, mqtt_user.c_str(), mqtt_password.c_str())) {
    subscribeToTopics();
    firstPublish = true;
    publishRelayStates();
  }
}

// Подписка на MQTT-топики
void subscribeToTopics() {
  for (uint8_t i = 0; i < RELAY_PIN_COUNT; i++) {
    char topic[32];
    snprintf(topic, sizeof(topic), MQTT_TOPIC_PREFIX "relay%d", i);
    client.subscribe(topic);
  }
}

// Обработчик MQTT-сообщений
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (isMyMessage || length < 2 || length > 3) return;

  String message;
  for (unsigned i = 0; i < length; i++) {
    message += (char)toupper(payload[i]);
  }
  message.trim();

  if (message != "ON" && message != "OFF") return;

  for (uint8_t i = 0; i < RELAY_PIN_COUNT; i++) {
    char t[32];
    snprintf(t, sizeof(t), MQTT_TOPIC_PREFIX "relay%d", i);
    if (String(topic) == t) {
      updateRelayState(i, message == "ON");
      break;
    }
  }
}

// Обновление состояния реле
void updateRelayState(uint8_t relay, bool state) {
  if (relay >= RELAY_PIN_COUNT) return;

  if (relayStates[relay] != state) {
    relayStates[relay] = state;
    
    // Управление с учетом глобального активного уровня
    digitalWrite(relayPins[relay], 
      (globalActiveHigh && state) || (!globalActiveHigh && !state) ? HIGH : LOW);
    
    saveRelayStates();
    publishSingleRelayState(relay);
  }
}

// Публикация состояний всех реле
void publishRelayStates() {
  isMyMessage = true;
  for (uint8_t i = 0; i < RELAY_PIN_COUNT; i++) {
    if (firstPublish || lastStates[i] != relayStates[i]) {
      publishSingleRelayState(i);
      lastStates[i] = relayStates[i];
    }
  }
  firstPublish = false;
  isMyMessage = false;
}

// Публикация состояния одного реле
void publishSingleRelayState(uint8_t relay) {
  if (relay >= RELAY_PIN_COUNT) return;
  
  char topic[32];
  snprintf(topic, sizeof(topic), MQTT_TOPIC_PREFIX "relay%d", relay);
  client.publish(topic, relayStates[relay] ? "ON" : "OFF", false);
}

// Загрузка настроек из файла
void loadConfiguration() {
  File configFile = LittleFS.open("/config.json", "r");
  if (!configFile) return;

  DynamicJsonDocument doc(1024);
  DeserializationError error = deserializeJson(doc, configFile);
  if (error) {
    configFile.close();
    return;
  }

  ssid = doc["ssid"] | "";
  password = doc["password"] | "";
  mqtt_server = doc["mqtt_server"] | "mqtt.eclipseprojects.io";
  mqtt_user = doc["mqtt_user"] | "";
  mqtt_password = doc["mqtt_password"] | "";
  globalActiveHigh = doc["active_high"] | true;

  configFile.close();
}

// Загрузка состояний реле
void loadRelayStates() {
  File stateFile = LittleFS.open("/states.json", "r");
  if (!stateFile) return;

  DynamicJsonDocument doc(512);
  DeserializationError error = deserializeJson(doc, stateFile);
  if (error) {
    stateFile.close();
    return;
  }

  for (uint8_t i = 0; i < RELAY_PIN_COUNT; i++) {
    relayStates[i] = doc["relay" + String(i)];
    digitalWrite(relayPins[i], 
      (globalActiveHigh && relayStates[i]) || (!globalActiveHigh && !relayStates[i]) ? HIGH : LOW);
  }

  stateFile.close();
}

// Сохранение состояний реле
void saveRelayStates() {
  DynamicJsonDocument doc(512);
  for (uint8_t i = 0; i < RELAY_PIN_COUNT; i++) {
    doc["relay" + String(i)] = relayStates[i];
  }
  
  File stateFile = LittleFS.open("/states.json", "w");
  if (stateFile) {
    serializeJson(doc, stateFile);
    stateFile.close();
  }
}

// Обработчики веб-интерфейса
void handleConfig(AsyncWebServerRequest *request) {
  File file = LittleFS.open("/config.html", "r");
  if (!file) {
    request->send(404, "text/plain; charset=UTF-8", "Файл не найден. Скорее всего не загружена папка Data");
    return;
  }
  
  String html = file.readString();
  html.replace("{{ ssid }}", ssid);
  html.replace("{{ password }}", password);
  html.replace("{{ mqtt_server }}", mqtt_server);
  html.replace("{{ mqtt_user }}", mqtt_user);
  html.replace("{{ mqtt_password }}", mqtt_password);
  html.replace("{{ selected_high }}", globalActiveHigh ? "selected" : "");
  html.replace("{{ selected_low }}", globalActiveHigh ? "" : "selected");
  
  request->send(200, "text/html", html);
  file.close();
}

void handleControl(AsyncWebServerRequest *request) {
  File file = LittleFS.open("/control.html", "r");
  if (!file) {
    request->send(404, "text/plain; charset=UTF-8", "Файл не найден. Скорее всего не загружена папка Data");
    return;
  }
  
  request->send(200, "text/html", file.readString());
  file.close();
}

void handleSaveSettings(AsyncWebServerRequest *request) {
  ssid = request->arg("ssid");
  password = request->arg("password");
  mqtt_server = request->arg("mqtt_server");
  mqtt_user = request->arg("mqtt_user");
  mqtt_password = request->arg("mqtt_password");
  globalActiveHigh = request->arg("active_high") == "true";

  DynamicJsonDocument doc(1024);
  doc["ssid"] = ssid;
  doc["password"] = password;
  doc["mqtt_server"] = mqtt_server;
  doc["mqtt_user"] = mqtt_user;
  doc["mqtt_password"] = mqtt_password;
  doc["active_high"] = globalActiveHigh;

  File configFile = LittleFS.open("/config.json", "w");
  if (configFile) {
    serializeJson(doc, configFile);
    configFile.close();
  }

  shouldReboot = true;
 request->send(200, "text/plain; charset=UTF-8", "Настройки сохранены. Перезагрузка...");
}

void handleRelayRequest(AsyncWebServerRequest *request) {
  if (!request->hasArg("num") || !request->hasArg("state")) {
    request->send(400, "text/plain", "Отсутствуют параметры");
    return;
  }

  int relay = request->arg("num").toInt();
  String state = request->arg("state");
  state.toUpperCase();

  if (state != "ON" && state != "OFF") {
    request->send(400, "text/plain; charset=UTF-8", "Неверное состояние");
    return;
  }

  updateRelayState(relay, state == "ON");
  request->send(200, "text/plain; charset=UTF-8", "Реле обновлено");
}

void handleRelayStateRequest(AsyncWebServerRequest *request) {
  if (!request->hasArg("num")) {
    request->send(400, "text/plain; charset=UTF-8", "Отсутствует параметр num");
    return;
  }

  int relay = request->arg("num").toInt();
  if (relay < 0 || relay >= RELAY_PIN_COUNT) {
    request->send(400, "text/plain; charset=UTF-8", "Неверный номер реле");
    return;
  }

  String stateStr = relayStates[relay] ? "ON" : "OFF";
  request->send(200, "application/json", "{\"state\": \"" + stateStr + "\"}");
}

void handleNotFound(AsyncWebServerRequest *request) {
  request->send(404, "text/plain; charset=UTF-8", "Не найдено");
}

void handleDNSServer() {
  if (WiFi.getMode() == WIFI_AP) {
    dns.processNextRequest();
  }
}