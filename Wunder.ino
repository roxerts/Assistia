/* ===================== ESP-2 (MQTT BRIDGE) =========
   - Conectada a WiFi y broker MQTT
   - Recibe comandos desde el HTML vía MQTT
   - Envía comandos al ESP-1 vía ESP-NOW
   - Publica estado del robot de vuelta al HTML
============================================================================== */

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <PubSubClient.h>

// ============== CONFIGURACIÓN WiFi ==============
const char* ssid = "Galaxy";          
const char* password = "12345678";   

// ============== CONFIGURACIÓN MQTT ==============
const char* mqtt_server = "192.168.4.159";  
const int mqtt_port = 1884;

const char* TOPIC_CONTROL = "/robot/control";
const char* TOPIC_STATUS = "/robot/status";
const char* TOPIC_SENSOR = "/robot/sensores";

WiFiClient espClient;
PubSubClient mqttClient(espClient);

// ============== CONFIGURACIÓN ESP-NOW ==============
uint8_t peerMac[6] = {0x4C, 0xC3, 0x82, 0x0C, 0x83, 0x7C}; //MAC del ESP-1

typedef struct {
  uint8_t cmd;    // 1 = IR_CUARTO, 2 = STOP, 3 = OPEN, 5 = MOVE, 6 = TURN
  int16_t p1;     // número de cuarto / velocidad
  int16_t p2;
} control_msg;

typedef struct {
  uint8_t tipo;   // 1 = estado, 2 = sensor
  int16_t destino;
  int32_t pos;
  char rfid[16];
  uint8_t obstaculo;
} status_msg;

// ============== FUNCIONES ESP-NOW ==============
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW Send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  if (len != sizeof(status_msg)) return;
  
  status_msg msg;
  memcpy(&msg, incomingData, sizeof(msg));
  
  Serial.println("📥 Recibido estado del ESP-1");
  
  // Publicar en MQTT para que el HTML lo vea
  if (msg.tipo == 1) {
    // Estado general
    char payload[200];
    snprintf(payload, sizeof(payload), 
             "{\"destino\":%d,\"pos\":%d,\"rfid\":\"%s\"}", 
             msg.destino, msg.pos, msg.rfid);
    mqttClient.publish(TOPIC_STATUS, payload);
    Serial.print("📤 MQTT Status: "); Serial.println(payload);
  } 
  else if (msg.tipo == 2) {
    // Sensores
    char payload[100];
    snprintf(payload, sizeof(payload), 
             "{\"obstaculo\":%s}", 
             msg.obstaculo ? "true" : "false");
    mqttClient.publish(TOPIC_SENSOR, payload);
    Serial.print("📤 MQTT Sensor: "); Serial.println(payload);
  }
}

void initESPNOW() {
  if (esp_now_init() != ESP_OK) {
    Serial.println("❌ ESP-NOW init failed");
    return;
  }
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);
  
  // Agregar peer (ESP-1)
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("✅ ESP-NOW peer agregado");
  } else {
    Serial.println("❌ Error agregando peer");
  }
}

// ============== FUNCIONES WiFi ==============
void initWiFi() {
  Serial.print("Conectando a WiFi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("\n✅ WiFi conectado");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
}

// ============== FUNCIONES MQTT ==============
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) message += (char)payload[i];
  
  Serial.print("📥 MQTT: "); Serial.println(message);
  
  if (String(topic) == TOPIC_CONTROL) {
    // Extracción manual de JSON (busca "cmd" y "val" o "cuarto")
    String cmdStr = "";
    int val = 0;
    
    // 1. Extraer CMD
    int cmdIdx = message.indexOf("\"cmd\"");
    if (cmdIdx != -1) {
      int start = message.indexOf("\"", cmdIdx + 6) + 1;
      int end = message.indexOf("\"", start);
      cmdStr = message.substring(start, end);
    }
    
    // 2. Extraer VALOR (puede venir como "val" o "cuarto")
    int valIdx = message.indexOf("\"val\"");
    if (valIdx == -1) valIdx = message.indexOf("\"cuarto\""); // Compatibilidad
    
    if (valIdx != -1) {
      int numStart = message.indexOf(":", valIdx) + 1;
      int numEnd = message.indexOf(",", numStart);
      if (numEnd == -1) numEnd = message.indexOf("}", numStart);
      String numStr = message.substring(numStart, numEnd);
      numStr.trim();
      val = numStr.toInt();
    }
    
    // 3. Preparar mensaje ESP-NOW
    control_msg msg;
    msg.cmd = 0;
    msg.p1 = 0; 
    msg.p2 = 0;
    
    if (cmdStr == "IR_CUARTO") {
      msg.cmd = 1;
      msg.p1 = val;
      Serial.println("🚀 Modo: AUTOMATICO");
    }
    else if (cmdStr == "STOP") {
      msg.cmd = 2;
      Serial.println("⛔ STOP");
    }
    else if (cmdStr == "OPEN") { // Abrir caja manual
      msg.cmd = 3;
      Serial.println("📦 OPEN - Abrir caja");
    }
    else if (cmdStr == "MOVE") { // Manual Avanzar/Atras
      msg.cmd = 5;
      msg.p1 = val; // Velocidad PWM (-255 a 255)
      Serial.print("🎮 Manual Move: "); Serial.println(val);
    }
    else if (cmdStr == "TURN") { // Manual Giro
      msg.cmd = 6;
      msg.p1 = val; // Velocidad Giro (-255 a 255)
      Serial.print("🔄 Manual Turn: "); Serial.println(val);
    }
    
    // Enviar si hay comando válido
    if (msg.cmd > 0) {
      esp_now_send(peerMac, (uint8_t*)&msg, sizeof(msg));
    }
  }
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("Conectando a MQTT...");
    
    String clientId = "ESP32_Bridge_";
    clientId += String(random(0xffff), HEX);
    
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println(" ✅ conectado");
      mqttClient.subscribe(TOPIC_CONTROL);
      Serial.print("Suscrito a: "); Serial.println(TOPIC_CONTROL);
    } else {
      Serial.print(" ❌ falló, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" reintentando en 5s...");
      delay(5000);
    }
  }
}

// ============== SETUP ==============
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n=== ESP-2 MQTT BRIDGE ===");
  
  // 1. Conectar WiFi
  initWiFi();
  
  // 2. Configurar MQTT
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(mqttCallback);
  
  // 3. Inicializar ESP-NOW
  initESPNOW();
  
  Serial.println("✅ Sistema listo");
}

// ============== LOOP ==============
void loop() {
  // Mantener conexión MQTT
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();
  
  delay(10);
}