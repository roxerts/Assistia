/* ===================== ESP-1 (CONTROL + WIFI SYNC) =========================
   - Implementación de comandos: IR, STOP, OPEN, MOVE, TURN
   - Se conecta a WiFi "Galaxy" para sincronizar canal.
   - Recibe control_msg por ESP-NOW.
============================================================================ */

#include <Arduino.h>
#include <SPI.h>
#include <MFRC522.h>
#include <ESP32Servo.h>
#include <esp_now.h>
#include <WiFi.h>

// ============== CREDENCIALES WIFI ==============
const char* ssid = "Galaxy";      // ⚠ DEBE SER IGUAL QUE EN ESP-2
const char* password = "12345678";

// ---------------- PINS ----------------
#define IN1 27
#define IN2 14
#define IN3 13
#define IN4 12

#define ENCODER_A1 33
#define ENCODER_A2 32
#define ENCODER_B1 26
#define ENCODER_B2 25

#define TRIG 15
#define ECHO 2

#define SS_PIN 5
#define RST_PIN 22
#define SCK_PIN 21
#define MOSI_PIN 23
#define MISO_PIN 19

#define SERVO_PIN 18
#define BUZZER_PIN 4

// ---------------- PWM / PID params ----------------
#define LEDC_BASE_FREQ 5000
#define LEDC_TIMER_BIT 8
#define PWM_CHANNEL_IN1 4
#define PWM_CHANNEL_IN2 5
#define PWM_CHANNEL_IN3 6
#define PWM_CHANNEL_IN4 7

float Kp = 5.2;
float Ki = 0.0001;
float Kd = 0.3;

#define BASE_PWM 75
#define MAX_PWM 210
#define MIN_DEADZONE 35      // Mínima potencia para que las ruedas giren
#define MAX_CORRECCION 80

// ---------------- hardware objects ----------------
MFRC522 rfid(SS_PIN, RST_PIN);
Servo servoMotor;

// ---------------- estado y tiempos ----------------
volatile long pulsosA = 0;
volatile long pulsosB = 0;
float integral = 0;
float errorAnterior = 0;
float salidaPID = 0;
bool habilitado = false;

enum EstadoRobot { ESTACIONADO, AVANZANDO, GIRANDO, DETENIDO_OBSTACULO, ABRIENDO_CAJA, CERRANDO_CAJA, ESPERANDO_CAJA };
EstadoRobot estadoMaquina = ESTACIONADO;

byte UID_C1[4] = {0x73,0x69,0x8A,0xBD};
byte UID_C2[4] = {0x83,0xF4,0x8B,0xBD};
byte UID_C3[4] = {0xA3,0xC9,0x8E,0xBD};

int estadoRuta = 0;
int anguloServo = 90;
unsigned long tiempoInicioAccion = 0;
unsigned long duracionGiro = 225; // ms
unsigned long ultimoPID = 0;
unsigned long ultimoTele = 0;
const long INTERVALO_PID = 20;
const long INTERVALO_TELEMETRIA = 500; // Actualizado a 500ms para mejor respuesta visual

// ---------------- ESP-NOW message structs ----------------
typedef struct {
  uint8_t cmd;    // 1=IR, 2=STOP, 3=OPEN, 5=MOVE, 6=TURN
  int16_t p1;     // Valor principal (-255 a 255 o ID cuarto)
  int16_t p2;     // Auxiliar
} control_msg;

typedef struct {
  uint8_t estadoMaquina;
  int16_t distancia;
  int32_t pulsosA;
  int32_t pulsosB;
  uint8_t anguloServo;
} telemetry_msg;

typedef struct {
  uint8_t tipo;   // 1 = estado, 2 = sensor
  int16_t destino;
  int32_t pos;    
  char rfid[16];
  uint8_t obstaculo; // 0/1
} status_msg;

// ---------------- MAC de ESP-2 ----------------
uint8_t macESP2[6] = { 0x3C, 0x8A, 0x1F, 0xD4, 0x8E, 0x74 }; // ⚠ VERIFICAR MAC

// ---------------- variables ESP-NOW ----------------
control_msg incomingControl;
volatile bool newControl = false;

// ---------------- Declaración de Funciones ----------------
void enviarEstado(int destino, long pos, const char* rfid); // Prototipo necesario

// ---------------- ISRs encoders ----------------
void IRAM_ATTR contarA() {
  int a = digitalRead(ENCODER_A1);
  int b = digitalRead(ENCODER_A2);
  if (a == b) pulsosA++; else pulsosA--;
}
void IRAM_ATTR contarB() {
  int a = digitalRead(ENCODER_B1);
  int b = digitalRead(ENCODER_B2);
  if (a == b) pulsosB++; else pulsosB--;
}

// ---------------- ESP-NOW callbacks ----------------
void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  if (len == sizeof(control_msg)) {
    memcpy(&incomingControl, incomingData, sizeof(control_msg));
    newControl = true;
  }
}
void onSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Callback opcional
}

// ---------------- utilidades ----------------
void detener() {
  ledcWrite(PWM_CHANNEL_IN1, 0);
  ledcWrite(PWM_CHANNEL_IN2, 0);
  ledcWrite(PWM_CHANNEL_IN3, 0);
  ledcWrite(PWM_CHANNEL_IN4, 0);
}

void resetPID() {
  noInterrupts();
  pulsosA = 0; pulsosB = 0;
  interrupts();
  integral = 0; errorAnterior = 0;
}

float medirDistancia() {
  digitalWrite(TRIG, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG, LOW);
  long duracion = pulseIn(ECHO, HIGH, 25000);
  if (duracion == 0) return 999;
  return duracion * 0.034 / 2.0;
}

bool compararUID(byte *a, byte *b) { for (int i=0;i<4;i++) if (a[i]!=b[i]) return false; return true; }

void iniciarAperturaServo() {
  estadoMaquina = ABRIENDO_CAJA;
  anguloServo = 90;
  tiempoInicioAccion = millis();
  detener();
  servoMotor.write(90);
  enviarEstado(estadoRuta, pulsosA, "OPEN_SEQ");
}

// PID y control motores
void avanzarPID() {
  static unsigned long lastMicros = micros();
  unsigned long now = micros();
  float dt = (now - lastMicros) / 1e6;
  lastMicros = now;
  if (dt <= 0 || dt > 0.1) dt = 0.02;

  static long lastA = 0, lastB = 0;
  long curA, curB;
  noInterrupts();
  curA = pulsosA; curB = pulsosB;
  interrupts();

  float velA = (curA - lastA) / dt;
  float velB = (curB - lastB) / dt;
  lastA = curA; lastB = curB;

  static float fA = 0, fB = 0;
  const float alpha = 0.85;
  fA = alpha*fA + (1-alpha)*velA; fB = alpha*fB + (1-alpha)*velB;

  float error = fA - fB;
  integral += error * dt; integral = constrain(integral, -80, 80);
  float derivada = (error - errorAnterior)/dt; derivada = constrain(derivada, -200, 200);
  salidaPID = Kp*error + Ki*integral + Kd*derivada; salidaPID = constrain(salidaPID, -MAX_CORRECCION, MAX_CORRECCION);
  errorAnterior = error;

  int pwmL = BASE_PWM - (int)round(salidaPID);
  int pwmR = BASE_PWM + (int)round(salidaPID);
  
  pwmL = constrain(pwmL, 0, MAX_PWM);
  pwmR = constrain(pwmR, 0, MAX_PWM);
  
  // Deadzone fix
  if (pwmL>0 && pwmL<MIN_DEADZONE) pwmL = MIN_DEADZONE;
  if (pwmR>0 && pwmR<MIN_DEADZONE) pwmR = MIN_DEADZONE;

  ledcWrite(PWM_CHANNEL_IN1, 0);
  ledcWrite(PWM_CHANNEL_IN2, pwmL);
  ledcWrite(PWM_CHANNEL_IN3, 0);
  ledcWrite(PWM_CHANNEL_IN4, pwmR);
}

// ---------------- Status / Telemetría ----------------
void enviarTelemetria() {
  telemetry_msg t;
  t.estadoMaquina = (uint8_t) estadoMaquina;
  float d = medirDistancia();
  t.distancia = (int16_t)d;
  noInterrupts();
  t.pulsosA = pulsosA;
  t.pulsosB = pulsosB;
  interrupts();
  t.anguloServo = (uint8_t) anguloServo;
  esp_now_send(macESP2, (uint8_t*)&t, sizeof(t));
}

void enviarEstado(int destino, long pos, const char* rfid) {
  status_msg msg;
  msg.tipo = 1;
  msg.destino = destino;
  msg.pos = (int32_t)pos; 
  strncpy(msg.rfid, rfid ? rfid : "NONE", sizeof(msg.rfid)-1);
  msg.rfid[sizeof(msg.rfid)-1] = '\0';
  msg.obstaculo = 0;
  esp_now_send(macESP2, (uint8_t*)&msg, sizeof(msg));
}

void enviarSensor(bool hayObstaculo) {
  status_msg msg;
  msg.tipo = 2;
  msg.destino = estadoRuta;
  msg.pos = pulsosA;
  msg.rfid[0] = '\0';
  msg.obstaculo = hayObstaculo ? 1 : 0;
  esp_now_send(macESP2, (uint8_t*)&msg, sizeof(msg));
}

// =================================================================================
// 🚀 IMPLEMENTACIÓN DE COMANDOS SOLICITADA
// =================================================================================
void aplicarComando() {
  if (!newControl) return;
  newControl = false;

  uint8_t cmd = incomingControl.cmd;
  int16_t p1 = incomingControl.p1;

  Serial.printf("CMD recibido: %d, P1: %d\n", cmd, p1);

  // --- CMD 1: IR A CUARTO (AUTO) ---
  if (cmd == 1) { 
    estadoRuta = p1;
    habilitado = true;
    resetPID(); // Importante resetear PID al iniciar nueva ruta
    estadoMaquina = AVANZANDO;
    enviarEstado(estadoRuta, 0, "START_AUTO");
  } 

  // --- CMD 2: PARO DE EMERGENCIA ---
  else if (cmd == 2) { 
    habilitado = false;
    detener();
    estadoRuta = 0;
    estadoMaquina = ESTACIONADO;
    enviarEstado(0, 0, "STOP_EMG");
  } 

  // --- CMD 3: ABRIR CAJA MANUALMENTE ---
  else if (cmd == 3) { 
    // Solo permitir si no se está moviendo
    detener();
    iniciarAperturaServo();
    enviarEstado(estadoRuta, pulsosA, "OPEN_MANUAL");
  } 

  // --- CMD 5: CONTROL MANUAL - AVANZAR/RETROCEDER ---
  else if (cmd == 5) { 
    estadoMaquina = ESTACIONADO; // Desactiva PID y lógica automática
    
    // Deadzone check: Si el valor es muy bajo, parar
    if (abs(p1) < 10) {
      detener();
      return;
    }

    // Mapeo inteligente: 0-255 -> MIN_DEADZONE-MAX_PWM
    int pwm = map(abs(p1), 0, 255, MIN_DEADZONE, MAX_PWM);
    pwm = constrain(pwm, 0, MAX_PWM);

    if (p1 > 0) { // Avanzar
      ledcWrite(PWM_CHANNEL_IN1, 0); ledcWrite(PWM_CHANNEL_IN2, pwm);
      ledcWrite(PWM_CHANNEL_IN3, 0); ledcWrite(PWM_CHANNEL_IN4, pwm);
    } else { // Retroceder
      ledcWrite(PWM_CHANNEL_IN1, pwm); ledcWrite(PWM_CHANNEL_IN2, 0);
      ledcWrite(PWM_CHANNEL_IN3, pwm); ledcWrite(PWM_CHANNEL_IN4, 0);
    }
  }

  // --- CMD 6: CONTROL MANUAL - GIRAR IZQ/DER ---
  else if (cmd == 6) {
    estadoMaquina = ESTACIONADO; // Desactiva PID

    if (abs(p1) < 10) {
      detener();
      return;
    }

    int pwm = map(abs(p1), 0, 255, MIN_DEADZONE, MAX_PWM);
    pwm = constrain(pwm, 0, MAX_PWM);

    if (p1 > 0) { // Derecha (Izq avanza, Der retrocede)
      ledcWrite(PWM_CHANNEL_IN1, 0); ledcWrite(PWM_CHANNEL_IN2, pwm);
      ledcWrite(PWM_CHANNEL_IN3, pwm); ledcWrite(PWM_CHANNEL_IN4, 0);
    } else { // Izquierda (Izq retrocede, Der avanza)
      ledcWrite(PWM_CHANNEL_IN1, pwm); ledcWrite(PWM_CHANNEL_IN2, 0);
      ledcWrite(PWM_CHANNEL_IN3, 0); ledcWrite(PWM_CHANNEL_IN4, pwm);
    }
  }
}

// ---------------- SETUP ----------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== ESP-1 CONTROL + WIFI STARTING ===");

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print("."); }
  Serial.println("\n✅ WiFi conectado.");
  Serial.print("Canal: "); Serial.println(WiFi.channel()); 

  pinMode(BUZZER_PIN, OUTPUT); noTone(BUZZER_PIN);
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, SS_PIN);
  rfid.PCD_Init();

  ESP32PWM::allocateTimer(0); ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2); ESP32PWM::allocateTimer(3);
  servoMotor.setPeriodHertz(50);
  servoMotor.attach(SERVO_PIN, 500, 2400);
  servoMotor.write(90);

  ledcSetup(PWM_CHANNEL_IN1, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(PWM_CHANNEL_IN2, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(PWM_CHANNEL_IN3, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcSetup(PWM_CHANNEL_IN4, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
  ledcAttachPin(IN1, PWM_CHANNEL_IN1); ledcAttachPin(IN2, PWM_CHANNEL_IN2);
  ledcAttachPin(IN3, PWM_CHANNEL_IN3); ledcAttachPin(IN4, PWM_CHANNEL_IN4);

  pinMode(ENCODER_A1, INPUT_PULLUP); pinMode(ENCODER_A2, INPUT_PULLUP);
  pinMode(ENCODER_B1, INPUT_PULLUP); pinMode(ENCODER_B2, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A1), contarA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B1), contarB, CHANGE);

  pinMode(TRIG, OUTPUT); pinMode(ECHO, INPUT);

  resetPID();

  if (esp_now_init() != ESP_OK) { Serial.println("❌ ESP-NOW init failed"); }
  else {
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onSend);
    Serial.println("✅ ESP-NOW OK.");
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, macESP2, 6);
  peerInfo.channel = 0; 
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) == ESP_OK) { Serial.println("✅ Peer ESP-2 OK."); } 
  else { Serial.println("❌ Error Peer ESP-2"); }
}

// ---------------- LOOP ----------------
void loop() {
  unsigned long ahora = millis();

  if (newControl) aplicarComando();

  if (ahora - ultimoTele >= INTERVALO_TELEMETRIA) {
    enviarTelemetria();
    ultimoTele = ahora;
  }

  // Máquina de estados
  switch (estadoMaquina) {
    case ESTACIONADO:
      // En este estado, los motores se controlan SOLO manualmente (cmd 5 o 6)
      // Si no hay comando manual activo, el último comando aplicado persiste
      // hasta que se envíe 0 (Stop manual).
      break;

    case AVANZANDO: {
      if (!habilitado) { detener(); break; }
      float d = medirDistancia();
      if (d < 20) {
        Serial.println("OBSTACULO");
        enviarSensor(true);
        detener();
        estadoMaquina = DETENIDO_OBSTACULO;
        break;
      }
      if (millis() - ultimoPID >= INTERVALO_PID) {
        avanzarPID();
        ultimoPID = millis();
      }
      break;
    }

    case DETENIDO_OBSTACULO:
      detener();
      break;

    case GIRANDO: {
       static unsigned long start = 0;
       if (start == 0) start = millis();
       if (millis() - start < duracionGiro) {
          // Giro preprogramado (ejemplo 90 grados a la ciega)
          ledcWrite(PWM_CHANNEL_IN1, 120); ledcWrite(PWM_CHANNEL_IN2, 0);
          ledcWrite(PWM_CHANNEL_IN3, 0); ledcWrite(PWM_CHANNEL_IN4, 120);
       } else {
          detener();
          resetPID();
          estadoMaquina = AVANZANDO;
          start = 0;
       }
       break;
    }

    case ABRIENDO_CAJA: {
      unsigned long t = millis() - tiempoInicioAccion;
      if (t < 2150) {
         int ts = t > 150 ? t - 150 : 0;
         anguloServo = map(ts, 0, 2000, 90, 180);
         anguloServo = constrain(anguloServo, 90, 180);
         servoMotor.write(anguloServo);
      } else if (t < 7150) {
         servoMotor.write(180); // Mantener abierto
      } else {
         estadoMaquina = CERRANDO_CAJA;
         tiempoInicioAccion = millis();
      }
      break;
    }

    case CERRANDO_CAJA: {
      unsigned long t = millis() - tiempoInicioAccion;
      if (t < 2000) {
        anguloServo = map(t, 0, 2000, 180, 90);
        anguloServo = constrain(anguloServo, 90, 180);
        servoMotor.write(anguloServo);
      } else {
        servoMotor.write(90);
        detener();
        resetPID();
        habilitado = false;
        enviarEstado(estadoRuta, pulsosA, "FIN");
        estadoRuta = 0;
        estadoMaquina = ESTACIONADO;
      }
      break;
    }
  }

  // RFID scan
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    byte u[4];
    for (int i=0;i<4;i++) u[i] = rfid.uid.uidByte[i];
    
    // Si estamos en modo automático y en ruta 1, y leemos tarjeta 1:
    if (compararUID(u, UID_C1)) {
        if (estadoRuta == 1 && estadoMaquina == AVANZANDO) { 
            iniciarAperturaServo(); 
            enviarEstado(1, pulsosA, "C1_FOUND"); 
        }
    } 
    rfid.PICC_HaltA(); rfid.PCD_StopCrypto1();
  }
  
  delay(1);
}