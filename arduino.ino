#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <HTTPUpdate.h>
#include <WebServer.h>  // NYTT: För att bygga det inbyggda gränssnittet
#include <HCSR04.h>


const byte DNS_PORT = 53;

// ==========================================
// NÄTVERKSINSTÄLLNINGAR & OTA
// ==========================================
const int WEB_PORT = 80;  // Standardport för webbplatser
const char* FIRMWARE_URL = "http://192.168.4.50:3000/firmware.bin";

const int safetyUDistance = 15;
const int safetyLDistance = 100;

WebServer webServer(WEB_PORT);  // NYTT: Starta webbservern

UltraSonicDistanceSensor distanceSensor1(4, 34);   // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor distanceSensor2(23, 35);  // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor distanceSensor3(25, 12);  // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor distanceSensor4(5, 32);   // Initialize sensor that uses digital pins 13 and 12.
UltraSonicDistanceSensor distanceSensor5(16, 33);  // Initialize sensor that uses digital pins 13 and 12.

// ==========================================
// MOTOR & PIN-KONFIGURATION
// ==========================================
#define LEFT_FWD 2
#define LEFT_BWD 17
#define RIGHT_FWD 18
#define RIGHT_BWD 19

int velocity = 500;

unsigned long autoStopTime = 0;
bool isAutoMode = false;
bool turnRightNext = true;  // Minnet för gräsklipparen (Scenario B)

// Kalibreringsvärden (Justera dessa efter din robot)
const int TIME_FOR_90_DEG = 2300;
const int TIME_FOR_FORWARD_STEP = 600;

// ==========================================
// VL53L0X KONFIGURATION
// ==========================================
const uint8_t SENSOR_COUNT = 4;
const int XSHUT_PINS[SENSOR_COUNT] = { 13, 14, 26, 27 };
const uint8_t NEW_ADDRESSES[SENSOR_COUNT] = { 0x30, 0x31, 0x32, 0x33 };
const uint8_t FRONT_SENSOR_END = 1;

const int DISTANCE_OFFSET_MM = 10;
const int EDGE_JUMP_THRESHOLD_MM = 120;
const int EDGE_MIN_DISTANCE_MM = 180;
const unsigned long SENSOR_READ_INTERVAL_MS = 50;

VL53L0X sensors[SENSOR_COUNT];
bool sensorInitialized[SENSOR_COUNT] = { false, false, false, false };
int lastDistanceMm[SENSOR_COUNT] = { -1, -1, -1, -1 };
int lastUDistance[5] = { -1, -1, -1, -1, -1 };
unsigned long lastUReadMs = 0;
unsigned long lastSensorReadMs = 0;

enum MotionState {
  MOTION_STOPPED,
  MOTION_FORWARD,
  MOTION_BACKWARD,
  MOTION_LEFT,
  MOTION_RIGHT
};
MotionState currentMotion = MOTION_STOPPED;

// ==========================================
// MOTORFUNKTIONER
// ==========================================

void updateUltrasonicSensors() {
  unsigned long now = millis();
  if (now - lastUReadMs < 100) return;  // Mät max var 100ms
  lastUReadMs = now;
  lastUDistance[0] = distanceSensor1.measureDistanceCm();
  lastUDistance[1] = distanceSensor2.measureDistanceCm();
  lastUDistance[2] = distanceSensor3.measureDistanceCm();
  lastUDistance[3] = distanceSensor4.measureDistanceCm();
  lastUDistance[4] = distanceSensor5.measureDistanceCm();
}

boolean checkForwardSafety() {
  int us2 = lastUDistance[1];
  int us3 = lastUDistance[2];
  int l0 = lastDistanceMm[0];
  int l1 = lastDistanceMm[1];
  if ((us2 > 0 && us2 < safetyUDistance) || (us3 > 0 && us3 < safetyUDistance) || l0 > safetyLDistance || l1 > safetyLDistance) {
    return false;
  }
  return true;
}

boolean checkBackwardsSafety() {
  int us1 = lastUDistance[0];
  int l2 = lastDistanceMm[2];
  int l3 = lastDistanceMm[3];
  if ((us1 > 0 && us1 < safetyUDistance) || l2 > safetyLDistance || l3 > safetyLDistance) {
    return false;
  }
  return true;
}

boolean checkRightSafety() {
  int lsensor1 = lastDistanceMm[0];
  int lsensor2 = lastDistanceMm[1];
  int lsensor3 = lastDistanceMm[2];
  int lsensor4 = lastDistanceMm[3];
  if (lsensor1 > safetyLDistance || lsensor2 > safetyLDistance || lsensor3 > safetyLDistance || lsensor4 > safetyLDistance) {
    return false;
  } else {
    return true;
  }
}

boolean checkLeftSafety() {
  int lsensor1 = lastDistanceMm[0];
  int lsensor2 = lastDistanceMm[1];
  int lsensor3 = lastDistanceMm[2];
  int lsensor4 = lastDistanceMm[3];
  if (lsensor1 > safetyLDistance || lsensor2 > safetyLDistance || lsensor3 > safetyLDistance || lsensor4 > safetyLDistance) {
    return false;
  } else {
    return true;
  }
}

void moveForward() {
  if (checkForwardSafety()) {
    analogWrite(LEFT_FWD, 0);
    analogWrite(LEFT_BWD, velocity);
    analogWrite(RIGHT_FWD, velocity);
    analogWrite(RIGHT_BWD, 0);
    currentMotion = MOTION_FORWARD;
  }
}
void moveBackward() {
  if (checkBackwardsSafety()) {
    analogWrite(LEFT_FWD, velocity);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_FWD, 0);
    analogWrite(RIGHT_BWD, velocity);
    currentMotion = MOTION_BACKWARD;
  }
}
void turnLeft() {
  if (checkLeftSafety()) {
    analogWrite(LEFT_FWD, 0);
    analogWrite(LEFT_BWD, velocity);
    analogWrite(RIGHT_FWD, 0);
    analogWrite(RIGHT_BWD, velocity);
    currentMotion = MOTION_LEFT;
  }
}
void turnRight() {
  if (checkRightSafety()) {
    analogWrite(LEFT_FWD, velocity);
    analogWrite(LEFT_BWD, 0);
    analogWrite(RIGHT_FWD, velocity);
    analogWrite(RIGHT_BWD, 0);
    currentMotion = MOTION_RIGHT;
  }
}
void stopAll() {
  analogWrite(LEFT_FWD, 0);
  analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0);
  analogWrite(RIGHT_BWD, 0);
  currentMotion = MOTION_STOPPED;
}

// ==========================================
// INBYGGT WEBBGRÄNSSNITT (Direkt på ESP32)
// ==========================================

// Den här HTML-koden skickas till din webbläsare
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="sv">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
  <title>Robotstyrning</title>
  <style>
    body {
      font-family: sans-serif; background-color: #1a1a1a; color: white;
      text-align: center; margin: 0; padding: 10px;
      display: flex; flex-direction: column;
      height: 100dvh; box-sizing: border-box;
      touch-action: none; overflow: hidden;
    }
    h1 { margin: 5px 0; font-size: 20px; }
    .page { display: none; flex-direction: column; flex-grow: 1; }
    .page.active { display: flex; }

    /* MENY */
    .menu { justify-content: center; align-items: center; gap: 20px; }
    .menu-btn {
      width: 200px; padding: 25px; font-size: 20px; font-weight: bold;
      background-color: #444; color: white; border: none; border-radius: 15px;
      box-shadow: 0 4px #222;
    }
    .menu-btn:active { background-color: #007bff; transform: translateY(4px); box-shadow: none; }

    /* DIREKTSTYRNING */
    .d-pad {
      display: grid; grid-template-columns: repeat(3, 70px); grid-template-rows: repeat(3, 70px);
      gap: 10px; flex-grow: 1; justify-content: center; align-content: center;
    }
    button {
      background-color: #444; color: white; border: none; border-radius: 15px;
      font-size: 22px; font-weight: bold; box-shadow: 0 4px #222;
      user-select: none; -webkit-user-select: none; transition: all 80ms;
    }
    button:active { background-color: #007bff; transform: translateY(4px); box-shadow: 0 0px #222; }
    .up { grid-column: 2; grid-row: 1; }
    .left { grid-column: 1; grid-row: 2; }
    .stop { grid-column: 2; grid-row: 2; background-color: #cc0000; box-shadow: 0 4px #880000; }
    .stop:active { background-color: #ff3333; }
    .right { grid-column: 3; grid-row: 2; }
    .down { grid-column: 2; grid-row: 3; }
    .speed-panel { padding: 10px; }
    .speed-panel label { font-size: 14px; }
    input { width: 80px; padding: 8px; font-size: 16px; border-radius: 5px; border: 1px solid #666; background: #111; color: white; }

    /* AUTO */
    .auto-panel { justify-content: center; align-items: center; gap: 20px; }
    .auto-panel label { font-size: 18px; }
    .auto-status { font-size: 16px; color: #aaa; margin-top: 10px; }
    .start-btn { background-color: #007b00; box-shadow: 0 4px #00440; width: 200px; padding: 20px; font-size: 20px; }
    .start-btn:active { background-color: #00aa00; }
    .stop-btn { background-color: #cc0000; box-shadow: 0 4px #880000; width: 200px; padding: 20px; font-size: 20px; }
    .stop-btn:active { background-color: #ff3333; }

    /* TILLBAKA-KNAPP */
    .back-btn { background-color: #555; padding: 10px; font-size: 14px; margin-top: auto; }
  </style>
</head>
<body>
  <h1>Robotstyrning</h1>

  <!-- MENY -->
  <div id="menuPage" class="page menu active">
    <button class="menu-btn" onclick="showPage('manualPage')">Direktstyrning</button>
    <button class="menu-btn" onclick="showPage('autoPage')">Auto</button>
  </div>

  <!-- DIREKTSTYRNING -->
  <div id="manualPage" class="page">
    <div class="d-pad">
      <button class="up" onpointerdown="send('w')" onpointerup="send('x')">W</button>
      <button class="left" onpointerdown="send('a')" onpointerup="send('x')">A</button>
      <button class="stop" onpointerdown="send('x')">🛑</button>
      <button class="right" onpointerdown="send('d')" onpointerup="send('x')">D</button>
      <button class="down" onpointerdown="send('s')" onpointerup="send('x')">S</button>
    </div>
    <div class="speed-panel">
      <label>Hastighet (0-1023):</label>
      <input type="number" id="spd" value="550" min="0" max="1023">
    </div>
    <button class="back-btn" onclick="showPage('menuPage')">Tillbaka</button>
  </div>

  <!-- AUTO -->
  <div id="autoPage" class="page auto-panel">
    <label>Tid (minuter, max 20):</label>
    <input type="number" id="autoTime" value="5" min="1" max="20">
    <button class="start-btn" onclick="startAuto()">Starta</button>
    <button class="stop-btn" onclick="stopAuto()">Stoppa</button>
    <div id="autoStatus" class="auto-status">Inaktiv</div>
    <button class="back-btn" onclick="showPage('menuPage')">Tillbaka</button>
  </div>

  <script>
    let autoTimer = null;

    function showPage(id) {
      document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
      document.getElementById(id).classList.add('active');
    }

    function send(dir) {
      const v = document.getElementById('spd').value;
      fetch(`/move?dir=${dir}&v=${v}`);
    }

    function startAuto() {
      let mins = parseInt(document.getElementById('autoTime').value);
      if (mins < 1) mins = 1;
      if (mins > 20) mins = 20;
      document.getElementById('autoTime').value = mins;

      fetch(`/auto?mins=${mins}`);
      document.getElementById('autoStatus').textContent = `Kör i ${mins} min...`;

      if (autoTimer) clearTimeout(autoTimer);
      autoTimer = setTimeout(() => {
        document.getElementById('autoStatus').textContent = 'Klar!';
      }, mins * 60000);
    }

    function stopAuto() {
      fetch('/move?dir=x&v=0');
      if (autoTimer) clearTimeout(autoTimer);
      document.getElementById('autoStatus').textContent = 'Stoppad';
    }
  </script>
</body>
</html>
)rawliteral";

void setupWebEndpoints() {
  // Huvudsidan
  webServer.on("/", []() {
    webServer.send(200, "text/html", htmlPage);
  });

  webServer.on("/ota", []() {
    webServer.send(200, "text/plain", "Startar OTA...");
    delay(500);
    performOTA();
  });

  webServer.on("/checkSafety", []() {
    String json = "{";
    json += "\"forward\":" + String(checkForwardSafety() ? "true" : "false") + ",";
    json += "\"backward\":" + String(checkBackwardsSafety() ? "true" : "false") + ",";
    json += "\"right\":" + String(checkRightSafety() ? "true" : "false") + ",";
    json += "\"left\":" + String(checkLeftSafety() ? "true" : "false");
    json += "}";
    webServer.send(200, "application/json", json);
  });

  webServer.on("/checkUDistances", []() {
    int d1 = distanceSensor1.measureDistanceCm();
    int d2 = distanceSensor2.measureDistanceCm();
    int d3 = distanceSensor3.measureDistanceCm();
    int d4 = distanceSensor4.measureDistanceCm();
    int d5 = distanceSensor5.measureDistanceCm();
    String json = "{";
    json += "\"sensor1\":" + String(d1) + ",";
    json += "\"sensor2\":" + String(d2) + ",";
    json += "\"sensor3\":" + String(d3) + ",";
    json += "\"sensor4\":" + String(d4) + ",";
    json += "\"sensor5\":" + String(d5);
    json += "}";
    webServer.send(200, "application/json", json);
  });

  webServer.on("/checkLDistances", []() {
    String json = "{";
    json += "\"laser0\":" + String(lastDistanceMm[0]) + ",";
    json += "\"laser1\":" + String(lastDistanceMm[1]) + ",";
    json += "\"laser2\":" + String(lastDistanceMm[2]) + ",";
    json += "\"laser3\":" + String(lastDistanceMm[3]);
    json += "}";
    webServer.send(200, "application/json", json);
  });

  // Gemensam endpoint för alla rörelser: /move?dir=w&v=255
  webServer.on("/move", []() {
    String dir = webServer.arg("dir");
    String v = webServer.arg("v");

    if (v != "") velocity = v.toInt();

    if (isAutoMode) {
      // Om en manuell knapp trycktes medan auto är igång:
      // Stäng av auto, tvärnita allt, och ignorera kommandot.
      isAutoMode = false;
      stopAll();
      Serial.println("Auto avbrutet via manuell input!");
      webServer.send(200, "text/plain", "OK");
      return;  // return gör att koden hoppar ur funktionen direkt och struntar i raderna nedanför
    }

    if (dir == "w") moveForward();
    else if (dir == "s") moveBackward();
    else if (dir == "a") turnLeft();
    else if (dir == "d") turnRight();
    else stopAll();

    webServer.send(200, "text/plain", "OK");
  });
  webServer.on("/auto", []() {
    String mins = webServer.arg("mins");
    int minutes = mins.toInt();
    if (minutes < 1) minutes = 1;
    if (minutes > 20) minutes = 20;

    // NYTT: Aktivera autoläget och sätt stoppklockan
    isAutoMode = true;
    autoStopTime = millis() + (minutes * 60000UL);  // 60000 millisekunder = 1 minut

    Serial.printf("Auto startat: %d minuter\n", minutes);
    webServer.send(200, "text/plain", "OK");
  });
}

// ==========================================
// KANTDETEKTERING & SENSORER
// ==========================================
void onFrontTableEdgeDetected(uint8_t sensorIndex, int distanceMm, int deltaMm) {
  if (currentMotion == MOTION_FORWARD) {
    stopAll();
    Serial.println("Nödstopp: frontkant upptäckt.");
  }
}

void onRearTableEdgeDetected(uint8_t sensorIndex, int distanceMm, int deltaMm) {
  if (currentMotion == MOTION_BACKWARD) {
    stopAll();
    Serial.println("Nödstopp: bakkant upptäckt.");
  }
}

void setupSensors() {
  Serial.println("Initierar VL53L0X-sensorer...");
  Wire.begin();
  Wire.setTimeOut(100);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    pinMode(XSHUT_PINS[i], OUTPUT);
    digitalWrite(XSHUT_PINS[i], LOW);
  }
  delay(50);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(10);
    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.printf("Sensor %u kunde inte initieras (XSHUT pin %d)\n", i, XSHUT_PINS[i]);
      digitalWrite(XSHUT_PINS[i], LOW);
      continue;
    }
    sensors[i].setAddress(NEW_ADDRESSES[i]);
    sensors[i].startContinuous();
    sensorInitialized[i] = true;
    Serial.printf("Sensor %u initierad pa adress 0x%02X\n", i, NEW_ADDRESSES[i]);
  }
}

void updateSensors() {
  unsigned long now = millis();
  if (now - lastSensorReadMs < SENSOR_READ_INTERVAL_MS) return;
  lastSensorReadMs = now;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if (!sensorInitialized[i]) continue;

    uint16_t rawDistance = sensors[i].readRangeContinuousMillimeters();
    if (sensors[i].timeoutOccurred()) continue;

    int correctedDistance = (int)rawDistance - DISTANCE_OFFSET_MM;
    if (correctedDistance < 0) correctedDistance = 0;

    if (lastDistanceMm[i] >= 0) {
      int delta = correctedDistance - lastDistanceMm[i];
      if (delta >= EDGE_JUMP_THRESHOLD_MM && correctedDistance >= EDGE_MIN_DISTANCE_MM) {
        if (i <= FRONT_SENSOR_END) onFrontTableEdgeDetected(i, correctedDistance, delta);
        else onRearTableEdgeDetected(i, correctedDistance, delta);
      }
    }
    //Serial.printf("Sensor %u: %d\n", i, correctedDistance);
    lastDistanceMm[i] = correctedDistance;
  }
  /*Serial.println("");
  Serial.println("");*/
}

void checkSafety() {
  switch (currentMotion) {
    case MOTION_FORWARD:
      {
        if (!checkForwardSafety()) {
          stopAll();
          delay(10);
          analogWrite(LEFT_FWD, velocity);
          analogWrite(RIGHT_BWD, velocity);
          delay(200);
          stopAll();
        }
        break;
      }
    case MOTION_BACKWARD:
      {
        if (!checkBackwardsSafety()) {
          stopAll();
          delay(10);
          // Gasa FRAMÅT för att bromsa farten bakåt:
          analogWrite(LEFT_BWD, velocity);
          analogWrite(RIGHT_FWD, velocity);
          delay(200);
          stopAll();
        }
        break;
      }
    case MOTION_LEFT:
      {
        if (!checkLeftSafety()) {
          stopAll();
          turnRight();
          delay(10);
          stopAll();
        }
        break;
      }
    case MOTION_RIGHT:
      {
        if (!checkRightSafety()) {
          stopAll();
          turnLeft();
          delay(10);
          stopAll();
        }
        break;
      }
  }

  /*Serial.println("Distance (1,2,3,4,5): ");
  Serial.println(distanceSensor1.measureDistanceCm());
  Serial.println(distanceSensor2.measureDistanceCm());
  Serial.println(distanceSensor3.measureDistanceCm());
  Serial.println(distanceSensor4.measureDistanceCm());
  Serial.println(distanceSensor5.measureDistanceCm());*/
}

// ==========================================
// OTA & NÄTVERKSHANTERING
// ==========================================
void performOTA() {
  Serial.println("Startar nedladdning av firmware...");
  WiFiClient otaClient;
  t_httpUpdate_return ret = httpUpdate.update(otaClient, FIRMWARE_URL);
  if (ret == HTTP_UPDATE_OK) Serial.println("Uppdatering klar! Startar om...");
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Boot: setup start");

  // Initiera motor-pins
  pinMode(LEFT_FWD, OUTPUT);
  pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT);
  pinMode(RIGHT_BWD, OUTPUT);
  WiFi.mode(WIFI_AP);
  WiFi.softAP("Halabi", "josef123");
  Serial.print("AP-IP: ");
  Serial.println(WiFi.softAPIP());


  // --- STARTA RESTEN AV SYSTEMET ---
  setupWebEndpoints();  // HTTP-server (för webbläsaren)
  Serial.println("Initierar sensorer...");
  setupSensors();  // VL53L0X

  Serial.println("Robot redo! Styr via 192.168.4.1 (AP) eller router-IP (STA)");
  delay(500);
  stopAll();
}

bool safeStepForward(int timeMs) {
  moveForward();
  unsigned long start = millis();

  while (millis() - start < timeMs) {
    updateSensors();  // Titta på lasrarna hela tiden!

    // Om någon laser upptäcker ett stup (KANT)
    if (lastDistanceMm[0] > safetyLDistance || lastDistanceMm[1] > safetyLDistance || lastDistanceMm[2] > safetyLDistance || lastDistanceMm[3] > safetyLDistance) {

      stopAll();
      moveBackward();  // Backa in på säker mark
      delay(300);
      stopAll();
      return false;  // Returnera false = Vi trillade in i ett hörn!
    }
    delay(10);  // Liten paus så processorn inte fryser
  }

  stopAll();
  return true;  // Returnera true = Steget var säkert, ny fil!
}

// SCENARIO B & C: Kanten/Hindret och Hörnfällan
void performAutoBounce() {
  // Steg 1: Tvärnit
  stopAll();
  delay(10);
  analogWrite(LEFT_FWD, velocity);
  analogWrite(RIGHT_BWD, velocity);
  delay(200);
  stopAll();

  // Steg 2: Backa för svängradie
  moveBackward();
  delay(300);
  stopAll();
  delay(100);

  // Steg 3: Välj håll baserat på minnet (turnRightNext)
  if (turnRightNext) {
    turnRight();
    delay(TIME_FOR_90_DEG);
    stopAll();
    delay(100);

    // Steg 4: Det säkra sidosteget
    if (safeStepForward(TIME_FOR_FORWARD_STEP) == true) {
      // Steg 5: Sidosteget lyckades, fullfölj U-svängen!
      turnRight();
      delay(TIME_FOR_90_DEG);
      stopAll();
      delay(100);
      turnRightNext = false;  // Invertera minnet till nästa gång
    } else {
      // SCENARIO C: Hörnfällan! (Sidosteget hittade en ny kant)
      // Gör en Flipper-studs (slumpmässig jättesväng) för att fly
      turnLeft();
      delay(random(700, 1100));
      stopAll();
      delay(100);
    }

  } else {
    // Samma logik för Vänster-svängen
    turnLeft();
    delay(TIME_FOR_90_DEG);
    stopAll();
    delay(100);

    if (safeStepForward(TIME_FOR_FORWARD_STEP) == true) {
      turnLeft();
      delay(TIME_FOR_90_DEG);
      stopAll();
      delay(100);
      turnRightNext = true;
    } else {
      // SCENARIO C: Hörnfällan!
      turnRight();
      delay(random(700, 1100));
      stopAll();
      delay(100);
    }
  }
}


void loop() {
  // Be WiFiManager att hantera eventuella försök att ansluta i bakgrunden
  // Utan denna rad kommer portalen aldrig att svara om du surfar till den

  webServer.handleClient();  // Webb-knapparna
  updateSensors();           // Kantsensorer
  updateUltrasonicSensors();

  if (isAutoMode) {

    // Scenario D: Är städtiden slut?
    if (millis() >= autoStopTime) {
      stopAll();
      isAutoMode = false;
      Serial.println("Auto-passet är klart.");
    }
    // Fortsätt köra Auto
    else {
      // Scenario A: Är kusten klar?
      if (checkForwardSafety()) {
        moveForward();  // Kör rakt fram och städa!
      } else {
        // Kusten är INTE klar (Vi träffade en kant/kopp)
        performAutoBounce();  // Starta Scenario B/C
      }
    }
  }
  // MANUELL KÖRNING (När Auto är avstängt via webben)
  else {
    checkSafety();  // Din vanliga säkerhetsspärr (för joystick/knappar)
  }
}