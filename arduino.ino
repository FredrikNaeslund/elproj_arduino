#include <Wire.h>
#include <VL53L0X.h>
#include <WiFi.h>
#include <WiFiManager.h>
#include <HTTPClient.h>
#include <HTTPUpdate.h>
#include <WebServer.h> // NYTT: För att bygga det inbyggda gränssnittet

// ==========================================
// NÄTVERKSINSTÄLLNINGAR & OTA
// ==========================================
const int   TCP_PORT      = 8080;
const int   WEB_PORT      = 80; // Standardport för webbplatser
const char* FIRMWARE_URL  = "http://10.255.212.171:3000/firmware.bin"; 

WiFiManager wm;
WiFiServer server(TCP_PORT);
WiFiClient activeClient; 
WebServer webServer(WEB_PORT); // NYTT: Starta webbservern

// ==========================================
// MOTOR & PIN-KONFIGURATION
// ==========================================
#define LEFT_FWD  2
#define LEFT_BWD  16
#define RIGHT_FWD 5
#define RIGHT_BWD 13
#define STBY      12

int velocity = 255; 

// ==========================================
// VL53L0X KONFIGURATION
// ==========================================
const uint8_t SENSOR_COUNT = 4;
const int XSHUT_PINS[SENSOR_COUNT] = {14, 27, 26, 25};
const uint8_t NEW_ADDRESSES[SENSOR_COUNT] = {0x30, 0x31, 0x32, 0x33};
const uint8_t FRONT_SENSOR_END = 1;

const int DISTANCE_OFFSET_MM = 10;
const int EDGE_JUMP_THRESHOLD_MM = 120;
const int EDGE_MIN_DISTANCE_MM = 180;
const unsigned long SENSOR_READ_INTERVAL_MS = 50;

VL53L0X sensors[SENSOR_COUNT];
bool sensorInitialized[SENSOR_COUNT] = {false, false, false, false};
int lastDistanceMm[SENSOR_COUNT] = {-1, -1, -1, -1};
unsigned long lastSensorReadMs = 0;

enum MotionState {
  MOTION_STOPPED, MOTION_FORWARD, MOTION_BACKWARD, MOTION_LEFT, MOTION_RIGHT
};
MotionState currentMotion = MOTION_STOPPED;

// ==========================================
// MOTORFUNKTIONER 
// ==========================================
void moveForward() {
  analogWrite(LEFT_FWD, velocity); analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, velocity); analogWrite(RIGHT_BWD, 0);
  currentMotion = MOTION_FORWARD;
}
void moveBackward() {
  analogWrite(LEFT_FWD, 0); analogWrite(LEFT_BWD, velocity);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_BWD, velocity);
  currentMotion = MOTION_BACKWARD;
}
void turnLeft() {
  analogWrite(LEFT_FWD, 0); analogWrite(LEFT_BWD, velocity);
  analogWrite(RIGHT_FWD, velocity); analogWrite(RIGHT_BWD, 0);
  currentMotion = MOTION_LEFT;
}
void turnRight() {
  analogWrite(LEFT_FWD, velocity); analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_BWD, velocity);
  currentMotion = MOTION_RIGHT;
}
void stopAll() {
  analogWrite(LEFT_FWD, 0); analogWrite(LEFT_BWD, 0);
  analogWrite(RIGHT_FWD, 0); analogWrite(RIGHT_BWD, 0);
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
  <title>Robot Direct Control</title>
  <style>
    body {
      font-family: sans-serif; background-color: #1a1a1a; color: white;
      text-align: center; margin: 0; display: flex; flex-direction: column; height: 100vh; touch-action: none;
    }
    .d-pad {
      display: grid; grid-template-columns: repeat(3, 80px); grid-template-rows: repeat(3, 80px);
      gap: 15px; flex-grow: 1; justify-content: center; align-content: center;
    }
    button {
      background-color: #444; color: white; border: none; border-radius: 15px;
      font-size: 24px; font-weight: bold; box-shadow: 0 4px #222;
      user-select: none; -webkit-user-select: none; transition: all 80ms;
    }
    button:active { background-color: #007bff; transform: translateY(4px); box-shadow: 0 0px #222; }
    .up { grid-column: 2; grid-row: 1; }
    .left { grid-column: 1; grid-row: 2; }
    .stop { grid-column: 2; grid-row: 2; background-color: #cc0000; box-shadow: 0 4px #880000; }
    .stop:active { background-color: #ff3333; }
    .right { grid-column: 3; grid-row: 2; }
    .down { grid-column: 2; grid-row: 3; }
    
    .speed-panel { background-color: #2a2a2a; padding: 20px; border-top: 2px solid #444; }
    input { width: 100px; padding: 10px; font-size: 18px; border-radius: 5px; border: 1px solid #666; background: #111; color: white; }
  </style>
</head>
<body>
  <h1>Direktstyrning</h1>
  <div class="d-pad">
    <button class="up" onpointerdown="send('w')" onpointerup="send('x')">W</button>
    <button class="left" onpointerdown="send('a')" onpointerup="send('x')">A</button>
    <button class="stop" onpointerdown="send('x')">🛑</button>
    <button class="right" onpointerdown="send('d')" onpointerup="send('x')">D</button>
    <button class="down" onpointerdown="send('s')" onpointerup="send('x')">S</button>
  </div>
  <div class="speed-panel">
    <label>Hastighet (0-255):</label><br>
    <input type="number" id="spd" value="200" min="0" max="255">
  </div>
  <script>
    function send(dir) {
      const v = document.getElementById('spd').value;
      fetch(`/move?dir=${dir}&v=${v}`);
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

  // Gemensam endpoint för alla rörelser: /move?dir=w&v=255
  webServer.on("/move", []() {
    String dir = webServer.arg("dir");
    String v = webServer.arg("v");
    
    if (v != "") velocity = v.toInt();
    
    if (dir == "w") moveForward();
    else if (dir == "s") moveBackward();
    else if (dir == "a") turnLeft();
    else if (dir == "d") turnRight();
    else stopAll();

    webServer.send(200, "text/plain", "OK");
  });

  webServer.begin();
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
  delay(10);

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    digitalWrite(XSHUT_PINS[i], HIGH);
    delay(10);
    sensors[i].setTimeout(500);
    if (!sensors[i].init()) {
      Serial.printf("Sensor %u kunde inte initieras (XSHUT pin %d)\n", i, XSHUT_PINS[i]);
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
    Serial.printf("Sensor %u: %d\n", i, correctedDistance);
    lastDistanceMm[i] = correctedDistance;
  }
  Serial.println("");
  Serial.println("");
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

void handleTCPCommands() {
  // 1. Kolla om det finns en NY klient som vill ansluta
  WiFiClient newClient = server.available();
  
  if (newClient) {
    // Om vi redan har en aktiv klient, stoppa den gamla (vi tillåter bara en TCP-koppling i taget)
    if (activeClient && activeClient.connected()) {
      Serial.println("Kopplar ner gammal klient, ny klient ansluter.");
      activeClient.stop();
    }
    
    // Spara den nya klienten
    activeClient = newClient;
    Serial.println("Ny TCP-klient ansluten!");
    
    // SKICKA VÄLKOMSTMEDDELANDET DIREKT
    activeClient.println("Ready. Commands: w=fwd s=bwd a=left d=right x=stop o=OTA");
    activeClient.flush(); // Se till att det skickas iväg
  }

  // 2. Om vi har en aktiv klient, kolla om den har SKICKAT data till oss
  if (activeClient && activeClient.connected()) {
    if (activeClient.available()) {
      
      // Läs in hela raden fram till radbrytning
      String data = activeClient.readStringUntil('\n');
      data.trim(); 

      if (data.length() > 0) {
        int spaceIndex = data.indexOf(' ');
        char command = data.charAt(0); 
        
        // Hämta ut hastigheten om den skickades med kommandot (t.ex. "w 255")
        if (spaceIndex != -1) {
          velocity = data.substring(spaceIndex + 1).toInt(); 
        }

        // --- Stabilitets-fixen (Brownout protection) ---
        activeClient.print("Mottog: ");
        activeClient.println(command);
        activeClient.flush(); // Tvinga TCP-paketet att skickas
        delay(50);            // Låt spänningen stabilisera sig
        // ---------------------------------------------

        // Utför rätt motorrörelse (eller OTA)
        switch (command) {
          case 'w': moveForward(); break;
          case 's': moveBackward(); break;
          case 'a': turnLeft(); break;
          case 'd': turnRight(); break;
          case 'x': stopAll(); break;
          case 'o': 
            stopAll();
            activeClient.println("Startar OTA... kopplar ner TCP.");
            activeClient.stop(); 
            delay(500);
            performOTA();
            break;
        }
      }
    }
  } else if (activeClient && !activeClient.connected()) {
      // Om klienten har kopplat ifrån, rensa anslutningen
      Serial.println("TCP-klient frånkopplad.");
      stopAll(); // Stanna roboten för säkerhets skull
      activeClient.stop();
  }
}

// ==========================================
// SETUP & MAIN LOOP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("Boot: setup start");

  // Initiera motor-pins
  pinMode(LEFT_FWD, OUTPUT); pinMode(LEFT_BWD, OUTPUT);
  pinMode(RIGHT_FWD, OUTPUT); pinMode(RIGHT_BWD, OUTPUT);
  pinMode(STBY, OUTPUT);
  stopAll();
  digitalWrite(STBY, HIGH); 
  // --- VIKTIGA INSTÄLLNINGAR ---
  
  // 1. Gör så att autoConnect INTE stannar upp hela programmet
  wm.setConfigPortalBlocking(false); 
  
  // 2. Försök ansluta till sparat WiFi i max 30 sekunder, annars gå vidare
  wm.setConfigPortalTimeout(30); 

  // 3. Starta läget där vi både är klient (STA) och basstation (AP)
  WiFi.mode(WIFI_AP_STA);

  // 4. Starta ditt permanenta nätverk
  WiFi.softAP("Elektronikskräpupplockarrobot", "josef");
  Serial.print("Direkt-IP (AP): "); Serial.println(WiFi.softAPIP());

  // 5. Försök ansluta till hemnätverket (Fredrik's S23)
  // Om det misslyckas körs loop() ändå tack vare setConfigPortalBlocking(false)
  Serial.println("WiFiManager: autoConnect start");
  wm.autoConnect("Elektronikskräpupplockarrobot", "josef");
  Serial.println("WiFiManager: autoConnect returnerade");

  // --- STARTA RESTEN AV SYSTEMET ---
  server.begin();      // TCP-server (för Node.js)
  setupWebEndpoints(); // HTTP-server (för webbläsaren)
  Serial.println("Initierar sensorer...");
  setupSensors();      // VL53L0X
  
  Serial.println("Robot redo! Styr via 192.168.4.1 (AP) eller router-IP (STA)");
}

void loop() {
  // Be WiFiManager att hantera eventuella försök att ansluta i bakgrunden
  // Utan denna rad kommer portalen aldrig att svara om du surfar till den
  wm.process(); 

  webServer.handleClient(); // Webb-knapparna
  handleTCPCommands();      // Node.js
  updateSensors();          // Kantsensorer
}