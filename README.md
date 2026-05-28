# ESP32 Robotstyrning — Fullständig Specifikation

## Översikt

Ett ESP32-baserat robotprojekt med fjärrstyrning via WiFi. Roboten har två driftlägen: manuell styrning via ett webbgränssnitt och ett autonomt läge ("Auto") med inbyggd kantdetektering och hinderdetektion. Systemet använder en kombination av VL53L0X-lasersensorer och HC-SR04 ultraljudssensorer för att säkerställa att roboten inte kör utanför kanter eller in i hinder.

Roboten fungerar som en WiFi-accesspunkt (AP) och exponerar ett webbgränssnitt samt ett REST-API för styrning och diagnostik.

---

## Hårdvara

### Mikrokontroller

ESP32 (WiFi AP-läge)

### Motorer

Fyra PWM-kanaler styr vänster och höger motorer i båda riktningar.

| Signal       | GPIO |
|-------------|------|
| LEFT_FWD    | 2    |
| LEFT_BWD    | 17   |
| RIGHT_FWD   | 18   |
| RIGHT_BWD   | 19   |

Motorstyrningen inkluderar en **kickstart-mekanism**: vid start från stillastående skickas en kort puls på högre hastighet (600 PWM i 40 ms) för att övervinna friktion, varefter den valda hastigheten används.

### Lasersensorer (VL53L0X)

Fyra stycken Time-of-Flight-sensorer anslutna via I2C. Används primärt för kantdetektering (t.ex. bordskant).

| Sensor | XSHUT-pin | I2C-adress | Funktion            |
|--------|-----------|------------|---------------------|
| 0      | 13        | 0x30       | Bak (kant)          |
| 1      | 14        | 0x31       | Bak (kant)          |
| 2      | 26        | 0x32       | Fram (kant)         |
| 3      | 27        | 0x33       | Fram (kant)         |

Sensor 0–1 klassas som **baksensorer** och sensor 2–3 som **framsensorer** (`FRONT_SENSOR_END = 1` anger att index 0–1 är bak).

Konfiguration:

- Mätintervall: 50 ms
- Offset-korrigering: −10 mm
- Kanttröskel: hopp ≥ 120 mm och avstånd ≥ 180 mm triggar kantstopp
- Värde > 8000 mm tolkas som "utanför räckvidd" och sätts till 999
- Om en sensor rapporterar 999 i mer än 1000 ms startas den om automatiskt

### Ultraljudssensorer (HC-SR04)

Fem stycken ultraljudssensorer för hinderdetektering.

| Sensor | Trig-pin | Echo-pin | Placering      |
|--------|----------|----------|----------------|
| 1      | 4        | 34       | Bak            |
| 2      | 23       | 35       | Fram vänster   |
| 3      | 25       | 12       | Fram höger     |
| 4      | 5        | 32       | —              |
| 5      | 16       | 33       | —              |

Mätintervall: minst 100 ms mellan läsningar.

---

## WiFi-konfiguration

Roboten startar som en **WiFi Access Point**.

| Parameter  | Värde               |
|-----------|----------------------|
| SSID       | `<DITT_WIFI_NAMN>`  |
| Lösenord   | `<DITT_LÖSENORD>`   |
| IP-adress  |   `192.168.4.1`     |
| Port       |         80          |

Anslut till WiFi-nätverket "<DITT_WIFI_NAMN>" och öppna `http://192.168.4.1` i en webbläsare.

---

## Webbgränssnitt

Det inbyggda gränssnittet är en enkelsidig HTML-app som serveras direkt från ESP32. Det har tre vyer:

### Huvudmeny

Två knappar: **Direktstyrning** och **Auto**.

### Direktstyrning

En D-pad (W/A/S/D/Stopp) för manuell styrning. Knapparna skickar kommandon vid `pointerdown` och stoppar vid `pointerup`. Ett fält för att ställa in hastighet (0–1023 PWM).

### Auto-läge

Ställ in körtid (1–20 minuter) och starta/stoppa det autonoma läget. Statustext visar aktuell status.

---

## REST API

Alla endpoints returnerar antingen `text/plain` eller `application/json`. Inga WebSockets används — all kommunikation sker via HTTP GET-requests.

### Styrning

#### `GET /move?dir={riktning}&v={hastighet}`

Styr roboten manuellt.

**Parametrar:**

- `dir` — Riktning: `w` (framåt), `s` (bakåt), `a` (vänster), `d` (höger), `x` (stopp)
- `v` — Hastighet som PWM-värde, 0–1023

**Beteende:** Om autoläget är aktivt när ett manuellt kommando skickas avbryts autoläget omedelbart och roboten stannar. Kommandot ignoreras; användaren måste skicka ett nytt kommando.

**Svar:** `200 OK`, body `OK`

#### `GET /auto?mins={minuter}`

Startar det autonoma körläget.

**Parametrar:**

- `mins` — Antal minuter roboten ska köra autonomt (1–20)

**Svar:** `200 OK`, body `OK`

### Diagnostik

#### `GET /checkSafety`

Returnerar huruvida det är säkert att röra sig i respektive riktning baserat på aktuella sensorvärden.

**Svar (JSON):**

```json
{
  "forward": true,
  "backward": false,
  "right": true,
  "left": true
}
```

#### `GET /checkUDistances`

Mäter och returnerar aktuella avstånd (cm) från alla fem ultraljudssensorer.

**Svar (JSON):**

```json
{
  "sensor1": 45,
  "sensor2": 12,
  "sensor3": -1,
  "sensor4": 88,
  "sensor5": 30
}
```

Värdet `-1` indikerar att sensorn inte kunde mäta ett avstånd.

#### `GET /laserStatus`

Returnerar aktuella avståndsvärden (mm) och initieringsstatus för alla fyra lasersensorer, samt en lista över hittade I2C-enheter.

**Svar (JSON):**

```json
{
  "laser0": 42,
  "laser1": 38,
  "laser2": 999,
  "laser3": 45,
  "init0": true,
  "init1": true,
  "init2": false,
  "init3": true,
  "i2c": ["0x30", "0x31", "0x33"]
}
```

Värdet `999` innebär att sensorn är utanför räckvidd eller inte fungerar.

#### `GET /velocity`

Returnerar nuvarande inställd hastighet.

**Svar (JSON):**

```json
{
  "velocity": 90
}
```

#### `GET /i2cscan`

Skannar I2C-bussen och returnerar alla enheter som svarar.

**Svar (JSON):**

```json
{
  "devices": ["0x30", "0x31", "0x32", "0x33"]
}
```

### Underhåll

#### `GET /restartLasers`

Startar om samtliga VL53L0X-lasersensorer. Stänger av alla via XSHUT, återinitierar I2C-bussen och konfigurerar om varje sensor.

**Svar (JSON):**

```json
{
  "message": "restartLasers done",
  "devices": ["0x30", "0x31", "0x32", "0x33"]
}
```

#### `GET /ota`

Triggar en OTA-firmwareuppdatering. ESP32 försöker ladda ner ny firmware från den konfigurerade URL:en (`http://192.168.4.50:3000/firmware.bin`). Enheten startar om automatiskt vid lyckad uppdatering.

**Svar:** `200 OK`, body `Startar OTA...`

---

## Säkerhetssystem

Roboten har ett flerskiktat säkerhetssystem som alltid är aktivt oavsett driftläge.

### Säkerhetströsklar

| Parameter            | Värde   | Beskrivning                              |
|---------------------|---------|------------------------------------------|
| `safetyUDistance`    | 10 cm   | Minsta tillåtna ultraljudsavstånd        |
| `safetyLDistance`    | 60 mm   | Maximal laserdistans (bortom = fara)     |

### Framåt-säkerhet

Blockeras om **ultraljudssensor 2 eller 3** mäter under 10 cm, **eller** om **lasersensor 2 eller 3** visar över 60 mm (indikerar att marken försvunnit, d.v.s. en kant).

### Bakåt-säkerhet

Blockeras om **ultraljudssensor 1** mäter under 10 cm, **eller** om **lasersensor 0 eller 1** visar över 60 mm.

### Sväng-säkerhet (vänster/höger)

Blockeras om **någon av de fyra lasersensorerna** visar över 60 mm. Båda riktningarna använder samma kontroll.

### Kantdetektering (edge detection)

Utöver de statiska säkerhetströsklarna finns en dynamisk kantdetektering som reagerar på plötsliga avståndshop i lasrarna. Om avståndet ökar med ≥ 120 mm mellan två mätningar och det nya värdet är ≥ 180 mm triggas ett nödstopp.

### Säkerhetsbeteende vid manuell körning

Om säkerheten triggas under manuell körning utförs en kort motrörelsepuls (180 ms) för att bromsa, följt av totalstopp.

### Vakthundsystem för lasrar

Varje lasersensor övervakas individuellt:

- **Timeout:** Om en sensor inte svarar startas den om direkt via `restartOneLaser()`.
- **Långvarig 999:** Om en sensor rapporterar "utanför räckvidd" (999) i mer än 1 sekund antas det vara ett sensorfel och den startas om.
- **Global omstart:** Alla sensorer kan startas om via API-endpointen `/restartLasers`.

---

## Autonomt läge (Auto)

### Grundbeteende

Roboten kör rakt framåt tills en kant eller ett hinder detekteras. Då utförs en automatisk undanmanöver.

### Undanmanöver (`performAutoBounce`)

Sekvensen vid detekterat hinder/kant:

1. **Nödstopp** — motbromspuls i 200 ms
2. **Backa** — 300 ms bakåt, bort från kanten
3. **Sväng höger 90°** — under `TIME_FOR_90_DEG` (400 ms)
4. **Sidosteg framåt** — kör framåt i max `TIME_FOR_FORWARD_STEP` (600 ms). Under hela steget kontrolleras säkerheten kontinuerligt. Om en ny kant hittas avbryts steget och roboten backar 250 ms för att få svängrum (hörnhantering).
5. **Sväng höger 90°** — fullbordar en U-sväng

Efter manövern återupptas rak framåtkörning.

### Tidsbegränsning

Autoläget körs i det antal minuter som angavs vid start (1–20 min). När tiden går ut stannar roboten och autoläget avaktiveras.

### Avbryta autoläget

Autoläget avbryts omedelbart om ett manuellt styrkommando skickas via `/move`. Roboten stannar och manuellt läge tar över.

---

## Konfigurationskonstanter

| Konstant                   | Värde | Beskrivning                                      |
|---------------------------|-------|--------------------------------------------------|
| `velocity`                 | 90    | Standard PWM-hastighet för framåt/bakåt           |
| `turnVelocity`             | 550   | PWM-hastighet vid sväng                           |
| `KICKSTART_VELOCITY`       | 600   | PWM-puls vid start från stillastående             |
| `KICKSTART_TIME_MS`        | 40    | Kickstart-pulsens längd i millisekunder           |
| `TIME_FOR_90_DEG`          | 400   | Tid i ms för en 90°-sväng (kalibreringsvärde)     |
| `TIME_FOR_FORWARD_STEP`    | 600   | Max tid i ms för sidosteg i auto-bounce           |
| `SENSOR_READ_INTERVAL_MS`  | 50    | Minsta intervall mellan lasermätningar             |
| `DISTANCE_OFFSET_MM`       | 10    | Offset som subtraheras från rå lasermätningar      |
| `EDGE_JUMP_THRESHOLD_MM`   | 120   | Minsta hopp i mm för kantdetektering              |
| `EDGE_MIN_DISTANCE_MM`     | 180   | Minsta absolutavstånd i mm för kantdetektering    |
| `LASER_999_RESTART_MS`     | 1000  | Tid i ms innan 999-värde triggar lasreomstart      |

---

## Beroenden (Arduino-bibliotek)

- `Wire.h` — I2C-kommunikation
- `VL53L0X.h` — Pololu-drivrutin för VL53L0X Time-of-Flight-sensorer
- `WiFi.h` — ESP32 WiFi (AP-läge)
- `HTTPUpdate.h` — OTA-firmwareuppdatering via HTTP
- `WebServer.h` — Inbyggd HTTP-server
- `HCSR04.h` — HC-SR04 ultraljudssensorer

---

## OTA-uppdatering

Firmware kan uppdateras trådlöst. Endpointen `/ota` laddar ner ny firmware från:

```
http://192.168.4.50:3000/firmware.bin
```

Detta förutsätter att en HTTP-server som serverar firmware-filen är åtkomlig från ESP32 på ovanstående adress. Efter lyckad nedladdning startar enheten om automatiskt.

---

## Programflöde (loop)

```
loop()
├── webServer.handleClient()      ← Hanterar HTTP-requests
├── updateSensors()               ← Läser lasersensorer, kör vakthund
├── updateUltrasonicSensors()     ← Läser ultraljudssensorer
│
├── [Auto-läge aktivt?]
│   ├── Tid slut?  → stopAll(), avaktivera auto
│   ├── Framåt säkert?  → moveForward()
│   └── Ej säkert?  → performAutoBounce()
│
└── [Manuellt läge]
    └── checkSafety()             ← Nödstopp + motrörelsepuls
```
