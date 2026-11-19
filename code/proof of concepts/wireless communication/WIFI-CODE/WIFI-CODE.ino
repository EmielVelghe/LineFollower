
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// =====================================
// ESP32 LINEFOLLOWER WIFI TUNER
// Arduino Nano ESP32
// QTR-8A + DRV8833
// =====================================

// WIFI
const char* wifiName = "LineFollower";
const char* wifiPassword = "KIES_EEN_EIGEN_WACHTWOORD"; // Vul dit lokaal in voor het uploaden.

// WEB SERVER
WebServer server(80);
Preferences prefs;

// QTR-8A SENSORPINS
const int sensorPins[8] = {
  A0, A1, A2, A3, A4, A5, A6, A7
};

// DRV8833 MOTOREN
const int LEFT_IN1  = D2;
const int LEFT_IN2  = D3;
const int RIGHT_IN1 = D4;
const int RIGHT_IN2 = D5;

// Pas dit aan als een motor verkeerdom draait
bool invertLeft = false;
bool invertRight = false;

// SENSORINSTELLINGEN
int sensorMin[8];
int sensorMax[8];

bool calibrated = false;
bool calibrating = false;
unsigned long calibrationStart = 0;

// true = zwarte lijn op witte achtergrond
bool darkLine = true;

// PID INSTELLINGEN
float KP = 30.0;
float KI = 0.0;
float KD = 0.35;

float integral = 0;
float previousError = 0;

float error = 0;
float lastPosition = 0;

// MOTORINSTELLINGEN
int baseSpeed = 130;
int maxSpeed = 220;

bool running = false;

// SENSOR DATA
int rawValues[8];
int lineValues[8];

unsigned long lastPID = 0;

// =====================================
// MOTOR CONTROL
// =====================================

void setMotor(int left, int right) {

  left = constrain(left, -255, 255);
  right = constrain(right, -255, 255);

  if (invertLeft) left = -left;
  if (invertRight) right = -right;

  // LINKERMOTOR
  if (left >= 0) {
    analogWrite(LEFT_IN1, left);
    analogWrite(LEFT_IN2, 0);
  } else {
    analogWrite(LEFT_IN1, 0);
    analogWrite(LEFT_IN2, -left);
  }

  // RECHTERMOTOR
  if (right >= 0) {
    analogWrite(RIGHT_IN1, right);
    analogWrite(RIGHT_IN2, 0);
  } else {
    analogWrite(RIGHT_IN1, 0);
    analogWrite(RIGHT_IN2, -right);
  }
}

void stopMotors() {
  analogWrite(LEFT_IN1, 0);
  analogWrite(LEFT_IN2, 0);
  analogWrite(RIGHT_IN1, 0);
  analogWrite(RIGHT_IN2, 0);
}

// =====================================
// SENSOR UITLEZEN
// =====================================

void readSensors() {

  for (int i = 0; i < 8; i++) {

    rawValues[i] = analogRead(sensorPins[i]);

    if (calibrating) {
      if (rawValues[i] < sensorMin[i])
        sensorMin[i] = rawValues[i];

      if (rawValues[i] > sensorMax[i])
        sensorMax[i] = rawValues[i];
    }

    int range = sensorMax[i] - sensorMin[i];

    int value = 0;

    if (range > 20) {
      value = map(
        rawValues[i],
        sensorMin[i],
        sensorMax[i],
        0,
        1000
      );
    }

    value = constrain(value, 0, 1000);

    if (darkLine) {
      value = 1000 - value;
    }

    lineValues[i] = value;
  }
}

// =====================================
// LIJNPOSITIE BEREKENEN
// =====================================

bool calculatePosition(float &position) {

  long weightedSum = 0;
  long total = 0;

  for (int i = 0; i < 8; i++) {

    int sensorPosition = i * 1000;

    weightedSum +=
      (long)lineValues[i] * sensorPosition;

    total += lineValues[i];
  }

  if (total < 200) {
    return false;
  }

  position = (float)weightedSum / total;

  position -= 3500;

  position /= 1000.0;

  return true;
}

// =====================================
// PID REGELING
// =====================================

void updatePID() {

  if (!running || !calibrated || calibrating) {
    stopMotors();
    return;
  }

  unsigned long now = millis();

  float dt = (now - lastPID) / 1000.0;

  if (dt < 0.005) return;

  lastPID = now;

  float position;

  if (!calculatePosition(position)) {
    // Lijn kwijt: stop voor de veiligheid
    stopMotors();
    integral = 0;
    return;
  }

  error = position;
  lastPosition = position;

  integral += error * dt;
  integral = constrain(integral, -50.0, 50.0);

  float derivative =
    (error - previousError) / dt;

  float correction =
    KP * error +
    KI * integral +
    KD * derivative;

  previousError = error;

  correction = constrain(
    correction,
    -255.0,
    255.0
  );

  int left = baseSpeed + correction;
  int right = baseSpeed - correction;

  left = constrain(left, -maxSpeed, maxSpeed);
  right = constrain(right, -maxSpeed, maxSpeed);

  setMotor(left, right);
}

// =====================================
// INSTELLINGEN OPSLAAN
// =====================================

void saveSettings() {

  prefs.begin("lfsettings", false);

  prefs.putFloat("kp", KP);
  prefs.putFloat("ki", KI);
  prefs.putFloat("kd", KD);

  prefs.putInt("speed", baseSpeed);
  prefs.putInt("maxspeed", maxSpeed);

  prefs.putBool("dark", darkLine);

  prefs.putBool("cal", calibrated);

  if (calibrated) {
    prefs.putBytes("mins", sensorMin, sizeof(sensorMin));
    prefs.putBytes("maxs", sensorMax, sizeof(sensorMax));
  }

  prefs.end();
}

void loadSettings() {

  prefs.begin("lfsettings", true);

  KP = prefs.getFloat("kp", 30.0);
  KI = prefs.getFloat("ki", 0.0);
  KD = prefs.getFloat("kd", 0.35);

  baseSpeed = prefs.getInt("speed", 130);
  maxSpeed = prefs.getInt("maxspeed", 220);

  darkLine = prefs.getBool("dark", true);

  calibrated = prefs.getBool("cal", false);

  if (calibrated) {
    size_t a = prefs.getBytes(
      "mins", sensorMin, sizeof(sensorMin)
    );

    size_t b = prefs.getBytes(
      "maxs", sensorMax, sizeof(sensorMax)
    );

    if (a != sizeof(sensorMin) ||
        b != sizeof(sensorMax)) {
      calibrated = false;
    }
  }

  prefs.end();

  if (!calibrated) {
    for (int i = 0; i < 8; i++) {
      sensorMin[i] = 4095;
      sensorMax[i] = 0;
    }
  }
}

// =====================================
// WEBPAGINA
// =====================================

const char webpage[] PROGMEM = R"rawliteral(

<!DOCTYPE html>
<html>
<head>

<meta name="viewport"
content="width=device-width, initial-scale=1">

<title>LineFollower Tuner</title>

<style>

body {
  background:#101827;
  color:white;
  font-family:Arial;
  margin:0;
  padding:16px;
}

.container {
  max-width:550px;
  margin:auto;
}

.card {
  background:#1e293b;
  padding:18px;
  border-radius:14px;
  margin-bottom:15px;
}

h1 {
  text-align:center;
  color:#38bdf8;
}

h3 {
  color:#38bdf8;
}

.value {
  float:right;
  color:#4ade80;
  font-weight:bold;
}

input[type=range] {
  width:100%;
  margin:15px 0;
  accent-color:#38bdf8;
}

button {
  width:100%;
  padding:15px;
  margin-top:8px;
  border:0;
  border-radius:10px;
  font-size:16px;
  font-weight:bold;
  color:white;
}

.start {
  background:#16a34a;
}

.stop {
  background:#dc2626;
}

.save {
  background:#2563eb;
}

.cal {
  background:#d97706;
}

.sensor {
  display:inline-block;
  width:10%;
  text-align:center;
  font-size:12px;
}

.bar {
  height:65px;
  background:#334155;
  border-radius:5px;
  margin-bottom:5px;
  display:flex;
  align-items:end;
  overflow:hidden;
}

.fill {
  background:#38bdf8;
  width:100%;
  height:0%;
}

#status {
  text-align:center;
  color:#4ade80;
  font-weight:bold;
}

</style>
</head>

<body>

<div class="container">

<h1>LINEFOLLOWER</h1>

<div class="card">
  <h3>Verbinding</h3>
  <div id="status">Verbonden met ESP32</div>
  <p>IP: 192.168.4.1</p>
</div>

<div class="card">

<h3>PID instellingen</h3>

<label>KP <span class="value" id="kpv">30</span></label>
<input type="range" id="kp"
min="0" max="100" step="0.1" value="30"
oninput="changeValue('kp',this.value)">

<label>KI <span class="value" id="kiv">0</span></label>
<input type="range" id="ki"
min="0" max="20" step="0.01" value="0"
oninput="changeValue('ki',this.value)">

<label>KD <span class="value" id="kdv">0.35</span></label>
<input type="range" id="kd"
min="0" max="5" step="0.01" value="0.35"
oninput="changeValue('kd',this.value)">

</div>

<div class="card">

<h3>Motorsnelheid</h3>

<label>Basissnelheid
<span class="value" id="speedv">130</span>
</label>

<input type="range" id="speed"
min="0" max="255" value="130"
oninput="changeValue('speed',this.value)">

<label>Maximale snelheid
<span class="value" id="maxspeedv">220</span>
</label>

<input type="range" id="maxspeed"
min="0" max="255" value="220"
oninput="changeValue('maxspeed',this.value)">

</div>

<div class="card">

<h3>8 IR sensoren</h3>

<div id="sensors"></div>

<p id="position">Positie: 0</p>

</div>

<div class="card">

<h3>Robot bediening</h3>

<button class="start"
onclick="send('/start')">
START
</button>

<button class="stop"
onclick="send('/stop')">
STOP
</button>

<button class="cal"
onclick="calibrate()">
SENSOREN KALIBREREN (5 SEC)
</button>

<button class="save"
onclick="send('/save')">
INSTELLINGEN OPSLAAN
</button>

</div>

</div>

<script>

function send(url) {
  fetch(url).catch(() => {});
}

function changeValue(name,value) {

  document.getElementById(name+'v').innerText =
    value;

  fetch('/set?'+name+'='+value)
    .catch(() => {});
}

function calibrate() {

  send('/calibrate?on=1');

  document.getElementById('status').innerText =
    'Kalibreren: beweeg de sensoren over lijn en achtergrond';

  setTimeout(() => {
    send('/calibrate?on=0');
    document.getElementById('status').innerText =
      'Kalibratie beëindigd';
  },5000);
}

function updateSensors() {

  fetch('/sensors')
    .then(response => response.json())
    .then(data => {

      let html = '';

      for(let i=0;i<8;i++) {

        let value = data.values[i];

        html += `
        <div class="sensor">
          <div class="bar">
            <div class="fill"
            style="height:${value/10}%"></div>
          </div>
          ${i+1}
          <br>${value}
        </div>`;
      }

      document.getElementById('sensors').innerHTML =
        html;

      document.getElementById('position').innerText =
        'Positie: '+data.position.toFixed(2);

      if(data.calibrating) {
        document.getElementById('status').innerText =
          'Sensoren kalibreren...';
      } else if(!data.calibrated) {
        document.getElementById('status').innerText =
          'Nog niet gekalibreerd';
      } else {
        document.getElementById('status').innerText =
          data.running ? 'ROBOT RIJDT' : 'ROBOT GESTOPT';
      }

    })
    .catch(() => {
      document.getElementById('status').innerText =
        'Geen verbinding';
    });
}

setInterval(updateSensors,200);

</script>

</body>
</html>

)rawliteral";

// =====================================
// WEB SERVER ROUTES
// =====================================

void setupWebServer() {

  server.on("/", []() {
    server.send_P(200, "text/html", webpage);
  });

  // INSTELLINGEN WIJZIGEN
  server.on("/set", []() {

    if (server.hasArg("kp"))
      KP = server.arg("kp").toFloat();

    if (server.hasArg("ki"))
      KI = server.arg("ki").toFloat();

    if (server.hasArg("kd"))
      KD = server.arg("kd").toFloat();

    if (server.hasArg("speed"))
      baseSpeed = constrain(
        server.arg("speed").toInt(), 0, 255
      );

    if (server.hasArg("maxspeed"))
      maxSpeed = constrain(
        server.arg("maxspeed").toInt(), 0, 255
      );

    integral = 0;
    previousError = 0;

    server.send(200, "text/plain", "OK");
  });

  // START
  server.on("/start", []() {

    if (calibrated && !calibrating) {
      running = true;
      integral = 0;
      previousError = 0;
      lastPID = millis();
    }

    server.send(200, "text/plain", "OK");
  });

  // STOP
  server.on("/stop", []() {

    running = false;
    stopMotors();

    server.send(200, "text/plain", "STOP");
  });

  // OPSLAAN
  server.on("/save", []() {

    saveSettings();

    server.send(200, "text/plain", "SAVED");
  });

  // KALIBRATIE
  server.on("/calibrate", []() {

    if (server.hasArg("on") &&
        server.arg("on") == "1") {

      running = false;
      stopMotors();

      for (int i = 0; i < 8; i++) {
        sensorMin[i] = 4095;
        sensorMax[i] = 0;
      }

      calibrating = true;
      calibrationStart = millis();
      calibrated = false;
    }

    if (server.hasArg("on") &&
        server.arg("on") == "0") {

      calibrating = false;

      calibrated = true;

      for (int i = 0; i < 8; i++) {
        if (sensorMax[i] - sensorMin[i] < 20) {
          calibrated = false;
        }
      }

      if (calibrated) saveSettings();
    }

    server.send(200, "text/plain", "OK");
  });

  // LIVE SENSORWAARDEN
  server.on("/sensors", []() {

    readSensors();

    float position = 0;
    calculatePosition(position);

    String json = "{";

    json += "\"values\":[";

    for (int i = 0; i < 8; i++) {
      json += String(lineValues[i]);

      if (i < 7) json += ",";
    }

    json += "],";

    json += "\"position\":";
    json += String(position, 2);

    json += ",\"running\":";
    json += running ? "true" : "false";

    json += ",\"calibrated\":";
    json += calibrated ? "true" : "false";

    json += ",\"calibrating\":";
    json += calibrating ? "true" : "false";

    json += "}";

    server.send(200, "application/json", json);
  });

  server.begin();
}

// =====================================
// SETUP
// =====================================

void setup() {

  Serial.begin(115200);

  // MOTORPINS
  pinMode(LEFT_IN1, OUTPUT);
  pinMode(LEFT_IN2, OUTPUT);
  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);

  analogWriteResolution(8);

  stopMotors();

  // ADC
  analogReadResolution(12);

  // SENSORPINS
  for (int i = 0; i < 8; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  // INSTELLINGEN LADEN
  loadSettings();

  // WIFI ACCESS POINT
  WiFi.mode(WIFI_AP);

  WiFi.softAP(wifiName, wifiPassword);

  Serial.println();
  Serial.println("LINEFOLLOWER WIFI");
  Serial.println(WiFi.softAPIP());

  // WEB SERVER
  setupWebServer();

  lastPID = millis();
}

// =====================================
// MAIN LOOP
// =====================================

void loop() {

  server.handleClient();

  if (calibrating) {

    readSensors();

    if (millis() - calibrationStart >= 5000) {

      calibrating = false;
      calibrated = true;

      for (int i = 0; i < 8; i++) {
        if (sensorMax[i] - sensorMin[i] < 20) {
          calibrated = false;
        }
      }

      if (calibrated) {
        saveSettings();
      }
    }

    stopMotors();
    return;
  }

  if (running) {
    readSensors();
    updatePID();
  } else {
    stopMotors();
  }

}
