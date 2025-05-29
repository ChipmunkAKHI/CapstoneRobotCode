#include <WiFi.h>
#include <WebServer.h>
#include <time.h>
#include <LiquidCrystal_I2C.h>  // LCD library
#include "actions.h"


const char* ssid = "Serwaa";
const char* password = "beserious101";

WebServer server(80);
LiquidCrystal_I2C lcd(0x27, 16, 2); 

String scheduledTime = "";
bool actionDone = false;
unsigned long lastCheck = 0;
unsigned long lastTimeSync = 0;

const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Room Mapping and Cleaning</title>
  <style>
    body {
      font-family: 'Arial', sans-serif;
      background: linear-gradient(to right, #f7f7f7, #e2e2e2);
      margin: 0;
      display: flex;
      justify-content: center;
      align-items: center;
      height: 100vh;
    }
    .container {
      background-color: white;
      padding: 30px;
      border-radius: 8px;
      box-shadow: 0 4px 10px rgba(0, 0, 0, 0.1);
      width: 300px;
      text-align: center;
    }
    h2 { color: #333; font-size: 24px; margin-bottom: 20px; font-weight: 600; }
    button {
      background-color: #4CAF50;
      color: white;
      border: none;
      padding: 12px 20px;
      margin-top: 15px;
      cursor: pointer;
      font-size: 16px;
      border-radius: 5px;
      box-shadow: 0 2px 5px rgba(0, 0, 0, 0.1);
      transition: background-color 0.3s ease;
      width: 100%;
    }
    button:hover { background-color: #45a049; }
    label { font-weight: bold; margin-top: 20px; display: block; font-size: 16px; }
    input[type="time"] {
      margin-top: 10px;
      padding: 10px;
      font-size: 16px;
      width: 100%;
      border: 1px solid #ccc;
      border-radius: 5px;
      box-sizing: border-box;
    }
    .section { margin-bottom: 20px; }
  </style>
</head>
<body>
  <div class="container">
    <h2>Room Mapping and Cleaning</h2>
    <div class="section">
      <button onclick="fetch('/startscan')">Start Scan</button>
    </div>
    <div class="section">
      <button onclick="fetch('/startclean')">Start Cleaning</button>
    </div>
    <div class="section">
      <label for="scheduleTime">Set Cleaning Time:</label>
      <input type="time" id="scheduleTime" name="scheduleTime" oninput="previewTime(this.value)">
      <button onclick="setSchedule()">Set Schedule</button>
    </div>
  </div>

  <script>
    function setSchedule() {
      const time = document.getElementById('scheduleTime').value;
      fetch('/schedule?time=' + time);
    }
    function previewTime(value) {
      fetch(`/livepreview?time=${encodeURIComponent(value)}`)
      .then(response => response.text())
      .then(data => console.log("LCD preview updated:", data));
    }
  </script>
</body>
</html>
)rawliteral";

// Time sync from NTP
void syncTime() {
  configTime(0, 0, "pool.ntp.org");
  Serial.print("Waiting for NTP time");
  while (time(nullptr) < 100000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nTime synchronized!");
}

// Format current time
String getCurrentTimeStr() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) return "";
  char buffer[6];
  sprintf(buffer, "%02d:%02d", timeinfo.tm_hour, timeinfo.tm_min);
  return String(buffer);
}



void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(MOTOR_LEFT_FWD, OUTPUT);
  pinMode(MOTOR_LEFT_BCK, OUTPUT);
  pinMode(MOTOR_RIGHT_FWD, OUTPUT);
  pinMode(MOTOR_RIGHT_BCK, OUTPUT);
  pinMode(VACUUM_MOTOR, OUTPUT);
  digitalWrite(VACUUM_MOTOR, LOW);
  pinMode(IR_LEFT, INPUT);
  pinMode(IR_RIGHT, INPUT);
  pinMode(IR_FRONT, INPUT);
  pinMode(IR_BOTTOM, INPUT);

  // LCD init
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Connecting...");

  // WiFi connect
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.println(WiFi.localIP());

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("WiFi connected!");
  lcd.setCursor(0, 1);
  lcd.print(WiFi.localIP());

  syncTime();

  // Web routes
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", htmlPage);
  });

  server.on("/startscan", HTTP_GET, []() {
    Serial.println("Starting room scan...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Room scan started");
    startScan();
    server.send(200, "text/plain", "Scan started");
  });

  server.on("/startclean", HTTP_GET, []() {
    Serial.println("Starting cleaning...");
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Cleaning started");
    startClean();
    server.send(200, "text/plain", "Cleaning started");
  });

  server.on("/schedule", HTTP_GET, []() {
    if (server.hasArg("time")) {
      scheduledTime = server.arg("time");
      actionDone = false;
      Serial.print("Scheduled cleaning time: ");
      Serial.println(scheduledTime);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Scheduled for:");
      lcd.setCursor(0, 1);
      lcd.print(scheduledTime);
      server.send(200, "text/plain", "Schedule set for " + scheduledTime);
    } else {
      server.send(400, "text/plain", "Missing time argument");
    }
  });

  server.on("/livepreview", HTTP_GET, []() {
    if (server.hasArg("time")) {
      String previewTime = server.arg("time");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Preview time:");
      lcd.setCursor(0, 1);
      lcd.print(previewTime);
      server.send(200, "text/plain", "Preview updated");
    } else {
      server.send(400, "text/plain", "Missing time argument");
    }
  });

  server.begin();
}

void loop() {
  server.handleClient();

  if (millis() - lastTimeSync > 3600000) {
    syncTime();
    lastTimeSync = millis();
  }

  if (millis() - lastCheck > 1000) {
    lastCheck = millis();
    String now = getCurrentTimeStr();
    if (scheduledTime == now && !actionDone) {
      Serial.println("Scheduled time reached! Cleaning has started.");
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Scheduled Clean:");
      lcd.setCursor(0, 1);
      lcd.print(now);
      startClean();
      actionDone = true;
    }

    if (now == "00:00") {
      actionDone = false;
    }
  }
}
