#include <NewPing.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>
#include <WiFiClientSecure.h>

// WiFi Configuration
const char* ssid = "Prime95_4G";          // CHANGE THIS
const char* password = "Jrdsk=2025";  // CHANGE THIS
const char* server = "pothole-detection-pcio.onrender.com";

// Hardware Pins - Tailored for your specific NodeMCU
#define TRIGGER_PIN D6  // Connect to HC-SR04 TRIG
#define ECHO_PIN D5     // Connect to HC-SR04 ECHO through voltage divider
#define GPS_RX_PIN D7   // Connect to GPS TX through voltage divider
#define GPS_TX_PIN D8   // Connect to GPS RX
#define LED_PIN D4      // Built-in LED (active low)

// System Parameters
#define MAX_DISTANCE 200          // Maximum distance to measure (cm)
#define FILTER_SIZE 5             // Size of moving average filter
#define SAMPLE_WINDOW 10          // Window for baseline calculation
#define STABILITY_THRESHOLD 0.5   // Threshold for stable baseline
#define MAX_DETECTION_RANGE 50    // Maximum range for pothole detection
#define POTHOLE_THRESHOLD 2.0     // Depth threshold for pothole detection (cm)
#define GPS_BAUD 9600             // Baud rate for GPS module (NEO-6M uses 9600)
#define POST_INTERVAL 30000       // Interval between server posts (ms)
#define SIMULATION_TIMEOUT 30000  // Timeout for GPS fix before simulation
#define HTTP_TIMEOUT 15000        // HTTP request timeout (ms)

// Sensor Objects
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);
TinyGPSPlus gps;
SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);

// State Variables
float baselineDistance = 0;
bool potholeDetected = false;
float filterBuffer[FILTER_SIZE] = {0};
float sampleBuffer[SAMPLE_WINDOW] = {0};
int filterIndex = 0;
int sampleIndex = 0;

// GPS Data
float latitude = 0;
float longitude = 0;
bool hasGPSFix = false;
bool simulateData = false;
unsigned long lastFixTime = 0;

// WiFi Status
unsigned long lastPostTime = 0;
int connectionFailures = 0;

// Function prototypes
float getSmoothDistance();
void updateBaseline();
void updateGPS();
void sendToServer(float lat, float lon, float depth, bool isPothole);
bool reconnectWiFi();
void testServerConnection();
void displayStatus();
void generateTestData();
void blinkLED(int times, int delayTime);

void setup() {
  Serial.begin(9600);  // Using 9600 baud rate as specified on your board
  Serial.println("\n\n--- Pothole Detection System Starting ---");
 
  // Initialize GPS serial
  gpsSerial.begin(GPS_BAUD);
 
  // Setup LED pin
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  // LED off initially (active low)
 
  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  int wifiAttempts = 0;
 
  // Flash LED while connecting
  while(WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    digitalWrite(LED_PIN, LOW);  // LED on
    delay(250);
    digitalWrite(LED_PIN, HIGH); // LED off
    delay(250);
    Serial.print(".");
    wifiAttempts++;
  }
 
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    blinkLED(3, 200);  // 3 quick blinks = connected
  } else {
    Serial.println("\nWiFi Connection Failed. Will retry later.");
    blinkLED(5, 500);  // 5 slow blinks = failed
  }

  // Initialize ultrasonic sensor
  delay(1000);
  Serial.println("Calibrating ultrasonic sensor...");
  baselineDistance = getSmoothDistance();
  Serial.print("Initial baseline distance: ");
  Serial.println(baselineDistance);
 
  // Fill buffers with initial readings
  for(int i=0; i<SAMPLE_WINDOW; i++) {
    sampleBuffer[i] = baselineDistance;
  }

  // Initialize simulation timer
  lastFixTime = millis();
 
  Serial.println("Waiting for GPS fix...");
 
  // Test connectivity to server
  if(WiFi.status() == WL_CONNECTED) {
    testServerConnection();
  }
 
  Serial.println("\n--- Setup complete. Starting pothole detection ---");
  Serial.println("Available commands:");
  Serial.println("  'test' - Toggle simulation mode");
  Serial.println("  'wifi' - Reconnect WiFi");
  Serial.println("  'server' - Test server connection");
  Serial.println("  'send' - Send test data");
  Serial.println("  'status' - Display system status");
  Serial.println("  'reset' - Reset baseline distance");
  Serial.println("  'help' - Show this help");
}

bool reconnectWiFi() {
  if(WiFi.status() == WL_CONNECTED) return true;
 
  Serial.println("WiFi disconnected - attempting reconnect");
  WiFi.disconnect();
  delay(1000);
  WiFi.begin(ssid, password);
 
  int attempts = 0;
  while(WiFi.status() != WL_CONNECTED && attempts < 10) {
    digitalWrite(LED_PIN, LOW);  // LED on
    delay(250);
    digitalWrite(LED_PIN, HIGH); // LED off
    delay(250);
    Serial.print(".");
    attempts++;
  }
 
  if(WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Reconnected");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
    blinkLED(2, 200);  // 2 quick blinks = reconnected
    return true;
  } else {
    Serial.println("\nReconnect failed");
    blinkLED(4, 400);  // 4 medium blinks = reconnection failed
    return false;
  }
}

void testServerConnection() {
  // Try HTTPS connection
  WiFiClientSecure secureClient;
  HTTPClient https;
 
  Serial.println("Testing HTTPS server connection...");
 
  // Disable certificate validation for testing
  secureClient.setInsecure();
 
  // Set timeout
  https.setTimeout(HTTP_TIMEOUT);
 
  if(https.begin(secureClient, "https://" + String(server) + "/api/detections")) {
    int httpCode = https.GET();
    String response = https.getString();
   
    Serial.print("HTTPS Test - Status code: ");
    Serial.println(httpCode);
    Serial.print("Response: ");
    Serial.println(response);
   
    if(httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_NO_CONTENT) {
      Serial.println("Server connection successful!");
      blinkLED(3, 200);  // 3 quick blinks = success
    } else {
      Serial.println("Server connection issue");
      blinkLED(2, 500);  // 2 slow blinks = issue
    }
   
    https.end();
  } else {
    Serial.println("Failed to connect to server for HTTPS test");
    blinkLED(5, 300);  // 5 medium blinks = connection failure
  }
}

float getSmoothDistance() {
  // Take multiple readings and average them for stability
  float sum = 0;
  int validReadings = 0;
 
  for(int i=0; i<3; i++) {
    float distance = sonar.ping_cm();
    if(distance > 0) {
      sum += distance;
      validReadings++;
    }
    delay(10);  // Short delay between pings
  }
 
  float currentReading;
  if(validReadings > 0) {
    currentReading = sum / validReadings;
  } else {
    currentReading = MAX_DISTANCE;  // Default to max if no valid readings
  }
 
  // Update filter buffer
  filterBuffer[filterIndex] = currentReading;
  filterIndex = (filterIndex + 1) % FILTER_SIZE;
 
  // Calculate moving average
  sum = 0;
  for(int i=0; i<FILTER_SIZE; i++) {
    sum += filterBuffer[i];
  }
  return sum / FILTER_SIZE;
}

void updateBaseline() {
  float sum = 0, mean = 0, variance = 0;
 
  // Calculate mean
  for(int i=0; i<SAMPLE_WINDOW; i++) {
    sum += sampleBuffer[i];
  }
  mean = sum / SAMPLE_WINDOW;
 
  // Calculate variance
  for(int i=0; i<SAMPLE_WINDOW; i++) {
    variance += sq(sampleBuffer[i] - mean);
  }
  variance /= SAMPLE_WINDOW;
 
  // Update baseline if measurements are stable
  if(sqrt(variance) < STABILITY_THRESHOLD) {
    baselineDistance = mean;
    Serial.print("Baseline updated: ");
    Serial.println(baselineDistance);
  }
}

void updateGPS() {
  // Process any available GPS data
  while(gpsSerial.available() > 0) {
    char c = gpsSerial.read();
    // Uncomment for GPS debugging
    // Serial.write(c);
    if(gps.encode(c)) {
      if(gps.location.isValid()) {
        latitude = gps.location.lat();
        longitude = gps.location.lng();
       
        // First fix or reconnection
        if(!hasGPSFix) {
          Serial.println("GPS Fix Acquired!");
          Serial.printf("Location: %.6f, %.6f\n", latitude, longitude);
          blinkLED(2, 200);  // 2 quick blinks = GPS fix
        }
       
        hasGPSFix = true;
        lastFixTime = millis();
        simulateData = false;  // Exit simulation mode if we were in it
      }
    }
  }

  // Enable simulation if no GPS fix for specified timeout
  if(!hasGPSFix && (millis() - lastFixTime > SIMULATION_TIMEOUT)) {
    if(!simulateData) {
      simulateData = true;
      Serial.println("No GPS fix. Entering simulation mode");
      blinkLED(3, 500);  // 3 slow blinks = simulation mode
    }
  }
}

void sendToServer(float lat, float lon, float depth, bool isPothole) {
  // Check WiFi connection and attempt reconnection if needed
  if(WiFi.status() != WL_CONNECTED) {
    if(!reconnectWiFi()) {
      connectionFailures++;
      Serial.print("Connection failures: ");
      Serial.println(connectionFailures);
     
      // Try to reset WiFi after multiple failures
      if(connectionFailures > 5) {
        Serial.println("Multiple connection failures. Resetting WiFi...");
        WiFi.disconnect();
        delay(3000);
        WiFi.begin(ssid, password);
        connectionFailures = 0;
      }
      return;
    }
  }

  // Indicate data transmission with LED
  digitalWrite(LED_PIN, LOW);  // LED on for transmission

  // Using WiFiClientSecure for HTTPS connections
  WiFiClientSecure secureClient;
  HTTPClient https;
 
  // Disable certificate validation for testing
  secureClient.setInsecure();
 
  // Set timeout
  https.setTimeout(HTTP_TIMEOUT);
 
  Serial.println("Sending data to server...");
 
  if(https.begin(secureClient, "https://" + String(server) + "/api/detections")) {
    https.addHeader("Content-Type", "application/json");
   
    // Create JSON payload
    String payload = String("{") +
                   "\"lat\":" + String(lat, 6) + "," +
                   "\"lon\":" + String(lon, 6) + "," +
                   "\"depth\":" + String(depth) + "," +
                   "\"is_pothole\":" + (isPothole ? "true" : "false") + "}";
   
    Serial.print("Sending payload: ");
    Serial.println(payload);
   
    // Send data and measure response time
    unsigned long startTime = millis();
    int httpCode = https.POST(payload);
    unsigned long responseTime = millis() - startTime;
   
    String response = https.getString();
   
    Serial.print("HTTP Code: ");
    Serial.println(httpCode);
    Serial.print("Response time (ms): ");
    Serial.println(responseTime);
    Serial.print("Response: ");
    Serial.println(response);
   
    if(httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_CREATED || httpCode == HTTP_CODE_NO_CONTENT) {
      Serial.println("Data successfully sent to server");
      connectionFailures = 0;
      blinkLED(3, 200);  // 3 quick blinks = success
    } else {
      Serial.printf("HTTP error %d: %s\n", httpCode, https.errorToString(httpCode).c_str());
      connectionFailures++;
      blinkLED(4, 300);  // 4 medium blinks = HTTP error
    }
   
    https.end();
  } else {
    Serial.println("Failed to connect to server");
    connectionFailures++;
    blinkLED(5, 300);  // 5 medium blinks = connection failure
  }
 
  digitalWrite(LED_PIN, HIGH);  // LED off after transmission
}

void generateTestData() {
  static float baseLat = 51.5074; // London coordinates as default
  static float baseLon = -0.1278;
 
  // Generate random offset (about 100-500 meter range)
  float latOffset = random(-500, 500) / 10000.0;
  float lonOffset = random(-500, 500) / 10000.0;
 
  // Random depth between 2.0-10.0cm
  float depth = random(20, 100) / 10.0;
 
  // Send simulated data
  sendToServer(baseLat + latOffset,
               baseLon + lonOffset,
               depth,
               true);
 
  Serial.println("Sent simulated pothole data");
}

void displayStatus() {
  Serial.println("\n--- SYSTEM STATUS ---");
 
  // Distance readings
  float currentDist = getSmoothDistance();
  Serial.printf("Current distance: %.1fcm\n", currentDist);
  Serial.printf("Baseline distance: %.1fcm\n", baselineDistance);
  Serial.printf("Difference: %.1fcm\n", currentDist - baselineDistance);
 
  // Detection status
  if(currentDist > (baselineDistance + POTHOLE_THRESHOLD) &&
     currentDist < MAX_DETECTION_RANGE) {
    Serial.println("Current reading indicates POTHOLE");
  } else {
    Serial.println("Current reading is NORMAL");
  }
 
  // GPS status
  Serial.print("GPS: ");
  if(hasGPSFix) {
    Serial.printf("Fix Acquired (%.6f, %.6f)\n", latitude, longitude);
   
    // If we have a fix, show date/time if available
    if(gps.date.isValid() && gps.time.isValid()) {
      Serial.printf("Date/Time: %02d/%02d/%d %02d:%02d:%02d UTC\n",
              gps.date.day(), gps.date.month(), gps.date.year(),
              gps.time.hour(), gps.time.minute(), gps.time.second());
    }
   
    // Show satellites and accuracy if available
    if(gps.satellites.isValid()) {
      Serial.printf("Satellites: %d\n", gps.satellites.value());
    }
   
  } else if(simulateData) {
    Serial.println("Simulation Mode");
  } else {
    Serial.println("No Fix");
  }
 
  // WiFi status
  Serial.print("WiFi: ");
  if(WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected (IP: ");
    Serial.print(WiFi.localIP());
    Serial.print(", RSSI: ");
    Serial.print(WiFi.RSSI());
    Serial.println(" dBm)");
  } else {
    Serial.println("Disconnected");
  }
 
  // Connection status
  Serial.print("Connection failures: ");
  Serial.println(connectionFailures);
 
  // Memory status
  Serial.print("Free heap: ");
  Serial.println(ESP.getFreeHeap());
 
  // Runtime
  Serial.print("Uptime: ");
  unsigned long uptime = millis() / 1000;
  Serial.print(uptime / 60);
  Serial.print(" minutes, ");
  Serial.print(uptime % 60);
  Serial.println(" seconds");
 
  Serial.println("---------------------");
}

void blinkLED(int times, int delayTime) {
  for(int i=0; i<times; i++) {
    digitalWrite(LED_PIN, LOW);  // LED on
    delay(delayTime);
    digitalWrite(LED_PIN, HIGH); // LED off
    delay(delayTime);
  }
}

void loop() {
  // Update GPS data
  updateGPS();
 
  // Get filtered distance
  float currentDistance = getSmoothDistance();
 
  // Update sample buffer
  sampleBuffer[sampleIndex] = currentDistance;
  sampleIndex = (sampleIndex + 1) % SAMPLE_WINDOW;
 
  // Periodically update baseline
  if(sampleIndex == 0) {
    updateBaseline();
  }
 
  // Detect potholes by comparing with baseline
  bool isPothole = currentDistance > (baselineDistance + POTHOLE_THRESHOLD) &&
                  currentDistance < MAX_DETECTION_RANGE;
 
  // Handle detection events
  if(!potholeDetected && isPothole) {
    Serial.println("\n⚠️ POTHOLE DETECTED ⚠️");
    Serial.printf("Depth: %.1fcm\n", currentDistance - baselineDistance);
   
    // Flash LED rapidly to indicate pothole detection
    blinkLED(5, 100);
   
    if(hasGPSFix || simulateData) {
      if(simulateData) {
        // Use simulated GPS data
        generateTestData();
      } else {
        // Use real GPS data
        Serial.printf("Location: %.6f, %.6f\n", latitude, longitude);
        if(millis() - lastPostTime > POST_INTERVAL) {
          sendToServer(latitude, longitude, currentDistance - baselineDistance, true);
          lastPostTime = millis();
        }
      }
    } else {
      Serial.println("No GPS fix - data not sent");
      blinkLED(2, 500);  // 2 slow blinks = no GPS fix
    }
   
    potholeDetected = true;
  }
  else if(!isPothole) {
    potholeDetected = false;
  }
 
  // Periodic status report (every 5 seconds)
  static unsigned long lastStatusTime = 0;
  if(millis() - lastStatusTime > 5000) {
    displayStatus();
    lastStatusTime = millis();
  }
 
  // Periodic WiFi check and server test (every 5 minutes)
  static unsigned long lastTestTime = 0;
  if(millis() - lastTestTime > 300000) {  // 5 minutes
    if(WiFi.status() != WL_CONNECTED) {
      reconnectWiFi();
    }
    if(WiFi.status() == WL_CONNECTED) {
      testServerConnection();
    }
    lastTestTime = millis();
  }
 
  // Manual test commands via Serial
  if(Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim();
   
    if(input == "test") {
      simulateData = !simulateData;
      Serial.println(simulateData ? "Simulation mode activated" : "Simulation mode deactivated");
      blinkLED(simulateData ? 3 : 2, 300);
    } else if(input == "wifi") {
      reconnectWiFi();
    } else if(input == "server") {
      testServerConnection();
    } else if(input == "send") {
      generateTestData();
    } else if(input == "status") {
      displayStatus();
    } else if(input == "reset") {
      Serial.println("Resetting baseline distance...");
      baselineDistance = getSmoothDistance();
      Serial.printf("New baseline: %.1fcm\n", baselineDistance);
      for(int i=0; i<SAMPLE_WINDOW; i++) {
        sampleBuffer[i] = baselineDistance;
      }
      blinkLED(3, 300);
    } else if(input == "help") {
      Serial.println("Available commands:");
      Serial.println("  'test' - Toggle simulation mode");
      Serial.println("  'wifi' - Reconnect WiFi");
      Serial.println("  'server' - Test server connection");
      Serial.println("  'send' - Send test data");
      Serial.println("  'status' - Display system status");
      Serial.println("  'reset' - Reset baseline distance");
      Serial.println("  'help' - Show this help");
    }
  }
 
  delay(100);  // Small delay for stability
}
