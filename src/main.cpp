#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PN532.h>
#include <WiFi.h>
#include "time.h"
#include <NoDelay.h>
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

// --- WiFi Credentials ---
const char *ssid = "pocof5";
const char *password = "soypobre";
const unsigned long WIFI_CONNECT_TIMEOUT_MS = 15000; // 15 seconds to connect to WiFi

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -7 * 3600; // Adjusted for -7 hours from UTC.
const int daylightOffset_sec = 0;

// --- System State (Room Occupancy) ---
enum RoomStateType
{
    ROOM_VACANT,
    ROOM_OCCUPIED
};
RoomStateType currentRoomState = ROOM_VACANT;

// --- Detailed Operational Sub-States ---
enum SystemSubStateType
{
    SUBSTATE_IDLE,
    SUBSTATE_AWAITING_DOOR_OPEN_FOR_ENTRY,
    SUBSTATE_AWAITING_DOOR_CLOSE_AFTER_ENTRY,
    SUBSTATE_AWAITING_PIR_CONFIRMATION_FOR_ENTRY,
    SUBSTATE_AWAITING_DOOR_OPEN_FOR_EXIT,
    SUBSTATE_AWAITING_DOOR_CLOSE_AFTER_EXIT,
    SUBSTATE_AWAITING_NO_PIR_CONFIRMATION_FOR_EXIT
};
SystemSubStateType currentSystemSubState = SUBSTATE_IDLE;

// --- Timeouts and Durations ---
// These are the editable values for the system's operational timeouts and durations.
unsigned long ENTRY_PIR_WAIT_TIMEOUT_MS = 15000;    // 15s to detect PIR after entry door close
unsigned long EXIT_NO_PIR_WAIT_DURATION_MS = 10000; // 10s to confirm no PIR after exit door close
unsigned long ALARM_DURATION_MS = 10000;            // Alarm sounds for 10 seconds

const unsigned long EXIT_PIR_ACTIVATION_DELAY_MS = 5000;            // 5s delay after door close on exit before PIR is considered for exit confirmation
const unsigned long PROLONGED_DOOR_OPEN_ALERT_THRESHOLD_MS = 10000; // 10s for door open alert
const unsigned long RFID_AUTHORIZATION_WINDOW_MS = 5000;            // 5s window for RFID to be considered authorizing a door open
const unsigned long AWAIT_DOOR_OPEN_TIMEOUT_MS = 10000;             // 10s to wait for door to open after RFID scan

// NC Switch with INPUT_PULLUP (actuated by door closing): LOW = Door Open, HIGH = Door Closed.
int doorSwitchState = LOW; // Initialize assuming door is open (NC switch: open -> LOW when not actuated)
int lastDoorSwitchState = LOW;
unsigned long doorOpenStartTime = 0;
bool doorCurrentlyOpen = true;                   // Initialize assuming door is open, will be updated in setup
unsigned long lastDoorSwitchChangeTime = 0;      // For debouncing
const unsigned long DOOR_DEBOUNCE_DELAY_MS = 50; // Debounce delay for door switch

// --- RFID Interaction State ---
unsigned long lastValidRfidTime = 0;     // Timestamp of the last valid RFID scan
String lastValidRfidDisplayTime = "N/A"; // Formatted time of the last valid RFID scan for display

// --- Entry/Exit Process State Variables ---
unsigned long entryPIRWaitStartTime = 0;
unsigned long exitNoPIRWaitStartTime = 0;
bool motionDetectedDuringExitWait = false;

// --- Prolonged Door Open Alert State ---
bool prolongedDoorAlertActive = false;

const int buzzerPin = 18;

const int switchPin = 19; // This is now the Door Limit Switch

const int pirPin = 15;

const int rele1Pin = 5;
#define RELE_ON LOW
#define RELE_OFF HIGH

#define PN532_IRQ (16)
#define PN532_RESET (17)
Adafruit_PN532 nfc(PN532_IRQ, PN532_RESET);

// Server for web interface
AsyncWebServer server(80);

// --- Global Variables ---

// PIR
int lastPirState = LOW;
// unsigned long pirCooldownEnd = 0; // Replaced by pirCooldownTimer
const unsigned long PIR_COOLDOWN_MS = 5000;       // 5 seconds cooldown for PIR
noDelay pirCooldownTimer(PIR_COOLDOWN_MS, false); // Initialize with 5s, initially stopped

// System State (overall)
enum SystemStateType
{
    CALIBRATING_PIR,
    RUNNING
};
SystemStateType systemState = CALIBRATING_PIR;
unsigned long calibrationStartTime = 0;
const unsigned long PIR_CALIBRATION_DURATION_MS = 5000; // 5 seconds for PIR calibration
unsigned long lastCalibrationPrintTime = 0;
const unsigned long CALIBRATION_PRINT_INTERVAL_MS = 250;
int calibrationDotsPrinted = 0;
const int MAX_CALIBRATION_DOTS = PIR_CALIBRATION_DURATION_MS / CALIBRATION_PRINT_INTERVAL_MS;

// Armed State - REMOVED (systemArmed is no longer used)
// bool systemArmed = false; // REMOVED
bool alarmActive = false;
unsigned long alarmStopTime = 0;

// Relay (for NFC access)
unsigned long relayAccessGrantedUntil = 0;
const unsigned long RELAY_ACCESS_DURATION_MS = 3000; // Relay stays on for 3 seconds for NFC access

// NFC
uint8_t validUid1[] = {0xD4, 0x80, 0x48, 0x05};
uint8_t validUid2[] = {0xBA, 0x2F, 0xB0, 0x01};
const uint8_t VALID_UID_LENGTH = 4;
uint8_t uid[] = {0, 0, 0, 0, 0, 0, 0};
uint8_t uidLength;
unsigned long nfcReadCooldownEnd = 0;               // Cooldown after a card is processed
const unsigned long NFC_PROCESS_COOLDOWN_MS = 1000; // 1 second
unsigned long nextNfcCheckTime = 0;                 // Interval to attempt an NFC read
const unsigned long NFC_CHECK_INTERVAL_MS = 100;    // Check for NFC card every 100ms

// Buzzer State (for non-blocking beeps)
enum BeepPattern
{
    BEEP_NONE,
    BEEP_CONFIRMATION,
    BEEP_ERROR,
    BEEP_PROLONGED_DOOR // Added
};
BeepPattern currentBeepPattern = BEEP_NONE;
int beepSequenceStep = 0;
unsigned long nextBeepActionTime = 0;
// Buzzer timings
const int BEEP_SHORT_DURATION_MS = 100;
const int BEEP_ERROR_PAUSE_MS = 50;

// --- Helper Functions ---

// Function to get formatted time string
String getFormattedTime()
{
    if (WiFi.status() != WL_CONNECTED || time(nullptr) < 1000000000) // Check if time is synchronized
    {
        return String(millis()); // Fallback to millis if no NTP time
    }
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo))
    {
        Serial.println("Failed to obtain time");
        return String(millis()); // Fallback
    }
    char timeStringBuff[50]; // Buffer to hold the formatted time string
    strftime(timeStringBuff, sizeof(timeStringBuff), "%Y-%m-%d %H:%M:%S", &timeinfo);
    return String(timeStringBuff);
}

void startConfirmationBeep()
{
    // Only start if not already beeping or if the alarm isn't also trying to use the buzzer
    // And also not if prolonged door alert is active with its own pattern
    if (currentBeepPattern == BEEP_NONE && !alarmActive && !prolongedDoorAlertActive)
    {
        currentBeepPattern = BEEP_CONFIRMATION;
        beepSequenceStep = 0;
        nextBeepActionTime = millis(); // Start immediately
    }
}

void startErrorBeep()
{
    if (currentBeepPattern == BEEP_NONE && !alarmActive && !prolongedDoorAlertActive)
    {
        currentBeepPattern = BEEP_ERROR;
        beepSequenceStep = 0;
        nextBeepActionTime = millis(); // Start immediately
    }
}

void handleBuzzer()
{
    if (alarmActive)
    {                                  // Alarm has priority for the buzzer
        digitalWrite(buzzerPin, HIGH); // Continuous buzzer during alarm
        if (currentBeepPattern != BEEP_NONE && currentBeepPattern != BEEP_PROLONGED_DOOR)
        {                                   // Don't reset if it was prolonged door, alarm overrides
            currentBeepPattern = BEEP_NONE; // Stop any other beep patterns
        }
        if (prolongedDoorAlertActive)
        { // If alarm starts, prolonged alert stops making sound via this
          // prolongedDoorAlertActive = false; // Alarm starting should also cancel the alert state if desired
        }
        return;
    }
    // If alarm just stopped, ensure buzzer is turned off if no other pattern is active
    // stopAlarm() handles turning buzzer LOW.

    if (currentBeepPattern == BEEP_NONE || millis() < nextBeepActionTime)
    {
        // If BEEP_NONE, but prolongedDoorAlertActive is true, it means the pattern was interrupted or not set.
        // This check ensures prolonged alert can restart if nothing else is running.
        if (currentBeepPattern == BEEP_NONE && prolongedDoorAlertActive && !alarmActive)
        {
            currentBeepPattern = BEEP_PROLONGED_DOOR;
            beepSequenceStep = 0;
            nextBeepActionTime = millis(); // Start immediately
        }
        else if (currentBeepPattern == BEEP_NONE)
        {
            return; // Truly nothing to do
        }
        if (millis() < nextBeepActionTime && currentBeepPattern != BEEP_PROLONGED_DOOR)
        { // Prolonged door manages its own looping timing
            return;
        }
    }

    switch (currentBeepPattern)
    {
    case BEEP_CONFIRMATION:
        if (beepSequenceStep == 0)
        { // Start beep
            digitalWrite(buzzerPin, HIGH);
            nextBeepActionTime = millis() + BEEP_SHORT_DURATION_MS;
            beepSequenceStep++;
        }
        else
        { // Stop beep
            digitalWrite(buzzerPin, LOW);
            currentBeepPattern = BEEP_NONE; // End of pattern
        }
        break;
    case BEEP_ERROR:
        switch (beepSequenceStep)
        {
        case 0: // Start first beep
            digitalWrite(buzzerPin, HIGH);
            nextBeepActionTime = millis() + BEEP_SHORT_DURATION_MS;
            beepSequenceStep++;
            break;
        case 1: // Stop first beep, start pause
            digitalWrite(buzzerPin, LOW);
            nextBeepActionTime = millis() + BEEP_ERROR_PAUSE_MS;
            beepSequenceStep++;
            break;
        case 2: // Start second beep
            digitalWrite(buzzerPin, HIGH);
            nextBeepActionTime = millis() + BEEP_SHORT_DURATION_MS;
            beepSequenceStep++;
            break;
        case 3: // Stop second beep
            digitalWrite(buzzerPin, LOW);
            currentBeepPattern = BEEP_NONE; // End of pattern
            break;
        }
        break;
    case BEEP_PROLONGED_DOOR:
        if (!prolongedDoorAlertActive && !alarmActive)
        { // If flag was turned off externally and no alarm
            currentBeepPattern = BEEP_NONE;
            digitalWrite(buzzerPin, LOW);
            break;
        }
        // Pattern: 200ms ON, 800ms OFF, repeats.
        if (beepSequenceStep == 0)
        { // Start beep
            digitalWrite(buzzerPin, HIGH);
            nextBeepActionTime = millis() + 200;
            beepSequenceStep = 1;
        }
        else
        { // Start pause
            digitalWrite(buzzerPin, LOW);
            nextBeepActionTime = millis() + 800;
            beepSequenceStep = 0; // Loop back
        }
        // This pattern continues as long as currentBeepPattern is BEEP_PROLONGED_DOOR and prolongedDoorAlertActive is true
        break;
    default:                          // BEEP_NONE or unexpected
        digitalWrite(buzzerPin, LOW); // Ensure buzzer is off
        currentBeepPattern = BEEP_NONE;
        break;
    }
}

void startAlarm()
{
    if (!alarmActive)
    {
        Serial.println(getFormattedTime() + ": ALARM ACTIVATED!");
        alarmActive = true;

        prolongedDoorAlertActive = false; // Alarm overrides prolonged door alert
        if (currentBeepPattern == BEEP_PROLONGED_DOOR)
        {
            currentBeepPattern = BEEP_NONE; // Stop prolonged beep pattern sound
            // Buzzer will be set HIGH by handleBuzzer due to alarmActive
        }
        alarmStopTime = millis() + ALARM_DURATION_MS;
    }
}

void stopAlarm()
{
    if (alarmActive)
    {
        Serial.println(getFormattedTime() + ": Alarm stopped.");
        alarmActive = false;
        digitalWrite(rele1Pin, RELE_OFF);
        digitalWrite(buzzerPin, LOW); // Explicitly turn off buzzer
        alarmStopTime = 0;
        pirCooldownTimer.start();       // Start PIR cooldown
        currentBeepPattern = BEEP_NONE; // Ensure any beep sequence is also stopped
    }
}

void grantAccessByNFC()
{
    Serial.println(getFormattedTime() + ": Access requested by NFC.");
    if (alarmActive)
    {
        stopAlarm(); // NFC can stop an active alarm
    }

    startConfirmationBeep(); // 1 short beep for valid RFID

    digitalWrite(rele1Pin, RELE_ON); // Unlock electric lock
    relayAccessGrantedUntil = millis() + RELAY_ACCESS_DURATION_MS;
    lastValidRfidTime = millis();                  // Mark time of valid RFID
    lastValidRfidDisplayTime = getFormattedTime(); // Store formatted time for display

    if (currentRoomState == ROOM_VACANT)
    {
        currentSystemSubState = SUBSTATE_AWAITING_DOOR_OPEN_FOR_ENTRY;
        Serial.println(getFormattedTime() + ": NFC valid for ENTRY. Waiting for door to open.");
    }
    else
    { // ROOM_OCCUPIED
        currentSystemSubState = SUBSTATE_AWAITING_DOOR_OPEN_FOR_EXIT;
        Serial.println(getFormattedTime() + ": NFC valid for EXIT. Waiting for door to open.");
    }
}

// --- WiFi Setup Function ---
void setupWiFi()
{
    Serial.println(getFormattedTime() + ": Setting up WiFi...");
    WiFi.mode(WIFI_STA); // Set ESP32 to Station mode
    WiFi.begin(ssid, password);
    Serial.print(getFormattedTime() + ": Connecting to WiFi network: ");
    Serial.println(ssid);

    unsigned long wifiStartTime = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - wifiStartTime > WIFI_CONNECT_TIMEOUT_MS)
        {
            Serial.println("\n" + getFormattedTime() + ": WiFi connection FAILED after timeout.");
            return; // Exit if timeout occurs
        }
    }

    Serial.println("\n" + getFormattedTime() + ": WiFi connected successfully!");
    Serial.print(getFormattedTime() + ": IP Address: ");
    Serial.println(WiFi.localIP());

    // --- Initialize NTP after WiFi connection ---
    Serial.println(getFormattedTime() + ": Initializing NTP...");
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo, 10000)) // Wait up to 10 seconds for time sync
    {
        Serial.println(getFormattedTime() + ": Failed to obtain initial NTP time.");
    }
    else
    {
        Serial.println(getFormattedTime() + ": NTP time synchronized.");
        Serial.println(&timeinfo, "Current time: %A, %B %d %Y %H:%M:%S");
    }
}

// --- Helper function to convert SystemStateType to String ---
String systemStateToString(SystemStateType state)
{
    switch (state)
    {
    case CALIBRATING_PIR:
        return "Calibrating PIR";
    case RUNNING:
        return "Running";
    default:
        return "Unknown";
    }
}

// --- Helper function to convert RoomStateType to String ---
String roomStateToString(RoomStateType state)
{
    switch (state)
    {
    case ROOM_VACANT:
        return "Vacant";
    case ROOM_OCCUPIED:
        return "Occupied";
    default:
        return "Unknown";
    }
}

// --- setup() ---
void setup()
{
    Serial.begin(115200);
    // Wait a bit for serial, but not indefinitely.
    unsigned long setupSerialStartTime = millis();
    while (!Serial && millis() - setupSerialStartTime < 2000)
    {
        // Wait for serial port to connect, with a timeout.
    }

    Serial.println("\\n" + getFormattedTime() + ": Initializing Security System...");

    // Initialize LittleFS
    if (!LittleFS.begin(true)) // Pass true to format LittleFS if it's not already formatted
    {
        Serial.println(getFormattedTime() + ": An Error has occurred while mounting LittleFS");
        // Consider halting or indicating error state, as web server relies on LittleFS
        return;
    }
    Serial.println(getFormattedTime() + ": LittleFS mounted successfully.");

    // --- Setup WiFi ---
    setupWiFi(); // Connect to WiFi

    // --- Web Server Routes ---
    // Route for root / web page
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                  if (LittleFS.exists("/index.html")) {
                      request->send(LittleFS, "/index.html", "text/html");
                  } else {
                      request->send(404, "text/plain", "index.html not found");
                  } });

    // Route for CSS file
    server.on("/style.css", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                  if (LittleFS.exists("/style.css")) {
                      request->send(LittleFS, "/style.css", "text/css");
                  } else {
                      request->send(404, "text/plain", "style.css not found");
                  } });

    // API Endpoint to get current status
    server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request)
              {
        JsonDocument doc; // Using ArduinoJson V7
        doc["currentTime"] = getFormattedTime();
        doc["systemState"] = systemStateToString(systemState);
        doc["wifiStatus"] = (WiFi.status() == WL_CONNECTED) ? "Connected" : "Disconnected";
        doc["ipAddress"] = WiFi.localIP().toString();
        doc["roomOccupancy"] = roomStateToString(currentRoomState);
        doc["doorStatus"] = doorCurrentlyOpen ? "OPEN" : "CLOSED";
        doc["pirStatus"] = (lastPirState == HIGH) ? "Motion Detected" : "No Motion";
        doc["alarmStatus"] = alarmActive ? "Active" : "Inactive";
        doc["lastRfidTime"] = lastValidRfidDisplayTime; // MODIFIED to use formatted display time

        String jsonResponse;
        size_t len = serializeJson(doc, jsonResponse);
        if (len > 0) {
            request->send(200, "application/json", jsonResponse);
        } else {
            Serial.println(getFormattedTime() + ": Failed to serialize /status JSON.");
            request->send(500, "text/plain", "Internal Server Error: Failed to serialize /status JSON");
        } });

    // Route to get current parameters
    server.on("/parameters", HTTP_GET, [](AsyncWebServerRequest *request)
              {
                  JsonDocument doc; // Adjust size as needed
                  doc["entryPirTimeout"] = ENTRY_PIR_WAIT_TIMEOUT_MS;
                  doc["exitNoPirWait"] = EXIT_NO_PIR_WAIT_DURATION_MS;
                  doc["alarmDuration"] = ALARM_DURATION_MS;

                  String output;
                  size_t len = serializeJson(doc, output);
                  if (len > 0)
                  {
                      request->send(200, "application/json", output);
                  }
                  else
                  {
                      Serial.println(getFormattedTime() + ": Failed to serialize /parameters JSON.");
                      request->send(500, "text/plain", "Internal Server Error: Failed to serialize /parameters JSON");
                  }
              });

    // Route to update parameters
    server.on("/update-parameters", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total)
              {
      JsonDocument doc; // Adjust size as needed
      DeserializationError error = deserializeJson(doc, (const char*)data);

      if (error) {
        Serial.print(F("deserializeJson() failed: "));
        Serial.println(error.f_str());
        request->send(400, "application/json", "{\"message\":\"Invalid JSON\"}");
        return;
      }

      bool updated = false;
      if (doc.containsKey("entryPirTimeout")) {
        ENTRY_PIR_WAIT_TIMEOUT_MS = doc["entryPirTimeout"].as<unsigned long>();
        updated = true;
      }
      if (doc.containsKey("exitNoPirWait")) {
        EXIT_NO_PIR_WAIT_DURATION_MS = doc["exitNoPirWait"].as<unsigned long>();
        updated = true;
      }
      if (doc.containsKey("alarmDuration")) {
        ALARM_DURATION_MS = doc["alarmDuration"].as<unsigned long>();
        updated = true;
      }

      if (updated) {
        Serial.println("Parameters updated:");
        Serial.print("ENTRY_PIR_WAIT_TIMEOUT_MS: "); Serial.println(ENTRY_PIR_WAIT_TIMEOUT_MS);
        Serial.print("EXIT_NO_PIR_WAIT_DURATION_MS: "); Serial.println(EXIT_NO_PIR_WAIT_DURATION_MS);
        Serial.print("ALARM_DURATION_MS: "); Serial.println(ALARM_DURATION_MS);
        // TODO: Persist parameters to NVM/LittleFS if needed
        request->send(200, "application/json", "{\"message\":\"Parameters updated successfully\"}");
      } else {
        request->send(200, "application/json", "{\"message\":\"No parameters provided or no changes made\"}");
      } });

    // Start server
    if (WiFi.status() == WL_CONNECTED)
    { // Only start server if WiFi is connected
        server.begin();
        Serial.println(getFormattedTime() + ": HTTP server started. Access at http://" + WiFi.localIP().toString() + "/");
    }
    else
    {
        Serial.println(getFormattedTime() + ": HTTP server not started due to WiFi connection failure.");
    }

    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);

    pinMode(switchPin, INPUT_PULLUP);             // switchPin is Door Limit Switch
    lastDoorSwitchState = digitalRead(switchPin); // Read initial door state
    doorSwitchState = lastDoorSwitchState;
    doorCurrentlyOpen = (doorSwitchState == LOW); // LOW = Open for NC switch (door open -> switch not actuated -> NC is closed -> LOW)
    if (doorCurrentlyOpen)
    {
        doorOpenStartTime = millis(); // If starts open, note the time
    }

    pinMode(pirPin, INPUT);
    pinMode(rele1Pin, OUTPUT);
    digitalWrite(rele1Pin, RELE_OFF);

    Serial.println(getFormattedTime() + ": Initializing PN532...");
    Wire.begin();
    nfc.begin();
    uint32_t versiondata = nfc.getFirmwareVersion();
    if (!versiondata)
    {
        Serial.println(getFormattedTime() + ": Didn't find PN532 board. Halting system.");
        while (1)
        { /* Halt */
        }
    }
    Serial.print(getFormattedTime() + ": Found chip PN532. Firmware ver. ");
    Serial.print((versiondata >> 16) & 0xFF, DEC);
    Serial.print('.');
    Serial.println((versiondata >> 8) & 0xFF, DEC);
    nfc.SAMConfig();
    Serial.println(getFormattedTime() + ": PN532 Initialized.");

    Serial.println(getFormattedTime() + ": Starting PIR sensor calibration (" + String(PIR_CALIBRATION_DURATION_MS / 1000) + " seconds)...");
    systemState = CALIBRATING_PIR;
    calibrationStartTime = millis();
    lastCalibrationPrintTime = millis(); // To print the first dot (or start interval)
    calibrationDotsPrinted = 0;

    currentRoomState = ROOM_VACANT;        // Initialize room state
    currentSystemSubState = SUBSTATE_IDLE; // Initialize sub-state

    Serial.println(getFormattedTime() + ": System initialized. Room: VACANT. Door: " + (doorCurrentlyOpen ? "OPEN" : "CLOSED"));
}

// --- loop() ---
void loop()
{
    unsigned long currentTime = millis();

    // Always handle buzzer state
    handleBuzzer();

    // State: Calibrating PIR
    if (systemState == CALIBRATING_PIR)
    {
        if (currentTime - calibrationStartTime < PIR_CALIBRATION_DURATION_MS)
        {
            if (currentTime - lastCalibrationPrintTime >= CALIBRATION_PRINT_INTERVAL_MS)
            {
                if (calibrationDotsPrinted < MAX_CALIBRATION_DOTS)
                {
                    Serial.print(".");
                    calibrationDotsPrinted++;
                }
                lastCalibrationPrintTime = currentTime;
            }
        }
        else
        {
            // Ensure all dots are printed if iterations were missed
            for (int i = calibrationDotsPrinted; i < MAX_CALIBRATION_DOTS; i++)
            {
                Serial.print(".");
            }
            Serial.println("\n" + getFormattedTime() + ": PIR Sensor calibrated and ready.");
            lastPirState = digitalRead(pirPin);
            systemState = RUNNING;
            // pirCooldownEnd = millis() + PIR_COOLDOWN_MS; // Set initial cooldown // Replaced
            pirCooldownTimer.start(); // Start initial PIR cooldown
            Serial.println(getFormattedTime() + ": System Ready.");
            Serial.println(getFormattedTime() + ": Room is currently: " + (currentRoomState == ROOM_VACANT ? "VACANT" : "OCCUPIED"));
            Serial.println(getFormattedTime() + ": Door is currently: " + (doorCurrentlyOpen ? "OPEN" : "CLOSED"));
        }
        return; // Don't run the rest of the loop during calibration
    }

    // --- Main Operations (systemState == RUNNING) ---

    // 1. Handle Alarm Timeout
    if (alarmActive && currentTime >= alarmStopTime)
    {
        stopAlarm();
    }

    // 2. Handle Relay Access Timeout (for NFC granted access)
    if (!alarmActive && relayAccessGrantedUntil > 0 && currentTime >= relayAccessGrantedUntil)
    {
        digitalWrite(rele1Pin, RELE_OFF);
        relayAccessGrantedUntil = 0;
        Serial.println(getFormattedTime() + ": Relay access period ended (lock re-engaged).");
    }

    // 3. Door State Management (Limit Switch: switchPin)
    // switchPin: LOW = Door Closed, HIGH = Door Open (due to INPUT_PULLUP)
    int currentDoorReading = digitalRead(switchPin);
    currentTime = millis(); // Ensure currentTime is up-to-date for debouncing logic

    if (currentDoorReading != lastDoorSwitchState)
    {
        lastDoorSwitchChangeTime = currentTime; // Record time of potential change
    }

    if ((currentTime - lastDoorSwitchChangeTime) > DOOR_DEBOUNCE_DELAY_MS)
    {
        // If enough time has passed since the last potential change, and the reading is stable
        if (currentDoorReading != doorSwitchState)
        {                                         // Actual confirmed state change
            doorSwitchState = currentDoorReading; // Update the stable door state

            if (doorSwitchState == LOW)
            { // Door just opened (confirmed, NC switch: LOW means open)
                doorCurrentlyOpen = true;
                doorOpenStartTime = currentTime; // Use current time of confirmed change
                Serial.println(getFormattedTime() + ": Door OPENED (debounced).");

                // Check for unauthorized door opening
                bool isExpectedOpening = (currentSystemSubState == SUBSTATE_AWAITING_DOOR_OPEN_FOR_ENTRY ||
                                          currentSystemSubState == SUBSTATE_AWAITING_DOOR_OPEN_FOR_EXIT);

                if (!isExpectedOpening && (currentTime - lastValidRfidTime > RFID_AUTHORIZATION_WINDOW_MS) && !alarmActive)
                {
                    Serial.println(getFormattedTime() + ": UNAUTHORIZED door opening detected!");
                    startAlarm();
                }
                else
                {
                    if (currentSystemSubState == SUBSTATE_AWAITING_DOOR_OPEN_FOR_ENTRY)
                    {
                        currentSystemSubState = SUBSTATE_AWAITING_DOOR_CLOSE_AFTER_ENTRY;
                        Serial.println(getFormattedTime() + ": Door opened for entry. Waiting for door to close.");
                    }
                    else if (currentSystemSubState == SUBSTATE_AWAITING_DOOR_OPEN_FOR_EXIT)
                    {
                        currentSystemSubState = SUBSTATE_AWAITING_DOOR_CLOSE_AFTER_EXIT;
                        Serial.println(getFormattedTime() + ": Door opened for exit. Waiting for door to close.");
                    }
                }
            }
            else
            { // Door just closed (doorSwitchState == HIGH) (confirmed, NC switch: HIGH means closed)
                doorCurrentlyOpen = false;
                Serial.println(getFormattedTime() + ": Door CLOSED (debounced).");

                if (prolongedDoorAlertActive)
                {
                    prolongedDoorAlertActive = false;
                    if (currentBeepPattern == BEEP_PROLONGED_DOOR)
                    {
                        currentBeepPattern = BEEP_NONE;
                        digitalWrite(buzzerPin, LOW);
                    }
                    Serial.println(getFormattedTime() + ": Prolonged door alert stopped (door closed).");
                }

                if (currentSystemSubState == SUBSTATE_AWAITING_DOOR_CLOSE_AFTER_ENTRY)
                {
                    currentSystemSubState = SUBSTATE_AWAITING_PIR_CONFIRMATION_FOR_ENTRY;
                    entryPIRWaitStartTime = currentTime; // Use current time of confirmed change
                    Serial.println(getFormattedTime() + ": Door closed after entry. Waiting for PIR for " + String(ENTRY_PIR_WAIT_TIMEOUT_MS / 1000) + "s.");
                }
                else if (currentSystemSubState == SUBSTATE_AWAITING_DOOR_CLOSE_AFTER_EXIT)
                {
                    currentSystemSubState = SUBSTATE_AWAITING_NO_PIR_CONFIRMATION_FOR_EXIT;
                    exitNoPIRWaitStartTime = currentTime; // Use current time of confirmed change
                    motionDetectedDuringExitWait = false;
                    Serial.println(getFormattedTime() + ": Door closed after exit. Waiting " + String(EXIT_NO_PIR_WAIT_DURATION_MS / 1000) + "s to confirm no motion.");
                }
                else if (currentSystemSubState != SUBSTATE_IDLE)
                {
                    // Consider if resetting to IDLE is always appropriate here
                }
            }
        }
    }
    lastDoorSwitchState = currentDoorReading; // Always update last reading for next iteration

    // 4. Handle Entry/Exit Sub-State Logic & Timeouts
    currentTime = millis(); // Ensure currentTime is fresh
    switch (currentSystemSubState)
    {
    case SUBSTATE_AWAITING_DOOR_OPEN_FOR_ENTRY:
    case SUBSTATE_AWAITING_DOOR_OPEN_FOR_EXIT:
        if (currentTime - lastValidRfidTime > AWAIT_DOOR_OPEN_TIMEOUT_MS)
        {
            Serial.println(getFormattedTime() + ": Timeout waiting for door to open after RFID. Resetting sub-state.");
            currentSystemSubState = SUBSTATE_IDLE;
            lastValidRfidTime = 0; // Consume the RFID event as it wasn't used for door opening
        }
        break;

    case SUBSTATE_AWAITING_PIR_CONFIRMATION_FOR_ENTRY:
        // PIR check itself is handled in PIR section. This is for timeout.
        if (currentTime - entryPIRWaitStartTime > ENTRY_PIR_WAIT_TIMEOUT_MS)
        {
            Serial.println(getFormattedTime() + ": No PIR motion detected within " + String(ENTRY_PIR_WAIT_TIMEOUT_MS / 1000) + "s for entry. Assuming no one entered.");
            currentSystemSubState = SUBSTATE_IDLE; // Room remains VACANT
            lastValidRfidTime = 0;                 // Consume RFID
        }
        break;

    case SUBSTATE_AWAITING_NO_PIR_CONFIRMATION_FOR_EXIT:
        // PIR check updates motionDetectedDuringExitWait. This is for timeout.
        if (currentTime - exitNoPIRWaitStartTime > EXIT_NO_PIR_WAIT_DURATION_MS)
        {
            if (!motionDetectedDuringExitWait)
            {
                Serial.println(getFormattedTime() + ": No PIR motion during exit wait. Room is now VACANT.");
                currentRoomState = ROOM_VACANT;
            }
            else
            {
                Serial.println(getFormattedTime() + ": PIR motion detected during exit wait. Assuming person did not exit. Room remains OCCUPIED.");
            }
            currentSystemSubState = SUBSTATE_IDLE;
            lastValidRfidTime = 0; // Consume RFID
            // lastValidRfidDisplayTime = "N/A"; // MODIFICATION: Removed to keep display time
        }
        break;
    default: // SUBSTATE_IDLE or other transient states
        break;
    }

    // 5. Read PIR Sensor
    if (pirCooldownTimer.update()) // Check if PIR cooldown has elapsed
    {
        int currentPirState = digitalRead(pirPin);
        if (currentPirState == HIGH && lastPirState == LOW)
        { // Motion detected (rising edge)
            Serial.println(getFormattedTime() + ": Motion detected by PIR!");
            // pirCooldownEnd = currentTime + PIR_COOLDOWN_MS; // Replaced
            pirCooldownTimer.start(); // Restart PIR cooldown

            if (currentSystemSubState == SUBSTATE_AWAITING_PIR_CONFIRMATION_FOR_ENTRY)
            {
                Serial.println(getFormattedTime() + ": PIR motion confirms entry. Room is now OCCUPIED.");
                currentRoomState = ROOM_OCCUPIED;
                currentSystemSubState = SUBSTATE_IDLE;
                lastValidRfidTime = 0; // Consume RFID
                // lastValidRfidDisplayTime = "N/A"; // MODIFICATION: Removed to keep display time
            }
            else if (currentSystemSubState == SUBSTATE_AWAITING_NO_PIR_CONFIRMATION_FOR_EXIT)
            {
                if (currentTime >= exitNoPIRWaitStartTime + EXIT_PIR_ACTIVATION_DELAY_MS)
                {
                    Serial.println(getFormattedTime() + ": PIR motion detected during active exit wait period.");
                    motionDetectedDuringExitWait = true;
                }
                else
                {
                    Serial.println(getFormattedTime() + ": PIR motion detected during initial exit delay. Ignoring for exit confirmation.");
                }
            }
            else if (currentRoomState == ROOM_VACANT && currentSystemSubState == SUBSTATE_IDLE && !alarmActive)
            {
                Serial.println(getFormattedTime() + ": Motion detected in VACANT room (unexpected)! Activating alarm.");
                startAlarm();
            }
            else if (!alarmActive)
            {
                // Log other motion if needed, but don't act on it if not in specific states
                // Serial.println(getFormattedTime() + ": Motion ignored (State: " + String(currentRoomState) + ", SubState: " + String(currentSystemSubState) + ")");
            }
        }
        lastPirState = currentPirState;
    }

    // 6. Read NFC
    currentTime = millis();
    if (currentTime >= nextNfcCheckTime && currentTime >= nfcReadCooldownEnd)
    {
        uint8_t successNFC = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50);

        if (successNFC)
        {
            String uidString = "";
            for (uint8_t i = 0; i < uidLength; i++)
            {
                if (uid[i] < 0x10)
                    uidString += "0";
                uidString += String(uid[i], HEX);
                if (i < uidLength - 1)
                    uidString += " ";
            }
            uidString.toUpperCase();
            Serial.println(getFormattedTime() + ": NFC Card detected. UID Length: " + String(uidLength) + " bytes. UID Value: " + uidString);

            bool cardIsValid = false;
            if (uidLength == VALID_UID_LENGTH)
            {
                bool match1 = true;
                for (uint8_t i = 0; i < VALID_UID_LENGTH; i++)
                {
                    if (uid[i] != validUid1[i])
                    {
                        match1 = false;
                        break;
                    }
                }
                if (match1)
                    cardIsValid = true;
                else
                {
                    bool match2 = true;
                    for (uint8_t i = 0; i < VALID_UID_LENGTH; i++)
                    {
                        if (uid[i] != validUid2[i])
                        {
                            match2 = false;
                            break;
                        }
                    }
                    if (match2)
                        cardIsValid = true;
                }
            }

            if (cardIsValid)
            {
                Serial.println(getFormattedTime() + ": Valid NFC card.");
                grantAccessByNFC(); // This now sets sub-state and handles alarm
            }
            else
            {
                Serial.println(getFormattedTime() + ": Invalid NFC card.");
                startErrorBeep(); // 2 short beeps
            }
            nfcReadCooldownEnd = currentTime + NFC_PROCESS_COOLDOWN_MS;
        }
        nextNfcCheckTime = currentTime + NFC_CHECK_INTERVAL_MS;
    }

    // 7. Prolonged Door Open Alert
    currentTime = millis();
    // Check if door was opened legitimately (within RFID window or part of an ongoing RFID-initiated process)
    bool legitimateOpeningContext = (currentTime - lastValidRfidTime < RFID_AUTHORIZATION_WINDOW_MS) ||
                                    currentSystemSubState == SUBSTATE_AWAITING_DOOR_CLOSE_AFTER_ENTRY ||
                                    currentSystemSubState == SUBSTATE_AWAITING_DOOR_CLOSE_AFTER_EXIT;

    if (doorCurrentlyOpen && legitimateOpeningContext &&
        (currentTime - doorOpenStartTime > PROLONGED_DOOR_OPEN_ALERT_THRESHOLD_MS) &&
        !alarmActive && !prolongedDoorAlertActive)
    {

        Serial.println(getFormattedTime() + ": Door has been open for too long. Starting alert.");
        if (currentBeepPattern == BEEP_NONE && !alarmActive)
        {
            currentBeepPattern = BEEP_PROLONGED_DOOR;
            beepSequenceStep = 0;
            nextBeepActionTime = millis();
            prolongedDoorAlertActive = true;
        }
    }

    // If door is closed OR alarm is active OR legitimate context ended, and alert was on, ensure it's turned off.
    if ((!doorCurrentlyOpen || alarmActive || !legitimateOpeningContext) && prolongedDoorAlertActive)
    {
        prolongedDoorAlertActive = false;
        if (currentBeepPattern == BEEP_PROLONGED_DOOR)
        {
            currentBeepPattern = BEEP_NONE;
            digitalWrite(buzzerPin, LOW);
        }
    }
}
