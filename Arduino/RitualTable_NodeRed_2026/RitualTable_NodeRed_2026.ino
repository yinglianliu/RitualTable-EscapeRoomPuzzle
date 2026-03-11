/*
 * =======================================================================================
 * Project:    Ritual Table v2 - Interactive RFID Controller
 * Author:     Yinglian Liu
 * Date:       Feb 2026
 * =======================================================================================
 * Description:
 * A bi-directional IoT controller designed for an interactive puzzle.
 * It monitors five RFID readers and synchronizes game states with a central server
 * via MQTT (Node-RED). It also supports STANDALONE mode, allowing local operation
 * without Wi-Fi or MQTT.
 *
 * Integrated Ecosystem:
 * This device acts as a sensor node within the control network:
 * - Node-RED: Central logic and MQTT broker
 * - QLab:     Audio playback (triggered via Node-RED)
 * - QLC+:     DMX lighting control (triggered via Node-RED)
 *
 * Key Features:
 * 1. Input: Monitors 5x MFRC522 RFID readers sequentially.
 * 2. Optimization: Uses state-change detection to minimize network traffic
 *    and prevent MQTT flooding.
 * 3. Administrative Control: Allows operators to remotely control the puzzle
 *    through the Node-RED dashboard:
 *    - Force Win: Immediately triggers the solved state
 *    - Remote Reset: Resets the puzzle to its initial state
 * 4. Standalone Mode: Continues operating locally even if the network fails.
 *
 * MQTT Architecture:
 * - Publishes to:   "demo/ritualtable/status"   (sensor state data)
 * - Subscribes to:  "demo/ritualtable/control"  (admin commands: RESET, FORCE WIN)
 * =======================================================================================
 */

#include <SPI.h>
#include <MFRC522.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>

// ================== Network Settings ==================
char ssid[] = "YING";         // Wi-Fi SSID
char pass[] = "][p709394";    // Wi-Fi password

// IP address of the computer running Node-RED
const char broker[] = "192.168.137.1";
int port = 1883;

// MQTT topics
const char topicStatus[]  = "demo/ritualtable/status";
const char topicControl[] = "demo/ritualtable/control";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const uint8_t Num_Readers = 5;
const uint8_t ssPins[Num_Readers] = {9, 8, 7, 6, 5};
const uint8_t rstPin = 10;
MFRC522 rfid[Num_Readers];

// Temporary test UIDs; replace these with the actual ritual item tags
byte targetUIDs[Num_Readers][7] = {
  {0x04, 0xA5, 0xAA, 0xC5, 0x79, 0x00, 0x00},  // Trophy
  {0x04, 0xA7, 0xAA, 0xC5, 0x79, 0x00, 0x00},  // Sheet Music
  {0x04, 0xA6, 0xAA, 0xC5, 0x79, 0x00, 0x00},  // Amulet
  {0x04, 0xBE, 0xAA, 0xC5, 0x79, 0x00, 0x00},  // Journal
  {0x04, 0xA3, 0xAA, 0xC5, 0x79, 0x00, 0x00},  // Ring Box
};

byte targetUIDLengths[Num_Readers] = {7, 7, 7, 7, 7};

byte resetUID[7] = {0xF3, 0x9A, 0x2B, 0xAB, 0x00, 0x00, 0x00};
const byte resetUIDLength = 4;

const uint8_t ledPins[] = {4, 3, 2, 19, 20};
const uint8_t relayPins[Num_Readers] = {A0, A1, A2, A3, A4};

bool cardDetected[] = {false, false, false, false, false};
bool puzzleSolved = false;
long lastReconnectAttempt = 0;

void setup() {
  Serial.begin(115200);

  for (uint8_t i = 0; i < Num_Readers; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);

    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LOW);

    pinMode(ssPins[i], OUTPUT);
    digitalWrite(ssPins[i], HIGH);
  }

  SPI.begin();

  for (uint8_t reader = 0; reader < Num_Readers; reader++) {
    rfid[reader].PCD_Init(ssPins[reader], rstPin);
    delay(4);
  }

  tryConnectNetwork();
}

void loop() {
  if (!mqttClient.connected()) {
    long now = millis();
    if (now - lastReconnectAttempt > 5000) {
      lastReconnectAttempt = now;
      tryConnectNetwork();
    }
  } else {
    mqttClient.poll();
  }

  bool allCorrectCardsDetected = true;

  // Core improvement:
  // Use a stateChanged flag so MQTT is only published once per loop cycle
  bool stateChanged = false;

  for (uint8_t reader = 0; reader < Num_Readers; reader++) {
    // Activate only the current reader
    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(ssPins[i], HIGH);
    }
    digitalWrite(ssPins[reader], LOW);
    delay(10);

    if (rfid[reader].PICC_IsNewCardPresent() && rfid[reader].PICC_ReadCardSerial()) {
      byte* uid = rfid[reader].uid.uidByte;
      byte uidSize = rfid[reader].uid.size;

      if (compareUID(uid, uidSize, targetUIDs[reader], targetUIDLengths[reader])) {
        // Correct card detected
        if (!cardDetected[reader]) {
          cardDetected[reader] = true;
          stateChanged = true; // Mark change, but do not publish yet
        }

        digitalWrite(ledPins[reader], HIGH);
        digitalWrite(relayPins[reader], HIGH);
      }
      else if (compareUID(uid, uidSize, resetUID, resetUIDLength)) {
        // Reset card detected
        resetToInitialState();
        return; // Exit immediately and begin a new loop cycle
      }
      else {
        // Wrong card detected -> trigger candle flash effect
        flashAllCandles(3, 100);

        if (cardDetected[reader]) {
          cardDetected[reader] = false;
          stateChanged = true; // Mark change, but do not publish yet
        }
      }

      rfid[reader].PICC_HaltA();
      rfid[reader].PCD_StopCrypto1();
    }

    // Deactivate the current reader
    digitalWrite(ssPins[reader], HIGH);

    if (cardDetected[reader]) {
      digitalWrite(ledPins[reader], HIGH);
      digitalWrite(relayPins[reader], HIGH);
    } else {
      digitalWrite(ledPins[reader], LOW);
      digitalWrite(relayPins[reader], LOW);
      allCorrectCardsDetected = false;
    }
  }

  // ================= Centralized State Resolution =================

  if (!allCorrectCardsDetected && puzzleSolved) {
    puzzleSolved = false;
    stateChanged = true;
  }

  if (allCorrectCardsDetected && !puzzleSolved) {
    puzzleSolved = true;
    stateChanged = true;
  }

  // Publish only once after the full loop finishes
  if (stateChanged) {
    sendMqttState();
  }

  // Run the win animation only after the network update is sent
  if (allCorrectCardsDetected && stateChanged) {
    playWinAnimation();
  }
}

// ================== Network / MQTT ==================

void tryConnectNetwork() {
  Serial.print("Checking Wi-Fi...");

  if (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    delay(1000);
  }

  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    Serial.print(" Wi-Fi connected. Connecting to MQTT... ");

    if (mqttClient.connect(broker, port)) {
      Serial.println("MQTT connected.");
      mqttClient.onMessage(onMqttMessage);
      mqttClient.subscribe(topicControl);

      delay(1500);
      mqttClient.poll();

      sendMqttState();
      Serial.println("Initial state update sent to Node-RED.");
    } else {
      Serial.print("MQTT connection failed: ");
      Serial.println(mqttClient.connectError());
    }
  }
  else if (WiFi.status() != WL_CONNECTED) {
    Serial.println(" Wi-Fi connection failed. Running in standalone mode.");
  }
}

void onMqttMessage(int messageSize) {
  String message = "";
  while (mqttClient.available()) {
    message += (char)mqttClient.read();
  }

  if (message == "R" || message == "r") {
    resetToInitialState();
  }
  else if (message == "F" || message == "f") {
    forceSolve();
  }
}

void sendMqttState() {
  if (!mqttClient.connected()) {
    return;
  }

  mqttClient.beginMessage(topicStatus);
  mqttClient.print("{\"data\":[");

  for (uint8_t i = 0; i < Num_Readers; i++) {
    mqttClient.print(cardDetected[i]);
    if (i < Num_Readers - 1) mqttClient.print(",");
  }

  mqttClient.print("],\"solved\":");
  mqttClient.print(puzzleSolved ? "true" : "false");
  mqttClient.print("}");
  mqttClient.endMessage();
}

void forceSolve() {
  for (uint8_t i = 0; i < Num_Readers; i++) {
    cardDetected[i] = true;
    digitalWrite(ledPins[i], HIGH);
    digitalWrite(relayPins[i], HIGH);
  }

  puzzleSolved = true;
  sendMqttState();
  playWinAnimation();
}

void resetToInitialState() {
  for (uint8_t i = 0; i < Num_Readers; i++) {
    cardDetected[i] = false;
    digitalWrite(ledPins[i], LOW);
    digitalWrite(relayPins[i], LOW);
  }

  puzzleSolved = false;
  sendMqttState();
  delay(1000);
}

void playWinAnimation() {
  delay(2000);

  for (uint8_t repeat = 0; repeat < 3; repeat++) {
    if (mqttClient.connected()) mqttClient.poll(); // Prevent disconnects during animation

    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(relayPins[i], HIGH);
      digitalWrite(ledPins[i], HIGH);
      delay(50);

      digitalWrite(relayPins[i], LOW);
      digitalWrite(ledPins[i], LOW);
      delay(50);
    }

    for (int i = Num_Readers - 1; i >= 0; i--) {
      digitalWrite(relayPins[i], HIGH);
      digitalWrite(ledPins[i], HIGH);
      delay(50);

      digitalWrite(relayPins[i], LOW);
      digitalWrite(ledPins[i], LOW);
      delay(50);
    }
  }

  for (uint8_t i = 0; i < Num_Readers; i++) {
    digitalWrite(relayPins[i], HIGH);
    digitalWrite(ledPins[i], HIGH);
  }
}

bool compareUID(byte* uid1, byte uid1Size, byte* uid2, byte uid2Size) {
  if (uid1Size != uid2Size) return false;

  for (byte i = 0; i < uid1Size; i++) {
    if (uid1[i] != uid2[i]) return false;
  }

  return true;
}

void flashAllCandles(uint8_t times, unsigned long duration) {
  for (uint8_t t = 0; t < times; t++) {
    if (mqttClient.connected()) mqttClient.poll(); // Prevent disconnects during animation

    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(ledPins[i], HIGH);
      digitalWrite(relayPins[i], HIGH);
    }

    delay(duration);

    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(ledPins[i], LOW);
      digitalWrite(relayPins[i], LOW);
    }

    delay(duration);
  }
}