/*
 * =======================================================================================
 * Project:    Ritual Table v4 - Interactive RFID Controller (Pure USB Serial + Raspberry Pi 5)
 * Author:     Yinglian Liu
 * Date:       March 2026
 * =======================================================================================
 * Description:
 * This version uses pure USB serial communication between the Arduino and Raspberry Pi 5.
 * The Arduino acts as the real-time hardware controller:
 * - Scans 5 RFID readers
 * - Drives local LEDs and relays for physical feedback
 * - Sends puzzle state updates to Node-RED as JSON over Serial
 * - Receives admin commands from Node-RED over Serial
 *
 * System Architecture:
 * - Arduino: Real-time hardware layer for sensing and physical output
 * - Raspberry Pi 5 / Node-RED: Central orchestration, dashboard, timing logic,
 *   and media / lighting control
 *
 * Key Features:
 * 1. Sequentially scans 5 MFRC522 RFID readers.
 * 2. Uses state-change detection to avoid unnecessary serial traffic.
 * 3. Sends a dedicated "wrong" flag immediately when an incorrect item is detected.
 * 4. Supports admin commands over Serial:
 *    - R = Reset puzzle
 *    - F = Force solved state
 * 5. Maintains local fallback behavior for physical feedback and win animation.
 *
 * Serial Message Format:
 * {
 *   "data":[1,0,1,0,0],
 *   "solved":false,
 *   "wrong":false
 * }
 * 6. Add arduino Heartbeat to node-red, it can be monitoring by node-red dashboard
 * =======================================================================================
 */

#include <SPI.h>
#include <MFRC522.h>

const uint8_t Num_Readers = 5;
const uint8_t ssPins[Num_Readers] = {9, 8, 7, 6, 5};
const uint8_t rstPin = 10;
MFRC522 rfid[Num_Readers];

// Test UIDs used during Node-RED integration
// Replace these with the actual ritual item tags for the final installation
byte targetUIDs[Num_Readers][7] = {
  {0x04, 0xA5, 0xAA, 0xC5, 0x79, 0x00, 0x00},  // Trophy
  {0x04, 0xA7, 0xAA, 0xC5, 0x79, 0x00, 0x00},  // Sheet Music
  {0x04, 0xA6, 0xAA, 0xC5, 0x79, 0x00, 0x00},  // Amulet
  {0x04, 0xBE, 0xAA, 0xC5, 0x79, 0x00, 0x00},  // Journal
  {0x04, 0xA3, 0xAA, 0xC5, 0x79, 0x00, 0x00},  // Ring Box
};
byte targetUIDLengths[Num_Readers] = {7, 7, 7, 7, 7};

// GM / admin reset card UID
byte resetUID[7] = {0xF3, 0x9A, 0x2B, 0xAB, 0x00, 0x00, 0x00};
const byte resetUIDLength = 4;

const uint8_t ledPins[] = {4, 3, 2, 19, 20};
const uint8_t relayPins[Num_Readers] = {A0, A1, A2, A3, A4};

bool cardDetected[] = {false, false, false, false, false};
bool puzzleSolved = false;

void sendSerialState(bool isWrong = false);

unsigned long lastHeartbeatTime = 0;
const unsigned long HEARTBEAT_INTERVAL = 2000;

void setup() {
  // Run Serial at 115200 baud to ensure fast and stable communication
  // with the Raspberry Pi 5
  Serial.begin(115200);

  for (uint8_t i = 0; i < Num_Readers; i++) {
    pinMode(relayPins[i], OUTPUT); digitalWrite(relayPins[i], LOW);
    pinMode(ledPins[i], OUTPUT);   digitalWrite(ledPins[i], LOW);
    pinMode(ssPins[i], OUTPUT);    digitalWrite(ssPins[i], HIGH);
  }

  SPI.begin();
  for (uint8_t reader = 0; reader < Num_Readers; reader++) {
    rfid[reader].PCD_Init(ssPins[reader], rstPin);
    delay(4);
  }

  // Force-send the initial state to Node-RED on startup
  sendSerialState();
}

void loop() {
  // 1. Always listen for commands coming from Node-RED
  //    This replaces the old MQTT subscription model
  checkSerialCommands();

  bool allCorrectCardsDetected = true;
  bool stateChanged = false; // Tracks whether any state changed during this loop cycle

  // 2. Poll the RFID readers one by one
  for (uint8_t reader = 0; reader < Num_Readers; reader++) {
    // Activate the current reader only
    for (uint8_t i = 0; i < Num_Readers; i++) digitalWrite(ssPins[i], HIGH);
    digitalWrite(ssPins[reader], LOW);
    delay(10);

    if (rfid[reader].PICC_IsNewCardPresent() && rfid[reader].PICC_ReadCardSerial()) {
      byte* uid = rfid[reader].uid.uidByte;
      byte uidSize = rfid[reader].uid.size;

      if (compareUID(uid, uidSize, targetUIDs[reader], targetUIDLengths[reader])) {
        // Correct card
        if (!cardDetected[reader]) {
          cardDetected[reader] = true;
          stateChanged = true;
        }
        digitalWrite(ledPins[reader], HIGH);
        digitalWrite(relayPins[reader], HIGH);
      }
      else if (compareUID(uid, uidSize, resetUID, resetUIDLength)) {
        // Reset card
        resetToInitialState();
        return; // Exit the current loop cycle immediately and restart cleanly
      }
      else {
        // Wrong card -> trigger the error flash animation

        // Important fix:
        // report the wrong placement to Node-RED immediately before flashing
        sendSerialState(true);

        flashAllCandles(3, 100);

        if (cardDetected[reader]) {
          cardDetected[reader] = false;
          stateChanged = true;
        }
      }

      rfid[reader].PICC_HaltA();
      rfid[reader].PCD_StopCrypto1();
    }

    // Deactivate the current reader
    digitalWrite(ssPins[reader], HIGH);

    // State resolution for this reader
    if (cardDetected[reader]) {
      digitalWrite(ledPins[reader], HIGH);
      digitalWrite(relayPins[reader], HIGH);
    } else {
      digitalWrite(ledPins[reader], LOW);
      digitalWrite(relayPins[reader], LOW);
      allCorrectCardsDetected = false;
    }
  }

  // ================= Centralized resolution stage =================

  if (!allCorrectCardsDetected && puzzleSolved) {
    puzzleSolved = false;
    stateChanged = true;
  }

  if (allCorrectCardsDetected && !puzzleSolved) {
    puzzleSolved = true;
    stateChanged = true;
  }

  // Most important step:
  // only send a clean JSON update over USB Serial if the state actually changed
  if (stateChanged) {
    sendSerialState();
  }

  // After sending data, run the blocking physical win animation if needed
  if (allCorrectCardsDetected && stateChanged) {
    playWinAnimation();
  }

  // ================= Heartbeat =================
  if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    sendSerialState(false); 
    lastHeartbeatTime = millis();
  }
}

// ================== Core functions ==================

// Replaces the old MQTT publish behavior
// Build the state packet as JSON and print it to Serial
// The parameter defaults to false, meaning "no wrong placement"
void sendSerialState(bool isWrong) {
  if(!Serial) return; //Prevents game cannot proceed due to serial port connection failure.

  Serial.print("{\"data\":[");
  for (uint8_t i = 0; i < Num_Readers; i++) {
    Serial.print(cardDetected[i] ? 1 : 0);
    if (i < Num_Readers - 1) Serial.print(",");
  }
  Serial.print("],\"solved\":");
  Serial.print(puzzleSolved ? "true" : "false");

  // Include the dedicated wrong-placement flag
  Serial.print(",\"wrong\":");
  Serial.print(isWrong ? "true" : "false");

  Serial.println("}");
}

// Replaces the old MQTT message handler
// Listen for single-character commands over USB Serial
void checkSerialCommands() {
  if (Serial.available() > 0) {
    char incomingChar = Serial.read();
    if (incomingChar == 'R' || incomingChar == 'r') {
      resetToInitialState();
    } else if (incomingChar == 'F' || incomingChar == 'f') {
      forceSolve();
    }
  }
}

void forceSolve() {
  for (uint8_t i = 0; i < Num_Readers; i++) {
    cardDetected[i] = true;
    digitalWrite(ledPins[i], HIGH);
    digitalWrite(relayPins[i], HIGH);
  }
  puzzleSolved = true;
  sendSerialState();
  playWinAnimation();
}

bool smartDelay(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    checkSerialCommands();

    // Fix B: maintain heartbeat during blocking animations so Node-RED
    // does not falsely report Arduino as Offline during playWinAnimation
    if (millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
      sendSerialState(false);
      lastHeartbeatTime = millis();
    }

    if (!puzzleSolved) {
      return false;
    }
  }
  return true;
}

void resetToInitialState() {
  for (uint8_t i = 0; i < Num_Readers; i++) {
    cardDetected[i] = false;
    digitalWrite(ledPins[i], LOW);
    digitalWrite(relayPins[i], LOW);
  }
  puzzleSolved = false;
  sendSerialState();
  //delay(1000);

  // //clear the serial port buffer
  // while (Serial.available() > 0) {
  //   Serial.read(); 
  // }

}

void playWinAnimation() {
  //delay(2000);
  if (!smartDelay(2000)) return;
  for (uint8_t repeat = 0; repeat < 5; repeat++) {
    // Network keepalive logic was removed here because USB serial does not drop like Wi-Fi / MQTT
    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(relayPins[i], HIGH); digitalWrite(ledPins[i], HIGH);
      if (!smartDelay(100)) return;
      //delay(100);
      digitalWrite(relayPins[i], LOW); digitalWrite(ledPins[i], LOW);
      if (!smartDelay(100)) return;
      //delay(100);
    }
    for (int i = Num_Readers - 1; i >= 0; i--) {
      digitalWrite(relayPins[i], HIGH); digitalWrite(ledPins[i], HIGH);
      if (!smartDelay(100)) return;
     // delay(100);
      digitalWrite(relayPins[i], LOW); digitalWrite(ledPins[i], LOW);
      if (!smartDelay(100)) return;
      //delay(100);
    }
  }

  for (uint8_t i = 0; i < Num_Readers; i++) {
    digitalWrite(relayPins[i], HIGH); digitalWrite(ledPins[i], HIGH);
  }

    //   //clear the serial port buffer
    // while (Serial.available() > 0) {
    //   Serial.read(); 
    //}
}

void flashAllCandles(uint8_t times, unsigned long duration) {
  // Fix D: use smartDelay instead of blocking delay() so serial commands
  // (e.g. Reset) remain responsive during the error flash animation
  for (uint8_t t = 0; t < times; t++) {
    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(ledPins[i], HIGH); digitalWrite(relayPins[i], HIGH);
    }
    if (!smartDelay(duration)) return;
    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(ledPins[i], LOW); digitalWrite(relayPins[i], LOW);
    }
    if (!smartDelay(duration)) return;
  }
}

bool compareUID(byte* uid1, byte uid1Size, byte* uid2, byte uid2Size) {
  if (uid1Size != uid2Size) return false;
  for (byte i = 0; i < uid1Size; i++) {
    if (uid1[i] != uid2[i]) return false;
  }
  return true;
}
