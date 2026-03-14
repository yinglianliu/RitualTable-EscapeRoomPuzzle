/*
 * =======================================================================================
 * Project:    Ritual Table v3 - USB Serial Master-Slave Controller
 * Author:     Yinglian Liu
 * Date:       March 2026
 * =======================================================================================
 * Description:
 * This version replaces Wi-Fi / MQTT communication with direct USB serial communication.
 * The Arduino acts as a lower-level hardware controller:
 * - Reads 5 RFID readers
 * - Controls LEDs and relays for physical feedback
 * - Reports puzzle state to Node-RED via JSON over Serial
 * - Receives admin commands from Node-RED via Serial
 *
 * System Architecture:
 * - Arduino: Sensor and actuator controller
 * - Node-RED: Central logic, admin control, and audio / lighting routing
 *
 * Key Features:
 * 1. Sequentially scans 5 MFRC522 RFID readers.
 * 2. Sends JSON state updates over Serial only when state changes occur.
 * 3. Supports startup state synchronization using an "isSync" flag.
 * 4. Accepts admin commands from Node-RED:
 *    - R = Reset puzzle
 *    - F = Force solved state
 * 5. Keeps all game logic centralized in Node-RED while Arduino handles hardware.
 *
 * Serial Message Format:
 * {
 *   "data":[false,false,true,false,false],
 *   "solved":false,
 *   "isSync":true
 * }
 * =======================================================================================
 */

#include <SPI.h>
#include <MFRC522.h>

const uint8_t Num_Readers = 5;
const uint8_t ssPins[Num_Readers] = {9, 8, 7, 6, 5};
const uint8_t rstPin = 10;
MFRC522 rfid[Num_Readers];

// Target card UIDs
// Temporary test UIDs; replace these with the actual ritual item tags
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

// Physical output pins
// A5 usually maps to 19, and A6 usually maps to 20
const uint8_t ledPins[] = {4, 3, 2, 19, 20};
const uint8_t relayPins[Num_Readers] = {A0, A1, A2, A3, A4};

bool cardDetected[] = {false, false, false, false, false};
bool puzzleSolved = false;

void setup() {
  // 1. Initialize Serial at 115200 baud
  //    This ensures JSON messages are transmitted quickly with minimal delay
  Serial.begin(115200);

  for (uint8_t i = 0; i < Num_Readers; i++) {
    pinMode(relayPins[i], OUTPUT); digitalWrite(relayPins[i], LOW);
    pinMode(ledPins[i], OUTPUT);   digitalWrite(ledPins[i], LOW);
    pinMode(ssPins[i], OUTPUT);    digitalWrite(ssPins[i], HIGH);
  }

  SPI.begin();
  for (uint8_t reader = 0; reader < Num_Readers; reader++) {
    rfid[reader].PCD_Init(ssPins[reader], rstPin);
    delay(4); // Give each reader a short startup delay
  }

  // 2. Startup ：
  //    after boot or power recovery, send one full state update with isSync = true
  delay(1000); 
  sendSerialState(true);
}

void loop() {
  // ==========================================
  // 3. Listen for admin commands from Node-RED
  // ==========================================
  if (Serial.available() > 0) {
    char incomingByte = Serial.read();
    if (incomingByte == 'R' || incomingByte == 'r') resetToInitialState();
    else if (incomingByte == 'F' || incomingByte == 'f') forceSolve();
  }

  // ==========================================
  // 4. Core RFID scanning logic
  // ==========================================
  bool allCorrectCardsDetected = true;

  for (uint8_t reader = 0; reader < Num_Readers; reader++) {
    for (uint8_t i = 0; i < Num_Readers; i++) digitalWrite(ssPins[i], HIGH);
    digitalWrite(ssPins[reader], LOW);
    delay(5); // With networking removed, this can be reduced to 5 ms for faster scanning

    if (rfid[reader].PICC_IsNewCardPresent() && rfid[reader].PICC_ReadCardSerial()) {
      byte* uid = rfid[reader].uid.uidByte;
      byte uidSize = rfid[reader].uid.size;

      if (compareUID(uid, uidSize, targetUIDs[reader], targetUIDLengths[reader])) {
        bool wasFalse = !cardDetected[reader];
        cardDetected[reader] = true;
        digitalWrite(ledPins[reader], HIGH);
        digitalWrite(relayPins[reader], HIGH);

        // Send JSON only when the state changes from "not detected" to "detected"
        if (wasFalse) sendSerialState(false);
      }
      else if (compareUID(uid, uidSize, resetUID, resetUIDLength)) {
        resetToInitialState();
        return;
      }
      else {
        flashAllCandles(3, 100);
        bool wasTrue = cardDetected[reader];
        cardDetected[reader] = false;

        // Send JSON only when the state changes from "detected" to "not detected"
        if (wasTrue) sendSerialState(false);
      }

      rfid[reader].PICC_HaltA();
      rfid[reader].PCD_StopCrypto1();
    }

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

  if (!allCorrectCardsDetected && puzzleSolved) {
    puzzleSolved = false;
    sendSerialState(false);
  }

  if (allCorrectCardsDetected && !puzzleSolved) {
    puzzleSolved = true;
    sendSerialState(false);
    playWinAnimation();
  }
}

// ==========================================
// 5. Core state transmission function:
//    package the current puzzle state as a JSON message
// ==========================================
void sendSerialState(bool isSync) {
  Serial.print("{\"data\":[");
  for (uint8_t i = 0; i < Num_Readers; i++) {
    Serial.print(cardDetected[i]);
    if (i < Num_Readers - 1) Serial.print(",");
  }
  Serial.print("],\"solved\":");
  Serial.print(puzzleSolved ? "true" : "false");
  Serial.print(",\"isSync\":");
  Serial.print(isSync ? "true" : "false");

  // Critical:
  // use println() so Node-RED can split packets by the trailing newline
  Serial.println("}");
}

void forceSolve() {
  for (uint8_t i = 0; i < Num_Readers; i++) {
    cardDetected[i] = true;
    digitalWrite(ledPins[i], HIGH);
    digitalWrite(relayPins[i], HIGH);
  }
  puzzleSolved = true;
  sendSerialState(false);
  playWinAnimation();
}

void resetToInitialState() {
  for (uint8_t i = 0; i < Num_Readers; i++) {
    cardDetected[i] = false;
    digitalWrite(ledPins[i], LOW);
    digitalWrite(relayPins[i], LOW);
  }
  puzzleSolved = false;
  sendSerialState(false);
  delay(1000);
}

void playWinAnimation() {
  delay(2000);
  for (uint8_t repeat = 0; repeat < 3; repeat++) {
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
    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(ledPins[i], HIGH); digitalWrite(relayPins[i], HIGH);
    }
    delay(duration);
    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(ledPins[i], LOW); digitalWrite(relayPins[i], LOW);
    }
    delay(duration);
  }
}