/*
 * =======================================================================================
 * Project:    Ritual Table v2 - Interactive RFID Controller
 * Author:     Yinglian Liu
 * Date:       January 2026
 * =======================================================================================
 * Description:
 * A bi-directional IoT controller designed for an interactive puzzle. 
 * It monitors 5 RFID readers and synchronizes game states with a central server 
 * via MQTT (Node-RED).
 *
 * Integrated Ecosystem:
 * This device acts as the sensor node within a Control network:
 * - Node-RED: Central logic & MQTT Broker.
 * - QLab:     Audio playback (triggered via Node-RED).
 * - QLC+:     DMX Lighting control (triggered via Node-RED).
 *
 * Key Features:
 * 1. Input: Monitors 5x MFRC522 RFID readers sequentially.
 * 2. Optimization: Implements "State Change Detection"
 * to minimize network traffic and prevent UDP/MQTT flooding.
 * 3. Administrative Control: Allows admins to remotely manipulate the game 
 * via Node-RED Dashboard:
 * - Force Win: Instantly triggers the "Win" state (Lighting/Audio cues).
 * - Remote Reset: Resets the puzzle.
 *
 * MQTT Architecture:
 * - Publishes to: "demo/ritualtable/status" (Sensor Data)
 * - Subscribes to: "demo/ritualtable/control" (Admin Commands: RESET, ForceWin)
 * =======================================================================================
 */

#include <SPI.h>
#include <MFRC522.h>
#include <WiFiNINA.h>
#include <ArduinoMqttClient.h>

// ================== Network setting ==================
char ssid[] = "YING";        // WiFi name
char pass[] = "][p709394";    // WiFi pw

// Node-RED running computer IP address
const char broker[] = "192.168.137.1"; 
int        port     = 1883;

// MQTT topices
const char topicStatus[]  = "demo/ritualtable/status";  
const char topicControl[] = "demo/ritualtable/control"; 

//
WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

const uint8_t Num_Readers = 5;
const uint8_t ssPins[Num_Readers] = {9,8,7,6,5}; 
const uint8_t rstPin = 10; 
MFRC522 rfid[Num_Readers];

// Node-RED testing UIDs, need to replace the ritual item tags 
byte targetUIDs[Num_Readers][7] = {
  {0x04, 0xA5, 0xAA, 0xC5, 0x79, 0x00, 0x00}, 
  {0x04, 0xA7, 0xAA, 0xC5, 0x79, 0x00, 0x00}, 
  {0x04, 0xA6, 0xAA, 0xC5, 0x79, 0x00, 0x00}, 
  {0x04, 0xBE, 0xAA, 0xC5, 0x79, 0x00, 0x00}, 
  {0x04, 0xA3, 0xAA, 0xC5, 0x79, 0x00, 0x00}, 
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
  
  // 
  for (uint8_t i = 0; i < Num_Readers; i++) {
    pinMode(relayPins[i], OUTPUT); digitalWrite(relayPins[i], LOW);
    pinMode(ledPins[i], OUTPUT);   digitalWrite(ledPins[i], LOW); 
    pinMode(ssPins[i], OUTPUT);    digitalWrite(ssPins[i], HIGH);
  }

  SPI.begin(); 
  for (uint8_t reader = 0; reader < Num_Readers; reader++) {
    rfid[reader].PCD_Init(ssPins[reader],rstPin);
    delay(4);
  }

  connectToNetwork();
}

void loop() {
  // keeping network is connected
  if (!mqttClient.connected()) {
     long now = millis();
     if (now - lastReconnectAttempt > 5000) {
       lastReconnectAttempt = now;
       connectToNetwork();
     }
  }
  mqttClient.poll(); // receive remote control

  // 
  bool allCorrectCardsDetected = true;

  for (uint8_t reader = 0; reader < Num_Readers; reader++) {
    // active current reader
    for (uint8_t i = 0; i < Num_Readers; i++) digitalWrite(ssPins[i], HIGH);
    digitalWrite(ssPins[reader], LOW);
    delay(10);

    if (rfid[reader].PICC_IsNewCardPresent() && rfid[reader].PICC_ReadCardSerial()) {
      byte* uid = rfid[reader].uid.uidByte;
      byte uidSize = rfid[reader].uid.size;

      // 
      if (compareUID(uid, uidSize, targetUIDs[reader], targetUIDLengths[reader])) {
        // [correct card]
        bool wasFalse = !cardDetected[reader]; // record previous state is False or not
        
        cardDetected[reader] = true;
        digitalWrite(ledPins[reader], HIGH);
        digitalWrite(relayPins[reader], HIGH);
        
        // only send the message to Node-RED when the state is changed
        if (wasFalse) sendMqttState(); 
        
      } 
      else if(compareUID(uid, uidSize, resetUID, resetUIDLength)) {
        // [reset card]
        resetToInitialState();
        return; // exit loop and start new loop
      } 
      else {
        // [wrond card] -> excute flash the light bulb
        flashAllCandles(3, 100); 
        
        bool wasTrue = cardDetected[reader]; 
        cardDetected[reader] = false;        
        
        // Send message to Node-RED only when the state is changed
        if (wasTrue) sendMqttState();
      }

      rfid[reader].PICC_HaltA(); 
      rfid[reader].PCD_StopCrypto1();
    }

    // Deactivate current reader
    digitalWrite(ssPins[reader], HIGH);

    // when excute flashAllCandles，all the light are off, force to keep the detected light on
    if (cardDetected[reader]) {
      digitalWrite(ledPins[reader], HIGH);
      digitalWrite(relayPins[reader], HIGH);
    } else {
      digitalWrite(ledPins[reader], LOW);
      digitalWrite(relayPins[reader], LOW);
      allCorrectCardsDetected = false;
    }
  }

  if (!allCorrectCardsDetected) {
    if(puzzleSolved == true){
      puzzleSolved = false;
      sendMqttState();
    }
     
  }

  if (allCorrectCardsDetected && !puzzleSolved ) {
    puzzleSolved = true;
    sendMqttState();
    playWinAnimation();
  } 
}

// ==================  ==================

void connectToNetwork() {
  Serial.print("Connecting WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    delay(1000);
  }
  Serial.println("Connected.");

  if (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT Fail: "); Serial.println(mqttClient.connectError());
  } else {
    Serial.println("MQTT OK.");
    mqttClient.onMessage(onMqttMessage);
    mqttClient.subscribe(topicControl);
    sendMqttState();
  }
}

//recevie the message from node-red R is reset, F is force win
void onMqttMessage(int messageSize) {
  String message = "";
  while (mqttClient.available()) message += (char)mqttClient.read();
  
  if (message == "R" || message == "r") resetToInitialState();
  else if (message == "F" || message == "f") forceSolve();
}

void sendMqttState() {
  mqttClient.beginMessage(topicStatus);
  mqttClient.print("{\"data\":[");
  for (uint8_t i = 0; i < Num_Readers; i++) {
    mqttClient.print(cardDetected[i]); // output 1 or 0 and use them in node-red
    if (i < Num_Readers - 1) mqttClient.print(",");
  }
  mqttClient.print("],\"solved\":");
  mqttClient.print(puzzleSolved ? "true" : "false");
  mqttClient.print("}");
  mqttClient.endMessage();
}

// the button(Force Win) function in node-red,use it to control manually
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
  delay(1000); //
}

void playWinAnimation() {
  delay(2000);
    for(uint8_t repeat = 0; repeat < 3; repeat++){
      mqttClient.poll(); 
      for(uint8_t i = 0; i < Num_Readers; i++){
        digitalWrite(relayPins[i], HIGH); digitalWrite(ledPins[i], HIGH);
        delay(50);
        digitalWrite(relayPins[i], LOW); digitalWrite(ledPins[i], LOW);
        delay(50);
      }
      for (int i = Num_Readers - 1; i >=0; i--){
        digitalWrite(relayPins[i], HIGH); digitalWrite(ledPins[i], HIGH);
        delay(50);
        digitalWrite(relayPins[i], LOW); digitalWrite(ledPins[i], LOW);
        delay(50);
      }
    }
    // 
    for (uint8_t i = 0; i < Num_Readers; i++) {
        digitalWrite(relayPins[i], HIGH); digitalWrite(ledPins[i], HIGH);
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
    mqttClient.poll();
    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(ledPins[i], HIGH); digitalWrite(relayPins[i],HIGH);
    }
    delay(duration);
    for (uint8_t i = 0; i < Num_Readers; i++) {
      digitalWrite(ledPins[i], LOW); digitalWrite(relayPins[i], LOW);
    }
    delay(duration);
  }
}