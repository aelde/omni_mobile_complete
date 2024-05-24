#include <Wire.h>
#include <ArduinoJson.h>
#include <SoftwareSerial.h>

DynamicJsonDocument doc(1024);
DynamicJsonDocument responseDoc(1024);

// Define wheel encoder sensor pins
#define FLA 34
#define FLB 35
#define FRA 32
#define FRB 33
#define RLA 25
#define RLB 26
#define RRA 27
#define RRB 14

SoftwareSerial MDDS60Serial(RXF, TXF);
SoftwareSerial MDDS60Serial1(RXR, TXR);

// Encoder counters
volatile int cFL = 0, lFL = 0, dFL = 0;
volatile int cFR = 0, lFR = 0, dFR = 0;
volatile int cRL = 0, lRL = 0, dRL = 0;
volatile int cRR = 0, lRR = 0, dRR = 0;

// Function to print JSON response
void jsonPrint() {
  responseDoc["FL_cFL"] = cFL;
  responseDoc["FR_cFR"] = cFR;
  responseDoc["RL_cRL"] = cRL;
  responseDoc["RR_cRR"] = cRR;

  String jsonResponse;
  serializeJson(responseDoc, jsonResponse);
  Serial.println(jsonResponse);
}

// Encoder interrupt service routines
void IRAM_ATTR decoder_fl() {
  cFL += (digitalRead(FLA) == digitalRead(FLB)) ? 1 : -1;
}
void IRAM_ATTR decoder_fr() {
  cFR += (digitalRead(FRA) == digitalRead(FRB)) ? -1 : 1;
}
void IRAM_ATTR decoder_rl() {
  cRL += (digitalRead(RLA) == digitalRead(RLB)) ? 1 : -1;
}
void IRAM_ATTR decoder_rr() {
  cRR += (digitalRead(RRA) == digitalRead(RRB)) ? -1 : 1;
}

void setup() {
  // Initialize pins
  pinMode(FLA, INPUT_PULLDOWN);
  pinMode(FLB, INPUT_PULLDOWN);
  pinMode(FRA, INPUT);
  pinMode(FRB, INPUT);
  pinMode(RLA, INPUT);
  pinMode(RLB, INPUT);
  pinMode(RRA, INPUT);
  pinMode(RRB, INPUT);

  // Attach interrupts
  attachInterrupt(FLA, decoder_fl, RISING);
  attachInterrupt(FRA, decoder_fr, RISING);
  attachInterrupt(RLA, decoder_rl, RISING);
  attachInterrupt(RRA, decoder_rr, RISING);

  // Initialize serial communication
  Serial.begin(115200);

}

void loop() {
  jsonPrint();
  delay(50);
}
