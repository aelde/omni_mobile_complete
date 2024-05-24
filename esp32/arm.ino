#include <Wire.h>
#include <ArduinoJson.h>
#include <Adafruit_PWMServoDriver.h>

DynamicJsonDocument doc(1024);
DynamicJsonDocument responseDoc(1024);

Adafruit_PWMServoDriver board1 = Adafruit_PWMServoDriver(0x40);
#define SERVOMIN 125
#define SERVOMAX 575

const int LED_PIN = 2;
const int OE_PIN = 16;

int step = 3, pre_OE = 1, cur_OE = 1, s = 0;
int autoM = 0, busy = 0;
char l[20]; // Changed l to an array of characters
float switch1Smoothed, switch1Prev, a6;
float smoothingFactor = 0.001, complementFactor = 0.999;

int maxV = 90, minV = 30;
int angle_default = 180;
int angle_grip = 110;

// a1, a2, a3, a4, a5, a6
int angles_Current[6] =    {10, 0, 90, 180, 180, 114}; // Initial angles
int angles_Default[6] =    {10, 0, 90, 180, 180, 114};  // Default angles
int angles_Grip[6] =       {10, 90, 90, 160, 92, 114};   // Grip angles
int angles_Gripping[6] =   {80, 90, 90, 160, 92, 114}; // gripping
int angles_After_Grip[6] = {80, 40, 90, 180, 135, 114}; // after grip angles
int angles_Handups[6] =    { 4, 70, 90,  40, 130, 110}; // hand ups

int aa[6] = {0, 0, 0, 0, 0, 0};
int aaa[6] = {0, 0, 0, 0, 0, 0};
// a1, a2, a3, a4, a5, a6.
int angleMins[] = {5, 10, 89, 90, 80, 0};
int angleMaxs[] = {80, 90, 90, 180, 180, 180};

int smoothMove(int currAngle, int targetAngle) {
  switch1Smoothed = (s * smoothingFactor) + (switch1Prev * complementFactor);
  switch1Prev = switch1Smoothed;
  int a = map(switch1Smoothed, 0, 100, currAngle, targetAngle);
  return a;
}

bool anyAngleDiffExceedsThreshold(int currAngles[], int targetAngles[], int threshold) {
  for (int i = 0; i < 6; i++) {
    if (abs(currAngles[i] - targetAngles[i]) >= threshold) {
      return true;
    }
  }
  return false;
}

void jsonPrint(int angles[]) {
    responseDoc["OE"] = cur_OE;
    responseDoc["autoM"] = autoM;
    for (int i = 0; i < 6; i++) {
      responseDoc["a" + String(i + 1)] = angles[i];
    }
    String jsonResponse;
    serializeJson(responseDoc, jsonResponse);
    Serial.println(jsonResponse);
}
void printAngles(const char *label, int angles[]) {
  Serial.print(label);
  for (int i = 0; i < 6; i++) {
    Serial.print(" a");
    Serial.print(i + 1);
    Serial.print(":");
    Serial.print(angles[i]);
    Serial.print(" ");
  }
  Serial.print(switch1Smoothed);
  Serial.print(" ");
  Serial.print(switch1Prev);
  Serial.println();
}
int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMIN,SERVOMAX);// map angle of 0 to 180 to Servo min and Servo max 
   return pulse;
}
void boardControl(int angles_Current[]){
    for (int i = 0; i < 6; i++) {
    board1.setPWM(i, 0, angleToPulse(angles_Current[i]));
  }
    // board1.setPWM(0, 0, angleToPulse(angles_Current[0]));
    // board1.setPWM(1, 0, angleToPulse(angles_Current[1]));
    // board1.setPWM(2, 0, angleToPulse(angles_Current[2]));
    // board1.setPWM(4, 0, angleToPulse(angles_Current[4]));
    // board1.setPWM(5, 0, angleToPulse(angles_Current[5]));
    // board1.setPWM(7, 0, angleToPulse(angles_Current[3]));
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(OE_PIN, OUTPUT);
  digitalWrite(OE_PIN, LOW);
  board1.begin();
  board1.setPWMFreq(60);
  digitalWrite(LED_PIN, HIGH);
}

void loop() {
  if (Serial.available() > 0) {
    String jsonData = Serial.readStringUntil('\n');
    DeserializationError error = deserializeJson(doc, jsonData);
    if (error) {
      Serial.println("Error parsing JSON");
      return;
    }

    cur_OE = doc["OE"];
    autoM = doc["autoM"];
    if (cur_OE != pre_OE) {
      pre_OE = cur_OE;
      digitalWrite(LED_PIN, cur_OE);
    }

    if (cur_OE == 0 && angles_Current[0] >= angleMins[0] && angles_Current[0] <= angleMaxs[0])  // arm1
    {
      int R2_a1 = doc["R2_a1"];
      angles_Current[0] = constrain(angles_Current[0] - (R2_a1 * step), angleMins[0], angleMaxs[0]);
    }
    if (cur_OE == 0 && angles_Current[1] >= angleMins[1] && angles_Current[1] <= angleMaxs[1])  // arm2
    {
      int L2_a2 = doc["L2_a2"];
      angles_Current[1] = constrain(angles_Current[1] + (L2_a2 * step), angleMins[1], angleMaxs[1]);
    }
    if (cur_OE == 0 && angles_Current[2] >= angleMins[2] && angles_Current[2] <= angleMaxs[2])  // arm3
    {
      int L1_a3 = doc["L1_a3"];
      angles_Current[2] = constrain(angles_Current[2] - (L1_a3 * step), angleMins[2], angleMaxs[2]);
    }
    if (cur_OE == 0 && angles_Current[3] >= angleMins[3] && angles_Current[3] <= angleMaxs[3])  // arm4
    {
      int R1_a4 = doc["R1_a4"];
      angles_Current[3] = constrain(angles_Current[3] - (R1_a4 * step), angleMins[3], angleMaxs[3]);
    }
    if (cur_OE == 0 && angles_Current[4] >= angleMins[4] && angles_Current[4] <= angleMaxs[4])  // arm5
    {
      int L2_a5 = doc["L2_a5"];
      angles_Current[4] = constrain(angles_Current[4] - (L2_a5 * step), angleMins[4], angleMaxs[4]);
    }
    if (cur_OE == 0 && angles_Current[5] >= angleMins[5] && angles_Current[5] <= angleMaxs[5])  // arm6
    {
      int L1_a6 = doc["L1_a6"];
      angles_Current[5] = constrain(angles_Current[5] + (L1_a6 * step), angleMins[5], angleMaxs[5]);
    }

    // jsonPrint((cur_OE == 0) ? angles_Current : aaa);
    jsonPrint(angles_Current);
  }

  if (cur_OE == 1 && busy == 0) {
    int *targetAngles;
    if (autoM == 0) {
      targetAngles = angles_Default;
      strcpy(l, "finished Default");
    } else if (autoM == 1) {
      targetAngles = angles_Grip;
      strcpy(l, "finish Grip");
    } else if (autoM == 2){
      targetAngles = angles_Gripping;
      strcpy(l, "finish Gripping");
    }
     else if (autoM == 3){
      targetAngles = angles_After_Grip;
      strcpy(l, "finish After Grip");
    } else {
      return;
    }
    // Serial.println("im in");
    s = 100, busy = 1;
    // if targetAngles == De
    while (anyAngleDiffExceedsThreshold(angles_Current, targetAngles, 3)) {
      for (int i = 0; i < 6; i++) {
        angles_Current[i] = smoothMove(angles_Current[i], targetAngles[i]);
      }
  //         for (int i = 0; i < 6; i++) {
  //   board1.setPWM(i, 0, angleToPulse(angles_Current[i]));
  // }
      boardControl(angles_Current);
      // jsonPrint(angles_Current);
      delay(20);
    }
    // if (targetAngles = angles_Grip){
    //   angles_Current[0] = smoothMove(110, 180);
    // }
    s = 0, busy = 0, switch1Smoothed = 0, switch1Prev = 0;
    memcpy(angles_Current, targetAngles, sizeof(angles_Current));  // angles_Current[6] = aa[6];
  }
  //   for (int i = 0; i < 6; i++) {
  //   board1.setPWM(i, 0, angleToPulse(angles_Current[i]));
  // }
  boardControl(angles_Current);
  printAngles("ei", angles_Current);
  delay(20);
}
