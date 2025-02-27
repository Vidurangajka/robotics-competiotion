#include <QTRSensors.h>
QTRSensors qtr;

#define WHITE 0
#define BLACK 1
#define NUM_LINES 10

const uint8_t SensorCount = 16;
// Array to store pin numbers for sensors

float sensorW[16] = { -1.28, -1.16, -0.88, -0.72, -0.60, -0.48, -0.32, -0.16, 0.16, 0.32, 0.48, 0.60, 0.72, 0.88, 1.16, 1.28 };
uint16_t sensorValues[SensorCount];
// Arrays to store the minimum and maximum values for each sensor
int sensorMin[SensorCount];
int sensorMax[SensorCount];
double weightedVal[SensorCount];
int dVal[SensorCount];
double digital_thres = 500;

// Variables for decoding
unsigned long startTime, duration;
int currentState, lastState = -1;
float sensorTimes[NUM_LINES];

// Define motor control pins (L298N)
int enA = 7;  //Right Motor
int in1 = 8;  //in1-LOW, in2-HIGH = FORWARD
int in2 = 9;

int enB = 10;  //Left Motor
int in3 = 11;  //in3-LOW, in4-HIGH = FORWARD
int in4 = 12;
int baseSpeed = 70;
int u = 0;  // For barcode
int n = 0;
int flag = 0;
bool status = false;
void setup() {
  Serial.begin(9600);
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);


  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){ 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48 }, SensorCount);
  qtr.setEmitterPin(2);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);

  for (int i = 0; i < SensorCount; i++) {
    sensorMin[i] = 1023;  // Set initial min to max possible value
    sensorMax[i] = 0;     // Set initial max to min possible value
  }

  // Calibrate sensors
  Serial.println("Calibrating sensors...");

  // For a certain period, read sensor values and update min and max values
  unsigned long startTime = millis();
  while (millis() - startTime < 10000) {  // Calibrate for 5 seconds
    for (int i = 0; i < SensorCount; i++) {
      qtr.read(sensorValues);
      int value = sensorValues[i];
      if (value < sensorMin[i]) {
        sensorMin[i] = value;
      }
      if (value > sensorMax[i]) {
        sensorMax[i] = value;
      }
    }
  }
}
int current = 1;
void loop() {
  if (flag == 0) {
    while (true) {
      sensorRead();
      for (int i = 1; i < SensorCount; i++) {
        if (dVal[i] != current) {
          status = true;
          break;
          Serial.println("allSensorsNotSame");
        }
      }
      if(status==true){
        break;
      }
      goForward();
    }
    flag = 1;
  }
  Serial.println("jj");
  if (n == NUM_LINES + 1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    // turn on motor B
    digitalWrite(in3, LOW);
    digitalWrite(in4, LOW);
    delay(1000);
  } else {
    goForward();
    if (u < 1) {
      barcode();
      decode(sensorTimes);
    }
  }
}
void goForward() {
  // this function will run the motors in
  //both directions at a fixed speed
  // turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  analogWrite(enA, baseSpeed*0.95);  //70
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  analogWrite(enB, baseSpeed);  //70
  // now change motor directions
}

void sensorRead() {
  double weightedSum = 0;
  double actualSum = 0;
  qtr.read(sensorValues);
  for (uint8_t i = 0; i < SensorCount; i++) {
    sensorValues[i] = map(sensorValues[i], sensorMin[i], sensorMax[i], 1000, 0);
    weightedVal[i] = sensorW[i] * sensorValues[i];
    actualSum += sensorValues[i];
    weightedSum += weightedVal[i];
    if (sensorValues[i] > digital_thres) {
      dVal[i] = 1;
    } else {
      dVal[i] = 0;
    }
  }
}

void barcode() {
  //int n = 0;
  while (u < 1) {
    if (n > NUM_LINES) {
      u = u + 1;
      break;
    }
    sensorRead();
    bool allSensorsSame = true;
    currentState = dVal[7];  // Read the first sensor

    // Check if all sensors detect the same state
    for (int i = 1; i < SensorCount; i++) {
      if (dVal[i] != currentState) {
        allSensorsSame = false;
        break;
        //Serial.println("allSensorsNotSame");
      }
    }
    if (allSensorsSame) {
      //Serial.println("allSensorsSame");
    }
    if (allSensorsSame && currentState == 1) {
      //Serial.println("allSensorsSame and Currentstate is 1");
      startTime = micros();
      // If all sensors detect the same state
      while (allSensorsSame && currentState == 1) {
        sensorRead();
        currentState = dVal[7];
        /*for (int k = 1; k < SensorCount ; k++) {
          Serial.println(dVal[k]);
          Serial.print("  ");
          }
          Serial.println();*/
      }
      duration = micros() - startTime;
      sensorTimes[n] = duration;
      n = n + 1;
      //Serial.print(duration);
      //Serial.print("  ");
    }
  }
  for (int g = 0; g < NUM_LINES; g++) {
    Serial.print(sensorTimes[g]);
    Serial.print(" ");
  }
  Serial.println("");
}
void decode(float timeArray[NUM_LINES]) {
  //Serial.print("aaa");
  for (int g = 0; g < NUM_LINES; g++) {
    Serial.print(timeArray[g]);
    Serial.print(" ");
  }
  float timeMin = 10000000;
  float timeMax = 0;
  int decodeArray[NUM_LINES - 2];
  /*for (int i = 0; i < NUM_LINES; i++) {
    int value = timeArray[i];
    if (value < timeMin) {
      timeMin = value;
    }
    if (value > timeMax) {
      timeMax = value;
    }
  }*/
  /*Serial.println("timeMin");
  Serial.print(timeMin);
  Serial.print("timeMax");
  Serial.print(timeMax);*/
  float timeThreshold = 600000;
  Serial.print("timeThreshold  ");
  Serial.println(timeThreshold);
  for (int i = 0; i < NUM_LINES - 2; i++) {
    float value = timeArray[i];
    if (value < timeThreshold) {
      decodeArray[i] = 0;
    } else if (value > timeThreshold) {
      decodeArray[i] = 1;
    }
  }
  //Serial.print("ccc");
  for (int j = 0; j < NUM_LINES - 2; j++) {
    Serial.print(decodeArray[j]);
  }
  int junction = binaryToDecimal(decodeArray, 8);
  junction = junction % 5;
  Serial.print("Virtual box position  ");
  Serial.print(junction);
}
int binaryToDecimal(int binaryArray[], int size) {
  int decimalValue = 0;

  for (int i = 0; i < size; i++) {
    decimalValue = decimalValue * 2 + binaryArray[i];
  }

  return decimalValue;
}