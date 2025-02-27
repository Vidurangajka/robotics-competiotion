
#include <QTRSensors.h>  //IR Sensor array
QTRSensors qtr;

const uint8_t SensorCount = 16;
float sensorW[16] = { -1.28, -1.16, -0.88, -0.72, -0.60, -0.48, -0.32, -0.16, 0.16, 0.32, 0.48, 0.60, 0.72, 0.88, 1.16, 1.28 };
uint16_t sensorValues[SensorCount];
// Arrays to store the minimum and maximum values for each sensor
int sensorMin[SensorCount];
int sensorMax[SensorCount];
double weightedVal[SensorCount];
int dVal[SensorCount];
double digital_thres = 500;

double position = 0;
double P, I, D, PID, PreErr = 0;
//double offset = 0;

double motorSpeedA;
double motorSpeedB;
double baseSpeed = 100;  //100
double Kp = 5;           //5.8
double Kd = 7;           //7.4

// Define motor control pins (L298N)
int enA = 7;  //Right Motor
int in1 = 8;  //in1-LOW, in2-HIGH = FORWARD
int in2 = 9;

int enB = 10;  //Left Motor
int in3 = 11;  //in3-LOW, in4-HIGH = FORWARD
int in4 = 12;

bool linefollow = true;

// Define Right encoder pins
const int encoderPinA1 = 2;  // Right Encoder Phase A
const int encoderPinB1 = 3;  // Right Encoder Phase B

volatile int encoderPos1 = 0;  // Variable to store Right encoder position
int direction1 = 0;            // Variable to store direction of Right Encoder

// Define Left encoder pins
const int encoderPinA2 = 19;  // Left Encoder Phase A
const int encoderPinB2 = 18;  // Left Encoder Phase B

volatile int encoderPos2 = 0;  // Variable to store Left encoder position
int direction2 = 0;            // Variable to store direction of Left Encoder

int targetPosition_Right = 500;
int targetPosition_Left = 500;

double offset = 0;
double backoffset = 2;

//Maze Variables
int junction = 2;
int junc_count = 1;  //For junction zero  count is 1
int box_indicator = 5;

//Right Ultra sonic sensor
const int trigPinR = 28;
const int echoPinR = 30;
//long durationR;
//int distanceR;

//Left Ultra sonic sensor
const int trigPinL = 16;
const int echoPinL = 17;
//long durationL;
//int distanceL;
String obstacle = "F";

// Interrupt service routine for encoder A
void encoderISR1() {
  if (digitalRead(encoderPinB1) == HIGH) {
    encoderPos1++;   // Clockwise rotation
    direction1 = 1;  // Direction is clockwise
  } else {
    encoderPos1--;    // Counterclockwise rotation
    direction1 = -1;  // Direction is counterclockwise
  }
}
void encoderISR2() {
  if (digitalRead(encoderPinB2) == HIGH) {
    encoderPos2++;   // Clockwise rotation
    direction2 = 1;  // Direction is clockwise
  } else {
    encoderPos2--;    // Counterclockwise rotation
    direction2 = -1;  // Direction is counterclockwise
  }
}

void setup() {
  pinMode(box_indicator, OUTPUT);
  pinMode(13, OUTPUT);
  // set all the motor control pins to outputs
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set encoder pins as INPUT
  pinMode(encoderPinA1, INPUT);  //Right Encoder
  pinMode(encoderPinB1, INPUT);

  pinMode(encoderPinA2, INPUT);  //Left Encoder
  pinMode(encoderPinB2, INPUT);

  pinMode(trigPinR, OUTPUT);  // Sets the right trigPin as an Output
  pinMode(echoPinR, INPUT);   // Sets the right echoPin as an Input

  pinMode(trigPinL, OUTPUT);  // Sets the left trigPin as an Output
  pinMode(echoPinL, INPUT);   // Sets the left echoPin as an Input
  // Attach interrupt to Encoder Phase A pin (for position tracking)
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), encoderISR1, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), encoderISR2, RISING);

  qtr.setTypeRC();  // Initialize IR Sensor Pins
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
  // Initialize Serial Monitor
  Serial.begin(9600);
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

  goForward(100);
}
float height = 0;
void loop() {

  switch (order) {
    case "Asending":
      while (true) {
        sensorRead();
        PID_control_Forward();
        if (dVal[0] == 1 && dVal[1] == 1 && dVal[2] == 1 && dVal[3] == 1 && dVal[4] == 1 && dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1) {  //Left line detect
          box_junc_count = box_junc_count + 1;
          Serial.println(box_junc_count);
          if (box_junc_count < 3) {
            box_common();
          } else if (height == "small") {
            a_small_first();
          } else if (height == "medium") {
            a_medium_first();
          } else if (height == "large") {
            a_large_first();
          }
        } else if (dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1 && dVal[11] == 1 && dVal[12] == 1 && dVal[13] == 1 && dVal[14] == 1) {  //Right line detect
          box_junc_count = box_junc_count + 1;
          Serial.println(box_junc_count);
          if (box_junc_count < 3) {
            box_common();
          } else if (height == "small") {
            if (box_junc_count < 8) {
              a_small_first();
            }
            if (height2 == "medium") {
              a_medium_second();
            } else if (height2 == "large") {
              a_large_second(); 
            }
          } else if (height == "medium") {
            a_medium_first();
          } else if (height == "large") {
            a_large_first();
          }
        }
      }
  }
}

void boxCommon() {
  if (box_junc_count == 5) {  //Left turn
    if (junc_count == 15) {
      goForward(550);
    }
    //Serial.println(junc_count);
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
      sensorRead();
      PID_control_Forward();
    }
    Left_normal();
  } else if (box_junc_count == 2) {
    height = measureHeight();
    grabBox();
    U_turn();
  }
}
void a_small_first() {
  if (box_junc_count == 5) {  //Right turn
    goForward(550);
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
      sensorRead();
      PID_control_Forward();
    }
    Right_normal();
    //PID_control_Forward();
  } else if (box_junc_count == 6) {  //Left turn
    goForward(550);

    //Serial.println(junc_count);
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
      sensorRead();
      PID_control_Forward();
    }
    Left_normal();
    //PID_control_Forward();
  } else if (box_junc_count == 4 || box_junc_count == 7) {  //U_turn
    if (box_junc_count == 4) {
      dropBox();
    } else if (box_junc_count == 7) {
      height2 = measureHeight();
      grabBox();
      U_turn();
    }
    U_turn();
  } else if (box_junc_count) {  //Forward
    goForward(600);
  }
}
void a_medium_first(){
}
void a_large_first(){
}
void a_medium_second(){
}
void a_large_secondo(){
}

void jj() {
  if (box_junc_count == 1 || junc_count == 4 || junc_count == 16) {  //Right turn
    if (junc_count == 16) {
      goForward(550);
    } else {
      goForward(550);
    }
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
      sensorRead();
      PID_control_Forward();
    }
    Right_normal();
    PID_control_Forward();

  } else if (junc_count == 8 || junc_count == 9 || junc_count == 15) {  //Left turn
    if (junc_count == 15) {
      goForward(550);
    } else if (junc_count == 8) {
      goForward(550);
    } else {
      goForward(550);
    }

    //Serial.println(junc_count);
    unsigned long startTime = millis();
    while (millis() - startTime < 1000) {
      sensorRead();
      PID_control_Forward();
    }
    Left_normal();
    //PID_control_Forward();
  } else if (junc_count == 2 || junc_count == 5 || junc_count == 6 || junc_count == 7 || junc_count == 10 || junc_count == 11 || junc_count == 12) {
    //Forward
    if (junc_count == 10) {
      goForward(600);
      int temp = measureDistance();

      unsigned long startTime = millis();

      while (millis() - startTime < 5000) {
        stop();
        Serial.println("ggg");
        if (temp > 10 && temp != 0) {
          sensorRead();
          PID_control_Forward();
          temp = measureDistance();
          Serial.println(temp);
        } else {
          obstacle = "T";
          digitalWrite(13, HIGH);
          Serial.println(obstacle);
          break;
        }
      }
    } else {
      goForward(600);
    }

  } else if (junc_count == 3 || junc_count == 13 || junc_count == 14) {  //Reverse
    if (junc_count == 14) {
      goBackword(700);
    } else {
      goBackword(300);
    }
  } else if (junc_count == 0) {
  }
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
  /*for (int i = 0; i < 16; i++) {
    Serial.print(dVal[i]);
    Serial.print(" ");
  }

  Serial.println("");
  //delay(500);
  //Serial.println("");*/

  position = weightedSum / actualSum;
}

void PID_control_Forward() {
  // Serial.println("Kp " + String(Kp,4) + "   KI " + String(Ki,4) + "   KD " + String(Kd,4) );
  //Serial.Println(positionLine);
  P = position * 100;
  D = P - PreErr;
  PID = Kp * P + Kd * D;

  PreErr = P;



  double MSpeedA = baseSpeed - PID;  //
  double MSpeedB = baseSpeed + PID;  //offset has been added to balance motor speeds


  // // constraints for speed

  if (MSpeedA > 140) {
    MSpeedA = 140;
  }

  if (MSpeedB > 140) {
    MSpeedB = 140;
  }

  if (MSpeedA < 40) {
    MSpeedA = 40;
  }

  if (MSpeedB < 40) {
    MSpeedB = 40;
  }

  //Serial.println("Position   " + String(position*100) + "  D  " + String(D) +  "  PID  "+ String(PID));
  // Serial.println("A   " + String(MSpeedA-offset)+ "   B   " +String(MSpeedB+offset));
  //enable pwm
  analogWrite(enA, MSpeedA);
  analogWrite(enB, MSpeedB);

  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  //delay(50);
}
void PID_control_Reverse() {
  // Serial.println("Kp " + String(Kp,4) + "   KI " + String(Ki,4) + "   KD " + String(Kd,4) );
  //Serial.Println(positionLine);
  P = position * 100;
  D = P - PreErr;
  PID = Kp * P + Kd * D;

  PreErr = P;



  double MSpeedA = baseSpeed - backoffset + PID;  //
  double MSpeedB = baseSpeed + backoffset - PID;  //offset has been added to balance motor speeds


  // // constraints for speed

  if (MSpeedA > 140) {
    MSpeedA = 140;
  }

  if (MSpeedB > 140) {
    MSpeedB = 140;
  }

  if (MSpeedA < 40) {
    MSpeedA = 40;
  }

  if (MSpeedB < 40) {
    MSpeedB = 40;
  }

  //Serial.println("Position   " + String(position*100) + "  D  " + String(D) +  "  PID  "+ String(PID));
  // Serial.println("A   " + String(MSpeedA-offset)+ "   B   " +String(MSpeedB+offset));
  //enable pwm
  analogWrite(enA, MSpeedA);
  analogWrite(enB, MSpeedB);

  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  //delay(50);
}
void goForward(double forward_delay) {
  // this function will run the motors in
  //both directions at a fixed speed
  // turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, baseSpeed - offset);  //70
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, baseSpeed);  //70
  delay(forward_delay);
  // now change motor directions
}

void goBackword(double backword_delay) {
  // this function will run the motors in
  //both directions at a fixed speed
  // turn on motor A
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, baseSpeed - 5);  //70
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, baseSpeed - 4);  //70
  delay(backword_delay);
  // now change motor directions
}

void Left_normal() {
  // Rotate the motor clockwise to achieve the right turn
  digitalWrite(in1, LOW);   // Clockwise direction
  digitalWrite(in2, HIGH);  // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enA, val);  // Full speed (255 is max PWM value)

  digitalWrite(in3, HIGH);  // Clockwise direction
  digitalWrite(in4, LOW);   // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enB, val);  // Full speed (255 is max PWM value)
  // Wait until the encoder reaches 90 pulses (for 90 degrees)
  while (encoderPos1 < targetPosition_Left) {
    // Optionally, you can monitor the position while turning
    Serial.print("Encoder Position: ");
    Serial.print(encoderPos1);
    Serial.print(" Direction: ");
    if (direction1 == 1) {
      Serial.println("Clockwise");
    } else {
      Serial.println("Counterclockwise");
    }
    //delay(50);  // Small delay for readability and to prevent serial buffer overflow
  }
  val = 0;
  // Once 90 degrees are reached, stop the motor
  //analogWrite(motorSpeedPin, 0);  // Stop the motor
  //Serial.println("Right angle turned (90 degrees).");

  // Reset encoder position for the next command
  //encoderPos = 0;

  // Add a small delay before the next loop iteration
  // delay(1000);
}


void Right_normal() {
  // Rotate the motor clockwise to achieve the right turn
  digitalWrite(in3, LOW);   // Clockwise direction
  digitalWrite(in4, HIGH);  // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enB, val);  // Full speed (255 is max PWM value)

  digitalWrite(in1, HIGH);  // Clockwise direction
  digitalWrite(in2, LOW);   // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enA, val);  // Full speed (255 is max PWM value)
  // Wait until the encoder reaches 90 pulses (for 90 degrees)
  //Serial.println(encoderPos2);
  while (encoderPos2 * (-1) < targetPosition_Right) {
    // Optionally, you can monitor the position while turning
    Serial.print("Encoder Position: ");
    Serial.print(encoderPos2);
    Serial.print(" Direction: ");
    if (direction2 == 1) {
      Serial.println("Clockwise");
    } else {
      Serial.println("Counterclockwise");
    }
    //delay(50);  // Small delay for readability and to prevent serial buffer overflow
  }
  val = 0;
  // Once 90 degrees are reached, stop the motor
  //analogWrite(motorSpeedPin, 0);  // Stop the motor
  //Serial.println("Right angle turned (90 degrees).");

  // Reset encoder position for the next command
  //encoderPos = 0;

  // Add a small delay before the next loop iteration
  // delay(1000);
}
/*
void Left_normal() {
  int val = 70;
  // Rotate the motors to achieve the Left side 90 degree angle
  digitalWrite(in1, LOW);   // Clockwise direction
  digitalWrite(in2, HIGH);  // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enA, val);  // Full speed (255 is max PWM value)

  digitalWrite(in3, HIGH);  // Clockwise direction
  digitalWrite(in4, LOW);   // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enB, val);  // Full speed (255 is max PWM value)
                          // Wait until the encoder reaches 90 pulses (for 90 degrees)
  encoderPos2 = 0;
  dVal[7] = 0;
  dVal[8] = 0;
  delay(2000);
  while (dVal[7] == 0 && dVal[8] == 0) {
    // Optionally, you can monitor the position while turning
    sensorRead();
    //delay(50);  // Small delay for readability and to prevent serial buffer overflow
  }
  val = 0;
  // Once 90 degrees are reached, stop the motor
  //analogWrite(motorSpeedPin, 0);  // Stop the motor
  //Serial.println("Right angle turned (90 degrees).");

  // Reset encoder position for the next command
  encoderPos2 = 0;
}


void Right_normal() {
  int val = 70;
  // Rotate the motors to achieve the Right side 90 degree angle
  digitalWrite(in3, LOW);   // Clockwise direction
  digitalWrite(in4, HIGH);  // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enB, val);  // Full speed (255 is max PWM value)

  digitalWrite(in1, HIGH);  // Clockwise direction
  digitalWrite(in2, LOW);   // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enA, val);  // Full speed (255 is max PWM value)
  // Wait until the encoder reaches 90 pulses (for 90 degrees)
  //Serial.println(encoderPos2);
  encoderPos2 = 0;
  dVal[7] = 0;
  dVal[8] = 0;
  delay(2000);
  while (dVal[7] == 0 || dVal[8] == 0) {
    // Optionally, you can monitor the position while turning
    sensorRead();
    //delay(50);  // Small delay for readability and to prevent serial buffer overflow
  }
  val = 0;
  // Once 90 degrees are reached, stop the motor
  //analogWrite(motorSpeedPin, 0);  // Stop the motor
  //Serial.println("Right angle turned (90 degrees).");

  // Reset encoder position for the next command
  encoderPos2 = 0;

  // Add a small delay before the next loop iteration
  // delay(1000);
}*/

long measureDistance() {
  long duration, distance;

  // Clear the TRIG_PIN
  digitalWrite(trigPinR, LOW);
  delayMicroseconds(2);

  // Set the TRIG_PIN on HIGH state for 10 microseconds
  digitalWrite(trigPinR, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinR, LOW);

  // Read the ECHO_PIN, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPinR, HIGH);

  // Calculate the distance
  distance = duration * 0.034 / 2;

  return distance;
}

long measureDistanceLeft() {
  long duration, distance;

  // Clear the TRIG_PIN
  digitalWrite(trigPinL, LOW);
  delayMicroseconds(2);

  // Set the TRIG_PIN on HIGH state for 10 microseconds
  digitalWrite(trigPinL, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPinL, LOW);

  // Read the ECHO_PIN, returns the sound wave travel time in microseconds
  duration = pulseIn(echoPinL, HIGH);

  // Calculate the distance
  distance = duration * 0.034 / 2;

  return distance;
}
void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  delay(500);
}
void TurnRight(double turn_delay) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 70 + offset);
  // turn on motor B
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 70 - offset);
  delay(turn_delay);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

void TurnLeft(double turn_delay) {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, 70 + offset);
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, 70 - offset);
  delay(turn_delay);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
