#include <QTRSensors.h>  //IR Sensor array
QTRSensors qtr;

//[-60,-52,-44,-36,-28,-20,-12,-4,4,12,20,28,36,44,52,60]
// [ -0.60, -0.52, -0.44, -0.36, -0.28, -0.20, -0.12, -0.04, 0.04, 0.12, 0.20, 0.28, 0.36, 0.44, 0.52, 0.60 ]
//-0.60, -0.52, -0.44, -0.40, -0.32, -0.24, -0.18, -0.04, 0.04, 0.18, 0.24, 0.32, 0.40, 0.44, 0.52, 0.60
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
double offset = 0;

double motorSpeedA;
double motorSpeedB;
double baseSpeed = 80;  //180
double Kp = 15;          //5.8
double Kd = 10;          //7.4

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
int targetPosition_Left = 600;

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



  goForward(1500);
}

void loop() {
  if (linefollow == true) {
    while (true) {
      sensorRead();
      if (dVal[3] == 1 && dVal[4] == 1 && dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1 && dVal[11] == 1 && dVal[12] == 1) {
        goForward(500);
        sensorRead();
        if (dVal[3] == 1 && dVal[4] == 1 && dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1 && dVal[11] == 1 && dVal[12] == 1) {
          //stop();
          //linefollow=false;
          //break;
        } else {
          TurnRight(500);
        }
      } else if (dVal[0] == 1 && dVal[1] == 1 && dVal[2] == 1 && dVal[3] == 1 && dVal[4] == 1 && dVal[5] == 1 && dVal[6] == 1 && dVal[7] == 1) {
        goForward(1000);  //Turn Left by 90 degrees
        Left_normal();

      } else if (dVal[7] == 1 && dVal[8] == 1 && dVal[9] == 1 && dVal[10] == 1 && dVal[11] == 1 && dVal[12] == 1 && dVal[13] == 1) {
        Serial.println("RightNormal");
        goForward(1200);  //Turn Right by 90 degrees
        Right_normal();

      } else if (dVal[0] == 0 && dVal[1] == 0 && dVal[2] == 0 && dVal[3] == 0 && dVal[4] == 0 && dVal[5] == 0 && dVal[6] == 0 && dVal[7] == 0 && dVal[8] == 0 && dVal[9] == 0 && dVal[10] == 0 && dVal[11] == 0 && dVal[12] == 0 && dVal[13] == 0 && dVal[14] == 0 && dVal[15] == 0) {
        Serial.println("rararara");
        TurnRight(400);
        sensorRead();
        if (dVal[0] == 1 || dVal[1] == 1 || dVal[2] == 1 || dVal[3] == 1 || dVal[4] == 1 || dVal[5] == 1 || dVal[6] == 1 || dVal[7] == 1 || dVal[8] == 1 || dVal[9] == 1 || dVal[10] == 1 || dVal[11] == 1 || dVal[12] == 1 || dVal[13] == 1 || dVal[14] == 1 || dVal[15] == 1) {
          PID_control_Forward();
        } else {
          Serial.println("lelelele");
          TurnLeft(800);
          sensorRead();
          if (dVal[0] == 1 || dVal[1] == 1 || dVal[2] == 1 || dVal[3] == 1 || dVal[4] == 1 || dVal[5] == 1 || dVal[6] == 1 || dVal[7] == 1 || dVal[8] == 1 || dVal[9] == 1 || dVal[10] == 1 || dVal[11] == 1 || dVal[12] == 1 || dVal[13] == 1 || dVal[14] == 1 || dVal[15] == 1) {
            PID_control_Forward();
          }
        }
      } else {
        PID_control_Forward();
      }
    }
    //delay(20);
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
  for (int i = 0; i < 16; i++) {
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

void goForward(double forward_delay) {
  // this function will run the motors in
  //both directions at a fixed speed
  // turn on motor A
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enA, baseSpeed);  //70
  // turn on motor B
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
  // set speed to 200 out of possible range 0~255
  analogWrite(enB, baseSpeed);  //70
  delay(forward_delay);
  // now change motor directions
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

void stop() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}
void Left_normal() {
  encoderPos1 = 0;
  encoderPos2 = 0;
  // Rotate the motor clockwise to achieve the right turn
  digitalWrite(in1, LOW);   // Clockwise direction
  digitalWrite(in2, HIGH);  // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enA, baseSpeed);  // Full speed (255 is max PWM value)

  digitalWrite(in3, HIGH);  // Clockwise direction
  digitalWrite(in4, LOW);   // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enB, baseSpeed);  // Full speed (255 is max PWM value)
  // Wait until the encoder reaches 90 pulses (for 90 degrees)
  while (encoderPos2 < targetPosition_Left) {
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
  //encoder_speed = 0;
  // Once 90 degrees are reached, stop the motor
  //analogWrite(motorSpeedPin, 0);  // Stop the motor
  //Serial.println("Right angle turned (90 degrees).");

  // Reset encoder position for the next command
  encoderPos1 = 0;
  encoderPos2 = 0;

  // Add a small delay before the next loop iteration
  // delay(1000);
}


void Right_normal() {
  Serial.println("encoder");
  encoderPos1 = 0;
  encoderPos2 = 0;
  // Rotate the motor clockwise to achieve the right turn
  digitalWrite(in3, LOW);   // Clockwise direction
  digitalWrite(in4, HIGH);  // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enB, baseSpeed);  // Full speed (255 is max PWM value)

  digitalWrite(in1, HIGH);  // Clockwise direction
  digitalWrite(in2, LOW);   // Counterclockwise direction

  // Motor speed control: PWM
  analogWrite(enA, baseSpeed);  // Full speed (255 is max PWM value)
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
  //encoder_speed = 0;
  // Once 90 degrees are reached, stop the motor
  //analogWrite(motorSpeedPin, 0);  // Stop the motor
  //Serial.println("Right angle turned (90 degrees).");

  // Reset encoder position for the next command
  encoderPos1 = 0;
  encoderPos2 = 0;
  // Add a small delay before the next loop iteration
  // delay(1000);
}
