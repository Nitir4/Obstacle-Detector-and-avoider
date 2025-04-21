#define SOUND_SPEED 0.034  // cm/µs (Speed of sound)
#define STOP_DISTANCE 10   // cm (Threshold distance to stop the robot)
#define MAX_DISTANCE 50    // cm (Max distance for deceleration)

// Sensor 1 Pins
const int TRIG1 = A0; // Center
const int ECHO1 = A1;

// Sensor 2 Pins
const int TRIG2 = A2; // Left
const int ECHO2 = A3;

// Sensor 3 Pins
const int TRIG3 = A4; // Right
const int ECHO3 = A5;

// Motor Pins
#define rightMotorF 8
#define rightMotorB 9
#define rightMotorPWM 10
#define leftMotorF 12
#define leftMotorB 13
#define leftMotorPWM 11

float distance1, distance2, distance3;

void setup() {
  Serial.begin(115200);

  // Sensor pins
  pinMode(TRIG1, OUTPUT);
  pinMode(ECHO1, INPUT);

  pinMode(TRIG2, OUTPUT);
  pinMode(ECHO2, INPUT);

  pinMode(TRIG3, OUTPUT);
  pinMode(ECHO3, INPUT);

  // Motor pins
  pinMode(rightMotorF, OUTPUT);
  pinMode(rightMotorB, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);

  pinMode(leftMotorF, OUTPUT);
  pinMode(leftMotorB, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
}

void loop() {
  distance1 = getDistance(TRIG1, ECHO1);  // Center
  distance2 = getDistance(TRIG2, ECHO2);  // Left
  distance3 = getDistance(TRIG3, ECHO3);  // Right

  Serial.print("Center: ");
  Serial.print(distance1);
  Serial.print(" cm | ");

  Serial.print("Left: ");
  Serial.print(distance2);
  Serial.print(" cm | ");

  Serial.print("Right: ");
  Serial.print(distance3);
  Serial.println(" cm");

  // Control Movement
  if (distance1 < STOP_DISTANCE) {
    stopMotors();  // Stop if center < 10 cm
  } else if (distance2 < STOP_DISTANCE) {
    turnRight(150);  // Turn right if left < 10 cm
  } else if (distance3 < STOP_DISTANCE) {
    turnLeft(150);  // Turn left if right < 10 cm
  } else if (distance1 <= MAX_DISTANCE) {
    int speed = map(distance1, STOP_DISTANCE, MAX_DISTANCE, 50, 200);  // Speed from 50 to 200
    moveForward(speed);
  } else {
    moveForward(200);  // Full speed if center > 50 cm
  }

  delay(100);
}

float getDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  return (duration * SOUND_SPEED) / 2;
}

void moveForward(int speed) {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, speed);

  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, speed);
}

void stopMotors() {
  digitalWrite(rightMotorF, LOW);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, 0);

  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, 0);
}

void turnLeft(int speed) {
  digitalWrite(rightMotorF, HIGH);
  digitalWrite(rightMotorB, LOW);
  analogWrite(rightMotorPWM, speed);

  digitalWrite(leftMotorF, LOW);
  digitalWrite(leftMotorB, HIGH);
  analogWrite(leftMotorPWM, speed);
}

void turnRight(int speed) {
  digitalWrite(rightMotorF, LOW);
  digitalWrite(rightMotorB, HIGH);
  analogWrite(rightMotorPWM, speed);

  digitalWrite(leftMotorF, HIGH);
  digitalWrite(leftMotorB, LOW);
  analogWrite(leftMotorPWM, speed);
}
