#define TRIG_CENTER 17
#define ECHO_CENTER 16

#define TRIG_LEFT 22
#define ECHO_LEFT 23

#define TRIG_RIGHT 15
#define ECHO_RIGHT 2

#define PWMA 13   // right Motor PWM
#define AIN2 12   // right Motor AIN2
#define AIN1 14   // right Motor AIN1
#define STBY 27   // Standby pin for Motor Driver
#define BIN1 26   // left Motor BIN1
#define BIN2 25   // left Motor BIN2
#define PWMB 33   // left Motor PWM

// Encoder Pins
#define ENC_LA 5  // Left Motor Encoder A
#define ENC_LB 18  // Left Motor Encoder B
#define ENC_RA 19  // Right Motor Encoder A
#define ENC_RB 21  // Right Motor Encoder B

#define SAFE_DISTANCE  7 // Stop if object is within 10cm
#define MAX_DISTANCE 250   // Start reducing speed at 50cm
#define MAX_SPEED 255     // Maximum motor speed
#define MIN_SPEED 50      // Minimum speed when close to object
#define TURN_SPEED 230    // Speed for turning
#define REVERSE_SPEED 80  // Speed for reversing

// PID Constants
float Kp = 1.8, Ki = 0.0, Kd = 0.7;

// Encoder Tick Counters
volatile int leftTicks = 0, rightTicks = 0;

#define TARGET_TICKS_90 198

void IRAM_ATTR leftEncoderISR() { leftTicks++; }
void IRAM_ATTR rightEncoderISR() { rightTicks++; }

void setup() {
    Serial.begin(115200);
    pinMode(TRIG_CENTER, OUTPUT);
    pinMode(ECHO_CENTER, INPUT);
    pinMode(TRIG_LEFT, OUTPUT);
    pinMode(ECHO_LEFT, INPUT);
    pinMode(TRIG_RIGHT, OUTPUT);
    pinMode(ECHO_RIGHT, INPUT);
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);
    attachInterrupt(digitalPinToInterrupt(ENC_LA), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RA), rightEncoderISR, RISING);
    digitalWrite(STBY, HIGH); 
    Serial.println("Robot Initialized");
}

long getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2;
    return distance;
}

void moveForward(int speed) {
    analogWrite(PWMA, speed);
    analogWrite(PWMB, speed);
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
}

void stopCar() {
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
}

void turnPID(int targetTicks, bool leftTurn) {
    leftTicks = 0;
    rightTicks = 0;
    float error, prevError = 0, integral = 0;
    int baseSpeed = 200;

    while (leftTicks < targetTicks && rightTicks < targetTicks) { // Problem: Should be OR instead of AND
        error = (leftTicks - rightTicks);  
        integral += error;
        float derivative = error - prevError;
        prevError = error;
        int correction = Kp * error + Ki * integral + Kd * derivative;
        
        int leftSpeed = baseSpeed - correction;
        int rightSpeed = baseSpeed + correction;
        leftSpeed = constrain(leftSpeed, 200, 255);
        rightSpeed = constrain(rightSpeed, 200, 254);

        if (leftTurn) {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            analogWrite(PWMA, leftSpeed);
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            analogWrite(PWMB, rightSpeed);
        } else {
            digitalWrite(AIN1, HIGH);
            digitalWrite(AIN2, LOW);
            analogWrite(PWMA, leftSpeed);
            digitalWrite(BIN1, LOW);
            digitalWrite(BIN2, HIGH);
            analogWrite(PWMB, rightSpeed);
        }

        // **NEW STOP CONDITION**
        if (leftTicks >= targetTicks || rightTicks >= targetTicks) {  
            break;
        }
    }
    delay(100);  
    stopCar();  
}



void reverseCar() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, REVERSE_SPEED);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, REVERSE_SPEED);
    delay(500);
    stopCar();
}

void loop() {
    long distanceCenter = getDistance(TRIG_CENTER, ECHO_CENTER);
    long distanceLeft = getDistance(TRIG_LEFT, ECHO_LEFT);
    long distanceRight = getDistance(TRIG_RIGHT, ECHO_RIGHT);

    Serial.print("Center: "); Serial.print(distanceCenter);
    Serial.print(" cm | Left: "); Serial.print(distanceLeft);
    Serial.print(" cm | Right: "); Serial.println(distanceRight);

    if (distanceCenter <= SAFE_DISTANCE) {
        Serial.println("Obstacle detected!");
        if (distanceLeft > SAFE_DISTANCE && distanceRight <= SAFE_DISTANCE) {
            Serial.println("Turning Left");
            turnPID(248, false);
        } else if (distanceRight > SAFE_DISTANCE && distanceLeft <= SAFE_DISTANCE) {
            Serial.println("Turning Right");
            turnPID(248, true);
        } else if (distanceLeft > SAFE_DISTANCE && distanceRight > SAFE_DISTANCE){
            if (distanceLeft > distanceRight ) {      
            turnPID(248, false);
        }
            else{
              turnPID(248,true);
            }
        }
            else {
            Serial.println("No clear path, reversing");
            turnPID(248,true);
            turnPID(248,true);
        }
    } else if (distanceCenter > SAFE_DISTANCE && distanceCenter <= MAX_DISTANCE) {
        int speed = map(distanceCenter, SAFE_DISTANCE, MAX_DISTANCE, MIN_SPEED, MAX_SPEED);
        Serial.print("Moving forward at speed: "); Serial.println(speed);
        moveForward(MAX_SPEED);
    } else {
        Serial.println("Moving at max speed");
        moveForward(MAX_SPEED);
    }
}
