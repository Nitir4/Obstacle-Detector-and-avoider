#define TRIG_CENTER 17
#define ECHO_CENTER 16

#define TRIG_LEFT 15
#define ECHO_LEFT 2

#define TRIG_RIGHT 22
#define ECHO_RIGHT 23

#define PWMA 13   // Left Motor PWM
#define AIN2 12   // Left Motor AIN2
#define AIN1 14   // Left Motor AIN1
#define STBY 27   // Standby pin for Motor Driver
#define BIN1 26   // Right Motor BIN1
#define BIN2 25   // Right Motor BIN2
#define PWMB 33   // Right Motor PWM

// Encoder Pins
#define ENC_LA 19  // Left Motor Encoder A
#define ENC_LB 21  // Left Motor Encoder B
#define ENC_RA 5   // Right Motor Encoder A
#define ENC_RB 18  // Right Motor Encoder B

#define SAFE_DISTANCE 10  // Stop if object is within 10cm
#define MAX_DISTANCE 50   // Start reducing speed at 50cm
#define MAX_SPEED 255     // Maximum motor speed
#define MIN_SPEED 30      // Minimum speed when close to object

void setup() {
    Serial.begin(115200);
    
    pinMode(TRIG_CENTER, OUTPUT);
    pinMode(ECHO_CENTER, INPUT);
    
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);
    
    digitalWrite(STBY, HIGH); // Enable motor driver
}

long getDistance(int trigPin, int echoPin) {
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    long duration = pulseIn(echoPin, HIGH);
    long distance = duration * 0.034 / 2; // Convert to cm
    return distance;
}

void moveForward(int speed) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, speed);

    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, speed);
}

void stopCar() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);
}

void loop() {
    long distance = getDistance(TRIG_CENTER, ECHO_CENTER);
    Serial.print("Distance: ");
    Serial.println(distance);

    if (distance <= SAFE_DISTANCE) {
        stopCar();
    } else if (distance > SAFE_DISTANCE && distance <= MAX_DISTANCE) {
        // Reduce speed based on distance using mapping function
        int speed = map(distance, SAFE_DISTANCE, MAX_DISTANCE, MIN_SPEED, MAX_SPEED);
        moveForward(speed);
    } else {
        moveForward(MAX_SPEED);
    }

    delay(100);
}
