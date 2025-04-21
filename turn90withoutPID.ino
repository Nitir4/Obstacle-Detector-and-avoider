// Motor Driver Pins
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

// Encoder Ticks for 90° Turn
#define TARGET_TICKS_90 250  // Adjust as per actual readings

// Encoder Tick Counters
volatile int leftTicks = 0, rightTicks = 0;

// Interrupt Service Routines for Encoders
void IRAM_ATTR leftEncoderISR() { leftTicks++; }
void IRAM_ATTR rightEncoderISR() { rightTicks++; }

void setup() {
    Serial.begin(115200);

    // Motor Pins Setup
    pinMode(AIN1, OUTPUT);
    pinMode(AIN2, OUTPUT);
    pinMode(PWMA, OUTPUT);
    pinMode(BIN1, OUTPUT);
    pinMode(BIN2, OUTPUT);
    pinMode(PWMB, OUTPUT);
    pinMode(STBY, OUTPUT);

    // Enable Motor Driver
    digitalWrite(STBY, HIGH);

    // Encoder Pins Setup
    pinMode(ENC_LA, INPUT_PULLUP);
    pinMode(ENC_LB, INPUT_PULLUP);
    pinMode(ENC_RA, INPUT_PULLUP);
    pinMode(ENC_RB, INPUT_PULLUP);

    // Attach Interrupts for Encoders
    attachInterrupt(digitalPinToInterrupt(ENC_LA), leftEncoderISR, RISING);
    attachInterrupt(digitalPinToInterrupt(ENC_RA), rightEncoderISR, RISING);

    Serial.println("🚀 System Initialized. Ready to turn.");
}

// Simple 90° Left Turn Function (No PID)
void turnLeft() {
    leftTicks = 0;
    rightTicks = 0;

    int speedLeft = 230;  // Speed for left motor
    int speedRight = 230; // Speed for right motor

    Serial.println("🔄 Starting 90-degree left turn...");
    
    while (leftTicks < TARGET_TICKS_90 || rightTicks < TARGET_TICKS_90) {
        if (leftTicks < TARGET_TICKS_90) {
            digitalWrite(AIN1, LOW);
            digitalWrite(AIN2, HIGH);
            analogWrite(PWMA, speedLeft);
        } else {
            analogWrite(PWMA, 0);
        }

        if (rightTicks < TARGET_TICKS_90) {
            digitalWrite(BIN1, HIGH);
            digitalWrite(BIN2, LOW);
            analogWrite(PWMB, speedRight);
        } else {
            analogWrite(PWMB, 0);
        }
    }

    stopMotors();
    Serial.println("✅ Turn complete.");
}

// Stop Motors Function
void stopMotors() {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, LOW);
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMA, 0);
    analogWrite(PWMB, 0);

    Serial.println("🛑 Motors stopped.");
}

void loop() {
    turnLeft();
    delay(2000);  // Wait 2 seconds before repeating
}
