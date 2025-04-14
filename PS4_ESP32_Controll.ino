#include <Bluepad32.h>
#include <ESP32Servo.h> 
#include <Arduino.h> 

ControllerPtr myControllers[BP32_MAX_GAMEPADS];
// Define motor control pins
const int enA = 5;        // PWM pin for Motor A speed control
const int enB = 4;        // PWM pin for Motor B speed control
// Motor A direction pins
const int up = 18;
const int down = 19;
// Motor B direction pins
const int right_forward = 21;  // Forward
const int right_reverse = 22;  // Reverse
const int servo1Pin = 26;
const int servo2Pin = 14;
const int servoChannel1 = 2;  // Use unused LEDC channels (0-15)
const int servoChannel2 = 3;
int L_xvalue;
int L_yvalue;
int R_xvalue; 
int R_yvalue;
const int motorX_IN1 = 12;  //VEX MOTOR CONTROL
const int motorX_IN2 = 13;
const int motorX_enable = 25;     // Enable pin for motor on pins 12 & 13
const int motorX_pwmChannel = 4;  // New PWM channel for motorX
const int motorX_pwmSpeed = 200;  // PWM duty (0-255) for LEFT/RIGHT D-pad

//VEX Motor Timing
unsigned long motorXStartTime = 0;
bool motorXActive = false;
//PWM Channels
const int pwmChannelA = 1;
const int pwmChannelB = 0; 
const int pwmFreq = 5000;
const int servoFreq = 50;     // 50Hz for most servos
const int servoResolution = 1;
const int pwmResolution = 8;
const int steeringDeadzone = 70;  // Adjust deadzone as needed

static bool servo2Action = false;
static unsigned long servo2ActionStart = 0;
static int initialServo2Pulse = 0;

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller connected at index=%d\n", i);
            myControllers[i] = ctl;
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            break;
        }
    }
}

void dumpGamepad(ControllerPtr ctl) {
    L_xvalue = ctl->axisX();
    L_yvalue = ctl->axisY();
    R_xvalue = ctl->axisRX(); 
    R_yvalue = ctl->axisRY() ; 
    uint16_t buttons = ctl->buttons();
    if (buttons & 0x0001) { // X button (typically bit 0)
        Serial.println("X pressed");
    }
    if (buttons & 0x0002) { // O button (typically bit 1)
        Serial.println("O pressed");
    }
    if (buttons & 0x0004) { // Square button (typically bit 2)
        Serial.println("Square pressed");
    }
    if (buttons & 0x0008) { // Triangle button (typically bit 3)
        Serial.println("Triangle pressed");
    }
}

void processGamepad(ControllerPtr ctl) {
    dumpGamepad(ctl);
}

void processControllers() {
    for (auto myController : myControllers) {
        if (myController && myController->isConnected()) {
            if (myController->isGamepad()) {
                processGamepad(myController);
            } else {
                Serial.println("Unsupported controller");
            }
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
    const uint8_t* addr = BP32.localBdAddress();
    Serial.printf("BD Address: %02X:%02X:%02X:%02X:%02X:%02X\n",
                  addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();

    // Initialize motor control pins
    pinMode(enA, OUTPUT);
    pinMode(enB, OUTPUT);
    pinMode(up, OUTPUT);
    pinMode(down, OUTPUT);
    pinMode(right_forward, OUTPUT);  // Changed from 'left'
    pinMode(right_reverse, OUTPUT);  // Changed from 'right'

    // Configure PWM channels
    ledcSetup(pwmChannelA, pwmFreq, pwmResolution);
    ledcAttachPin(enA, pwmChannelA);
    ledcSetup(pwmChannelB, pwmFreq, pwmResolution);
    ledcAttachPin(enB, pwmChannelB);

    // Initialize motors to stop
    digitalWrite(up, LOW);
    digitalWrite(down, LOW);
    digitalWrite(right_forward, LOW);
    digitalWrite(right_reverse, LOW);
    ledcWrite(pwmChannelA, 0);
    ledcWrite(pwmChannelB, 0);
   ledcSetup(servoChannel1, servoFreq, servoResolution);
  ledcAttachPin(servo1Pin, servoChannel1);
  ledcSetup(servoChannel2, servoFreq, servoResolution);
  ledcAttachPin(servo2Pin, servoChannel2);
  
   pinMode(motorX_IN1, OUTPUT);
    pinMode(motorX_IN2, OUTPUT);
    ledcSetup(motorX_pwmChannel, pwmFreq, pwmResolution);
    ledcAttachPin(motorX_enable, motorX_pwmChannel);

    // Stop all motors initially
    stopMotors();

}

void stopMotors() {
    digitalWrite(up, LOW);
    digitalWrite(down, LOW);
    digitalWrite(right_forward, LOW);  // Fixed
    digitalWrite(right_reverse, LOW);  // Fixed
    ledcWrite(pwmChannelA, 0);
    ledcWrite(pwmChannelB, 0);
    Serial.println("Stopping motors");
}

void loop() {
    BP32.update();
    processControllers();

    // Declare these variables at the top of loop()
    static int forwardSpeed = 0;
    static int backwardSpeed = 0;
    static int servo1Pulse = 0;
    static int servo2Pulse = 0;
  static bool dpadUpPressed = false;


    if (myControllers[0] && myControllers[0]->isConnected()) {
        int throttle = myControllers[0]->throttle();
        int brake = myControllers[0]->brake();
        int RX = myControllers[0]->axisX();
        uint16_t buttons = myControllers[0]->buttons();
        // Update the variables instead of redeclaring
        forwardSpeed = map(throttle, 0, 1023, 0, 255);
        backwardSpeed = map(brake, 0, 1023, 0, 255);
        servo1Pulse = map(R_xvalue, -512, 512, 1634, 7864);
        servo2Pulse = map(R_yvalue, -512, 512, 1634, 7864);

        // Control servo2 with R_yvalue to toggle between 0 and 90 degrees
        if (R_yvalue > 100) {
            servo2Pulse = 7864;  // 90 degrees
        } else {
            servo2Pulse = 1634;  // 0 degrees
        }
        ledcWrite(servoChannel2, servo2Pulse);


        // Handle Square (forward) and Triangle (backward)
        if (buttons & 0x0004) { // Square pressed
            digitalWrite(motorX_IN1, HIGH);
            digitalWrite(motorX_IN2, LOW);
            ledcWrite(motorX_pwmChannel, motorX_pwmSpeed);
            Serial.println("Square - MotorX Forward");
        }
        else if (buttons & 0x0008) { // Triangle pressed
            digitalWrite(motorX_IN1, LOW);
            digitalWrite(motorX_IN2, HIGH);
            ledcWrite(motorX_pwmChannel, motorX_pwmSpeed);
            Serial.println("Triangle - MotorX Backward");
        }
        else if (!motorXActive) { // Only stop if X button isn't active
            digitalWrite(motorX_IN1, LOW);
            digitalWrite(motorX_IN2, LOW);
            ledcWrite(motorX_pwmChannel, 0);
        }

        // X button handling (keep existing functionality)
        static bool xPrev = false;
        bool xCurrent = buttons & 0x0001;
        if (xCurrent && !xPrev) {
            digitalWrite(motorX_IN1, HIGH);
            digitalWrite(motorX_IN2, LOW);
            ledcWrite(motorX_pwmChannel, 255);
            motorXStartTime = millis();
            motorXActive = true;
            Serial.println("X pressed - MotorX Boost");
        }
        xPrev = xCurrent;        // Handle servo2 action
 

        // Rest of your existing control logic
        if (abs(RX) > steeringDeadzone) {
            handleSteering(RX);
        }
        else if (forwardSpeed > 5) {
            moveForward(forwardSpeed);
        }
        else if (backwardSpeed > 5) {
            moveBackward(backwardSpeed);
        }
        else {
            stopMotors();
        }
    } else {
        stopMotors();
    }

    // Motor timeout check
    if (motorXActive && (millis() - motorXStartTime >= 420)) {
        digitalWrite(motorX_IN1, LOW);
        digitalWrite(motorX_IN2, LOW);
        motorXActive = false;
    }

    delay(50);
}

void handleSteering(int RX) {
    // Convert RX to steering intensity
    int steerIntensity = map(RX, -512, 512, -255, 255);
    int baseSpeed = 255;  // Base speed when steering
    
    if (steerIntensity < 0) {
        // Right turn
        digitalWrite(up, HIGH);          // Left forward
        digitalWrite(down, LOW);
        digitalWrite(right_forward, LOW);  // Right reverse
        digitalWrite(right_reverse, HIGH);
        
        ledcWrite(pwmChannelA, baseSpeed + abs(steerIntensity));
        ledcWrite(pwmChannelB, baseSpeed + abs(steerIntensity));
    } else {
        // Left turn
        digitalWrite(up, LOW);           // Left reverse
        digitalWrite(down, HIGH);
        digitalWrite(right_forward, HIGH); // Right forward
        digitalWrite(right_reverse, LOW);
        
        ledcWrite(pwmChannelA, baseSpeed + abs(steerIntensity));
        ledcWrite(pwmChannelB, baseSpeed + abs(steerIntensity));
    }
}

void moveForward(int speed) {
    digitalWrite(up, HIGH);
    digitalWrite(down, LOW);
    digitalWrite(right_forward, HIGH);
    digitalWrite(right_reverse, LOW);
    ledcWrite(pwmChannelA, speed);
    ledcWrite(pwmChannelB, speed);
    Serial.println("Moving Straight");
}

void moveBackward(int speed) {
    digitalWrite(down, HIGH);
    digitalWrite(up, LOW);
    digitalWrite(right_reverse, HIGH);
    digitalWrite(right_forward, LOW);
    ledcWrite(pwmChannelA, speed);
    ledcWrite(pwmChannelB, speed);
    Serial.println("Moving Backward");
}
