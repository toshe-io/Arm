#include "Arm.h"

void Arm::begin() {
    servoPWM.begin();
    servoPWM.setPWMFreq(60);  // Analog servos run at ~60 Hz updates~

    delay(100);
}

void Arm::beginSCServo(HardwareSerial &scServoSerial) {
    pinMode(scServoTXEnablePin, OUTPUT);
    pinMode(scServoRXEnablePin, OUTPUT);
    digitalWrite(scServoTXEnablePin, HIGH);
    digitalWrite(scServoRXEnablePin, HIGH);

    delay(100);

    scServoSerial.begin(115200);

    scServo.pSerial = &scServoSerial;
    delay(500);
}

// Initialize servos with their limits, should be called from the interface
void Arm::initServos() {
    initServo(10, 700, 2900, SERVO_TYPE_SC); // Shoulder1
    initServo(11, 300, 2300, SERVO_TYPE_SC); // Shoulder2
    initServo(0, 310, 520, SERVO_TYPE_PWM); // UpperArm
    initServo(2, 210, 580, SERVO_TYPE_PWM); // Elbow
    initServo(4, 110, 500, SERVO_TYPE_PWM); // ForeArm

    setStartPWM(10, 2000);
    setStartPWM(11, 2300);
    setStartPWM(0, 510);
    setStartPWM(2, 160);
    setStartPWM(4, 100);
}

void Arm::initServo(int servo, int minPWM, int maxPWM, int type) {
    calibrate(servo, minPWM, maxPWM);
    servos[servo].type = type;

    if (type == SERVO_TYPE_SC) {
        scServo.EnableTorque(servo, 1);
    }
}

void Arm::setStartPWM(int servo, int pwm) {
    servos[servo].previousPWM = pwm;
    servos[servo].currentPWM = pwm;
    servos[servo].nextPWM = pwm;
}

void Arm::calibrate(uint8_t servo, uint16_t min, uint16_t max) {
    servos[servo].minPWM = min;
    servos[servo].maxPWM = max;
}

void Arm::move(int servo, int pwm) {
    servos[servo].currentPWMStep = 0;
    
    servos[servo].previousPWM = servos[servo].currentPWM;
    servos[servo].nextPWM = pwm;
}

void Arm::updateServo(int servo) {
    if (servos[servo].type == SERVO_TYPE_PWM) {
        servoPWM.setPWM(servo, 0, servos[servo].currentPWM);
    }
    else if (servos[servo].type == SERVO_TYPE_SC) {
        scServo.WritePos(servo, servos[servo].currentPWM, 0, 5000);
    }
}

void Arm::update() {
    if(updateTimer.repeat()){
        for(int servo=0;servo < MAX_SERVO_COUNT;servo++) {
            if (servos[servo].nextPWM != servos[servo].previousPWM) {
                servos[servo].currentPWM = Easing::easeInOutQuad(servos[servo].currentPWMStep, servos[servo].previousPWM, servos[servo].nextPWM-servos[servo].previousPWM, easingDuration);

                updateServo(servo);

                // Serial.printf("%i,%i,%i,%i\n", servos[servo].currentPWM, servos[servo].nextPWM, servos[servo].previousPWM, servos[servo].currentPWMStep); // DEBUG

                servos[servo].currentPWMStep++;

                if (servos[servo].currentPWM == servos[servo].nextPWM) {
                    servos[servo].previousPWM = servos[servo].nextPWM;
                    servos[servo].currentPWMStep = 0;
                }
            }
        }
    }
}