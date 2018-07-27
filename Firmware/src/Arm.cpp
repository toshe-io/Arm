#include "Arm.h"

void Arm::begin() {
    servoPWM.begin();
    servoPWM.setPWMFreq(60);  // Analog servos run at ~60 Hz updates~

    // Set scServo RS485 to transmit
    pinMode(scServoTXEnablePin, OUTPUT);
    pinMode(scServoRXEnablePin, OUTPUT);
    digitalWrite(scServoTXEnablePin, HIGH);
    digitalWrite(scServoRXEnablePin, HIGH);

    //HardwareSerial sc(2);
    //sc.begin(115200);
    //scServo.pSerial = &sc;
    delay(500);

    //scServo.EnableTorque(scServoID1, 1);
    //scServo.EnableTorque(scServoID2, 1);
    delay(100);
}

void Arm::beginSCServo(HardwareSerial &scServoSerial) {
    scServoSerial.begin(115200);

    scServoSerial.begin(115200);
    scServo.pSerial = &scServoSerial;
    delay(500);

    scServo.EnableTorque(scServoID1, 1);
    scServo.EnableTorque(scServoID2, 1);
    delay(100);
}

void Arm::setAnalogServoPWMFreq(float freq) {
    servoPWM.setPWMFreq(freq);
}

void Arm::setAnalogServoPWM(uint8_t servo, uint16_t off, uint16_t on) {
    servoPWM.setPWM(servo, off, on);
}

void Arm::setScServoPWM(uint8_t servo, uint16_t pwm) {
    scServo.WritePos(servo, pwm, 0, 5000);
}