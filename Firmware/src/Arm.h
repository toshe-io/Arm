#ifndef Arm_h
#define Arm_h

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SCServo.h>


class Arm {
    public:
        void begin();
        void beginSCServo(HardwareSerial &scServoSerial);

        // Analog PWM servos
        void setAnalogServoPWMFreq(float freq);
        void setAnalogServoPWM(uint8_t servo, uint16_t off, uint16_t on);

        void setScServoPWM(uint8_t servo, uint16_t pwm);

    private:
        // PWM Servos
        Adafruit_PWMServoDriver servoPWM = Adafruit_PWMServoDriver(0x40);

        // SC Servo
        SCServo scServo;
        uint8_t scServoTXEnablePin = 34;
        uint8_t scServoRXEnablePin = 35;

        
        uint8_t scServoID1 = 0xb;
        uint8_t scServoID2 = 0xa;
        /*
        uint16_t scServoMin = 0;
        uint16_t scServoMax = 4096;
        */
};

#endif
