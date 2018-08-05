#ifndef Arm_h
#define Arm_h

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SCServo.h>

#include <Easing.h>
#include <neotimer.h>


class Arm {
    public:
        void begin();
        void beginSCServo(HardwareSerial &scServoSerial);

        void initServo(int servoID, int minPWM, int maxPWM, int type);
        void initServos();

        void setStartPWM(int servo, int pwm);

        // Analog PWM servos
        void calibrate(uint8_t servo, uint16_t min, uint16_t max);

        void move(int servo, int pwm);

        void update();


        uint8_t getCalibrateMin(uint8_t servo) { return servos[servo].minPWM; };
        uint8_t getCalibrateMax(uint8_t servo) { return servos[servo].maxPWM; };
        uint8_t getPWM(uint8_t servo) { return servos[servo].currentPWM; };

    private:
        #define MAX_SERVO_COUNT 20

        enum ServoType {
            SERVO_TYPE_SC,
            SERVO_TYPE_PWM
        };

        struct Servo {
            int minPWM;
            int maxPWM;

            int previousPWM;
            int currentPWM;
            int nextPWM;

            int currentPWMStep;

            int easingDuration;

            int type;

            bool isMoving;
        };

        Servo servos[MAX_SERVO_COUNT];

        void updateServo(int servo);

        // PWM Servos
        Adafruit_PWMServoDriver servoPWM = Adafruit_PWMServoDriver(0x40);

        // SC Servo
        SCServo scServo;
        uint8_t scServoTXEnablePin = 34;
        uint8_t scServoRXEnablePin = 35;

        int easingDuration = 200;

        Neotimer updateTimer = Neotimer(15);
};

#endif
