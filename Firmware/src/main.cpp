#include <WiFi.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include <Arm.h>

#ifndef BUILTIN_LED
#define BUILTIN_LED  4
#endif

// Update these with values suitable for your network.
String clientId = "LeftArm";
const char* ssid = "Singularity";
const char* password = "r78ny8qt";

const bool debug = false;

WebSocketsServer webSocket = WebSocketsServer(81);

// Causing "InstrFetchProhibited" Kernel panic if not declared in main scope
HardwareSerial scServoSerial(2);

WiFiClient espClient;
Arm arm;

void setup_wifi() {
    delay(10);
    // We start by connecting to a WiFi network
    Serial.println();
    Serial.print("Connecting to ");
    Serial.println(ssid);

    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }

    randomSeed(micros());

    Serial.println("");
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());
}

/*
void callback(char* topic, byte* payload, unsigned int length) {
    char inData[length];

    for (int i = 0; i < length; i++) {
        inData[i] = (char)payload[i];
    }
    
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(inData);

    if (String(topic) == "arm/left/set/servo") {
        int servo = root["id"];
        int pwm = root["pwm"];
        bool scservo = root["scservo"];

        if (!debug) {
            if (scservo) {
                arm.setScServoPWM(servo, pwm);
            }
            else {
                arm.setAnalogServoPWM(servo, 0, pwm);
            }
        }

        if (debug) {
            Serial.print("Servo: ");
            Serial.print(servo);
            Serial.print(", ");
            Serial.print(pwm);
            Serial.print(", ");
            Serial.println(scservo);
        }
    }
}
*/

void handleSocketData(uint8_t * payload, size_t length) {
    String payload_str = String((char*) payload);
    
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.parseObject(payload_str);

    String action = root["action"];

    if (action.equals("move")) {
        int servo = root["id"];
        int pwm = root["pwm"];
        
        //Serial.printf("Servo: %d, PWM: %d\n", servo, pwm);

        if (!debug) {
            arm.move(servo, pwm);
        }
        Serial.println("");
    }
    else if (action.equals("calibrate")) {
        int servo = root["id"];
        int min = root["min"];
        int max = root["max"];
        arm.calibrate(servo, min, max);
    }
}

void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)
{    
    switch(type) 
    {
        case WStype_DISCONNECTED:
            Serial.printf("Disconnected: %u\n", num);
            break;
        case WStype_CONNECTED:
            {
                IPAddress ip = webSocket.remoteIP(num);
                Serial.printf("New socket client: %d.%d.%d.%d\n", ip[0], ip[1], ip[2], ip[3]);
                webSocket.sendTXT(num, "Connected");
            }
            break;
        case WStype_TEXT:
            //Serial.print("RXT");
            handleSocketData(payload, length);

            break;
        case WStype_BIN:
            //Serial.print("RXB");
            handleSocketData(payload, length);
            
            break;
        default:
            Serial.printf("Invalid WStype [%d]\r\n", type);
            break;
    }
}

void setup() {
    pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
    Serial.begin(115200);
    setup_wifi();
    webSocket.begin();
    webSocket.onEvent(webSocketEvent);

    arm.begin();
    arm.beginSCServo(scServoSerial);
    arm.initServos();
}

void loop() {
    webSocket.loop();

    arm.update();
}