#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Arm.h>

#ifndef BUILTIN_LED
#define BUILTIN_LED  4
#endif

// Update these with values suitable for your network.
String clientId = "LeftArm";
const char* ssid = "Singularity";
const char* password = "r78ny8qt";
const char* mqtt_server = "192.168.0.211";

const bool debug = false;

const char* subscribeTopics[] ={"arm/left/set/servo"
                                };

// Causing "InstrFetchProhibited" Kernel panic if not declared in main scope
HardwareSerial scServoSerial(2);

WiFiClient espClient;
PubSubClient client(espClient);
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

void subscribe() {
    for (int i = 0; i < sizeof(subscribeTopics) / sizeof(int) ; i++) {
        client.subscribe(subscribeTopics[i]);
    }
}

void reconnect() {
    // Loop until we're reconnected
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        // Create a random client ID
        clientId += "-" + String(random(0xffff), HEX);
        // Attempt to connect
        if (client.connect(clientId.c_str())) {
            Serial.println("connected");
            // Once connected, publish an announcement...
            client.publish("status/connect", clientId.c_str());
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            digitalWrite(LED_BUILTIN, LOW);
            // Wait 5 seconds before retrying
            delay(5000);
        }
    }
}

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

void setup() {
    pinMode(BUILTIN_LED, OUTPUT);     // Initialize the BUILTIN_LED pin as an output
    Serial.begin(115200);
    setup_wifi();
    client.setServer(mqtt_server, 1883);
    client.setCallback(callback);

    arm.begin();
    arm.beginSCServo(scServoSerial);
}

void loop() {

    if (!client.connected()) {
        reconnect();
        subscribe();
    }
    client.loop();
}