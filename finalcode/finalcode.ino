#include <LiquidCrystal_I2C.h>
#include "DHTesp.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;
int isRelayActivated = 0;

#define DHT_PIN 14
#define PIR_PIN 12
#define RELAY_PIN 15
#define BUTTON_PIN 13
#define WATER_PIN 3
#define SOIL_PIN A0

DHTesp dht;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

int isConfigured = 0;
int pirValue;
float humidity;
float humLimit = -1;
float temperature;
float tempLimit = -1;
float soil;
float soilLimit = -1;
int water;


const char* ssid = "THREE O'CLOCK THD";
const char* password = "3open24h";

unsigned long lastWaterNotiMillis = 0;
const char* mqttServer = "broker.hivemq.com";
int mqttPort = 1883;
const char* mqttTopic_sub = "to-esp32";
const char* mqttTopic_pub = "from-esp32";
const char* mqttTopic_status_pub = "from-esp32-s";
const char* mqttClientId = "ESP32Client-";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

unsigned long previousLCDMillis = 0;  // Initialize the variable
unsigned long lcdInterval = 1000;     // Initialize with a value (for example)

unsigned long previousWaterMillis = 0;  // Initialize the variable
unsigned long waterInterval = 2000;     // Initialize with a value (for example)

int buttonState = 0;   // Initialize the variable
int relayState = LOW;  // Initialize the variable
int pirState;          // Declare the variable
int lastPirState;      // Declare the variable

const char* host = "maker.ifttt.com";
const int port = 80;
const char* pirRequest = "/trigger/piractive/json/with/key/lgwuGgs9_LPCi7tDBBPzsWGh52RgphlK4WqwbEuydrf";
const char* noWaterRequest = "/trigger/nowater/json/with/key/lgwuGgs9_LPCi7tDBBPzsWGh52RgphlK4WqwbEuydrf";

unsigned long connectionTime = 0;
unsigned long buttonDelayTime = 0;
unsigned long motionTime = 0;
unsigned long waterWarningTime = 0;
unsigned long generalStatusTime = 0;
unsigned long waterTime = 0;

int detectMotion = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIR_PIN, INPUT);
  dht.setup(DHT_PIN, DHTesp::DHT11);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(WATER_PIN, INPUT);
  pinMode(BUTTON_PIN, INPUT);
  lcd.init();
  lcd.backlight();

  wifiConnect();
  mqttClient.setServer(mqttServer, mqttPort);
  mqttClient.setCallback(callback);

  // Connect to MQTT
  mqttConnect();
}


void sendRequest(const char* request, unsigned long& previousMillis, unsigned long interval) {
  WiFiClient client;
  if (millis() - previousMillis >= interval) {
    while (!client.connect(host, port)) {
      Serial.println("connection fail");
      previousMillis = millis();
      return;
    }
    client.print(String("GET ") + request + " HTTP/1.1\r\n"
                 + "Host: " + host + "\r\n"
                 + "Connection: close\r\n\r\n");

    delay(500);  // You may still need a small delay here for stability, as connecting is critical

    while (client.available()) {
      String line = client.readStringUntil('\r');
      Serial.print(line);
    }
    previousMillis = millis();
  }
}

void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected to WiFi!");
}

void mqttConnect() {
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection...");
    String clientId = mqttClientId;
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      Serial.println("Connected to MQTT");
      mqttClient.subscribe(mqttTopic_sub);
    } else {
      Serial.println("Failed, try again in 5 seconds");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* message, unsigned int length) {
  Serial.println(topic);
  String strMsg;
  for (int i = 0; i < length; i++) {
    strMsg += (char)message[i];
  }
  if (strMsg != NULL) {
    isConfigured = 1;
    Serial.println(strMsg);

    // Phân tách các giá trị từ thông điệp
    float receivedValues[3];
    int count = 0;
    char* value = strtok((char*)strMsg.c_str(), ",");
    while (value != NULL && count < 3) {
      receivedValues[count++] = atoi(value);
      value = strtok(NULL, ",");
    }

    soilLimit = receivedValues[0];
    tempLimit = receivedValues[1];
    humLimit = receivedValues[2];
  }
}

void waterPlant(float soil, float temperature, float humidity) {
  if (water == 1) {
    if (isConfigured == 0) {
      if (soil < 50.0) {
        if (!isRelayActivated) {
          isRelayActivated = 1;
          digitalWrite(RELAY_PIN, HIGH);
          previousWaterMillis = millis();
        }
        if (isRelayActivated && millis() - previousWaterMillis >= 2000) {
          digitalWrite(RELAY_PIN, LOW);
          isRelayActivated = 0;
        }
      }
    } else if (isConfigured == 1) {
      if (soil < soilLimit || (temperature < tempLimit && humidity < humLimit)) {
        if (!isRelayActivated) {
          isRelayActivated = 1;
          digitalWrite(RELAY_PIN, HIGH);
          previousWaterMillis = millis();
        }
        if (isRelayActivated && millis() - previousWaterMillis >= 2000) {
          digitalWrite(RELAY_PIN, LOW);
          isRelayActivated = 0;
        }
      }
    }
  }
}


void printLCD(float humidity, float temp, float soil, int pir, int water) {
  if (millis() - previousLCDMillis >= lcdInterval) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("T:");
    lcd.print(temp, 1);
    lcd.print(" ");
    lcd.print("H:");
    lcd.print(humidity, 1);
    lcd.setCursor(0, 1);
    lcd.print("S:");
    lcd.print(soil, 0);
    lcd.print(" ");
    if (pir == HIGH) {
      lcd.print("M: Y");
    } else {
      lcd.print("M: N");
    }
    previousLCDMillis = millis();
  }
}

void waterButton() {
  if (water == 1) {
    buttonState = digitalRead(BUTTON_PIN);

    Serial.println(buttonState);

    if (buttonState == HIGH) {
      relayState = HIGH;
      buttonDelayTime = millis();
      isRelayActivated = 1;
    }

    if (relayState == HIGH) {
      digitalWrite(RELAY_PIN, HIGH);
      if (millis() - buttonDelayTime >= 5000) {
        relayState = LOW;
        digitalWrite(RELAY_PIN, LOW);
      }
    }
  }
}

int getSoilPercent() {
  int sensorValue = analogRead(SOIL_PIN);
  Serial.println(sensorValue);

  int moisturePercent = map(sensorValue, 0, 1023, 100, 0);
  return moisturePercent;
}

void getMotion() {
  pirState = digitalRead(PIR_PIN);

  if (pirState == HIGH && detectMotion == 0) {
    sendRequest(pirRequest, previousLCDMillis, lcdInterval);
    Serial.println("Intruder detected!");
    digitalWrite(RELAY_PIN, LOW);
    detectMotion = 1;
    motionTime = millis();
  }

  if (detectMotion && millis() - motionTime >= 5000) {
    detectMotion = 0;  // Reset motion detection
  }
}


void warningWater() {
  if (water == 0 && millis() - waterWarningTime >= 3000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OUT OF WATER");
    Serial.println("OUT OF WATER");
    sendRequest(noWaterRequest, previousLCDMillis, lcdInterval);
    previousLCDMillis = millis();
    waterWarningTime = millis();
  }
}

void loop() {
  pirValue = digitalRead(PIR_PIN);
  humidity = dht.getHumidity();
  temperature = dht.getTemperature();
  soil = getSoilPercent();
  water = digitalRead(WATER_PIN);

  waterPlant(soil, temperature, humidity);
  printLCD(humidity, temperature, soil, pirValue, water);
  //   waterButton();
  getMotion();
  warningWater();


  //-------------------------------------------------------
  if (!mqttClient.connected()) {
    mqttConnect();
  }

  if (isRelayActivated == 0) {
    // Send general status
    if (millis() - generalStatusTime >= 5000) {
      String payload = String(temperature) + "," + String(humidity) + "," + String(soil) + "," + String(pirValue) + "," + String(water);
      mqttClient.publish(mqttTopic_status_pub, payload.c_str());
      generalStatusTime = millis();
    }

    if (isRelayActivated && millis() - generalStatusTime >= 5000) {
      String payload = String(soil) + "," + String(temperature) + "," + String(humidity);
      mqttClient.publish(mqttTopic_pub, payload.c_str());
      isRelayActivated = 0;
      generalStatusTime = millis();
    }
  }
  mqttClient.loop();
}
