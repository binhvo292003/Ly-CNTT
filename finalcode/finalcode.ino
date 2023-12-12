#include <LiquidCrystal_I2C.h>
#include "DHTesp.h"
#include <WiFi.h>

int lcdColumns = 16;
int lcdRows = 2;
#define DHT_PIN 14
#define PIR_PIN 12
#define RELAY_PIN 1
#define BUTTON_PIN 2
#define WATER_PIN 3
#define SOIL_PIN A0

DHTesp dht;
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

int pirValue;
float humidity;
float temperature;
float soil;
int water;

const char* ssid = "YourWiFiNetwork"; // Change this to your WiFi network name
const char* password = "YourWiFiPassword"; // Change this to your WiFi password


unsigned long previousLCDMillis = 0;
const long lcdInterval = 2000;

unsigned long previousWaterMillis = 0;
const long waterInterval = 100;

int buttonState = 0;
int relayState = LOW;

int pirState = LOW;
int lastPirState = LOW;


void setup() {
   Serial.begin(9600);
  pinMode(PIR_PIN, INPUT);
  dht.setup(DHT_PIN, DHTesp::DHT11);
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(WATER_PIN, INPUT);
  lcd.init();
  lcd.backlight();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi!");
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

void waterPlant(int soil) {
  if (soil < 50.0) {
    if (millis() - previousWaterMillis >= waterInterval) {
      digitalWrite(RELAY_PIN, HIGH);
      previousWaterMillis = millis();
    }
  } else {
    digitalWrite(RELAY_PIN, LOW);
  }
}

void waterButton() {
  buttonState = digitalRead(BUTTON_PIN);

  if (buttonState == HIGH) {
    if (relayState == LOW) {
      relayState = HIGH;
    } else {
      relayState = LOW;
    }
    digitalWrite(RELAY_PIN, relayState);
    delay(200);
  }
}

int getSoilPercent() {
  int sensorValue = analogRead(SOIL_PIN);
  int moisturePercent = map(sensorValue, 0, 1023, 100, 0);
  return moisturePercent;
}

void getMotion() {
  pirState = digitalRead(PIR_PIN);  // Read the state of the PIR sensor

  if (pirState != lastPirState) {  // If the PIR sensor state changes
    if (pirState == HIGH) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Intruder detected!");
      Serial.println("Intruder detected!");
      previousLCDMillis = millis();
    }
    lastPirState = pirState;  // Store the current state for comparison
  }
}

void warningWater() {
  if (water == 0) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("OUT OF WATER");
    Serial.println("OUT OF WATER");
    previousLCDMillis = millis();
  }
}


void loop() {
  pirValue = digitalRead(PIR_PIN);
  humidity = dht.getHumidity();
  temperature = dht.getTemperature();
  soil = getSoilPercent();
  water = digitalRead(WATER_PIN);

  Serial.println(humidity);
  Serial.println(temperature);

  waterPlant(soil);
  printLCD(humidity, temperature, soil, pirValue, water);
  waterButton();
  getMotion();
  warningWater();

  delay(200);
}
