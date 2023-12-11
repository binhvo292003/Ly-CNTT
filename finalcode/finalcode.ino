#include <LiquidCrystal_I2C.h>
#include "DHTesp.h"
// set the LCD number of columns and rows
int lcdColumns = 16;
int lcdRows = 2;
#define DHT_PIN 14
#define PIR_PIN 12
/*--VQB--*/
#define RELAY_PIN 1
#define BUTTON_PIN 2
#define WATER_PIN 3
#define SOIL_PIN A0
/*--VQB--*/

DHTesp dht;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows);

void setup() {
  Serial.begin(9600);
  // setup dht
  pinMode(PIR_PIN, INPUT);
  dht.setup(DHT_PIN, DHTesp::DHT11);

  // setup relay
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);

  // setup water
  pinMode(WATER_PIN, INPUT);

  // setup soil


  // initialize LCD
  lcd.init();
  // turn on LCD backlight
  lcd.backlight();
}

void printLCD(float humidity, float temp, float soil, int pir, int water) {
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
  } else if (pir == LOW) {
    lcd.print("M: N");
  }
  delay(2000);
  lcd.clear();
  if (water == HIGH) {
    lcd.setCursor(0, 0);
    lcd.print("Water available.");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("Water available.");
  }
}

void waterPlant(int soil) {
  if (soil < 50.0) {
    digitalWrite(RELAY_PIN, HIGH);
    delay(100);
  } else {
    digitalWrite(RELAY_PIN, LOW);
    delay(100);
  }
}

int getSoilPercent() {
  int sensorValue = analogRead(SOIL_PIN);                   // Read the analog sensor value
  int moisturePercent = map(sensorValue, 0, 1023, 100, 0);  // Map the sensor value to a percentage (adjust as needed)

  return moisturePercent;
}


void loop() {
  // Read humidity and temp from DHT11
  int pirvalue = digitalRead(PIR_PIN);
  float humidity = dht.getHumidity();
  float temperature = dht.getTemperature();
  float soil = getSoilPercent();
  int water = digitalRead(WATER_PIN);

  Serial.println(humidity);
  Serial.println(temperature);

  waterPlant(soil);
  printLCD(humidity, temperature, soil, pirvalue, water);
  delay(200);
}