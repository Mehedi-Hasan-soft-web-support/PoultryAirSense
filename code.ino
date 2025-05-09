#include <WiFi.h>
#include "ThingSpeak.h"
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

const char* ssid = "Me";
const char* password = "mehedi113";
const char* thingSpeakApiKey = "WQ38HT9QZWRE4FFX";
unsigned long channelNumber = 2948761;

const int mq7Pin = 34;
const int mq135Pin = 35;
const int dhtPin = 4;
#define DHTTYPE DHT22

const int statusLed = 12;
const int relayPin = 14;

const int RELAY_ON = HIGH;
const int RELAY_OFF = LOW;

LiquidCrystal_I2C lcd(0x27, 16, 2);
DHT dht(dhtPin, DHTTYPE);
WiFiClient client;

const int mq135Threshold = 500;
const int coThreshold = 700;

const int numSamples = 10;
unsigned long lastDisplayUpdate = 0;
unsigned long lastThingSpeakUpdate = 0;
const unsigned long displayInterval = 1000;
const unsigned long thingSpeakInterval = 15000;

float temperature = 0;
float humidity = 0;
int mq7Value = 0;
int mq135Value = 0;

bool relayState = false;  // <-- Global Relay State

void connectToWiFi() {
  WiFi.begin(ssid, password);
  int attempts = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Connecting WiFi");

  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(500);
    lcd.print(".");
    attempts++;
  }

  lcd.clear();
  if (WiFi.status() == WL_CONNECTED) {
    lcd.setCursor(0, 0);
    lcd.print("WiFi Connected");
    for (int i = 0; i < 3; i++) {
      digitalWrite(statusLed, HIGH);
      delay(100);
      digitalWrite(statusLed, LOW);
      delay(100);
    }
  } else {
    lcd.setCursor(0, 0);
    lcd.print("WiFi Failed");
  }

  delay(1000);
}

int readAveragedAnalog(int pin) {
  long sum = 0;
  for (int i = 0; i < numSamples; i++) {
    sum += analogRead(pin);
    delay(2);
  }
  return sum / numSamples;
}

void updateSensorValues() {
  float tempReading = dht.readTemperature();
  float humReading = dht.readHumidity();

  if (!isnan(tempReading)) temperature = tempReading;
  if (!isnan(humReading)) humidity = humReading;

  mq7Value = readAveragedAnalog(mq7Pin);
  mq135Value = readAveragedAnalog(mq135Pin);

  Serial.printf("MQ7: %d, MQ135: %d\n", mq7Value, mq135Value);
}

void updateDisplay() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(temperature, 1);
  lcd.print(" H:");
  lcd.print((int)humidity);
  lcd.print("%");

  lcd.setCursor(0, 1);
  lcd.print("CO:");
  lcd.print(mq7Value);
  lcd.print(" NH3:");
  lcd.print(mq135Value);
}

void sendDataToThingSpeak() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi disconnected. Reconnecting...");
    connectToWiFi();
    if (WiFi.status() != WL_CONNECTED) return;
  }

  ThingSpeak.setField(1, temperature);
  ThingSpeak.setField(2, humidity);
  ThingSpeak.setField(3, mq7Value);
  ThingSpeak.setField(4, mq135Value);

  int response = ThingSpeak.writeFields(channelNumber, thingSpeakApiKey);

  if (response == 200) {
    Serial.println("Data sent to ThingSpeak.");
    digitalWrite(statusLed, HIGH);
    delay(200);
    digitalWrite(statusLed, LOW);
  } else {
    Serial.printf("Failed to send data. Code: %d\n", response);
  }
}

void controlRelay() {
  if (mq7Value >= coThreshold) {
    if (!relayState) {
      digitalWrite(relayPin, RELAY_ON);
      relayState = true;
      Serial.printf("Relay ON - CO level: %d (Above threshold)\n", mq7Value);
    }
  } else {
    if (relayState) {
      digitalWrite(relayPin, RELAY_OFF);
      relayState = false;
      Serial.printf("Relay OFF - CO level: %d (Below threshold)\n", mq7Value);
    }
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(statusLed, OUTPUT);
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, RELAY_OFF);
  relayState = false;

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Poultry Monitor");
  lcd.setCursor(0, 1);
  lcd.print("Starting...");

  dht.begin();
  connectToWiFi();
  ThingSpeak.begin(client);

  Serial.println("System startup - Relay is OFF");
  delay(2000);
}

void loop() {
  unsigned long currentMillis = millis();

  updateSensorValues();
  controlRelay();  // <-- Simple and smart

  if (currentMillis - lastDisplayUpdate >= displayInterval) {
    lastDisplayUpdate = currentMillis;
    updateDisplay();
  }

  if (currentMillis - lastThingSpeakUpdate >= thingSpeakInterval) {
    lastThingSpeakUpdate = currentMillis;
    sendDataToThingSpeak();
  }

  delay(1);
}
