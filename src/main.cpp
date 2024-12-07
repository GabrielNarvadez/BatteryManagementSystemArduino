#include <Wire.h>
#include <Adafruit_INA3221.h>
#include <ArduinoJson.h>

#define PIN_LED1 3
#define PIN_LED2 4
#define PIN_LED3 5
#define PIN_SOURCE 7
#define BATTERY_1_RELAY 9
#define BATTERY_2_RELAY 10
#define BATTERY_3_RELAY 11

Adafruit_INA3221 ina3221;

float percentages[] = {0.0, 0.0, 0.0};
float voltages[] = {3.30, 4.08, 3.22};
bool status_flags[] = {false, false, false};
uint8_t relay_pins[] = {BATTERY_1_RELAY, BATTERY_2_RELAY, BATTERY_3_RELAY};
String field_names[] = {"shade_net", "ph_downer", "misting_process"};

const bool testing = false;

void printPercentages();

void controlLEDs();

float getPercentage(float voltage);

void handleBatteryAndPowerSource(float percentage, uint8_t relayPin, float &bat_voltage, bool &bat_status);

JsonDocument createBatteryPayload(int id, float percent);

void sendPayloads();

void setup() {
  Serial.begin(9600);
  Wire.begin();

  pinMode(PIN_LED1, OUTPUT);
  pinMode(PIN_LED2, OUTPUT);
  pinMode(PIN_LED3, OUTPUT);
  pinMode(BATTERY_1_RELAY, OUTPUT);
  pinMode(BATTERY_2_RELAY, OUTPUT);
  pinMode(BATTERY_3_RELAY, OUTPUT);
  pinMode(PIN_SOURCE, INPUT);

  digitalWrite(BATTERY_1_RELAY, HIGH);
  digitalWrite(BATTERY_2_RELAY, HIGH);
  digitalWrite(BATTERY_3_RELAY, HIGH);

  if (!ina3221.begin(0x41) && !testing) {
    Serial.println("Failed to initialize INA3221. Check connections.");
    while(1);
  }

  Serial.println("INA3221 Initialized");
}

void loop() {
  if (!testing) {
    for (int i = 0; i < 3; i++) {
      voltages[i] = ina3221.getBusVoltage(i);
    }
  }

  for (int i = 0; i < 3; i++) {
    percentages[i] = getPercentage(voltages[i]);
    handleBatteryAndPowerSource(percentages[i], relay_pins[i], voltages[i], status_flags[i]);
  }

  if (testing) printPercentages();

  controlLEDs();

  if (testing) {
    Serial.println("\n-----------------\n");
  } else {
    sendPayloads();
  }

  delay(10000);
}

JsonDocument createBatteryPayload(int id, float percent) {
  JsonDocument doc_bat;
  doc_bat["metric_type"] = "battery_percentage";
  doc_bat["metric_value"] = (int)percent;
  doc_bat["timestamp"] = "2024-11-30T10:30:00Z";
  doc_bat["battery_id"] = id;
  return doc_bat;
}

void handleBatteryAndPowerSource(float percentage, uint8_t relayPin, float &bat_voltage, bool &bat_status) {
  bool has_source = digitalRead(PIN_SOURCE) == HIGH;
  if (percentage >= 80.0 && has_source) {
    if (testing) {
      Serial.print("Battery relay ");
      Serial.print(relayPin);
      Serial.print(": ");
      Serial.println("OFF");
    }    
    digitalWrite(relayPin, LOW);
    bat_status = true;
  }
  else if (percentage < 80.0 && has_source && percentage > 20) {
    if (testing) {
      Serial.print("Battery relay ");
      Serial.print(relayPin);
      Serial.print(": ");
      Serial.println("ON");
      bat_voltage += .01;
    }
    digitalWrite(relayPin, HIGH);
    bat_status = false;
  }
  else if (percentage <= 20 && has_source) {
    if (testing) {
      Serial.print("Battery relay ");
      Serial.print(relayPin);
      Serial.print(": ");
      Serial.println("OFF");
      bat_voltage += .01;
    }
    digitalWrite(relayPin, LOW);
    bat_status = true;
  }
  else if (percentage <= 20) {
    if (testing) {
      Serial.print("Battery relay ");
      Serial.print(relayPin);
      Serial.print(": ");
      Serial.println("OFF");
    }
    digitalWrite(relayPin, LOW);
    bat_status = true;
  }
  else if (!has_source) {
    if (testing) {
      Serial.print("Battery relay ");
      Serial.print(relayPin);
      Serial.print(": ");
      Serial.println("ON");
      bat_voltage -= .01;
    }
    digitalWrite(relayPin, HIGH);
    bat_status = false;
  }
}

void controlLEDs() {
  float average = (percentages[0] + percentages[1] + percentages[2]) / 3.0;
  if (testing) {
    Serial.print("Average Battery Percentage: ");
    Serial.println(average);
  }

  if (average >= 80.0) {
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_LED2, HIGH);
    digitalWrite(PIN_LED3, HIGH);
  } else if (average >= 60.0) {
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_LED2, HIGH);
    digitalWrite(PIN_LED3, LOW);
  } else if (average >= 40.0) {
    digitalWrite(PIN_LED1, HIGH);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_LED3, LOW);
  } else {
    digitalWrite(PIN_LED1, LOW);
    digitalWrite(PIN_LED2, LOW);
    digitalWrite(PIN_LED3, LOW);
  }
}

float getPercentage(float voltage) {
  return (1 - (4.2 - voltage) / (4.2 - 3.0)) * 100;
}

void printPercentages() {
  Serial.print("Bat 1: ");
  Serial.print(percentages[0]);
  Serial.print("%   | V: ");
  Serial.println(voltages[0]);

  Serial.print("Bat 2: ");
  Serial.print(percentages[1]);
  Serial.print("%   | V: ");
  Serial.println(voltages[1]);

  Serial.print("Bat 3: ");
  Serial.print(percentages[2]);
  Serial.print("%   | V: ");
  Serial.println(voltages[2]);

  Serial.println();
}

void sendPayloads() {
  for (int i = 0; i < 3; i++) {
    JsonDocument doc_bat_status;
    doc_bat_status["component"] = field_names[i];
    doc_bat_status["status"] = status_flags[i] ? "OFF" : "ON";
    doc_bat_status["timestamp"] = "2024-11-30T10:30:00Z";
    serializeJson(doc_bat_status, Serial);
    Serial.println();
  }

  JsonDocument doc_system_status;
  doc_system_status["component"] = "system_status";
  doc_system_status["status"] = digitalRead(PIN_SOURCE) == HIGH ? "ON" : "OFF";
  doc_system_status["timestamp"] = "2024-11-30T10:30:00Z";
  serializeJson(doc_system_status, Serial);
  Serial.println();

  for (int i = 0; i < 3; i++) {
    JsonDocument bat_payload = createBatteryPayload(i + 1, percentages[i]);
    serializeJson(bat_payload, Serial);
    Serial.println();
  }
  Serial.println();
}
