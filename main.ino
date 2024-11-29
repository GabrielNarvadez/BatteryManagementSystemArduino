 #include <Wire.h>
#include "DFRobot_SHT20.h"       // For SHT20
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>  // For TSL2561
#include <Adafruit_INA3221.h>    // For Adafruit INA3221
#include <RTClib.h>              // For RTC
#include <WiFiS3.h>
#include <ArduinoHttpClient.h>
#include <ArduinoJson.h>

#define RELAY_WATER_PUMP 8       // Relay for Water Pump - Channel 4
#define RELAY_MISTING 12         // Relay for Misting - Channel 32  
#define RELAY_PH_DOWNER A1       // Relay for pH Downer - Channel 2
#define RELAY_FAN A0             // Relay for Fan - Channel 1

#define TB6600_STEP_PIN 9        // Step Pin for Stepper Motor Driver
#define TB6600_DIR_PIN 10        // Direction Pin for Stepper Motor Driver
#define TB6600_ENABLE_PIN 11     // Enable Pin for Stepper Motor Driver

#define TDS_SENSOR_PIN A3        // Analog Pin for TDS Sensor
#define PH_SENSOR_PIN A2         // Analog Pin for pH Sensor
// Function to send JSON data to the server

DFRobot_SHT20 sht20;
Adafruit_TSL2561_Unified tsl = Adafruit_TSL2561_Unified(TSL2561_ADDR_FLOAT, 12345);
Adafruit_INA3221 ina3221_1;  
Adafruit_INA3221 ina3221_2;  
RTC_DS3231 rtc;

String twoDigits(int number) {
  if (number < 10) {
    return "0" + String(number);
  }
  return String(number);
}


// ===========================================================
//                      Calibration Parameters
//                 (Editable for Sensor Calibration)
// ===========================================================

// --- Reporting Interval ---
const unsigned long REPORT_INTERVAL = 10 * 60 * 1000; // 10 minutes in milliseconds

// --- Acceptable Ranges for Sensors ---
const float PH_MIN_ACCEPTABLE = 5.0;
const float PH_MAX_ACCEPTABLE = 8.0;

const float TEMP_MIN_ACCEPTABLE = 10.0;   // °C
const float TEMP_MAX_ACCEPTABLE = 50.0;   // °C

const float HUMID_MIN_ACCEPTABLE = 0.0;   // %
const float HUMID_MAX_ACCEPTABLE = 100.0; // %

const float TDS_MIN_ACCEPTABLE = 0.0;     // ppm
const float TDS_MAX_ACCEPTABLE = 2000.0;  // ppm

const float LIGHT_MIN_ACCEPTABLE = 0.0;       // lux
const float LIGHT_MAX_ACCEPTABLE = 1000000.0;  // lux

// --- Control Thresholds for Actuators ---
const float light_max = 32400; // Maximum light level to trigger shade (in lux)
const float light_min = 22400; // Minimum light level to retract shade (in lux)

const float temp_max = 30; // Maximum temperature to trigger cooling (in °C)
const float temp_min = 27; // Minimum temperature to stop cooling (in °C)

const float humid_max = 85; // Maximum humidity to stop misting (in %)
const float humid_min = 65; // Minimum humidity to start misting (in %)

const float ph_max = 7; // Maximum pH to trigger pH downer
const float ph_min = 6; // Minimum pH (not used for actuator control)

// --- Sensor Thresholds for Significant Changes ---
const float lightThreshold = 200;  // Significant change in light (in lux)
const float tempThreshold = 0.5;   // Significant change in temperature (in °C)
const float humidThreshold = 1.0;  // Significant change in humidity (in %)
const float phThreshold = 0.1;     // Significant change in pH
const float tdsThreshold = 50.0;   // Significant change in TDS (in ppm)

// --- Threshold Durations ---
const unsigned long thresholdDuration = 5000;      // Duration to confirm temp/humidity threshold (in milliseconds)
const unsigned long luxThresholdDuration = 5000;   // Duration to confirm light threshold (in milliseconds)

// --- Stepper Motor Parameters ---
const int stepDelay = 1000;       // Delay between steps (in microseconds)
const int stepsCount = 200*115;      // Number of steps to rotate
const int stepsIdeal = 200;      // Number of steps to rotate
const int rotationsCount = 115;      // Number of steps to rotate

// ===========================================================
//                        System Constants
//           (Modify with Caution - Affects System Behavior)
// ===========================================================

bool shadeNetDeployed = false; // false means retracted, true means deployed


// --- Retry Settings for Server Communication ---
const int maxRetries = 3;         // Maximum number of retries for sending data
const int retryDelay = 2000;      // Delay between retries (in milliseconds)

// --- Server Details ---
const char* serverAddress = "192.168.4.1";  // Server IP address
const int serverPort = 3001;  
const char* sensorUrl = "/api/sensor-data";
const char* componentUrl = "/api/component-status";
const char* consumptionUrl = "/api/save-metric";

// --- Access Point Credentials ---
const char* ssid = "GreenHouseManagement";
// ===========================================================
//                          State Variables
//             (Do Not Edit - Used for Program Logic)
// ===========================================================

// --- Timing Variables ---
unsigned long lastReportTime = 0;

unsigned long tempAboveMaxStartTime = 0;
unsigned long tempBelowMinStartTime = 0;
unsigned long humidBelowMinStartTime = 0;

unsigned long luxAboveMaxStartTime = 0;
unsigned long luxBelowMinStartTime = 0;

// --- Flags for Misting Control ---
bool mistingDueToTemp = false;
bool mistingDueToHumid = false;
bool mistingActive = false; // Tracks the state of the misting system

// --- Variables to Store Last Sent Sensor Values ---
float lastLightLevel = -1;
float lastTemperature = -1;
float lastHumidity = -1;
float lastPH = -1;
float lastTDS = -1;

void setupWifi() {
  Serial.print("Connecting to Raspberry Pi AP: ");
  Serial.println(ssid);

  WiFi.begin(ssid);

  int retries = 0;
  while (WiFi.status() != WL_CONNECTED && retries < 50) {
    delay(500);
    Serial.print(".");
    retries++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to Raspberry Pi AP.");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nFailed to connect to Raspberry Pi AP.");
    while (true); // Halt if Wi-Fi connection fails
  }
}

void setupSensors(){
  Serial.println("Initializing sensors and displaying I2C addresses...");

  sht20.initSHT20();
  delay(100);
  sht20.checkSHT20();
  Serial.println("SHT20 initialized.");

  if (!tsl.begin()) {
    Serial.println("TSL2561 not found.");
  } else {
    tsl.enableAutoRange(true);
    tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
    Serial.println("TSL2561 initialized.");
  }

  if (!ina3221_1.begin(0x41)) {
    Serial.println("INA3221_1 not found.");
  } else {
    Serial.println("INA3221_1 initialized.");
  }

  if (!ina3221_2.begin(0x43)) {
    Serial.println("INA3221_2 not found.");
  } else {
    Serial.println("INA3221_2 initialized.");
  }

  if (!rtc.begin()) {
    Serial.println("RTC not found.");
  } else {
    Serial.println("RTC DS3231 initialized.");
  }

  Serial.println("Initialization complete.\n");

  pinMode(TB6600_STEP_PIN, OUTPUT);
  pinMode(TB6600_DIR_PIN, OUTPUT);
  pinMode(TB6600_ENABLE_PIN, OUTPUT);

  // Enable the stepper motor driver
  digitalWrite(TB6600_ENABLE_PIN, HIGH); // LOW to enable (varies with driver)
}











void setupActuators(){
  pinMode(RELAY_WATER_PUMP , OUTPUT);
  digitalWrite(RELAY_WATER_PUMP , HIGH); // Ensure relay is off initiall

  pinMode(RELAY_MISTING , OUTPUT);
  digitalWrite(RELAY_MISTING , HIGH); // Ensure relay is off initiall

  pinMode(RELAY_PH_DOWNER, OUTPUT);
  digitalWrite(RELAY_PH_DOWNER, HIGH); // Ensure relay is off initiall

  pinMode(RELAY_FAN , OUTPUT);
  digitalWrite(RELAY_FAN , HIGH); // Ensure relay is off initiall
}

void setup() {
  Serial.begin(9600);
  Serial.println("Setting things up...");
  Wire.begin();
  
  Serial.println("Trying to connect to wifi...");
  // Ensure Wi-Fi connectivity first
  setupWifi();

  // Initialize other systems after Wi-Fi is established
  setupSensors();
  setupActuators();

  Serial.println("System Initialized. Sending initial report...");
  reportData();
  shadeNetDeployed = false; // Assuming the shade net is retracted at startup

  lastReportTime = millis();
}


// Function to send JSON data to the server
void sendSensorData(String sensor, String output, String parameter, float value) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi not connected, cannot send data");
    return;
  }

  // Get the current timestamp
  DateTime now = rtc.now();
  String timestamp = String(now.year()) + "-" +
                     twoDigits(now.month()) + "-" +
                     twoDigits(now.day()) + "T" +
                     twoDigits(now.hour()) + ":" +
                     twoDigits(now.minute()) + ":" +
                     twoDigits(now.second()) + "Z";

  // Create JSON payload
  StaticJsonDocument<512> jsonDoc;
  jsonDoc["sensor"] = sensor;
  jsonDoc["output"] = output;
  jsonDoc["parameter"] = parameter;
  jsonDoc["value"] = value;
  jsonDoc["timestamp"] = timestamp;

    // Handle the value based on the sensor type
  if (sensor == "light_sensor") {
    // For light sensor, cast value to integer to remove decimals
    jsonDoc["value"] = (int)value;
  } else {
    // For other sensors, keep the value as is
    jsonDoc["value"] = value;
  }

  // Convert JSON document to string
  String jsonPayload;
  serializeJson(jsonDoc, jsonPayload);

  // Print JSON payload for debugging
  Serial.print("Sending JSON Payload: ");
  Serial.println(jsonPayload);

  // Create a new WiFiClient object
  WiFiClient wifiClientLocal;
  HttpClient client(wifiClientLocal, serverAddress, serverPort);

  // Retry logic
  int retries = 0;
  while (retries < maxRetries) {
    client.beginRequest();
    client.post(sensorUrl);
    client.sendHeader("Content-Type", "application/json");
    client.sendHeader("Content-Length", jsonPayload.length());
    client.beginBody();
    client.print(jsonPayload);
    client.endRequest();

    int statusCode = client.responseStatusCode();
    if (statusCode > 0) {
      Serial.print("Data sent successfully. Response code: ");
      Serial.println(statusCode);
      client.stop();
      return; // Exit function if successful
    } else {
      Serial.print("Error sending data. Response code: ");
      Serial.println(statusCode);
      retries++;
      client.stop(); // Close connection before retrying
      delay(retryDelay); // Wait before retrying
    }
  }

  Serial.println("Failed to send data after maximum retries.");
  client.stop();
}





void sendComponentStatus(String component, String status) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi not connected, cannot send data");
    return;
  }

  // Get the current timestamp
  DateTime now = rtc.now();
  String timestamp = String(now.year()) + "-" +
                     twoDigits(now.month()) + "-" +
                     twoDigits(now.day()) + "T" +
                     twoDigits(now.hour()) + ":" +
                     twoDigits(now.minute()) + ":" +
                     twoDigits(now.second()) + "Z";

  // Create JSON payload
  StaticJsonDocument<200> jsonDoc;
  jsonDoc["component"] = component;
  jsonDoc["status"] = status;
  jsonDoc["timestamp"] = timestamp;

  // Convert JSON document to string
  String jsonPayload;
  serializeJson(jsonDoc, jsonPayload);

  // Print JSON payload for debugging
  Serial.print("Sending JSON Payload: ");
  Serial.println(jsonPayload);

  // Create a new WiFiClient object
  WiFiClient wifiClientLocal;
  HttpClient client(wifiClientLocal, serverAddress, serverPort);

  int retries = 0;
  while (retries < maxRetries) {
    client.beginRequest();
    client.post(componentUrl);
    client.sendHeader("Content-Type", "application/json");
    client.sendHeader("Content-Length", jsonPayload.length());
    client.beginBody();
    client.print(jsonPayload);
    client.endRequest();

    int statusCode = client.responseStatusCode();
    if (statusCode > 0) {
      Serial.print("Component status sent successfully. Response code: ");
      Serial.println(statusCode);
      client.stop(); // Close the connection
      return; // Exit function if successful
    } else {
      Serial.print("Error sending component status. Response code: ");
      Serial.println(statusCode);
      retries++;
      client.stop(); // Close connection before retrying
      delay(retryDelay);
    }
  }

  Serial.println("Failed to send component status after maximum retries.");
  client.stop(); // Ensure the connection is closed
}



void sendMetricData(String metricType, String metricValue) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Wi-Fi not connected, cannot send data");
    return;
  }

  // Get current timestamp
  DateTime now = rtc.now();
    String timestamp = String(now.year()) + "-" +
                     twoDigits(now.month()) + "-" +
                     twoDigits(now.day()) + "T" +
                     twoDigits(now.hour()) + ":" +
                     twoDigits(now.minute()) + ":" +
                     twoDigits(now.second()) + "Z";

  // Create JSON payload
  StaticJsonDocument<512> jsonDoc;
  jsonDoc["metric_type"] = metricType;
  jsonDoc["metric_value"] = metricValue;
  jsonDoc["timestamp"] = timestamp;

  // Convert JSON document to string
  String jsonPayload;
  serializeJson(jsonDoc, jsonPayload);

  // Print JSON payload to Serial for debugging
  Serial.print("Sending JSON Payload: ");
  Serial.println(jsonPayload);

  // Create a new WiFiClient object
  WiFiClient wifiClientLocal;
  HttpClient client(wifiClientLocal, serverAddress, serverPort);

  int retries = 0;
  while (retries < maxRetries) {
    client.beginRequest();
    client.post(consumptionUrl); // Specify the endpoint
    client.sendHeader("Content-Type", "application/json");
    client.sendHeader("Content-Length", jsonPayload.length());
    client.beginBody();
    client.print(jsonPayload); // Send JSON payload
    client.endRequest();

    int statusCode = client.responseStatusCode();
    String response = client.responseBody();

    if (statusCode > 0) {
      Serial.print("Metric data sent, response code: ");
      Serial.println(statusCode);
      Serial.print("Server response: ");
      Serial.println(response);
      client.stop(); // Close the connection
      return; // Exit function if successful
    } else {
      Serial.print("Error sending metric data. Response code: ");
      Serial.println(statusCode);
      retries++;
      client.stop(); // Close connection before retrying
      delay(retryDelay);
    }
  }

  Serial.println("Failed to send metric data after maximum retries.");
  client.stop(); // Ensure the connection is closed
}


// Example usage functions
void sendBatteryPercentage(float percentage) {
  sendMetricData("battery_percentage", String(percentage));
}

void sendHarvestedPower(float power) {
  sendMetricData("harvested_power", String(power));
}

void sendSystemPowerConsumption(String status) {
  sendMetricData("system_power_consumption", status);
}

void sendWaterPumpConsumption(String status) {
  sendMetricData("water_pump_consumption", status);
}

void sendMistingConsumption(String status) {
  sendMetricData("misting_consumption", status);
}

void sendShadeNetConsumption(String status) {
  sendMetricData("shade_net_consumption", status);
}

//component Data Sending
void sendTempData(float temperature) {
  sendSensorData("temp", "Temperature Sensor Output", "celsius", temperature);
}

void sendHumidityData(float humidity) {
  sendSensorData("humidity", "Humidity Sensor Output", "%", humidity);
}

void sendTDSData(float tds) {
  sendSensorData("tds", "TDS Sensor Output", "ppm", tds);
}

void sendLightSensorData(float lightLevel) {
  sendSensorData("light_sensor", "Light Sensor Output", "lm", lightLevel);
}

void sendPHData(float phValue) {
  sendSensorData("ph_sensor", "pH Sensor Output", "pH", phValue);
}

//componentStatus Sending Data
void sendShadeNetStatus(String status) {
  sendComponentStatus("shade_net", status);
}

void sendPHDownerStatus(String status) {
  sendComponentStatus("ph_downer", status);
}

void sendMistingProcessStatus(String status) {
  sendComponentStatus("misting_process", status);
}

void sendSystemStatus(String status) {
  sendComponentStatus("system_status", status);
}


// Control misting system for temperature and humidity
void controlMisting(float temperature, float humidity) {
  // Activate misting if temperature exceeds max or humidity is below min
  if ((temperature > temp_max || humidity < humid_min) && !mistingActive) {
    Serial.println("Activating Misting System due to temperature or humidity thresholds.");
    digitalWrite(RELAY_MISTING, LOW); // Turn on misting system
    mistingActive = true;
  }
  // Deactivate misting if temperature falls below min or humidity exceeds max
  else if ((temperature < temp_min || humidity > humid_max) && mistingActive) {
    Serial.println("Deactivating Misting System.");
    digitalWrite(RELAY_MISTING, HIGH); // Turn off misting system
    mistingActive = false;
  }
}


// Control pH downer system
void controlPHDowner(float phValue) {
  if (phValue > ph_max) {
    Serial.println("Activating pH Downer.");
    sendPHDownerStatus("ON");
    digitalWrite(RELAY_PH_DOWNER, LOW);
    delay(3000); // Keep it active for 3 seconds
    digitalWrite(RELAY_PH_DOWNER, HIGH);
    sendPHDownerStatus("OFF");
    delay(3000); // Keep it active for 3 seconds
  }
  // No need to track phDownerActive unless it's required for additional logic
}




void checkTemperatureAndHumidity(float temperature, float humidity, float tempMin, float tempMax, float humidMin, float humidMax) {
  // Handle temperature
  if (temperature > tempMax) {
    Serial.println("Temperature: TOO HIGH. Activating Misting System.");
    controlMisting(temperature, humidity);
  } else if (temperature < tempMin) {
    Serial.println("Temperature: TOO LOW. No action taken.");
  } else {
    Serial.println("Temperature: NORMAL.");
  }

  // Handle humidity
  if (humidity < humidMin) {
    Serial.println("Humidity: TOO LOW. Activating Misting System.");
    controlMisting(temperature, humidity);
  } else if (humidity > humidMax) {
    Serial.println("Humidity: TOO HIGH. Stopping Misting System.");
    digitalWrite(RELAY_MISTING, HIGH); // Directly stop misting
    mistingActive = false;
  } else {
    Serial.println("Humidity: NORMAL.");
  }
}



void rotateStepperForward(int rotationsCount, int stepsIdeal, int stepDelay) {
  Serial.println("Rotating stepper motor forward...");

  // Enable the stepper driver
  digitalWrite(TB6600_ENABLE_PIN, LOW); // Enable the driver (active LOW)

  // Set direction to forward
  digitalWrite(TB6600_DIR_PIN, HIGH);
  delay(1); // Allow some time for the direction change to take effect

  int stepsCount = rotationsCount * stepsIdeal; // Total steps to achieve desired rotations

  for (int i = 0; i < stepsCount; i++) {
    digitalWrite(TB6600_STEP_PIN, HIGH); // Generate step pulse
    delayMicroseconds(stepDelay);       // Step pulse ON duration
    digitalWrite(TB6600_STEP_PIN, LOW); // End step pulse
    delayMicroseconds(stepDelay);       // Step pulse OFF duration
  }

  // Disable the stepper driver
  digitalWrite(TB6600_ENABLE_PIN, HIGH); // Disable the driver (active HIGH)

  Serial.println("Stepper motor forward rotation complete.");
}


void rotateStepperBackward(int rotationsCount, int stepsIdeal, int stepDelay) {
  Serial.println("Rotating stepper motor backward...");

  // Enable the stepper driver (active LOW)
  digitalWrite(TB6600_ENABLE_PIN, LOW);

  // Set direction to backward
  digitalWrite(TB6600_DIR_PIN, LOW);
  delay(1); // Short delay for direction to settle

  int stepsCount = rotationsCount * stepsIdeal; // Total steps for desired rotations

  for (int i = 0; i < stepsCount; i++) {
    digitalWrite(TB6600_STEP_PIN, HIGH); // Generate step pulse
    delayMicroseconds(stepDelay);        // Step pulse duration
    digitalWrite(TB6600_STEP_PIN, LOW);  // End step pulse
    delayMicroseconds(stepDelay);        // Pause between pulses
  }

  // Disable the stepper driver (active HIGH)
  digitalWrite(TB6600_ENABLE_PIN, HIGH);

  Serial.println("Stepper motor backward rotation complete.");
}





void readTemp() {
  float temperature = sht20.readTemperature();
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  sendTempData(temperature);  
}

void readHumidity() {
  float humidity = sht20.readHumidity();
  Serial.print("Humidity: ");
  Serial.print(humidity);
  Serial.println(" %");
  sendHumidityData(humidity);

}

void readLight() {
  sensors_event_t event;
  tsl.getEvent(&event);
  if (event.light) {
    Serial.print("Light: ");
    Serial.print(event.light);
    Serial.println(" lux");
    sendLightSensorData(event.light);    
  } else {
    Serial.println("TSL2561 measurement error.");
  }
}

void readPH() {
  float analogValue = analogRead(PH_SENSOR_PIN);
  float phValue = (analogValue * 14.0) / 1024.0; // Example conversion
  Serial.print("pH: ");
  Serial.println(phValue);
  sendPHData(phValue);
}

void readTDS() {
  float analogValue = analogRead(TDS_SENSOR_PIN);
  float tdsValue = (analogValue * 500.0) / 1024.0; // Example conversion for TDS (adjust as per your sensor's datasheet)
  Serial.print("TDS: ");
  Serial.print(tdsValue);
  Serial.println(" ppm"); // TDS is typically measured in ppm
  sendTDSData(tdsValue);
}

void readBatteryAndSolarConsumption() {
  Serial.println("Battery and Solar Consumption Readings (INA3221_1):");

  // Define acceptable voltage range for battery
  const float BATTERY_VOLTAGE_MIN = 11.0; // Minimum acceptable voltage
  const float BATTERY_VOLTAGE_MAX = 14.0; // Maximum acceptable voltage

  // Define channel names
  const char* channels[] = {"Solar Panel", "Battery", "Load"};

  for (int i = 0; i < 3; i++) {
    float busVoltage = ina3221_1.getBusVoltage(i);
    float currentAmps = ina3221_1.getCurrentAmps(i);
    float powerWatts = busVoltage * currentAmps; // Calculate wattage

    Serial.print(channels[i]);
    Serial.print(" - ");

    if (strcmp(channels[i], "Solar Panel") == 0) {
      // Send harvested power data
      Serial.print("Harvested Power: ");
      Serial.print(powerWatts);
      Serial.println(" W");
      sendHarvestedPower(powerWatts);
    } else if (strcmp(channels[i], "Battery") == 0) {
      // Check if battery voltage is within acceptable range
      if (busVoltage < BATTERY_VOLTAGE_MIN || busVoltage > BATTERY_VOLTAGE_MAX) {
        Serial.print("Abnormal battery voltage detected: ");
        Serial.println(busVoltage);
        // Do not send data
        continue;
      }

      Serial.print("Voltage: ");
      Serial.print(busVoltage, 2);
      Serial.print(" V, ");
      Serial.print("Power: ");
      Serial.print(powerWatts);
      Serial.println(" W");

      // Calculate battery percentage based on voltage
      int batteryPercentage = 0;

      if (busVoltage >= 12.70) {
        batteryPercentage = 100;
      } else if (busVoltage >= 12.40) {
        batteryPercentage = 75;
      } else if (busVoltage >= 12.20) {
        batteryPercentage = 50;
      } else if (busVoltage >= 12.00) {
        batteryPercentage = 25;
      } else if (busVoltage >= 11.80) {
        batteryPercentage = 0;
      } else {
        batteryPercentage = 0;
        Serial.println("Warning: Battery voltage is critically low!");
      }

      Serial.print("Battery Percentage: ");
      Serial.print(batteryPercentage);
      Serial.println("%");

      // Send battery percentage
      sendBatteryPercentage(batteryPercentage);
    } else if (strcmp(channels[i], "Load") == 0) {
      Serial.print("System Power Consumption: ");
      Serial.print(powerWatts);
      Serial.println(" W");
      sendSystemPowerConsumption(String(powerWatts));
    }
  }
}




void readActuatorConsumption() {
  Serial.println("Actuator Consumption Readings (INA3221_2):");

  // Define acceptable power range (example values)
  const float POWER_MIN_ACCEPTABLE = 0.0;  // Minimum acceptable power in Watts
  const float POWER_MAX_ACCEPTABLE = 100.0; // Maximum acceptable power in Watts

  // Define channel names
  const char* channels[] = {"Water Pump", "Misting System", "Fan"};

  for (int i = 0; i < 3; i++) {
    float busVoltage = ina3221_2.getBusVoltage(i);
    float currentAmps = ina3221_2.getCurrentAmps(i);
    float powerWatts = busVoltage * currentAmps; // Calculate wattage

    // Check if power consumption is within acceptable range
    if (powerWatts < POWER_MIN_ACCEPTABLE || powerWatts > POWER_MAX_ACCEPTABLE) {
      Serial.print("Abnormal power consumption detected on ");
      Serial.print(channels[i]);
      Serial.print(": ");
      Serial.println(powerWatts);
      // Do not send data
      continue;
    }

    Serial.print(channels[i]);
    Serial.print(" - Power: ");
    Serial.print(powerWatts);
    Serial.println(" W");

    if (strcmp(channels[i], "Water Pump") == 0) {
      sendWaterPumpConsumption(String(powerWatts));
    } else if (strcmp(channels[i], "Misting System") == 0) {
      sendMistingConsumption(String(powerWatts));
    } else if (strcmp(channels[i], "Fan") == 0) {
      sendShadeNetConsumption(String(powerWatts));
    }
  }
}




void handleAboveMaxLight() {

    if (shadeNetDeployed) {
    // Shade net is already deployed; no action needed.
    return;
  }

  if (luxAboveMaxStartTime == 0) {
    luxAboveMaxStartTime = millis();
    Serial.println("Light level above maximum threshold. Starting timer.");
  } else if (millis() - luxAboveMaxStartTime >= luxThresholdDuration) {
    Serial.println("Light level has been above maximum for 5 seconds. Rotating stepper forward.");
    
    // Actuator actions
    sendShadeNetStatus("ON");
    rotateStepperForward(115, 200, 500);
    sendShadeNetStatus("OFF");

        // Update the state variable
    shadeNetDeployed = true;
    
    // Reset timer
    luxAboveMaxStartTime = 0;
  }
  luxBelowMinStartTime = 0; // Reset the below-minimum timer
}

void handleBelowMinLight() {
      if (!shadeNetDeployed) {
    // Shade net is already deployed; no action needed.
    return;
  }
  if (luxBelowMinStartTime == 0) {
    luxBelowMinStartTime = millis();
    Serial.println("Light level below minimum threshold. Starting timer.");
  } else if (millis() - luxBelowMinStartTime >= luxThresholdDuration) {
    Serial.println("Light level has been below minimum for 5 seconds. Rotating stepper backward.");
    
    // Actuator actions
    sendShadeNetStatus("ON");
    rotateStepperBackward(115, 200, 500);
    sendShadeNetStatus("OFF");

        // Update the state variable
    shadeNetDeployed = false;
    
    // Reset timer
    luxBelowMinStartTime = 0;
  }
  luxAboveMaxStartTime = 0; // Reset the above-maximum timer
}

void resetTimers() {
  if (luxAboveMaxStartTime != 0 || luxBelowMinStartTime != 0) {
    Serial.println("Light level returned to acceptable range. Timers reset.");
  }
  luxAboveMaxStartTime = 0;
  luxBelowMinStartTime = 0;
}


// Update the processLightSensor() function
void processLightSensor() {
  sensors_event_t event;
  tsl.getEvent(&event);

  if (event.light) {
    float lightValue = event.light;

    // Check if light is above or below thresholds
    if (lightValue > light_max) {
      handleAboveMaxLight();
    } else if (lightValue < light_min) {
      handleBelowMinLight();
    } else {
      resetTimers();
    }

    // Send data if significant change is detected
    if (abs(lightValue - lastLightLevel) >= lightThreshold) {
      Serial.print("Significant change in light detected. Sending: ");
      Serial.println(lightValue);
      sendLightSensorData(lightValue);
      lastLightLevel = lightValue;
    }
  } else {
    Serial.println("TSL2561 measurement error.");
  }
}



void controlMisting(bool turnOn) {
  if (turnOn) {
    if (!mistingActive) {
      Serial.println("Activating Misting System.");
      sendMistingProcessStatus("ON");
      digitalWrite(RELAY_MISTING, LOW); // Turn on misting system
      mistingActive = true;
    }
  } else {
    if (mistingActive) {
      sendMistingProcessStatus("OFF");
      Serial.println("Deactivating Misting System.");
      digitalWrite(RELAY_MISTING, HIGH); // Turn off misting system
      mistingActive = false;
    }
  }
}


// Read and process temperature and humidity sensors
void processTempAndHumidity() {
  float temperature = sht20.readTemperature();
  float humidity = sht20.readHumidity();

  // Check if temperature is within acceptable range
  if (temperature < TEMP_MIN_ACCEPTABLE || temperature > TEMP_MAX_ACCEPTABLE) {
    Serial.print("Abnormal temperature reading detected: ");
    Serial.println(temperature);
    // Do not send data or proceed with control logic

      // Turn off actuators related to temperature control
  controlMisting(false); // Ensure misting system is off
  sendMistingProcessStatus("OFF"); // Update status if necessary

    return;
  }

  // Check if humidity is within acceptable range
  if (humidity < HUMID_MIN_ACCEPTABLE || humidity > HUMID_MAX_ACCEPTABLE) {
    Serial.print("Abnormal humidity reading detected: ");
    Serial.println(humidity);
    // Do not send data or proceed with control logic

      // Turn off actuators related to humidity control
  controlMisting(false); // Ensure misting system is off
  sendMistingProcessStatus("OFF"); // Update status if necessary

    return;
  }

  // --- Temperature Control Logic ---
  if (temperature > temp_max) {
    if (tempAboveMaxStartTime == 0) {
      tempAboveMaxStartTime = millis();
      Serial.println("Temperature above maximum threshold. Starting timer.");
    } else if (millis() - tempAboveMaxStartTime >= thresholdDuration && !mistingDueToTemp) {
      Serial.println("Temperature has been above maximum for 5 seconds. Activating misting due to temperature.");
      mistingDueToTemp = true;
      controlMisting(true);
    }
    tempBelowMinStartTime = 0;
  } else if (temperature < temp_min) {
    if (tempBelowMinStartTime == 0) {
      tempBelowMinStartTime = millis();
      Serial.println("Temperature below minimum threshold. Starting timer.");
    } else if (millis() - tempBelowMinStartTime >= thresholdDuration && mistingDueToTemp) {
      Serial.println("Temperature has been below minimum for 5 seconds. Deactivating misting due to temperature.");
      mistingDueToTemp = false;
      controlMisting(mistingDueToHumid);
    }
    tempAboveMaxStartTime = 0;
  } else {
    if (tempAboveMaxStartTime != 0 || tempBelowMinStartTime != 0) {
      Serial.println("Temperature returned to normal range. Timers reset.");
    }
    tempAboveMaxStartTime = 0;
    tempBelowMinStartTime = 0;
  }

  // --- Humidity Control Logic ---
  if (humidity < humid_min) {
    if (humidBelowMinStartTime == 0) {
      humidBelowMinStartTime = millis();
      Serial.println("Humidity below minimum threshold. Starting timer.");
    } else if (millis() - humidBelowMinStartTime >= thresholdDuration && !mistingDueToHumid) {
      Serial.println("Humidity has been below minimum for 5 seconds. Activating misting due to humidity.");
      mistingDueToHumid = true;
      controlMisting(true);
    }
  } else {
    if (humidBelowMinStartTime != 0) {
      Serial.println("Humidity returned to normal range. Timer reset.");
    }
    humidBelowMinStartTime = 0;
    if (mistingDueToHumid) {
      Serial.println("Deactivating misting due to humidity.");
      mistingDueToHumid = false;
      controlMisting(mistingDueToTemp);
    }
  }

  // --- Data Transmission Logic ---
  if (abs(temperature - lastTemperature) >= tempThreshold) {
    Serial.print("Significant change in temperature detected. Sending: ");
    Serial.println(temperature);
    sendTempData(temperature);
    lastTemperature = temperature;
  }

  if (abs(humidity - lastHumidity) >= humidThreshold) {
    Serial.print("Significant change in humidity detected. Sending: ");
    Serial.println(humidity);
    sendHumidityData(humidity);
    lastHumidity = humidity;
  }
}



// Read and process pH sensor
void processPHSensor() {
  float analogValue = analogRead(PH_SENSOR_PIN);
  float phValue = (analogValue * 14.0) / 1024.0; // Example conversion

  Serial.print("Analog Value: ");
  Serial.println(analogValue);
  Serial.print("pH Value: ");
  Serial.println(phValue);

  // Check if pH value is within acceptable range
  if (phValue < PH_MIN_ACCEPTABLE || phValue > PH_MAX_ACCEPTABLE) {
    Serial.print("Abnormal pH reading detected: ");
    Serial.println(phValue);

    // Turn off pH Downer actuator
    digitalWrite(RELAY_PH_DOWNER, HIGH); // Ensure pH downer is off
    Serial.println("pH Downer turned OFF.");
    sendPHDownerStatus("OFF"); // Update status if necessary
    return;
  }

  // Control actuators based on thresholds
  Serial.println("Controlling pH Downer...");
  controlPHDowner(phValue);

  // Send data if significant change is detected
  if (abs(phValue - lastPH) >= phThreshold) {
    Serial.print("Significant change in pH detected. Sending: ");
    Serial.println(phValue);
    sendPHData(phValue);
    lastPH = phValue;
  }
}





// Read and process TDS sensor
void processTDSSensor() {
  float analogValue = analogRead(TDS_SENSOR_PIN);
  float tdsValue = (analogValue * 500.0) / 1024.0; // Example conversion

  // Check if TDS value is within acceptable range
  if (tdsValue < TDS_MIN_ACCEPTABLE || tdsValue > TDS_MAX_ACCEPTABLE) {
    Serial.print("Abnormal TDS reading detected: ");
    Serial.println(tdsValue);
    // Do not send data
    return;
  }

  if (abs(tdsValue - lastTDS) >= tdsThreshold) {
    Serial.print("Significant change in TDS detected. Sending: ");
    Serial.println(tdsValue);
    sendTDSData(tdsValue);
    lastTDS = tdsValue;
  }
}


// Read battery and actuator consumption
void processBatteryAndActuators() {
  readBatteryAndSolarConsumption();
  readActuatorConsumption();
}

// Non-blocking delay function
unsigned long lastReadTime = 0;
const unsigned long sensorReadInterval = 2000; // 2 seconds

void readRTC() {
  DateTime now = rtc.now();
  Serial.print("Date: ");
  Serial.print(now.year(), DEC);
  Serial.print('/');
  Serial.print(now.month(), DEC);
  Serial.print('/');
  Serial.print(now.day(), DEC);
  Serial.print(" Time: ");
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.println();
}

void separator() {
  Serial.println("-----------------------------");
}

void reportData() {

  readRTC();
  separator();
  readLight();
  readTemp();
  readPH();
  readHumidity();
  readTDS();
  separator();
  readBatteryAndSolarConsumption();
  readActuatorConsumption();
  separator();
  sendSystemStatus("ON");
}

void loop() {
  unsigned long currentTime = millis();

  // Check sensor readings and process every sensorReadInterval (2 seconds)
  if (currentTime - lastReadTime >= sensorReadInterval) {
    lastReadTime = currentTime;

    // Process all sensors
    processLightSensor();
    processTempAndHumidity();
    processPHSensor();
    processTDSSensor();

    // Optionally, check actuator consumption periodically
    processBatteryAndActuators();
  }

  // Report data every REPORT_INTERVAL (10 minutes)
  if (currentTime - lastReportTime >= REPORT_INTERVAL) {
    Serial.println("Periodic Report:");
    reportData();
    lastReportTime = currentTime;
  }

   static unsigned long previousMillis = 0;
  static bool pumpState = false;  // Tracks whether the pump is ON or OFF
  
  unsigned long currentMillis = millis();
  DateTime now = rtc.now();
  
  // Calculate minutes since the hour
  int minutesSinceHour = now.minute() % 60;
  
  if (!pumpState && minutesSinceHour == 0) {
    // Turn ON the pump at the start of the 55-minute cycle
    digitalWrite(RELAY_WATER_PUMP, LOW);
    pumpState = true;
    previousMillis = currentMillis;  // Start the timer
  }
  
  if (pumpState && currentMillis - previousMillis >= 5 * 60 * 1000) {
    // Turn OFF the pump after 5 minutes
    digitalWrite(RELAY_WATER_PUMP, HIGH);
    pumpState = false;
  }


}