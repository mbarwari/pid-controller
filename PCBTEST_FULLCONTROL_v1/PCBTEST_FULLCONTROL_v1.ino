/*******************************************************
  Combined ESP32-S2/S3 Example with WiFi WPA2-Enterprise and ADC Calibration,
  with non-blocking INA260 sensor initialization and a continuously updated webpage.
  
  Changes:
    1. DAC output is now controlled via a slider (0 to 255, step 10) and an update button.
    2. PWM frequency is controlled via a slider (0 to 60 Hz, step 10) and an update button.
  
  Endpoints:
    "/"       - Serves an HTML page that continuously fetches JSON sensor data and provides PWM/DAC controls.
    "/data"   - Returns JSON sensor/output data.
    "/setPWM" - Accepts a "value" parameter to set the PWM frequency.
    "/setDAC" - Accepts a "value" parameter to set the DAC output.
********************************************************/

#include <Arduino.h>
#include <Wire.h>

// ================= I2C Sensors =================
#include <Adafruit_INA260.h>
Adafruit_INA260 currentSensor1, currentSensor2;
uint8_t inaAddress1 = 0x40;  // INA260 sensor #1
uint8_t inaAddress2 = 0x41;  // INA260 sensor #2
bool ina1Found = false;
bool ina2Found = false;

#include <SensirionI2cSf06Lf.h>
SensirionI2cSf06Lf sensor;

// ================= PWM and DAC =================
#include "driver/ledc.h"  // PWM (LEDC)
#include "driver/dac.h"   // DAC on ESP32-S2/S3

// ================= ADC & MUX =================
#include <driver/adc.h>
#include <esp_adc_cal.h>  // ADC calibration
#include <math.h>

// ================= WiFi and HTTP Server =================
#include <WiFi.h>
#include <WebServer.h>

// ----- WPA2-Enterprise Credentials -----
#define EAP_IDENTITY "s206427"
#define EAP_USERNAME "s206427"
#define EAP_PASSWORD "AlmondsFozzy2014"
const char* ssid = "utwpa2";

// Create an HTTP server on port 80
WebServer server(80);

// ================= PWM Configuration (GPIO16) =================
#define PWM_PIN         16
#define PWM_FREQ        60                // Default initial frequency (Hz)
#define PWM_RESOLUTION  LEDC_TIMER_8_BIT  // 8-bit resolution
#define PWM_CHANNEL     LEDC_CHANNEL_0
#define SPEED_MODE      LEDC_LOW_SPEED_MODE
#define TIMER_NUM       LEDC_TIMER_0
#define DUTY_50_PERCENT 128               // Fixed 50% duty cycle

// ================= DAC Configuration (GPIO17) =================
int dacValue = 0;  // Controlled via slider (0 to 255)

// ================= MUX + ADC Configuration =================
// MUX1 is connected to ADC1_CHANNEL_0 and MUX2 to ADC1_CHANNEL_1
#define MUX1_ADC1_CHANNEL ADC1_CHANNEL_0
#define MUX2_ADC1_CHANNEL ADC1_CHANNEL_1

// MUX1 control pins
const int mux1S0 = 33;
const int mux1S1 = 26;
const int mux1S2 = 21;
const int mux1S3 = 20;

// MUX2 control pins
const int mux2S0 = 41;
const int mux2S1 = 40;
const int mux2S2 = 39;
const int mux2S3 = 38;

// Pressure sensor parameters
const float vSupply = 3.3;
const float pMin = 0.0;
const float pMax = 15.0;
const int sampleSize = 10;

// Thermistor parameters
#define NUM_SAMPLES           10
#define SERIES_RESISTOR       10000.0  // 10k ohm resistor
#define B_COEFFICIENT         3892.0
#define THERMISTOR_NOMINAL    10000.0  // at 25°C
#define TEMPERATURE_NOMINAL   25.0     // °C

// ================= ADC Calibration =================
#define DEFAULT_VREF    1100  // in mV
esp_adc_cal_characteristics_t adc1_chars;

// ========== Function Declarations ==========
void initPWM();
uint32_t setFrequency(uint32_t freq);
void initDAC();           // Now simply sets initial DAC state
void updateDACOutput();   // Writes current dacValue to DAC

void initMuxPins();
void setMux(int muxIndex, int channel);
float readPressure(int muxIndex);
float readThermistor(int muxIndex);

String getAllData();
void handleAllData();
void handleRoot();
void handleSetPWM();
void handleSetDAC();

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }
  Serial.println("\n=== Combined ESP32-S2/S3 Example with WiFi WPA2-Enterprise and ADC Calibration ===");

  // ----------- WiFi WPA2-Enterprise Setup -----------
  Serial.println("Connecting to WPA2-Enterprise WiFi...");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(1000);
  WiFi.begin(ssid, WPA2_AUTH_PEAP, EAP_IDENTITY, EAP_USERNAME, EAP_PASSWORD);
  
  int counter = 0;
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    counter++;
    if (counter >= 60) {
      Serial.println("\nConnection timed out. Restarting...");
      ESP.restart();
    }
  }
  Serial.println();
  Serial.print("Connected! IP address: ");
  Serial.println(WiFi.localIP());
  
  // ----------- HTTP Server Setup -----------
  server.on("/", handleRoot);
  server.on("/data", handleAllData);
  server.on("/setPWM", handleSetPWM);
  server.on("/setDAC", handleSetDAC);
  server.begin();
  Serial.println("HTTP server started on port 80.");

  // ----------- I2C and Sensor Setup -----------
  Wire.begin();
  
  ina1Found = currentSensor1.begin(inaAddress1, &Wire);
  if (ina1Found)
    Serial.println("Found INA260 sensor #1.");
  else
    Serial.println("Couldn't find INA260 sensor #1! Continuing...");
  
  ina2Found = currentSensor2.begin(inaAddress2, &Wire);
  if (ina2Found)
    Serial.println("Found INA260 sensor #2.");
  else
    Serial.println("Couldn't find INA260 sensor #2! Continuing...");
  
  sensor.begin(Wire, SLF3C_1300F_I2C_ADDR_08);
  delay(100);
  sensor.startH2oContinuousMeasurement();
  Serial.println("Sensirion SF06 initialized.");

  // ----------- PWM and DAC Setup -----------
  initPWM();
  initDAC(); // Initializes DAC with current dacValue

  // ----------- MUX Pins Setup -----------
  initMuxPins();

  // ----------- ADC Calibration Setup -----------
  adc1_config_width(ADC_WIDTH_BIT_13);
  adc1_config_channel_atten(MUX1_ADC1_CHANNEL, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(MUX2_ADC1_CHANNEL, ADC_ATTEN_DB_11);
  esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_11,
                           ADC_WIDTH_BIT_13, DEFAULT_VREF, &adc1_chars);

  Serial.println("Setup complete.\n");
}

// ================= LOOP =================
void loop() {
  server.handleClient();
  // Note: We no longer update the DAC output automatically.
  // PWM frequency and DAC output are updated only when you press the buttons.

  // ----------- INA260 Sensor Readings -----------
  float ina1_current = 0.0, ina1_busVoltage = 0.0, ina1_power = 0.0;
  if (ina1Found) {
    ina1_current = currentSensor1.readCurrent();
    ina1_busVoltage = currentSensor1.readBusVoltage() / 1000.0;
    ina1_power = currentSensor1.readPower();
  }
  Serial.print("[INA260 #1] Current (mA): ");
  Serial.print(ina1_current);
  Serial.print("  Bus Voltage (V): ");
  Serial.print(ina1_busVoltage);
  Serial.print("  Power (mW): ");
  Serial.println(ina1_power);

  float ina2_current = 0.0, ina2_busVoltage = 0.0, ina2_power = 0.0;
  if (ina2Found) {
    ina2_current = currentSensor2.readCurrent();
    ina2_busVoltage = currentSensor2.readBusVoltage() / 1000.0;
    ina2_power = currentSensor2.readPower();
  }
  Serial.print("[INA260 #2] Current (mA): ");
  Serial.print(ina2_current);
  Serial.print("  Bus Voltage (V): ");
  Serial.print(ina2_busVoltage);
  Serial.print("  Power (mW): ");
  Serial.println(ina2_power);

  // ----------- Sensirion SF06 Reading -----------
  float sf06_flow = 0.0, sf06_temperature = 0.0;
  uint16_t sf06_flags = 0;
  sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3C_1300F,
                               sf06_flow, sf06_temperature, sf06_flags);
  Serial.print("[SF06] Flow (ml/min): ");
  Serial.print(sf06_flow);
  Serial.print("  Temp (C): ");
  Serial.print(sf06_temperature);
  Serial.print("  Flags: ");
  Serial.println(sf06_flags);

  // ----------- MUX1 Readings (Thermistors) -----------
  Serial.println("=== MUX1 (Thermistors) ===");
  for (int ch = 0; ch < 16; ch++) {
    setMux(1, ch);
    float tempC = readThermistor(1);
    Serial.print(" MUX1 Ch");
    Serial.print(ch);
    Serial.print(" => ");
    Serial.print(tempC, 2);
    Serial.println(" °C");
    delay(10);
  }

  // ----------- MUX2 Readings (Pressure 0-3, Thermistors 4-15) -----------
  Serial.println("=== MUX2 (Pressure 0-3, Thermistors 4-15) ===");
  for (int ch = 0; ch < 16; ch++) {
    setMux(2, ch);
    if (ch <= 3) {
      float p = readPressure(2);
      Serial.print(" MUX2 Ch");
      Serial.print(ch);
      Serial.print(" => ");
      Serial.print(p, 2);
      Serial.println(" PSI");
    } else {
      float tempC = readThermistor(2);
      Serial.print(" MUX2 Ch");
      Serial.print(ch);
      Serial.print(" => ");
      Serial.print(tempC, 2);
      Serial.println(" °C");
    }
    delay(10);
  }
  
  Serial.println("----- End of cycle -----\n");
  delay(1000);
}

// ================= HTTP Server Handlers =================

// Root page: displays sensor data and provides sliders with push buttons for PWM and DAC control.
void handleRoot() {
  String page = "<!DOCTYPE html><html><head><title>ESP32 Sensor Data</title>";
  page += "<style>";
  page += "body { font-family: Arial, sans-serif; margin: 20px; }";
  page += "pre { background: #f4f4f4; padding: 10px; }";
  page += "input { width: 300px; margin: 10px 0; }";
  page += "</style></head><body>";
  page += "<h1>ESP32 Sensor Data</h1>";
  page += "<pre id='data'>Loading data...</pre>";
  page += "<h2>Control PWM Frequency</h2>";
  page += "<input type='range' id='pwmSlider' min='0' max='60' step='10' value='" + String(PWM_FREQ) + "'>";
  page += "<p>PWM Frequency: <span id='pwmValue'>" + String(PWM_FREQ) + "</span> Hz</p>";
  page += "<button id='updatePWM'>Update PWM</button>";
  page += "<h2>Control DAC Output</h2>";
  page += "<input type='range' id='dacSlider' min='0' max='255' step='10' value='" + String(dacValue) + "'>";
  page += "<p>DAC Value: <span id='dacValue'>" + String(dacValue) + "</span></p>";
  page += "<button id='updateDAC'>Update DAC</button>";
  page += "<script>";
  // Function to fetch sensor data every second
  page += "function fetchData() {";
  page += "  fetch('/data').then(response => response.json()).then(data => {";
  page += "    document.getElementById('data').innerText = JSON.stringify(data, null, 2);";
  page += "  }).catch(err => {";
  page += "    document.getElementById('data').innerText = 'Error: ' + err;";
  page += "  });";
  page += "}";
  page += "fetchData();";
  page += "setInterval(fetchData, 1000);"; // Update every second

  // PWM update button event
  page += "document.getElementById('updatePWM').addEventListener('click', function() {";
  page += "  var val = document.getElementById('pwmSlider').value;";
  page += "  document.getElementById('pwmValue').innerText = val;";
  page += "  fetch('/setPWM?value=' + val);";
  page += "});";

  // DAC update button event
  page += "document.getElementById('updateDAC').addEventListener('click', function() {";
  page += "  var val = document.getElementById('dacSlider').value;";
  page += "  document.getElementById('dacValue').innerText = val;";
  page += "  fetch('/setDAC?value=' + val);";
  page += "});";

  page += "</script></body></html>";
  server.send(200, "text/html", page);
}

// Returns JSON sensor and output data.
void handleAllData() {
  String data = getAllData();
  server.send(200, "application/json", data);
}

// Sets the PWM frequency based on the "value" parameter.
void handleSetPWM() {
  if (server.hasArg("value")) {
    int newFreq = server.arg("value").toInt();
    uint32_t actualFreq = setFrequency(newFreq);
    Serial.printf("PWM frequency set to: %d Hz (actual: %d Hz)\n", newFreq, actualFreq);
  }
  server.send(204);
}

// Sets the DAC output based on the "value" parameter.
void handleSetDAC() {
  if (server.hasArg("value")) {
    dacValue = server.arg("value").toInt();
    updateDACOutput();
    Serial.printf("DAC value set to: %d\n", dacValue);
  }
  server.send(204);
}

// ================= PWM Functions =================
void initPWM() {
  ledc_timer_config_t timer_conf = {
    .speed_mode = SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = TIMER_NUM,
    .freq_hz = PWM_FREQ,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);

  ledc_channel_config_t channel_conf = {
    .gpio_num = PWM_PIN,
    .speed_mode = SPEED_MODE,
    .channel = PWM_CHANNEL,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = TIMER_NUM,
    .duty = DUTY_50_PERCENT,
    .hpoint = 0
  };
  ledc_channel_config(&channel_conf);

  uint32_t actualFreq = ledc_get_freq(SPEED_MODE, TIMER_NUM);
  Serial.printf("PWM on GPIO%d at ~%d Hz (50%% duty cycle)\n", PWM_PIN, actualFreq);
}

uint32_t setFrequency(uint32_t freq) {
  ledc_timer_config_t timer_conf = {
    .speed_mode = SPEED_MODE,
    .duty_resolution = PWM_RESOLUTION,
    .timer_num = TIMER_NUM,
    .freq_hz = freq,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer_conf);
  return ledc_get_freq(SPEED_MODE, TIMER_NUM);
}

// ================= DAC Functions =================
void initDAC() {
  // Initialize DAC output with current dacValue
  dac_output_enable(DAC_CHANNEL_1);
  updateDACOutput();
  Serial.printf("DAC on GPIO17 initialized with value %d\n", dacValue);
}

void updateDACOutput() {
  dac_output_voltage(DAC_CHANNEL_1, dacValue);
}

// ================= MUX & ADC Functions =================
void initMuxPins() {
  // Initialize MUX1 control pins
  pinMode(mux1S0, OUTPUT); pinMode(mux1S1, OUTPUT);
  pinMode(mux1S2, OUTPUT); pinMode(mux1S3, OUTPUT);
  digitalWrite(mux1S0, LOW); digitalWrite(mux1S1, LOW);
  digitalWrite(mux1S2, LOW); digitalWrite(mux1S3, LOW);

  // Initialize MUX2 control pins
  pinMode(mux2S0, OUTPUT); pinMode(mux2S1, OUTPUT);
  pinMode(mux2S2, OUTPUT); pinMode(mux2S3, OUTPUT);
  digitalWrite(mux2S0, LOW); digitalWrite(mux2S1, LOW);
  digitalWrite(mux2S2, LOW); digitalWrite(mux2S3, LOW);
}

void setMux(int muxIndex, int channel) {
  int controlPins[4];
  if (muxIndex == 1) {
    controlPins[0] = mux1S0; controlPins[1] = mux1S1;
    controlPins[2] = mux1S2; controlPins[3] = mux1S3;
  } else {
    controlPins[0] = mux2S0; controlPins[1] = mux2S1;
    controlPins[2] = mux2S2; controlPins[3] = mux2S3;
  }
  digitalWrite(controlPins[0], (channel & 0x01) ? HIGH : LOW);
  digitalWrite(controlPins[1], (channel & 0x02) ? HIGH : LOW);
  digitalWrite(controlPins[2], (channel & 0x04) ? HIGH : LOW);
  digitalWrite(controlPins[3], (channel & 0x08) ? HIGH : LOW);
}

float readPressure(int muxIndex) {
  adc1_channel_t adcCh = (muxIndex == 1) ? MUX1_ADC1_CHANNEL : MUX2_ADC1_CHANNEL;
  uint32_t total = 0;
  for (int i = 0; i < sampleSize; i++) {
    total += adc1_get_raw(adcCh);
    delay(1);
  }
  uint32_t averageRaw = total / sampleSize;
  uint32_t mV = esp_adc_cal_raw_to_voltage(averageRaw, &adc1_chars);
  float measuredVoltage = mV / 1000.0f;
  float pressure = ((pMax - pMin) / (0.8f * vSupply))
                   * (measuredVoltage - (0.10f * vSupply)) + pMin;
  return pressure;
}

float readThermistor(int muxIndex) {
  adc1_channel_t adcCh = (muxIndex == 1) ? MUX1_ADC1_CHANNEL : MUX2_ADC1_CHANNEL;
  uint32_t sumRaw = 0;
  for (int i = 0; i < NUM_SAMPLES; i++) {
    sumRaw += adc1_get_raw(adcCh);
    delay(1);
  }
  uint32_t avgRaw = sumRaw / NUM_SAMPLES;
  uint32_t mV = esp_adc_cal_raw_to_voltage(avgRaw, &adc1_chars);
  float thermVoltage = mV / 1000.0f;
  if (thermVoltage <= 0.0f || thermVoltage >= (vSupply - 0.01f))
    return -273.15f;  // sensor error or out-of-range
  float R_thermistor = SERIES_RESISTOR * (thermVoltage / (vSupply - thermVoltage));
  float steinhart = log(R_thermistor / THERMISTOR_NOMINAL) / B_COEFFICIENT;
  steinhart += 1.0f / (TEMPERATURE_NOMINAL + 273.15f);
  steinhart = 1.0f / steinhart - 273.15f;
  return steinhart;
}

// ================= HTTP Server Data Function =================
String getAllData() {
  String data = "{";

  // INA260 Sensor #1 data
  data += "\"INA260_1\":{";
  if (ina1Found) {
    data += "\"current\":" + String(currentSensor1.readCurrent()) + ",";
    data += "\"bus_voltage\":" + String(currentSensor1.readBusVoltage() / 1000.0, 3) + ",";
    data += "\"power\":" + String(currentSensor1.readPower());
  } else {
    data += "\"current\":0.0,\"bus_voltage\":0.0,\"power\":0.0";
  }
  data += "},";

  // INA260 Sensor #2 data
  data += "\"INA260_2\":{";
  if (ina2Found) {
    data += "\"current\":" + String(currentSensor2.readCurrent()) + ",";
    data += "\"bus_voltage\":" + String(currentSensor2.readBusVoltage() / 1000.0, 3) + ",";
    data += "\"power\":" + String(currentSensor2.readPower());
  } else {
    data += "\"current\":0.0,\"bus_voltage\":0.0,\"power\":0.0";
  }
  data += "},";

  // SF06 Sensor data
  float sf06_flow = 0.0, sf06_temperature = 0.0;
  uint16_t sf06_flags = 0;
  sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3C_1300F,
                               sf06_flow, sf06_temperature, sf06_flags);
  data += "\"SF06\":{";
  data += "\"flow\":" + String(sf06_flow, 2) + ",";
  data += "\"temperature\":" + String(sf06_temperature, 2) + ",";
  data += "\"flags\":" + String(sf06_flags);
  data += "},";

  // PWM Data
  uint32_t pwm_freq = ledc_get_freq(SPEED_MODE, TIMER_NUM);
  data += "\"PWM\":{";
  data += "\"frequency\":" + String(pwm_freq) + ",";
  data += "\"duty\":" + String(DUTY_50_PERCENT);
  data += "},";

  // DAC Data
  data += "\"DAC\":{";
  data += "\"value\":" + String(dacValue) + ",";
  data += "\"step_size\":10,";
  data += "\"direction\":\"N/A\"";
  data += "},";

  // MUX1 Data (Thermistors)
  data += "\"MUX1\":{";
  for (int ch = 0; ch < 16; ch++) {
    setMux(1, ch);
    float tempC = readThermistor(1);
    data += "\"Ch" + String(ch) + "\":" + String(tempC, 2);
    if (ch < 15) data += ",";
  }
  data += "},";

  // MUX2 Data (Channels 0-3: Pressure, 4-15: Thermistors)
  data += "\"MUX2\":{";
  for (int ch = 0; ch < 16; ch++) {
    setMux(2, ch);
    if (ch <= 3) {
      float p = readPressure(2);
      data += "\"Ch" + String(ch) + "\":" + String(p, 2);
    } else {
      float tempC = readThermistor(2);
      data += "\"Ch" + String(ch) + "\":" + String(tempC, 2);
    }
    if (ch < 15) data += ",";
  }
  data += "}";
  
  data += "}";
  return data;
}
