#include <driver/adc.h>
#include "Multiplexer.h"
#include <Arduino.h>


void Multiplexer::muxSetup() {
  // --- Initialize MUX 1 control pins ---
  pinMode(mux1S0, OUTPUT);
  pinMode(mux1S1, OUTPUT);
  pinMode(mux1S2, OUTPUT);
  pinMode(mux1S3, OUTPUT);
  digitalWrite(mux1S0, LOW);
  digitalWrite(mux1S1, LOW);
  digitalWrite(mux1S2, LOW);
  digitalWrite(mux1S3, LOW);

  // --- Initialize MUX 2 control pins ---
  pinMode(mux2S0, OUTPUT);
  pinMode(mux2S1, OUTPUT);
  pinMode(mux2S2, OUTPUT);
  pinMode(mux2S3, OUTPUT);
  digitalWrite(mux2S0, LOW);
  digitalWrite(mux2S1, LOW);
  digitalWrite(mux2S2, LOW);
  digitalWrite(mux2S3, LOW);

  // --- Configure ADC settings ---
  //adc_gpio_init(ADC_UNIT_1, ADC_CHANNEL_0);  // For GPIO1 -> ADC1_CH0
  adc1_config_width(ADC_WIDTH_BIT_13);  // 13-bit ADC (0–8191)
  adc1_config_channel_atten(MUX1_ADC1_CHANNEL, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(MUX2_ADC1_CHANNEL, ADC_ATTEN_DB_11);

  Serial.println("Setup complete.");
}

void Multiplexer::muxPrint() {
  tempCount = 0;
  pressureCount = 0;
  int j = 0; 

  // --- Read all 16 channels from MUX 1 (assumed thermistors) ---
  for (int i = 15; i >= 0; i--) {
    tempCount++;
    setMux(1, i);                // Select channel i on MUX 1
    float temp = readThermistor(1);  // Read thermistor on MUX 1
    myArray[j] = temp;
    j++;
  }

  // --- Read all 16 channels from MUX 2 ---
  // Channels 0–3: pressure sensors; Channels 4–15: thermistors.
  for (int i = 15; i >= 0; i--) {
    setMux(2, i);  // Select channel i on MUX 2
    if (i > 3) {
      tempCount++;
      float temp = readThermistor(2);
      myArray[j] = temp;
      j++;

    } else {
      pressureCount++;
      float pressure = readPressure(2);
      myArray[j] = pressure;
      j++;
    }
  }

  // Print the updated array
  Serial.println("MUX Array Values:");
  for (int i = 0; i < 32; i++) {
    Serial.print("Index ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(myArray[i], 3); // Print with 2 decimal places
  }
  
}


void Multiplexer::setMux(int mux, int channel) {
  int controlPins[4];

  if (mux == 1) {
    controlPins[0] = mux1S0;
    controlPins[1] = mux1S1;
    controlPins[2] = mux1S2;
    controlPins[3] = mux1S3;
  } else if (mux == 2) {
    controlPins[0] = mux2S0;
    controlPins[1] = mux2S1;
    controlPins[2] = mux2S2;
    controlPins[3] = mux2S3;
  } else {
    return;
  }

  // Set each control pin according to the channel number bits:
  digitalWrite(controlPins[0], (channel & 0x01) ? HIGH : LOW);
  digitalWrite(controlPins[1], (channel & 0x02) ? HIGH : LOW);
  digitalWrite(controlPins[2], (channel & 0x04) ? HIGH : LOW);
  digitalWrite(controlPins[3], (channel & 0x08) ? HIGH : LOW);
}


float Multiplexer::readPressure(int mux) {
  delay(50);
  int total = 0;
  int adcChannel;
  
  if (mux == 1)
    adcChannel = MUX1_ADC1_CHANNEL;
  else if (mux == 2)
    adcChannel = MUX2_ADC1_CHANNEL;
  else
    return -1;
  
  // Take several samples and average them.
  for (int i = 0; i < sampleSize; i++) {
    int sensorValue = adc1_get_raw((adc1_channel_t)adcChannel);
    total += sensorValue;
    delay(10);
  }
  
  int averageSensorValue = total / sampleSize;
  float outputVoltage = averageSensorValue * (vSupply / 8191.0);
  float pressureApplied = ((pMax - pMin) / (0.8 * vSupply)) * (outputVoltage - 0.10 * vSupply) + pMin;
  
  return pressureApplied;
}



float Multiplexer::readThermistor(int mux) {
  uint16_t readings[NUM_SAMPLES];
  int adcChannel;
  
  if (mux == 1)
    adcChannel = MUX1_ADC1_CHANNEL;
  else if (mux == 2)
    adcChannel = MUX2_ADC1_CHANNEL;
  else
    return -273.15;
  
  for (uint8_t j = 0; j < NUM_SAMPLES; j++) {
    readings[j] = adc1_get_raw((adc1_channel_t)adcChannel);
    delay(10);
  }
  
  float sum = 0;
  for (int k = 0; k < NUM_SAMPLES; k++) {
    sum += readings[k];
  }
  float avgADC = (sum / NUM_SAMPLES);
  
  // Avoid division by zero or log of 0
  if (avgADC < 1) {
    return -273.15;
  }
  
  float R_thermistor = SERIES_RESISTOR / ((8191.0 / avgADC) - 1);
  float steinhart = log(R_thermistor / THERMISTOR_NOMINAL) / B_COEFFICIENT;
  steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15);
  steinhart = 1.0 / steinhart;
  steinhart -= 273.15;
  
  return steinhart;
}