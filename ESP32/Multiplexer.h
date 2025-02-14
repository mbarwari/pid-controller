#pragma once
#define MUX1_ADC1_CHANNEL ADC1_CHANNEL_0  // for MUX1 (GPIO1)
#define MUX2_ADC1_CHANNEL ADC1_CHANNEL_1  // for MUX2 (GPIO2)
#define NUM_SAMPLES 10
#define SERIES_RESISTOR 10000.0      // 10K resistor in voltage divider
#define B_COEFFICIENT 3892.0         // Thermistor B-coefficient
#define THERMISTOR_NOMINAL 10000.0   // Thermistor nominal resistance at 25°C
#define TEMPERATURE_NOMINAL 25.0     // Nominal temperature in °C

class Multiplexer{
  private:
    const int mux1S0 = 33;
    const int mux1S1 = 26;
    const int mux1S2 = 21;
    const int mux1S3 = 20;
    const int mux2S0 = 41;
    const int mux2S1 = 40;
    const int mux2S2 = 39;
    const int mux2S3 = 38;
    const float vSupply = 3.3;   
    const float pMin = 0.0;      
    const float pMax = 15.0;     
    const int sampleSize = 10;  
    int tempCount = 0;
    int pressureCount = 0;
    float correctionfactor = 1;
    float myArray[32] = {0.0};
  public: 
    void muxSetup();
    void muxPrint();
    void setMux(int mux, int channel);
    float readPressure(int mux);
    float readThermistor(int mux);
}; 
