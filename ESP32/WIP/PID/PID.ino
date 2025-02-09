/**************************************************************************
 * ESP32 PID Controller Example (Using t.Brain & t.SWB)
 *
 * In this sketch:
 *   - The measured temperature for the Peltier controller is stored in t.Brain.
 *   - The measured temperature for the Pump controller is stored in t.SWB.
 *
 * The desired setpoints are stored in:
 *   - setpoint.Brain (for the Peltier)
 *   - setpoint.SWB   (for the Pump)
 *
 * The PID outputs (which remain named "OutputPeltier" and "OutputPump")
 * are used as follows:
 *   - The Peltier output (inverted) is sent via the DAC on GPIO17.
 *   - The Pump output is mapped to a PWM frequency (0–60 Hz) that drives the
 *     PWM output on GPIO16 (using a fixed 50% duty cycle).
 *
 * The low-level ESP-IDF drivers for LEDC (PWM) and DAC are used.
 **************************************************************************/

#include <Arduino.h>
#include <PID_v1.h>
#include "driver/ledc.h"  // For low-level LEDC configuration
#include "driver/dac.h"   // For DAC output on ESP32 (GPIO17 = DAC_CHANNEL_1)

// --------------------- Data Structures for Temperatures ---------------------
// Change the type from float to double so that PID_v1 (which expects double)
// will accept these pointers.
struct Temps {
  double Brain;  // Measured temperature for the Peltier controller
  double SWB;    // Measured temperature for the Pump controller (SWB)
};

struct Setpoints {
  double Brain;  // Desired temperature for the Peltier controller
  double SWB;    // Desired temperature for the Pump controller (SWB)
};

Temps t;
Setpoints setpoint;

// --------------------- Global PID Output Variables ---------------------
double OutputPeltier = 0.0;  // PID output for the Peltier (DAC output)
double OutputPump    = 0.0;  // PID output for the Pump (mapped to frequency)

// --------------------- PID Tuning Parameters ---------------------
double Kp_peltier = 17.0, Ki_peltier = 3.0, Kd_peltier = 2.0;
double Kp_pump    = 15.0, Ki_pump    = 10.0, Kd_pump    = 0.0;

// --------------------- PID Objects ---------------------
// Use t.Brain and setpoint.Brain for the Peltier PID, and
// use t.SWB and setpoint.SWB for the Pump PID.
// (The PID objects remain named pidPeltier and pidPump.)
PID pidPeltier(&t.Brain, &OutputPeltier, &setpoint.Brain,
               Kp_peltier, Ki_peltier, Kd_peltier, DIRECT, REVERSE);

PID pidPump(&t.SWB, &OutputPump, &setpoint.SWB,
            Kp_pump, Ki_pump, Kd_pump, DIRECT, REVERSE);

// --------------------- Pump PWM (LEDC) Configuration ---------------------
#define PUMP_PWM_PIN         16                      // GPIO16 for PWM output
#define PUMP_PWM_FREQ        60                      // Initial frequency (Hz)
#define PUMP_PWM_RESOLUTION  LEDC_TIMER_8_BIT        // 8-bit resolution (0–255)
#define PUMP_PWM_CHANNEL     LEDC_CHANNEL_0          // Use channel 0 for Pump
#define PUMP_SPEED_MODE      LEDC_LOW_SPEED_MODE
#define PUMP_TIMER_NUM       LEDC_TIMER_0
#define PUMP_DUTY_50_PERCENT 128                     // 50% duty cycle

// Global variable to track the current Pump PWM frequency
int currentPumpFreq = PUMP_PWM_FREQ;

// Function to reconfigure the Pump PWM frequency using LEDC
uint32_t setPumpFrequency(uint32_t freq) {
  ledc_timer_config_t pumpTimer = {
    .speed_mode      = PUMP_SPEED_MODE,
    .duty_resolution = PUMP_PWM_RESOLUTION,
    .timer_num       = PUMP_TIMER_NUM,
    .freq_hz         = freq,
    .clk_cfg         = LEDC_AUTO_CLK
  };
  ledc_timer_config(&pumpTimer);
  return ledc_get_freq(PUMP_SPEED_MODE, PUMP_TIMER_NUM);
}

// --------------------- Sensor Input Pins (Simulated) ---------------------
#define PELTIER_SENSOR_PIN   34   // ADC input for Peltier temperature
#define PUMP_SENSOR_PIN      35   // ADC input for Pump temperature

// --------------------- Setup -------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("ESP32 PID Controller Example (Using t.Brain & t.SWB)");

  // ----- Set desired setpoints -----
  setpoint.Brain = 25.0;  // Desired brain temperature (for Peltier)
  setpoint.SWB   = 37.0;  // Desired SWB temperature (for Pump)

  // ----- Initialize DAC for Peltier output (GPIO17) -----
  dac_output_enable(DAC_CHANNEL_1);

  // ----- Setup LEDC for Pump PWM on GPIO16 -----
  ledc_timer_config_t pumpTimer = {
    .speed_mode      = PUMP_SPEED_MODE,
    .duty_resolution = PUMP_PWM_RESOLUTION,
    .timer_num       = PUMP_TIMER_NUM,
    .freq_hz         = currentPumpFreq,
    .clk_cfg         = LEDC_AUTO_CLK
  };
  ledc_timer_config(&pumpTimer);

  ledc_channel_config_t pumpChannel = {
    .gpio_num   = PUMP_PWM_PIN,
    .speed_mode = PUMP_SPEED_MODE,
    .channel    = PUMP_PWM_CHANNEL,
    .intr_type  = LEDC_INTR_DISABLE,
    .timer_sel  = PUMP_TIMER_NUM,
    .duty       = PUMP_DUTY_50_PERCENT,  // Fixed 50% duty cycle
    .hpoint     = 0
  };
  ledc_channel_config(&pumpChannel);

  // ----- Initialize PID Controllers -----
  pidPeltier.SetMode(AUTOMATIC);
  pidPeltier.SetOutputLimits(0, 255);

  pidPump.SetMode(AUTOMATIC);
  pidPump.SetOutputLimits(0, 255);
}

// --------------------- Loop -------------------------------------
void loop() {
  // ---- Simulated Sensor Readings ----
  // Replace these analogRead() calls with your actual sensor routines.
  int rawPeltier = analogRead(PELTIER_SENSOR_PIN); // returns 0–4095
  int rawPump    = analogRead(PUMP_SENSOR_PIN);    // returns 0–4095

  // Convert ADC reading to temperature.
  // (For example, assume a 0–50°C range for the ADC values.)
  t.Brain = (rawPeltier / 4095.0) * 50.0;  // measured brain temperature
  t.SWB   = (rawPump    / 4095.0) * 50.0;   // measured SWB temperature

  // ---- Run the Peltier PID Controller ----
  pidPeltier.Compute();
  // Invert the PID output (as in your original code) before sending to the DAC.
  int dacValue = 255 - (int)OutputPeltier;
  dac_output_voltage(DAC_CHANNEL_1, dacValue);

  // ---- Run the Pump PID Controller ----
  pidPump.Compute();
  // Map the PID output (0–255) to a frequency range (0–60 Hz)
  int newPumpFreq = map((int)OutputPump, 0, 255, 0, 60);
  if (newPumpFreq < 1) newPumpFreq = 1;  // ensure a minimum frequency of 1 Hz
  // Update the PWM frequency if the change is significant (e.g., ≥ 5 Hz)
  if (abs(newPumpFreq - currentPumpFreq) >= 5) {
    currentPumpFreq = newPumpFreq;
    uint32_t actualFreq = setPumpFrequency(currentPumpFreq);
    Serial.print("Updated Pump Frequency: ");
    Serial.println(actualFreq);
  }
  // Maintain a constant 50% duty cycle on the Pump PWM output
  ledcWrite(PUMP_PWM_CHANNEL, PUMP_DUTY_50_PERCENT);

  // ---- Debug Output ----
  Serial.print("Peltier (Brain) Temp: ");
  Serial.print(t.Brain, 1);
  Serial.print(" °C | Setpoint: ");
  Serial.print(setpoint.Brain, 1);
  Serial.print(" °C | PID Out: ");
  Serial.print(OutputPeltier, 1);
  Serial.print(" | DAC: ");
  Serial.println(dacValue);

  Serial.print("Pump (SWB) Temp: ");
  Serial.print(t.SWB, 1);
  Serial.print(" °C | Setpoint: ");
  Serial.print(setpoint.SWB, 1);
  Serial.print(" °C | PID Out: ");
  Serial.print(OutputPump, 1);
  Serial.print(" | Freq: ");
  Serial.println(currentPumpFreq);

  delay(1000);
}
