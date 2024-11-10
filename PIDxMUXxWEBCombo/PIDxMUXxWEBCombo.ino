/*
  TODO: add feature - adjust setpoint for waterblock (done) and device via webpage 
*/

/*
  TODO: add feature - output daq voltage and pwm feq   
*/

/*
  TODO: add feature - temp rejection 
    1. Setup with only sensors that are functioning
    2. check each iteration of the temp readings with the last iteration to check if there is a large difference. 
       if there is a large difference then that specific temp sensor should be considered faulty. include that in log file and then do not displauy that specific sensor(s)
    3. make sure this is noted in our log.txt file 
*/

/*
  TODO: add feature - flow sensor failure - Done
    1. our flow sensors output both flow rate and temp so if we get both temp and flow then the sensor works. if we dont then consider it faulty
    Have it so that if both the flow rate and temp are the default 0.0, then set a variable to one. When the flow sensor detection happens, and if the variable is set to 0, it goes through, if 1, skip it
    Do the same for both sensors
    - Done, needs testing

*/



/*
Aberrant temp rejection code
Setup with only sensors that are functioning
Dac output peltier (at 5v scale)

*/

/*
fix file naming issue - mahabad 
*/


// include necessary libraries
#include <Wire.h>
#include <WiFi.h>
#include <Arduino.h>
#include <SensirionI2cSf06Lf.h>
#include <Adafruit_INA260.h>
#include <SPI.h>
#include <SD.h>
#include <PID_v1.h>
#include <mbed.h>
#include <mbed_mktime.h>
#include <AdvancedDAC.h>
#include <Arduino_AdvancedAnalog.h>
#include "WebDashboard.h"

//Define global variables for webdashboard and .h files
float WBTemp_atm;
float braintemp_atm;
float aFlow = 0.0;
float bFlow = 0.0;
float aTemperature = 0.0;
float bTemperature = 0.0;
float pumpVoltage;
float pumpCurrent;
float peltierVoltage;
float peltierCurrent;
float pressureApplied1;
float pressureApplied2;
int aFlow_bad;
int bFlow_bad;

// Define the flow sensor objects
SensirionI2cSf06Lf flowSensorA;
SensirionI2cSf06Lf flowSensorB;

// Define current sensor objects and I2C addresses
Adafruit_INA260 pumpINA260 = Adafruit_INA260();
Adafruit_INA260 peltierINA260 = Adafruit_INA260();
uint8_t pumpI2CAddress = 0x40;
uint8_t peltierI2CAddress = 0x41;

// Pressure sensor specifications
const float vSupply = 3.3;  // Supply voltage
const float pMin = 0.0;     // Minimum pressure in PSI
const float pMax = 15.0;    // Maximum pressure in PSI

// Thermistor related macros
#define SERIESRESISTOR 10000
#define BCOEFFICIENT 3895
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25

// General global variables
unsigned long waqt;
int tempCount, pressureCount, flowCount, currentCount, noneCount;
const int sampleSize = 10;  // Sample size
const int NUMSAMPLES = 10;
File myFile;  //Initialization of SD card reader
#define DACPIN A12
int decimalPlaces = 3;

// WIFI related global variables, macros, and object
#define SECRET_SSID "test1"
#define SECRET_PASS "123456789"
char ssid[] = SECRET_SSID;  // your network SSID (name)
char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
int status = WL_IDLE_STATUS;
WiFiServer server(80);  //Keep 80 if connecting from a webpage

//Multiplexer Pin Numbers
// MUX1 control pins, S0-S3 (digital pins)
int mux1S0 = 9;
int mux1S1 = 8;
int mux1S2 = 7;
int mux1S3 = 6;

// MUX2 control pins, S0-S3 (digital pins)
int mux2S0 = 5;
int mux2S1 = 4;
int mux2S2 = 3;
int mux2S3 = 2;

// MUX1 and MUX2 signal pins, SIG (analog pins)
int mux1Sig = 0;
int mux2Sig = 1;

//Initializing PID loops
//Peltier PID Loop
double SetpointPeltier, InputPeltierPID, OutputPeltierPID;                                                                      //Generating the input variables for the peltier plate cooling PID -- Setpoint is goal; input is what is detected; output is PID output 0-255
double Kp_peltier = 17, Ki_peltier = 3, Kd_peltier = 2;                                                                         //Specify the initial tuning parameters. Ignoring Kd since not used often. -- watch youtube video: https://www.youtube.com/watch?v=IB1Ir4oCP5k
PID myPID_peltier(&InputPeltierPID, &OutputPeltierPID, &SetpointPeltier, Kp_peltier, Ki_peltier, Kd_peltier, DIRECT, REVERSE);  //By default, PID "warms" up, so we have to reverse it -- unknown what DIRECT is for

//Water Block PID Loop
double SetpointWB, InputWB, OutputWB;  // setpoint is desired temp, input is current temp, output is 0-255
double Kp_WB = 15, Ki_WB = 10, Kd_WB = 0;
PID myPID_WB(&InputWB, &OutputWB, &SetpointWB, Kp_WB, Ki_WB, Kd_WB, DIRECT, REVERSE);
int freq = 0;

//Temperature controls for PID loops
//These are parameters that can be adjusted for temperature cutoffs -- TempIdeals are equivalent to Setpoint
int BrainTempIdeal = 25;  //Ideal brain temperature, currently 12 for testing purposes (should be 25)
int WBTempMax = 50;       //The maximum peltier plate temperature, currently 25 for testing purposes (should be 40)
int WBTempIdeal = -60;     //The ideal temperature of the WB, currently 22 for testing purposes (should be 35)

//These are vectors to save analog sensor data. Total of 32 of sensors
uint16_t samples1[NUMSAMPLES];
uint16_t samples2[NUMSAMPLES];
uint16_t samples3[NUMSAMPLES];
uint16_t samples4[NUMSAMPLES];
uint16_t samples5[NUMSAMPLES];
uint16_t samples6[NUMSAMPLES];
uint16_t samples7[NUMSAMPLES];
uint16_t samples8[NUMSAMPLES];
uint16_t samples9[NUMSAMPLES];
uint16_t samples10[NUMSAMPLES];
uint16_t samples11[NUMSAMPLES];
uint16_t samples12[NUMSAMPLES];
uint16_t samples13[NUMSAMPLES];
uint16_t samples14[NUMSAMPLES];
uint16_t samples15[NUMSAMPLES];
uint16_t samples16[NUMSAMPLES];
uint16_t samples17[NUMSAMPLES];
uint16_t samples18[NUMSAMPLES];
uint16_t samples19[NUMSAMPLES];
uint16_t samples20[NUMSAMPLES];
uint16_t samples21[NUMSAMPLES];
uint16_t samples22[NUMSAMPLES];
uint16_t samples23[NUMSAMPLES];
uint16_t samples24[NUMSAMPLES];
uint16_t samples25[NUMSAMPLES];
uint16_t samples26[NUMSAMPLES];
uint16_t samples27[NUMSAMPLES];
uint16_t samples28[NUMSAMPLES];
uint16_t samples29[NUMSAMPLES];
uint16_t samples30[NUMSAMPLES];
uint16_t samples31[NUMSAMPLES];
uint16_t samples32[NUMSAMPLES];

//Relay Pin Placement
int peltierRelay = 14;
int pumpRelay = 15;

//Max Water Block Temp and Freq Countdown info
int startTime = 0;

//PWM Frequency Generator Variables
float myfreq;                      // Desired frequency
float desperiod;                   // Desired period in seconds
float previousFreq = 0.0;          // Previous frequency to track changes
const float freqThreshold = 10.0;  // Frequency change threshold and interval (e.g., 10 Hz)
const float hysteresis = 5.0;      // Hysteresis margin (e.g., 5 Hz to prevent frequent toggling)
PinName pin = digitalPinToPinName(D11); //This is for the water block controller. Can be any digital pin, make changes to correct pin here
mbed::PwmOut* pwm = new mbed::PwmOut(pin);

//File variables 
String data_filename = "";
String log_filename = "";

//Forward declarations
void setupWiFiAP();
int setMux(int mux, int channel);
String getLocaltime();


void setup() {

  // put your setup code here, to run once:
  Serial.begin(9600);

  //while (!Serial) {};
  //Turning on Multiplexer and Initializing Pins
  // set the control pins of MUX1 (S0-S3) to be output pins
  // meaning the control pins will be used to select which one of the 16 channels to read from
  pinMode(mux1S0, OUTPUT);
  pinMode(mux1S1, OUTPUT);
  pinMode(mux1S2, OUTPUT);
  pinMode(mux1S3, OUTPUT);

  // set the initial state of MUX1 (S0-S3) control pins to LOW (0V)
  // this initializes the multiplexer to connect to channel 0.
  digitalWrite(mux1S0, LOW);
  digitalWrite(mux1S1, LOW);
  digitalWrite(mux1S2, LOW);
  digitalWrite(mux1S3, LOW);

  // set the control pins of MUX2 (S0-S3) to be output pins
  // meaning the control pins will be used to select which one of the 16 channels to read from
  pinMode(mux2S0, OUTPUT);
  pinMode(mux2S1, OUTPUT);
  pinMode(mux2S2, OUTPUT);
  pinMode(mux2S3, OUTPUT);

  // set the initial state of MUX2 (S0-S3) control pins to LOW (0V)
  // this initializes the multiplexer to connect to channel 0.  digitalWrite(mux2_s0, LOW);
  digitalWrite(mux2S0, LOW);
  digitalWrite(mux2S1, LOW);
  digitalWrite(mux2S2, LOW);
  digitalWrite(mux2S3, LOW);

  //Setting up I2C Devices
  Serial.println("I2C setup: ");

  Wire.begin();
  flowSensorA.begin(Wire, SLF3S_4000B_I2C_ADDR_08);
  Wire1.begin();
  flowSensorB.begin(Wire1, SLF3S_4000B_I2C_ADDR_08);
  delay(100);
  flowSensorA.startH2oContinuousMeasurement();
  flowSensorB.startH2oContinuousMeasurement();

  //Starting up Current and Voltage Sensors
  if (!pumpINA260.begin(pumpI2CAddress, &Wire1)) {
    Serial.println("Couldn't find pump current sensor");
    myFile.println("Pump INA not found");
  }
  Serial.println("Found pump current sensor");

  if (!peltierINA260.begin(peltierI2CAddress, &Wire1)) {
    Serial.println("Couldn't find peltier current sensor");
    myFile.println("Peltier INA not found");
  }
  Serial.println("Found peltier current sensor");
  Serial.println();


  //Setting up WiFi
  Serial.println("WIFI setup: ");

  // set up the WiFi Access Point
  setupWiFiAP();

  //Set up Web interface
  setupWebDashboard(server);  // Use the existing server object

  //Initializing SD Card
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    myFile.println("SD card not found");
  }
  Serial.println("initialization done.");

  data_filename += getLocaltime();
  data_filename += "_DATA.txt";

  log_filename += getLocaltime();
  log_filename += "_LOG.txt";


  //Opening the file we want to save in
  myFile = SD.open("testing.txt", FILE_WRITE);  //Change file name here
  //The line below establishes column headers for save file. Always seperate with comma when adding new ones*/
  myFile.println("Time,IntraArray1 (C),IntraArray2 (C),IntraArray3 (C),IntraArray4 (C),ExtraArray1 (C),ExtraArray2 (C),ExtraArray3 (C),ScalpWB1 (C),ScalpWB2 (C),ScalpWB3 (C),EntrSWB1 (C),EntrSWB2 (C),EntrSWB3 (C),ExitSWB1 (C),ExitSWB2 (C),ExitSWB3 (C),EntrBWB1 (C),EntrBWB2 (C),EntrBWB3 (C),BWB1 (C),BWB2 (C),BWB3 (C),BWB4 (C),BWB5 (C),BWB6 (C),ExitBWB1 (C),ExitBWB2 (C),ExitBWB3 (C),Peltier Current (A),Peltier Voltage (V),Pump Current (A),Pump Voltage (V),Flow Sensor 1 (mL/min),Flow Sensor 2 (mL/min),Pressure1 (psi),Pressure2 (psi),Pressure3 (psi),Pressure4 (psi),P,I,D,SetpointPeltier Temp,WiFI Client Status (0 = USB, 1 = WiFi)");
  myFile.close();

  //Turning on Power to Pump and Peltier
  pinMode(peltierRelay, OUTPUT);
  pinMode(pumpRelay, OUTPUT);
  digitalWrite(peltierRelay, LOW);  //For a normally open circuit and these relay specs, low means relay on and high means relay off.
  digitalWrite(pumpRelay, LOW);

  //Initializing PWM pin at 60 hz initially
  pwm->period(1.0 / 60.0);  // Initial period for 60 Hz frequency
  pwm->write(0.5f);         // 50% duty cycle

  //Turning PID on
  SetpointPeltier = BrainTempIdeal;
  myPID_peltier.SetMode(AUTOMATIC);
  SetpointWB = WBTempIdeal;
  myPID_WB.SetMode(AUTOMATIC);

  //Initializing DAC
  pinMode(DACPIN, OUTPUT);
}



void loop() {
  // put your main code here, to run repeatedly:

  //Determining if Wifi is Connected or Not
  if (status != WiFi.status()) {
    // WIFI status has changed so update status variable
    status = WiFi.status();

    if (status == WL_AP_CONNECTED) {
      // a device has connected to the AP
      Serial.println("Device connected to AP");
      myFile.println("Device connected to AP");
    } else {
      // a device has disconnected from the AP, and we are back in listening mode
      Serial.println("Device disconnected from AP");
      myFile.println("Device disconnected from AP");
    }
  }

  //Establishing Variable for printing depending on connection or not
  WiFiClient client = server.available();
  int clientStatus;
  if (client) {
    // if you get a client, set clientStatus to 1, if not, set clientStatus to 0
    clientStatus = 1;
  } 
  else {
    clientStatus = 0;
  }

  //Getting all analog sensor data at once
  uint8_t i;
  waqt = millis() / 1000;
  for (i = 0; i < NUMSAMPLES; i++) {
    //Intraarray Temperatures
    int intraArray1 = setMux(1, 6);
    samples1[i] = analogRead(intraArray1);
    int intraArray2 = setMux(1, 7);
    samples2[i] = analogRead(intraArray2);
    int intraArray3 = setMux(1, 8);
    samples3[i] = analogRead(intraArray3);
    int intraArray4 = setMux(1, 9);
    samples4[i] = analogRead(intraArray4);

    //Extraarray Temperatures
    int extraArray1 = setMux(1, 3);
    samples5[i] = analogRead(extraArray1);
    int extraArray2 = setMux(1, 4);
    samples6[i] = analogRead(extraArray2);
    int extraArray3 = setMux(1, 5);
    samples7[i] = analogRead(extraArray3);

    //Scalp Water Block (SWB) Temperatures
    int SWB1 = setMux(1, 0);
    samples8[i] = analogRead(SWB1);
    int SWB2 = setMux(1, 1);
    samples9[i] = analogRead(SWB2);
    int SWB3 = setMux(1, 2);
    samples10[i] = analogRead(SWB3);

    /*
    //Scalp Water Block Entrance (entrSWB) Temperatures
    int entrSWB1 = setMux(2, 0);
    samples11[i] = analogRead(entrSWB1);
    int entrSWB2 = setMux(2, 1);
    samples12[i] = analogRead(entrSWB2);
    int entrSWB3 = setMux(2, 2);
    samples13[i] = analogRead(entrSWB3);

    //Scalp Water Block Exit (exitSWB) Temperatures
    int exitSWB1 = setMux(2, 3);
    samples14[i] = analogRead(exitSWB1);
    int exitSWB2 = setMux(2, 4);
    samples15[i] = analogRead(exitSWB2);
    int exitSWB3 = setMux(2, 5);
    samples16[i] = analogRead(exitSWB3);
    */

    //Body Water Block Entrance (entrBWB) Temperatures
    int entrBWB1 = setMux(2, 0);
    samples17[i] = analogRead(entrBWB1);
    int entrBWB2 = setMux(2, 1);
    samples18[i] = analogRead(entrBWB2);
    int entrBWB3 = setMux(2, 2);
    samples19[i] = analogRead(entrBWB3);

    //Body Water Block (BWB) Temperatures
    int BWB1 = setMux(2, 3);
    samples20[i] = analogRead(BWB1);
    int BWB2 = setMux(2, 4);
    samples21[i] = analogRead(BWB2);
    int BWB3 = setMux(2, 5);
    samples22[i] = analogRead(BWB3);
    int BWB4 = setMux(2, 6);
    samples23[i] = analogRead(BWB4);
    int BWB5 = setMux(2, 7);
    samples24[i] = analogRead(BWB5);
    int BWB6 = setMux(2, 8);
    samples25[i] = analogRead(BWB6);

    //Body Water Block Exit (exitBWB) Temperatures
    int exitBWB1 = setMux(2, 9);
    samples26[i] = analogRead(exitBWB1);
    int exitBWB2 = setMux(2, 10);
    samples27[i] = analogRead(exitBWB2);
    int exitBWB3 = setMux(2, 11);
    samples28[i] = analogRead(exitBWB3);

    //Pressure Sensors
    int pressureSensor1 = setMux(2, 14);
    samples29[i] = analogRead(pressureSensor1); 
    int pressureSensor2 = setMux(2, 15);
    samples30[i] = analogRead(pressureSensor2);
    /*
    int pressureSensor3 = setMux(2, 14);
    samples31[i] = analogRead(pressureSensor3);
    int pressureSensor4 = setMux(2, 15);
    samples32[i] = analogRead(pressureSensor4);
    */
    delay(10);
  }

  //Average each sensors data
  float avgIntraArray1;
  float avgIntraArray2;
  float avgIntraArray3;
  float avgIntraArray4;
  float avgExtraArray1;
  float avgExtraArray2;
  float avgExtraArray3;
  float avgSWB1;
  float avgSWB2;
  float avgSWB3;
  float avgEntrSWB1;
  float avgEntrSWB2;
  float avgEntrSWB3;
  float avgExitSWB1;
  float avgExitSWB2;
  float avgExitSWB3;
  float avgEntrBWB1;
  float avgEntrBWB2;
  float avgEntrBWB3;
  float avgBWB1;
  float avgBWB2;
  float avgBWB3;
  float avgBWB4;
  float avgBWB5;
  float avgBWB6;
  float avgExitBWB1;
  float avgExitBWB2;
  float avgExitBWB3;
  float avgPressureSensor1;
  float avgPressureSensor2;

  avgIntraArray1 = 0;
  avgIntraArray2 = 0;
  avgIntraArray3 = 0;
  avgIntraArray4 = 0;
  avgExtraArray1 = 0;
  avgExtraArray2 = 0;
  avgExtraArray3 = 0;
  avgSWB1 = 0;
  avgSWB2 = 0;
  avgSWB3 = 0;
  avgEntrSWB1 = 0;
  avgEntrSWB2 = 0;
  avgEntrSWB3 = 0;
  avgExitSWB1 = 0;
  avgExitSWB2 = 0;
  avgExitSWB3 = 0;
  avgEntrBWB1 = 0;
  avgEntrBWB2 = 0;
  avgEntrBWB3 = 0;
  avgBWB1 = 0;
  avgBWB2 = 0;
  avgBWB3 = 0;
  avgBWB4 = 0;
  avgBWB5 = 0;
  avgBWB6 = 0;
  avgExitBWB1 = 0;
  avgExitBWB2 = 0;
  avgExitBWB3 = 0;
  avgPressureSensor1 = 0;
  avgPressureSensor2 = 0;

  for (i = 0; i < NUMSAMPLES; i++) {
    avgIntraArray1 += samples1[i];
    avgIntraArray2 += samples2[i];
    avgIntraArray3 += samples3[i];
    avgIntraArray4 += samples4[i];
    avgExtraArray1 += samples5[i];
    avgExtraArray2 += samples6[i];
    avgExtraArray3 += samples7[i];
    avgSWB1 += samples8[i];
    avgSWB2 += samples9[i];
    avgSWB3 += samples10[i];
    avgEntrSWB1 += samples11[i];
    avgEntrSWB2 += samples12[i];
    avgEntrSWB3 += samples13[i];
    avgExitSWB1 += samples14[i];
    avgExitSWB2 += samples15[i];
    avgExitSWB3 += samples16[i];
    avgEntrBWB1 += samples17[i];
    avgEntrBWB2 += samples18[i];
    avgEntrBWB3 += samples19[i];
    avgBWB1 += samples20[i];
    avgBWB2 += samples21[i];
    avgBWB3 += samples22[i];
    avgBWB4 += samples23[i];
    avgBWB5 += samples24[i];
    avgBWB6 += samples25[i];
    avgExitBWB1 += samples26[i];
    avgExitBWB2 += samples27[i];
    avgExitBWB3 += samples28[i];
    avgPressureSensor1 += samples29[i];
    avgPressureSensor2 += samples30[i];
  }

  avgIntraArray1 = avgIntraArray1 / NUMSAMPLES;
  avgIntraArray2 = avgIntraArray2 / NUMSAMPLES;
  avgIntraArray3 = avgIntraArray3 / NUMSAMPLES;
  avgIntraArray4 = avgIntraArray4 / NUMSAMPLES;
  avgExtraArray1 = avgExtraArray1 / NUMSAMPLES;
  avgExtraArray2 = avgExtraArray2 / NUMSAMPLES;
  avgExtraArray3 = avgExtraArray3 / NUMSAMPLES;
  avgSWB1 = avgSWB1 / NUMSAMPLES;
  avgSWB2 = avgSWB2 / NUMSAMPLES;
  avgSWB3 = avgSWB3 / NUMSAMPLES;
  avgEntrSWB1 = avgEntrSWB1 / NUMSAMPLES;
  avgEntrSWB2 = avgEntrSWB2 / NUMSAMPLES;
  avgEntrSWB3 = avgEntrSWB3 / NUMSAMPLES;
  avgExitSWB1 = avgExitSWB1 / NUMSAMPLES;
  avgExitSWB2 = avgExitSWB2 / NUMSAMPLES;
  avgExitSWB3 = avgExitSWB3 / NUMSAMPLES;
  avgEntrBWB1 = avgEntrBWB1 / NUMSAMPLES;
  avgEntrBWB2 = avgEntrBWB2 / NUMSAMPLES;
  avgEntrBWB3 = avgEntrBWB3 / NUMSAMPLES;
  avgBWB1 = avgBWB1 / NUMSAMPLES;
  avgBWB2 = avgBWB2 / NUMSAMPLES;
  avgBWB3 = avgBWB3 / NUMSAMPLES;
  avgBWB4 = avgBWB4 / NUMSAMPLES;
  avgBWB5 = avgBWB5 / NUMSAMPLES;
  avgBWB6 = avgBWB6 / NUMSAMPLES;
  avgExitBWB1 = avgExitBWB1 / NUMSAMPLES;
  avgExitBWB2 = avgExitBWB2 / NUMSAMPLES;
  avgExitBWB3 = avgExitBWB3 / NUMSAMPLES;
  avgPressureSensor1 = avgPressureSensor1 / NUMSAMPLES;
  avgPressureSensor2 = avgPressureSensor2 / NUMSAMPLES;

  //Converting temp sensors data into actual degrees (Celsius)
  //First convert to a resistance value
  float avgIntraArray1Resist = SERIESRESISTOR / (1023 / (avgIntraArray1)-1);
  float avgIntraArray2Resist = SERIESRESISTOR / (1023 / (avgIntraArray2)-1);
  float avgIntraArray3Resist = SERIESRESISTOR / (1023 / (avgIntraArray3)-1);
  float avgIntraArray4Resist = SERIESRESISTOR / (1023 / (avgIntraArray4)-1);
  float avgExtraArray1Resist = SERIESRESISTOR / (1023 / (avgExtraArray1)-1);
  float avgExtraArray2Resist = SERIESRESISTOR / (1023 / (avgExtraArray2)-1);
  float avgExtraArray3Resist = SERIESRESISTOR / (1023 / (avgExtraArray3)-1);
  float avgSWB1Resist = SERIESRESISTOR / (1023 / (avgSWB1)-1);
  float avgSWB2Resist = SERIESRESISTOR / (1023 / (avgSWB2)-1);
  float avgSWB3Resist = SERIESRESISTOR / (1023 / (avgSWB3)-1);
  float avgEntrSWB1Resist = SERIESRESISTOR / (1023 / (avgEntrSWB1)-1);
  float avgEntrSWB2Resist = SERIESRESISTOR / (1023 / (avgEntrSWB2)-1);
  float avgEntrSWB3Resist = SERIESRESISTOR / (1023 / (avgEntrSWB3)-1);
  float avgExitSWB1Resist = SERIESRESISTOR / (1023 / (avgExitSWB1)-1);
  float avgExitSWB2Resist = SERIESRESISTOR / (1023 / (avgExitSWB2)-1);
  float avgExitSWB3Resist = SERIESRESISTOR / (1023 / (avgExitSWB3)-1);
  float avgEntrBWB1Resist = SERIESRESISTOR / (1023 / (avgEntrBWB1)-1);
  float avgEntrBWB2Resist = SERIESRESISTOR / (1023 / (avgEntrBWB2)-1);
  float avgEntrBWB3Resist = SERIESRESISTOR / (1023 / (avgEntrBWB3)-1);
  float avgBWB1Resist = SERIESRESISTOR / (1023 / (avgBWB1)-1);
  float avgBWB2Resist = SERIESRESISTOR / (1023 / (avgBWB2)-1);
  float avgBWB3Resist = SERIESRESISTOR / (1023 / (avgBWB3)-1);
  float avgBWB4Resist = SERIESRESISTOR / (1023 / (avgBWB4)-1);
  float avgBWB5Resist = SERIESRESISTOR / (1023 / (avgBWB5)-1);
  float avgBWB6Resist = SERIESRESISTOR / (1023 / (avgBWB6)-1);
  float avgExitBWB1Resist = SERIESRESISTOR / (1023 / (avgExitBWB1)-1);
  float avgExitBWB2Resist = SERIESRESISTOR / (1023 / (avgExitBWB2)-1);
  float avgExitBWB3Resist = SERIESRESISTOR / (1023 / (avgExitBWB3)-1);
  //Convert Resistance to Temperature
  float steinhartIntraArray1 = 1 / ((log(avgIntraArray1Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartIntraArray2 = 1 / ((log(avgIntraArray2Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartIntraArray3 = 1 / ((log(avgIntraArray3Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartIntraArray4 = 1 / ((log(avgIntraArray4Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartExtraArray1 = 1 / ((log(avgExtraArray1Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartExtraArray2 = 1 / ((log(avgExtraArray2Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartExtraArray3 = 1 / ((log(avgExtraArray3Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartSWB1 = 1 / ((log(avgSWB1Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartSWB2 = 1 / ((log(avgSWB2Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartSWB3 = 1 / ((log(avgSWB3Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartEntrSWB1 = 1 / ((log(avgEntrSWB1Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartEntrSWB2 = 1 / ((log(avgEntrSWB2Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartEntrSWB3 = 1 / ((log(avgEntrSWB3Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartExitSWB1 = 1 / ((log(avgExitSWB1Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartExitSWB2 = 1 / ((log(avgExitSWB2Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartExitSWB3 = 1 / ((log(avgExitSWB3Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartEntrBWB1 = 1 / ((log(avgEntrBWB1Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartEntrBWB2 = 1 / ((log(avgEntrBWB2Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartEntrBWB3 = 1 / ((log(avgEntrBWB3Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartBWB1 = 1 / ((log(avgBWB1Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartBWB2 = 1 / ((log(avgBWB2Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartBWB3 = 1 / ((log(avgBWB3Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartBWB4 = 1 / ((log(avgBWB4Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartBWB5 = 1 / ((log(avgBWB5Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartBWB6 = 1 / ((log(avgBWB6Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartExitBWB1 = 1 / ((log(avgExitBWB1Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartExitBWB2 = 1 / ((log(avgExitBWB2Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  float steinhartExitBWB3 = 1 / ((log(avgExitBWB3Resist / THERMISTORNOMINAL)) / BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15)) - 273.15;
  
  //Average multiple temp sensors into 1 temperature
  float IntraArrayTemp = (steinhartIntraArray1 + steinhartIntraArray2 + steinhartIntraArray3 + steinhartIntraArray4) / 4;
  float ExtraArrayTemp = (steinhartExtraArray1 + steinhartExtraArray2 + steinhartExtraArray3) / 3;
  float SWBTemp = (steinhartSWB1 + steinhartSWB2) / 2;  //steinhartSWB3 deleted due to bad sensor error
  float EntrSWBTemp = (steinhartEntrSWB1 + steinhartEntrSWB2 + steinhartEntrSWB3) / 3;
  float ExitSWBTemp = (steinhartExitSWB1 + steinhartExitSWB2 + steinhartExitSWB3) / 3;
  float EntrBWBTemp = (steinhartEntrBWB1 + steinhartEntrBWB2 + steinhartEntrBWB3) / 3;
  float BWBTemp = (steinhartBWB1 + steinhartBWB2 + steinhartBWB3 + steinhartBWB4 + steinhartBWB5 + steinhartBWB6) / 6;
  float ExitBWBTemp = (steinhartExitBWB1 + steinhartExitBWB2 + steinhartExitBWB3) / 3;

  //Peltier Plate PID
  //float braintemp_atm;
  braintemp_atm = IntraArrayTemp;  //This is the current brain temperature
  InputPeltierPID = braintemp_atm;
  myPID_peltier.Compute();
  analogWrite(DACPIN, 255 - OutputPeltierPID);

  //Water Pump/Block PID
  //float WBTemp_atm;
  WBTemp_atm = SWBTemp;
  if (WBTemp_atm >= WBTempIdeal) {
    InputWB = WBTemp_atm;
    myPID_WB.Compute();
    freq = map(OutputWB, 0, 255, 0, 60);
    float quantizedFreq = round(freq / freqThreshold) * freqThreshold;
    if (abs(quantizedFreq - previousFreq) >= freqThreshold + hysteresis) {
      // Update the period based on the new frequency
      if (quantizedFreq > 0) {
        desperiod = 1.0 / quantizedFreq;  // Calculate period in seconds
        pwm->period(desperiod);           // Update PWM period
        Serial.print("Updated frequency to: ");
        Serial.println(quantizedFreq);
      } else {
        pwm->period(1.0);  // Set to 1 Hz if the frequency goes to 0 to avoid errors
        Serial.println("Frequency too low, set to 1 Hz.");
      }
      // Update the previous frequency tracker
      previousFreq = quantizedFreq;
    }
    // Keep the duty cycle constant (50%)
    pwm->write(0.5f);
  }

  //Peltier Overcurrent and Overvoltage Protection
  peltierCurrent = peltierINA260.readCurrent() / 1000;
  peltierVoltage = peltierINA260.readBusVoltage() / 1000;
  pumpCurrent = pumpINA260.readCurrent() / 1000;
  pumpVoltage = pumpINA260.readBusVoltage() / 1000;
  //Turn off Power to peltier if too high


/*
  if (peltierCurrent >= 2 || peltierVoltage >= 8) {
    analogWrite(DACPIN, 255);
    digitalWrite(peltierRelay, LOW);

    Serial.print("ERROR: Peltier Overcurrenting/volting");
    updateErrorStatus("ERROR: Peltier Overcurrenting/volting");
  }


  //Pump Overcurrent and Overvoltage Protection
  //Turn off Power to pump if too high
  if (pumpVoltage >= 6.1) {
    digitalWrite(pumpRelay, HIGH);
    analogWrite(DACPIN, 255);
    digitalWrite(peltierRelay, HIGH);
    Serial.println("ERROR: Water Pump Overvolting");
    updateErrorStatus("ERROR: Water Pump Overvolting");
  }
*/


  //Flow Rate Sensor Detection
  
  uint16_t aSignalingFlags = 0u;
  flowSensorA.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3S_4000B, aFlow, aTemperature, aSignalingFlags);
  uint16_t bSignalingFlags = 0u;
  flowSensorB.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3S_4000B, bFlow, bTemperature, bSignalingFlags);

  //Rejection of Bad Flow Sensor
  if (aFlow == 0.0 && aTemperature == 0.0){
    aFlow_bad = 1;
    Serial.println("Flow Sensor A Bad");
  }
  else {
    aFlow_bad = 0;
  }

  if (bFlow == 0.0 && bTemperature == 0.0){
    bFlow_bad = 1;
    Serial.println("Flow Sensor B Bad");
  }
  else {
    bFlow_bad = 0;
  }

  //Sudden Flow Rate Drop Detection
  if (aFlow_bad == 0 && bFlow_bad == 0){
    if ((aFlow <= 5 || bFlow <= 5) && WBTemp_atm >= WBTempMax) {
      digitalWrite(pumpRelay, HIGH);
      analogWrite(DACPIN, 255);
      digitalWrite(peltierRelay, HIGH);

      Serial.println("ERROR: No Flow Detected");
      updateErrorStatus("ERROR: No Flow Detected");
    }
  }
  else if (aFlow_bad == 1 && bFlow_bad == 0){
    if (bFlow <= 5 && WBTemp_atm >= WBTempMax) {
      digitalWrite(pumpRelay, HIGH);
      analogWrite(DACPIN, 255);
      digitalWrite(peltierRelay, HIGH);

      Serial.println("ERROR: No Flow Detected");
      updateErrorStatus("ERROR: No Flow Detected");
    }
  }
  else if (aFlow_bad == 0 && bFlow_bad == 1){
    if (aFlow <= 5 && WBTemp_atm >= WBTempMax) {
      digitalWrite(pumpRelay, HIGH);
      analogWrite(DACPIN, 255);
      digitalWrite(peltierRelay, HIGH);

      Serial.println("ERROR: No Flow Detected");
      updateErrorStatus("ERROR: No Flow Detected");
    }
  }
  else if (aFlow_bad == 1 && bFlow_bad == 1){
    Serial.println("ERROR: Bad Flow Sensors");
    myFile.println("No good flow sensors");
  }

  //Detecting Pressure Values
  //Converting from input to voltage (conversion from spec sheet)
  float pressureSensor1Volt = avgPressureSensor1 * (vSupply / 1023.0);
  float pressureSensor2Volt = avgPressureSensor2 * (vSupply / 1023.0);

  //Converting Voltage to pressure (conversion from spec sheet)
  pressureApplied1 = (15 / (0.8 * vSupply)) * (pressureSensor1Volt - 0.1 * vSupply);
  pressureApplied2 = (15 / (0.8 * vSupply)) * (pressureSensor2Volt - 0.1 * vSupply);

  //Turn off system if abnormal pressure
  float lowPressure = 0;
  float highPressure = 5;

  /*
  if (pressureApplied1 > highPressure || pressureApplied2 > highPressure) {
    analogWrite(DACPIN, 255);  //DACPIN 0 means completely on, 255 means off
    digitalWrite(pumpRelay, HIGH);
    digitalWrite(peltierRelay, HIGH);

    Serial.print("ERROR: Aberrant Pressure Levels");
    updateErrorStatus("ERROR: Aberrant Pressure Levels");
  }
  */

  //Water block temp 6 degrees above baseline and max flow rate --> Countdown for 3 minutes activates
  if (WBTemp_atm >= 41 && freq == 60) {
    startTime = startTime;  //If maxxing out, keep the starttime of the max the same
  } else {
    int newTime = millis();  //If normal operations, update startime with current time
    startTime = newTime;
  }
  int TimeATM = millis();
  if (TimeATM - startTime >= 180000) {  //1000 ms/s * 60 s/min * 3 min
    //If it is greater than 3 minutes, shutting everything down
    analogWrite(DACPIN, 255);
    freq = 0;
    float quantizedFreq = round(freq / freqThreshold) * freqThreshold;
    if (abs(quantizedFreq - previousFreq) >= freqThreshold + hysteresis) {
      // Update the period based on the new frequency
      if (quantizedFreq > 0) {
        desperiod = 1.0 / quantizedFreq;  // Calculate period in seconds
        pwm->period(desperiod);           // Update PWM period
        Serial.print("Updated frequency to: ");
        Serial.println(quantizedFreq);
      } else {
        pwm->period(1.0);  // Set to 1 Hz if the frequency goes to 0 to avoid errors
        Serial.println("Frequency too low, set to 1 Hz.");
      }
      // Update the previous frequency tracker
      previousFreq = quantizedFreq;
      

    }
    Serial.println(quantizedFreq);
    Serial.println(freq);
    // Keep the duty cycle constant (50%)
    pwm->write(0.5f);


    digitalWrite(peltierRelay, HIGH);
    digitalWrite(pumpRelay, HIGH);

    Serial.println("CRITICAL ERROR: Cooling failed, Shutting system down");
    updateErrorStatus("CRITICAL ERROR: Cooling failed, Shutting system down");
  }

  //Update webpage and datamonitor
  handleWebRequests(server);


  //Save Data to SD Card
  myFile = SD.open("testing.txt", FILE_WRITE);  //Opening the file here

  myFile.print(getLocaltime());
  myFile.print(",");
  myFile.print(steinhartIntraArray1);
  myFile.print(",");
  myFile.print(steinhartIntraArray2);  //Saving the data here. It is done like this so easy import to CSV format
  myFile.print(",");
  myFile.print(steinhartIntraArray3);
  myFile.print(",");
  myFile.print(steinhartIntraArray4);
  myFile.print(",");
  myFile.print(steinhartExtraArray1);
  myFile.print(",");
  myFile.print(steinhartExtraArray2);
  myFile.print(",");
  myFile.print(steinhartExtraArray3);
  myFile.print(",");
  myFile.print(steinhartSWB1);
  myFile.print(",");
  myFile.print(steinhartSWB2);
  myFile.print(",");
  myFile.print(steinhartSWB3);
  myFile.print(",");
  myFile.print(steinhartEntrSWB1);  //Saving the data here. It is done like this so easy import to CSV format
  myFile.print(",");
  myFile.print(steinhartEntrSWB2);
  myFile.print(",");
  myFile.print(steinhartEntrSWB3);
  myFile.print(",");
  myFile.print(steinhartExitSWB1);
  myFile.print(",");
  myFile.print(steinhartExitSWB2);
  myFile.print(",");
  myFile.print(steinhartExitSWB3);
  myFile.print(",");
  myFile.print(steinhartEntrBWB1);
  myFile.print(",");
  myFile.print(steinhartEntrBWB2);
  myFile.print(",");
  myFile.print(steinhartEntrBWB3);
  myFile.print(",");
  myFile.print(steinhartBWB1);  //Saving the data here. It is done like this so easy import to CSV format
  myFile.print(",");
  myFile.print(steinhartBWB2);
  myFile.print(",");
  myFile.print(steinhartBWB3);
  myFile.print(",");
  myFile.print(steinhartBWB4);
  myFile.print(",");
  myFile.print(steinhartBWB5);
  myFile.print(",");
  myFile.print(steinhartBWB6);
  myFile.print(",");
  myFile.print(steinhartExitBWB1);
  myFile.print(",");
  myFile.print(steinhartExitBWB2);
  myFile.print(",");
  myFile.print(steinhartExitBWB3);
  myFile.print(",");
  myFile.print(peltierCurrent);  //Saving the data here. It is done like this so easy import to CSV format
  myFile.print(",");
  myFile.print(peltierVoltage);
  myFile.print(",");
  myFile.print(pumpCurrent);
  myFile.print(",");
  myFile.print(pumpVoltage);
  myFile.print(",");
  myFile.print(aFlow);
  myFile.print(",");
  myFile.print(bFlow);
  myFile.print(",");
  myFile.print(pressureApplied1);
  myFile.print(",");
  myFile.print(pressureApplied2);
  myFile.print(",");
  myFile.print(Kp_peltier);
  myFile.print(",");
  myFile.print(Ki_peltier);
  myFile.print(",");
  myFile.print(Kd_peltier);
  myFile.print(",");
  myFile.print(SetpointPeltier);
  myFile.print(",");
  myFile.println(clientStatus);
  myFile.close();

  //Sending data to Serial

  Serial.print("IntraArray:");
  Serial.print(steinhartIntraArray1);
  Serial.print(",");
  Serial.print(steinhartIntraArray2);
  Serial.print(",");
  Serial.print(steinhartIntraArray3);
  Serial.print(",");
  Serial.println(steinhartIntraArray4);

  Serial.print("ExtraArray:");
  Serial.print(steinhartExtraArray1);
  Serial.print(",");
  Serial.print(steinhartExtraArray2);
  Serial.print(",");
  Serial.println(steinhartExtraArray3);

  Serial.print("SWB:");
  Serial.print(steinhartSWB1);
  Serial.print(",");
  Serial.print(steinhartSWB2);
  Serial.print(",");
  Serial.println(steinhartSWB3);

  Serial.print("EntrSWB:");
  Serial.print(steinhartEntrSWB1);  //Saving the data here. It is done like this so easy import to CSV format
  Serial.print(",");
  Serial.print(steinhartEntrSWB2);
  Serial.print(",");
  Serial.println(steinhartEntrSWB3);

  Serial.print("ExitSWB:");
  Serial.print(steinhartExitSWB1);
  Serial.print(",");
  Serial.print(steinhartExitSWB2);
  Serial.print(",");
  Serial.println(steinhartExitSWB3);

  Serial.print("EntrBWB:");
  Serial.print(steinhartEntrBWB1);
  Serial.print(",");
  Serial.print(steinhartEntrBWB2);
  Serial.print(",");
  Serial.println(steinhartEntrBWB3);

  Serial.print("BWB:");
  Serial.print(steinhartBWB1);  //Saving the data here. It is done like this so easy import to CSV format
  Serial.print(",");
  Serial.print(steinhartBWB2);
  Serial.print(",");
  Serial.print(steinhartBWB3);
  Serial.print(",");
  Serial.print(steinhartBWB4);
  Serial.print(",");
  Serial.print(steinhartBWB5);
  Serial.print(",");
  Serial.println(steinhartBWB6);

  Serial.print("ExitBWB:");
  Serial.print(steinhartExitBWB1);
  Serial.print(",");
  Serial.print(steinhartExitBWB2);
  Serial.print(",");
  Serial.println(steinhartExitBWB3);

  Serial.print("Peltier Current:");
  Serial.print(peltierCurrent);  //Saving the data here. It is done like this so easy import to CSV format
  Serial.print(",");
  Serial.print("Peltier Voltage:");
  Serial.print(peltierVoltage);
  Serial.print(",");
  Serial.print("Pump Current:");
  Serial.print(pumpCurrent);
  Serial.print(",");
  Serial.print("Pump Voltage:");
  Serial.println(pumpVoltage);

  Serial.print("Flow Sensors temp:");
  Serial.print(aTemperature);
  Serial.print(",");
  Serial.println(bTemperature);

  Serial.print("Flow rate:");
  Serial.print(aFlow);
  Serial.print(",");
  Serial.println(bFlow);

  Serial.print("Frequency output:");
  Serial.print(previousFreq);
  Serial.println();

  Serial.print("Pressure Sensors:");
  Serial.print(pressureApplied1);
  Serial.print(",");
  Serial.println(pressureApplied2);

  Serial.print("Pressure Voltage:");
  Serial.print(pressureSensor1Volt);
  Serial.print(",");
  Serial.println(pressureSensor2Volt);

  Serial.print("PID values:");
  Serial.print(Kp_peltier);
  Serial.print(",");
  Serial.print(Ki_peltier);
  Serial.print(",");
  Serial.print(Kd_peltier);
  Serial.print(",");
  Serial.println(SetpointPeltier);

  Serial.print("Client Status:");
  Serial.println(clientStatus);

  /*//Sending Data to Client
  
    //Printing Client Stats
      client.print("Client Status");
      client.print(clientStatus);
        client.print("\t");
    //Printing Time and Data
      client.print(getLocaltime());
        client.print("\t");
    //Printing Temperature Info
      client.print("Brain Temp:");
      client.print(braintemp_atm);
        client.print("\t");
      client.print("Water Block Temp:");
      client.print(WBTemp_atm);
        client.print("\t");
    //PID Strength
      client.print("P_OUT(div10):");
      client.print(OutputPeltierPID/10);
        client.print("\t");  
      client.print("WB_OUT(div10):");
      client.print(OutputWB/10);
        client.print("\t");
      client.print("FREQ:");
      client.print(freq);
        client.print("\t");
    //Current and Voltage Sensors
      client.print("Peltier Current:");
      client.print(peltierCurrent);
        client.print("\t");
      client.print("Peltier Voltage");
      //client.print(peltierINA260.readBusVoltage()/1000);
        client.print("\t");
      client.print("Water Pump Current:");
      //client.print(pumpINA260.readCurrent()/1000);
        client.print("\t");
      client.print("Water Pump Voltage:");
      //client.print(pumpINA260.readBusVoltage()/1000);
        client.print("\t");
    //Flow Rate Sensors
      client.print("aFlow: ");
      client.print(aFlow);
        client.print("\t");
    //Pressure Sensors
      client.print(pressureApplied1, decimalPlaces);
      client.println("psi");
        client.print("\t");*/
  delay(1000);
}




/* ---------------------------- FUNCTIONS ---------------------------- */


String getLocaltime() {
  char buffer[32];
  tm t;
  _rtc_localtime(time(NULL), &t, RTC_4_YEAR_LEAP_YEAR_SUPPORT);
  strftime(buffer, 32, "%m-%d-%Y_%k:%M:%S", &t);
  return String(buffer);
}


/*
setMux(int mux, int channel) - sets the MUX channel based off the parameters  
Parameters - 
  int mux - represents which MUX(1-2) to read from. int must be [1,2] for function to work properly.  
  int channel - represents which channel(0-15) to read from. int must be [0,15] for function to work properly.
Return - (function return type: int)
  int sig_pin -  
*/
int setMux(int mux, int channel) {

  // declare local array for control pins (S0-S3) and variable for SIG pin
  int controlPin[4];
  int sigPin;

  // set the correct values for each MUX's control pins and SIG pins
  if (mux == 1) {
    controlPin[0] = mux1S0;
    controlPin[1] = mux1S1;
    controlPin[2] = mux1S2;
    controlPin[3] = mux1S3;
    sigPin = mux1Sig;
  } else if (mux == 2) {
    controlPin[0] = mux2S0;
    controlPin[1] = mux2S1;
    controlPin[2] = mux2S2;
    controlPin[3] = mux2S3;
    sigPin = mux2Sig;
  }

  // 2D integer array for MUX channels
  // arrayName[row][column]
  int muxChannel[16][4] = {
    { 0, 0, 0, 0 },  //channel 0
    { 1, 0, 0, 0 },  //channel 1
    { 0, 1, 0, 0 },  //channel 2
    { 1, 1, 0, 0 },  //channel 3
    { 0, 0, 1, 0 },  //channel 4
    { 1, 0, 1, 0 },  //channel 5
    { 0, 1, 1, 0 },  //channel 6
    { 1, 1, 1, 0 },  //channel 7
    { 0, 0, 0, 1 },  //channel 8
    { 1, 0, 0, 1 },  //channel 9
    { 0, 1, 0, 1 },  //channel 10
    { 1, 1, 0, 1 },  //channel 11
    { 0, 0, 1, 1 },  //channel 12
    { 1, 0, 1, 1 },  //channel 13
    { 0, 1, 1, 1 },  //channel 14
    { 1, 1, 1, 1 }   //channel 15
  };

  // loop through the 4 SIG
  for (int i = 0; i < 4; i++) {                                                                                   
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }

  return sigPin;
}


/*
setupWiFiAP() - sets up the WiFi Access Point and includes error handling 
Parameters -  none (uses global variable status)
Return - none (function return type: void)
*/
void setupWiFiAP() {

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  // check firmware version
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.48.56.2
  // you can override it with the following:
  WiFi.config(IPAddress(192, 48, 56, 2));

  // print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  // Create open network. Change this line if you want to create an WEP network:
  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    Serial.println("Creating access point failed");
    // don't continue
    while (true);
  }

  // wait 1 seconds for connection:
  delay(1000);
}