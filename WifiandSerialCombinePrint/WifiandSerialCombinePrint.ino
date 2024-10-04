#define DACPIN A0
//#define THERMISTORPIN1 A1
//#define THERMISTORPIN2 A2
//#define THERMISTORPIN3 A3
#define THERMISTORNOMINAL 10000  
#define TEMPERATURENOMINAL 25 
#define NUMSAMPLES 10
#define BCOEFFICIENTL 3895
#define BCOEFFICIENTA 3895
#define SERIESRESISTOR 10000 


//This portion is to test the peltier buck converter with serial monitor

//String stringa = String(1);
int CompSetPeltier;
//#define PWMPeltier 3 //Choosing pin randomly for output for Peltier
//#define PWMPump 5 //Choosing pin randomly for output for pump

#include <Adafruit_INA260.h>
#include <Wire.h>
#include <pwm.h>

#include <RTC.h> //This is just to make sure we have accurate time for arduino

//Needed for SD card reader
#include <SPI.h>
#include <SD.h>
File myFile;

//Needed for WIFI
#include "WiFiS3.h" //This is the one for the uno R4
#include "WiFi.h" //This is the one for Wifi Giga

//Needed for the flow sensor
#include <Arduino.h>
#include <SensirionI2cSf06Lf.h>
SensirionI2cSf06Lf flow_sensor;

PwmOut pwm(D5);

Adafruit_INA260 ina260_peltier = Adafruit_INA260();
Adafruit_INA260 ina260_waterpump = Adafruit_INA260();

//Creating the output frequency using the DAC pin
//#include "analogWave.h" // Include the library for analog waveform generation
//analogWave wave(DAC);   // Create an instance of the analogWave class, using the DAC pin
int freq = 10;  // in hertz, change accordingly, randomly set at 0 here, can increase below

//PID Loop creation and variables
  #include <PID_v1.h> //Initializing the PID loop

  double SetpointPeltier, InputPeltierPID, OutputPeltierPID; //Generating the input variables for the peltier plate cooling PID -- Setpoint is goal; input is what is detected; output is PID output 0-255
  double Kp_peltier=17, Ki_peltier=3, Kd_peltier=2; //Specify the initial tuning parameters. Ignoring Kd since not used often. -- watch youtube video: https://www.youtube.com/watch?v=IB1Ir4oCP5k
  PID myPID_peltier(&InputPeltierPID, &OutputPeltierPID, &SetpointPeltier, Kp_peltier, Ki_peltier, Kd_peltier, DIRECT, REVERSE); //By default, PID "warms" up, so we have to reverse it -- unknown what DIRECT is for

  //This PID is to make sure that the water block is adequtely cooling down. WB stands for water block
  double SetpointWB, InputWB, OutputWB; // setpoint is desired temp, input is current temp, output is 0-255
  double Kp_WB=15, Ki_WB=10, Kd_WB=0;
  PID myPID_WB(&InputWB, &OutputWB, &SetpointWB, Kp_WB, Ki_WB, Kd_WB, DIRECT, REVERSE);


  //These are parameters that can be adjusted for temperature cutoffs -- TempIdeals are equivalent to Setpoint
  int BrainTempIdeal = 10; //Ideal brain temperature, currently 12 for testing purposes (should be 25)
  int WBTempMax = 100; //The maximum peltier plate temperature, currently 25 for testing purposes (should be 40)
  int WBTempIdeal = 20; //The ideal temperature of the WB, currently 22 for testing purposes (should be 35)

unsigned long waqt;
uint16_t samplesa1[NUMSAMPLES];
uint16_t samplesa2[NUMSAMPLES];
uint16_t samplesa3[NUMSAMPLES];

//Pressure Sensor information

  // Define the analog input pin for the sensor
  const int pressureSensorPin = A3;

  // Sensor specifications
  const float Vsupply = 5.0;    // Supply voltage
  const float Pmin = 0.0;       // Minimum pressure in PSI
  const float Pmax = 15.0;      // Maximum pressure in PSI

  const int sampleSize = 10;   // Sample size
  const int decimalPlaces = 3;  // Decimal places for Serial.print()

//Setup for Multiplexer
  // Multiplexer control pins, S0-S3 (digital pins)
  int mux1S0 = 9;
  int mux1S1 = 8;
  int mux1S2 = 7;
  int mux1S3 = 6;
  // Multiplexer signal pins
  int mux1Sig = A1;

// WIFI related global variables, macros, and object
  #define SECRET_SSID "pigTrial"
  #define SECRET_PASS "123456789"
  char ssid[] = SECRET_SSID;  // your network SSID (name)
  char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
  int status = WL_IDLE_STATUS;
  WiFiServer server(80);

void setup() {
// put your setup code here, to run once:

Serial.begin(115200);

//Starting up Real Time Clock. Will have to adjust date and time when implanting
  RTC.begin();
  RTCTime startTime(26, Month::SEPTEMBER, 2024, 15, 42, 00, DayOfWeek::THURSDAY, SaveLight::SAVING_TIME_ACTIVE);
  RTC.setTime(startTime);


//Starting up INA Chips and Detecting them
    while (!Serial) { delay(10); }
  Wire.begin();
  Serial.println("Adafruit INA260 Test");
 if (!ina260_peltier.begin(0x40)) {
    Serial.print("Couldn't find INA260_peltier chip at 0x40");
    while (1);
}
if (!ina260_waterpump.begin(0x41)) {
    Serial.print("Couldn't find INA260_waterpump chip at 0x41");
    while (1);
}
  Serial.println("Found INA260 chips");



//Initializing SD Card
 /* Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");
  myFile = SD.open("testcode.txt", FILE_WRITE); */

  

pwm.begin(100.0f, 50.0f);
  //analogReference(EXTERNAL);
analogReference(AR_EXTERNAL); //This needs to be wired to AREF at 3.3V, and this is the powersupply to the thermestimors
pinMode(DACPIN, OUTPUT);
  //wave.square(freq);
  
//Turning on PID loops

  SetpointPeltier = BrainTempIdeal;
  //turn the PID on
  myPID_peltier.SetMode(AUTOMATIC);
  //myPID_pump.SetMode(AUTOMATIC);

  SetpointWB = WBTempIdeal;
  myPID_WB.SetMode(AUTOMATIC);


//Initializing flow sensor
  flow_sensor.begin(Wire, SLF3C_1300F_I2C_ADDR_08);
  delay(100);
  flow_sensor.startH2oContinuousMeasurement();


//Initializing Multiplexer
  //Defining pin outputs
  pinMode(mux1S0, OUTPUT);
  pinMode(mux1S1, OUTPUT);
  pinMode(mux1S2, OUTPUT);
  pinMode(mux1S3, OUTPUT);

  //Set control pins to 0 so it starts reading signals on channel 0 of multiplexer
  digitalWrite(mux1S0, LOW);
  digitalWrite(mux1S1, LOW);
  digitalWrite(mux1S2, LOW);
  digitalWrite(mux1S3, LOW);

//Initializing SD Card
if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  //Opening the file we want to save in
  myFile = SD.open("Testing.txt", FILE_WRITE);
  myFile.println("Date,Time,Peltier Temp (C),Water Block Temp (C),Peltier Current (A),Peltier Voltage (V),Water Pump Current (A),Water Pump Voltage (V),Flow (mL),Pressure (psi),WiFI Client Status (0 = USB, 1 = WiFi");
  myFile.close(); 

//Wifi Setup
  // set up the WiFi Access Point
  setupWiFiAP();

  // start the WiFi server
  // the server will listen for incoming connections on port 80 (the default port for HTTP)
  server.begin();

}




void loop() {
  // put your main code here, to run repeatedly:

//Starting Wifi and Checking if anyone connected
  // compare previous WIFI status to current WIFI status
    if (status != WiFi.status()) {
      // WIFI status has changed so update status variable
      status = WiFi.status();

      if (status == WL_AP_CONNECTED) {
        // a device has connected to the AP
        Serial.println("Device connected to AP");
      } else {
        // a device has disconnected from the AP, and we are back in listening mode
        Serial.println("Device disconnected from AP");
      }
    }


//Anything below here is printed to client

  // listen for incoming clients
    WiFiClient client = server.available();

    if (client){
      
        RTCTime currentTime;
        RTC.getTime(currentTime);

      int clientStatus = 1;
      Serial.print("Client Status");
      Serial.print(clientStatus);
      Serial.print("\t");

 

      client.print(currentTime.getDayOfMonth());
      client.print("/");
      client.print(Month2int(currentTime.getMonth()));
      client.print("/");
      client.print(currentTime.getYear());
      client.print("\t");

      client.print(currentTime.getHour());
      client.print(":");
      client.print(currentTime.getMinutes());
      client.print(":");
      client.print(currentTime.getSeconds());
      client.print("\t");


//Getting temperatures
  uint8_t i;
  //Finding the right channel for thermistors on multiplexer
  
    waqt = millis()/1000;
      for (i=0; i < NUMSAMPLES; i++) {          // take N samples in a row, with a slight delay
      int sig_therm1 = setMux(1, 0); //Channel 0 for Peltier thermistor
      samplesa1[i] = analogRead(sig_therm1); //Reading Peltier thermistor

      int sig_therm2 = setMux(1, 1); //Channel 1 for Water block thermistor
      samplesa2[i] = analogRead(sig_therm2); //Reading water block thermistor
      //samplesa3[i] = analogRead(THERMISTORPIN3);
    delay(100);
  }



  float avga1;
  float avga2;
  //float avga3;

 // average all the samples out
  avga1 = 0;
  avga2 = 0;
  //avga3 = 0;
 
  for (i=0; i< NUMSAMPLES; i++) {
     avga1 += samplesa1[i];
     avga2 += samplesa2[i];
     //avga3 += samplesa3[i];
  }

  avga1 = SERIESRESISTOR / (1023 /(avga1/NUMSAMPLES) - 1);
  avga2 = SERIESRESISTOR / (1023 /(avga2/NUMSAMPLES) - 1);
  //avga3 = avga3/NUMSAMPLES;

  //float flow3 = 100 * (avga3 - 0.045);

  float steinharta1;
  float steinharta2;
  //float steinharta3;


   // (R/Ro)
  steinharta1 = 1/((log(avga1 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta2 = 1/((log(avga2 / THERMISTORNOMINAL))/BCOEFFICIENTL + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
 // steinharta3 = 1/((log(avga3 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;


  //Serial.print(waqt);
  client.print("P_T:");
  client.print(steinharta1);
  client.print("\t");
 // Serial.print(waqt);
  //Serial.println();
 //Serial.print(waqt);
  client.print("WB_T:");
  client.print(steinharta2);
  client.print("\t");
  //Serial.println();
  //Serial.print(avga3);
  //Serial.print(",");
  //Serial.print(flow3);
  //Serial.print(",");

  /*
  //Serial.print(waqt);
  Serial.print(" Block Inlet Temp ");
  Serial.print(steinharta3);
  Serial.print(",");
*/



//analogWrite(PWMPeltier, 250);

//Creating PID to always keep the peltier running

  float braintemp_atm;
  braintemp_atm = steinharta1; //This is the current brain temperature
  InputPeltierPID = braintemp_atm;
  myPID_peltier.Compute();
  //OutputPeltierPID = 0;

  if (Serial.available()>0) {
  //stringa = Serial.readString();
  //CompSetPeltier = stringa.toInt();
}

  analogWrite(DACPIN, 255-OutputPeltierPID);
 //analogWrite(DACPIN, 255-CompSetPeltier);
  client.print("P_OUT(div10):");
  client.print(OutputPeltierPID/10);
  //Serial.print(CompSetPeltier/10);
  client.print("\t");
  


//PID Loop control for Water Pumps
  float WBTemp_atm;
  WBTemp_atm = steinharta2;

  if (WBTemp_atm >= WBTempIdeal) {
    InputWB = WBTemp_atm;
    myPID_WB.Compute();
    freq = (OutputWB * (60)) / 255;
    client.print("WB_OUT(div10):");
    client.print(OutputWB/10);
    pwm.period_raw(50000000/freq); ///50000000 might need some adjustmnet, it is rough ballpark
    pwm.pulse_perc(50.0f);
    client.print("\t");
    client.print("FREQ:");
    client.print(freq);
}
/*else {freq = 0;

Serial.print("WB_OUT(div10):");
 Serial.print(10/10);
 pwm.period_raw(50000000/freq); ///50000000 might need some adjustmnet, it is rough ballpark
 pwm.pulse_perc(50.0f);
 Serial.print("\t");
 Serial.print("FREQ:");
 Serial.print(freq); */


//Peltier Current and Voltage Sensing
  float peltier_current;
  peltier_current = ina260_peltier.readCurrent()/1000;

  //Turning off Peltier if drawing too much current
  if (peltier_current >= 2) {
  OutputPeltierPID = 0;
  client.print("ERROR: Peltier Overcurrenting");
  }


  client.print("\t");
  client.print("P_Curr:");
  client.print(peltier_current);
 
 
  client.print("\t");
  client.print("P_Volt");
  client.print(ina260_peltier.readBusVoltage()/1000);

//Water Pump Voltage and Current Sensing
  client.print("\t");
  client.print("WP_Curr:");
  client.print(ina260_waterpump.readCurrent()/1000);

  client.print("\t");
  client.print("WP_Volt");
  client.print(ina260_waterpump.readBusVoltage()/1000);


//Getting info from flow sensor
 float aFlow = 0.0;
  float aTemperature = 0.0;    
  uint16_t aSignalingFlags = 0u;
 flow_sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3S_4000B, aFlow, aTemperature, aSignalingFlags);
  client.print("\t");
  client.print("aFlow: ");
  client.print(aFlow);
  client.print("aTemperature: ");
  client.print(aTemperature);
  //Serial.print("\t");
  //Serial.print("aSignalingFlags: ");
 // Serial.print(aSignalingFlags);


//Pressure Sensor Detection
  int total = 0;
  int sig_pressure = setMux(1, 3);
  for (int i = 0; i < sampleSize; i++) {
    // Read the voltage from the pressure sensor
    // analogRead() returns an integer between 0-1023 (or 0-16383 if resolution changed to 14-bit)
    int sensorValue = analogRead(sig_pressure);
    total += sensorValue;
    delay(0);
  }

  // Find the average of the sensor value readings 
  int averageSensorValue = total / sampleSize;
  client.print("\t");
  client.println(averageSensorValue);

  // Convert the analog reading to voltage (0-5V)
  //float outputVoltage = averageSensorValue * (5.0 / 16383.0);
  //float outputVoltage = averageSensorValue * (5.0 / 1023.0);
  float outputVoltage = averageSensorValue * (3.2 / 1023.0);

  // pressureApplied = 15/(0.8*5)*(Vout-0.5) + 0
  float pressureApplied = (15 / (0.8 * 5)) * (outputVoltage - 0.5) + 0;

  // Print the pressure to the serial monitor
  client.print("\t");
  client.print(pressureApplied, decimalPlaces);
  client.println("psi");

  
// If pressure is too high/low, it turns the system off
  float lowpressure = 0.75; // Set pressures once actual known
  float highpressure = 2;
  if (pressureApplied < lowpressure || pressureApplied > highpressure )
    {
      analogWrite(DACPIN, 0); //Turning peltier off
      freq = 0; //Turning off water pumps
      pwm.period_raw(50000000/freq);
      pwm.pulse_perc(50.0f);
      client.print("ERROR: Aberrant Pressure Levels");
      client.println(pressureApplied);
  }

// If water block temperature is 6 degrees above baseline and flow rate max, sets a countdown to change or else will turn off
  if (WBTemp_atm >= WBTempMax && freq == 60) 
    {
      //Start a countdown that lasts 3 min. Then, start measuring. If it does not cool down 3 degrees in those 3 minutes, stop pumps and piezo
      int starttime = millis();
      int endtime = starttime;
      int loopcount;
      while ((endtime - starttime) <= 180000) // 1000 ms/s * 60s/min * 3 min
        {   
          // code here
          analogWrite(DACPIN, 255); //Turning peltier off
          freq = 60; //Turning off water pumps
          pwm.period_raw(50000000/freq);
          pwm.pulse_perc(50.0f);
          
          loopcount = loopcount+1;
          endtime = millis();
          }

      if (WBTemp_atm >= (WBTempIdeal + 3))
      { 
        analogWrite(DACPIN, 0); //Turning peltier off
        freq = 0; //Turning off water pumps
        pwm.period_raw(50000000/freq);
        pwm.pulse_perc(50.0f);
        client.println("CRITICAL ERROR: Cooling failed, Shutting system down");
        }
      }



//Writing data to the SD Card
    myFile = SD.open("Testing.txt", FILE_WRITE); //Opening the file here
    
    myFile.print(currentTime.getDayOfMonth());
    myFile.print("/");
    myFile.print(Month2int(currentTime.getMonth()));
    myFile.print("/");
    myFile.print(currentTime.getYear());
    myFile.print(",");
    myFile.print(currentTime.getHour());
    myFile.print(":");
    myFile.print(currentTime.getMinutes());
    myFile.print(":");
    myFile.print(currentTime.getSeconds());
    myFile.print(",");

    myFile.print(steinharta1);
    myFile.print(",");
    myFile.print(steinharta2); //Saving the data here. It is done like this so easy import to CSV format
    myFile.print(",");
    myFile.print(peltier_current);
    myFile.print(",");
    myFile.print(ina260_peltier.readBusVoltage()/1000);
    myFile.print(",");
    myFile.print(ina260_waterpump.readCurrent()/1000);
    myFile.print(",");
    myFile.print(ina260_waterpump.readBusVoltage()/1000);
    myFile.print(",");
    myFile.print(aFlow);
    myFile.print(",");
    myFile.print(pressureApplied);
    myFile.print(",");
    myFile.println(clientStatus);
    myFile.close();
    
    }




//Anything below is when no client connected




    else{

      RTCTime currentTime;
        RTC.getTime(currentTime);

      int clientStatus = 0;
      Serial.print("Client Status");
      Serial.print(clientStatus);
      Serial.print("\t");

 

      Serial.print(currentTime.getDayOfMonth());
      Serial.print("/");
      Serial.print(Month2int(currentTime.getMonth()));
      Serial.print("/");
      Serial.print(currentTime.getYear());
      Serial.print("\t");

      Serial.print(currentTime.getHour());
      Serial.print(":");
      Serial.print(currentTime.getMinutes());
      Serial.print(":");
      Serial.print(currentTime.getSeconds());
      Serial.print("\t");

//Getting temperatures
  uint8_t i;
  //Finding the right channel for thermistors on multiplexer
  
    waqt = millis()/1000;
      for (i=0; i < NUMSAMPLES; i++) {          // take N samples in a row, with a slight delay
      int sig_therm1 = setMux(1, 0); //Channel 0 for Peltier thermistor
      samplesa1[i] = analogRead(sig_therm1); //Reading Peltier thermistor

      int sig_therm2 = setMux(1, 1); //Channel 1 for Water block thermistor
      samplesa2[i] = analogRead(sig_therm2); //Reading water block thermistor
      //samplesa3[i] = analogRead(THERMISTORPIN3);
    delay(100);
  }



  float avga1;
  float avga2;
  //float avga3;

 // average all the samples out
  avga1 = 0;
  avga2 = 0;
  //avga3 = 0;
 
  for (i=0; i< NUMSAMPLES; i++) {
     avga1 += samplesa1[i];
     avga2 += samplesa2[i];
     //avga3 += samplesa3[i];
  }

  avga1 = SERIESRESISTOR / (1023 /(avga1/NUMSAMPLES) - 1);
  avga2 = SERIESRESISTOR / (1023 /(avga2/NUMSAMPLES) - 1);
  //avga3 = avga3/NUMSAMPLES;

  //float flow3 = 100 * (avga3 - 0.045);

  float steinharta1;
  float steinharta2;
  //float steinharta3;


   // (R/Ro)
  steinharta1 = 1/((log(avga1 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
  steinharta2 = 1/((log(avga2 / THERMISTORNOMINAL))/BCOEFFICIENTL + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
 // steinharta3 = 1/((log(avga3 / THERMISTORNOMINAL))/BCOEFFICIENTA + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;


  //Serial.print(waqt);
  Serial.print("P_T:");
  Serial.print(steinharta1);
  Serial.print("\t");
 // Serial.print(waqt);
  //Serial.println();
 //Serial.print(waqt);
  Serial.print("WB_T:");
  Serial.print(steinharta2);
  Serial.print("\t");
  //Serial.println();
  //Serial.print(avga3);
  //Serial.print(",");
  //Serial.print(flow3);
  //Serial.print(",");

  /*
  //Serial.print(waqt);
  Serial.print(" Block Inlet Temp ");
  Serial.print(steinharta3);
  Serial.print(",");
*/



//analogWrite(PWMPeltier, 250);

//Creating PID to always keep the peltier running

  float braintemp_atm;
  braintemp_atm = steinharta1; //This is the current brain temperature
  InputPeltierPID = braintemp_atm;
  myPID_peltier.Compute();
  //OutputPeltierPID = 0;

  if (Serial.available()>0) {
  //stringa = Serial.readString();
  //CompSetPeltier = stringa.toInt();
}

  analogWrite(DACPIN, 255-OutputPeltierPID);
 //analogWrite(DACPIN, 255-CompSetPeltier);
  Serial.print("P_OUT(div10):");
  Serial.print(OutputPeltierPID/10);
  //Serial.print(CompSetPeltier/10);
  Serial.print("\t");
  


//PID Loop control for Water Pumps
  float WBTemp_atm;
  WBTemp_atm = steinharta2;

  if (WBTemp_atm >= WBTempIdeal) {
    InputWB = WBTemp_atm;
    myPID_WB.Compute();
    freq = (OutputWB * (60)) / 255;
    Serial.print("WB_OUT(div10):");
    Serial.print(OutputWB/10);
    pwm.period_raw(50000000/freq); ///50000000 might need some adjustmnet, it is rough ballpark
    pwm.pulse_perc(50.0f);
    Serial.print("\t");
    Serial.print("FREQ:");
    Serial.print(freq);
}
/*else {freq = 0;

Serial.print("WB_OUT(div10):");
 Serial.print(10/10);
 pwm.period_raw(50000000/freq); ///50000000 might need some adjustmnet, it is rough ballpark
 pwm.pulse_perc(50.0f);
 Serial.print("\t");
 Serial.print("FREQ:");
 Serial.print(freq); */


//Peltier Current and Voltage Sensing
  float peltier_current;
  peltier_current = ina260_peltier.readCurrent()/1000;

  //Turning off Peltier if drawing too much current
  if (peltier_current >= 2) {
  OutputPeltierPID = 0;
  Serial.print("ERROR: Peltier Overcurrenting");
  }


  Serial.print("\t");
  Serial.print("P_Curr:");
  Serial.print(peltier_current);
 
 
  Serial.print("\t");
  Serial.print("P_Volt");
  Serial.print(ina260_peltier.readBusVoltage()/1000);

//Water Pump Voltage and Current Sensing
  Serial.print("\t");
  Serial.print("WP_Curr:");
  Serial.print(ina260_waterpump.readCurrent()/1000);

  Serial.print("\t");
  Serial.print("WP_Volt");
  Serial.print(ina260_waterpump.readBusVoltage()/1000);


//Getting info from flow sensor
 float aFlow = 0.0;
  float aTemperature = 0.0;    
  uint16_t aSignalingFlags = 0u;
 flow_sensor.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3S_4000B, aFlow, aTemperature, aSignalingFlags);
  Serial.print("\t");
  Serial.print("aFlow: ");
  Serial.print(aFlow);
  Serial.print("aTemperature: ");
  Serial.print(aTemperature);
  //Serial.print("\t");
  //Serial.print("aSignalingFlags: ");
 // Serial.print(aSignalingFlags);


//Pressure Sensor Detection
  int total = 0;
  int sig_pressure = setMux(1, 3);
  for (int i = 0; i < sampleSize; i++) {
    // Read the voltage from the pressure sensor
    // analogRead() returns an integer between 0-1023 (or 0-16383 if resolution changed to 14-bit)
    int sensorValue = analogRead(sig_pressure);
    total += sensorValue;
    delay(0);
  }

  // Find the average of the sensor value readings 
  int averageSensorValue = total / sampleSize;
  Serial.print("\t");
  Serial.println(averageSensorValue);

  // Convert the analog reading to voltage (0-5V)
  //float outputVoltage = averageSensorValue * (5.0 / 16383.0);
  //float outputVoltage = averageSensorValue * (5.0 / 1023.0);
  float outputVoltage = averageSensorValue * (3.2 / 1023.0);

  // pressureApplied = 15/(0.8*5)*(Vout-0.5) + 0
  float pressureApplied = (15 / (0.8 * 5)) * (outputVoltage - 0.5) + 0;

  // Print the pressure to the serial monitor
  Serial.print("\t");
  Serial.print(pressureApplied, decimalPlaces);
  Serial.println("psi");

  
// If pressure is too high/low, it turns the system off
  float lowpressure = 0.75; // Set pressures once actual known
  float highpressure = 2;
  if (pressureApplied < lowpressure || pressureApplied > highpressure )
    {
      analogWrite(DACPIN, 0); //Turning peltier off
      freq = 0; //Turning off water pumps
      pwm.period_raw(50000000/freq);
      pwm.pulse_perc(50.0f);
      Serial.print("ERROR: Aberrant Pressure Levels");
      Serial.println(pressureApplied);
  }

// If water block temperature is 6 degrees above baseline and flow rate max, sets a countdown to change or else will turn off
  if (WBTemp_atm >= WBTempMax && freq == 60) 
    {
      //Start a countdown that lasts 3 min. Then, start measuring. If it does not cool down 3 degrees in those 3 minutes, stop pumps and piezo
      int starttime = millis();
      int endtime = starttime;
      int loopcount;
      while ((endtime - starttime) <= 180000) // 1000 ms/s * 60s/min * 3 min
        {   
          // code here
          analogWrite(DACPIN, 255); //Turning peltier off
          freq = 60; //Turning off water pumps
          pwm.period_raw(50000000/freq);
          pwm.pulse_perc(50.0f);
          
          loopcount = loopcount+1;
          endtime = millis();
          }

      if (WBTemp_atm >= (WBTempIdeal + 3))
      { 
        analogWrite(DACPIN, 0); //Turning peltier off
        freq = 0; //Turning off water pumps
        pwm.period_raw(50000000/freq);
        pwm.pulse_perc(50.0f);
        Serial.println("CRITICAL ERROR: Cooling failed, Shutting system down");
        }
      }


//Writing data to the SD Card
    myFile = SD.open("Testing.txt", FILE_WRITE); //Opening the file here
    
    myFile.print(currentTime.getDayOfMonth());
    myFile.print("/");
    myFile.print(Month2int(currentTime.getMonth()));
    myFile.print("/");
    myFile.print(currentTime.getYear());
    myFile.print(",");
    myFile.print(currentTime.getHour());
    myFile.print(":");
    myFile.print(currentTime.getMinutes());
    myFile.print(":");
    myFile.print(currentTime.getSeconds());
    myFile.print(",");

    myFile.print(steinharta1);
    myFile.print(",");
    myFile.print(steinharta2); //Saving the data here. It is done like this so easy import to CSV format
    myFile.print(",");
    myFile.print(peltier_current);
    myFile.print(",");
    myFile.print(ina260_peltier.readBusVoltage()/1000);
    myFile.print(",");
    myFile.print(ina260_waterpump.readCurrent()/1000);
    myFile.print(",");
    myFile.print(ina260_waterpump.readBusVoltage()/1000);
    myFile.print(",");
    myFile.print(aFlow);
    myFile.print(",");
    myFile.print(pressureApplied);
    myFile.print(",");
    myFile.println(clientStatus);
    myFile.close();


    }

}

//This code helps us find the right channel for the analog sensors. 
//Mux is what multiplexer it is.
//Channel is what channel it is connected to on multiplexer.
int setMux(int mux, int channel) {

  // declare local array for control pins (S0-S3) and variable for SIG pin
  int controlPin[4];
  int sigPin;

  // set the correct values for each MUX's control pins and SIG pins.
  //Currently, coded for one multiplexer. If second one connected, need to copy next few lines and change mux == 2 and mux1 to mux2
  if (mux == 1) {
    controlPin[0] = mux1S0;
    controlPin[1] = mux1S1;
    controlPin[2] = mux1S2;
    controlPin[3] = mux1S3;
    sigPin = mux1Sig;
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

//Code to set up the wifi access point appropriately
void setupWiFiAP() {

  // check for the WiFi module:
  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true)
      ;
  }

  // check firmware version
  String fv = WiFi.firmwareVersion();
  if (fv < WIFI_FIRMWARE_LATEST_VERSION) {
    Serial.println("Please upgrade the firmware");
  }

  // by default the local IP address will be 192.168.4.1
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
    while (true)
      ;
  }


}
