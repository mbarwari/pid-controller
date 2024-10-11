
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
  #include <PwmOut.h>
  #include <AdvancedDAC.h>
  #include <Arduino_AdvancedAnalog.h>

// Digital pins for the IRQn pins of the flow sensors to connect to
  #define IRQN_PIN_FLOW_SENSOR_A 16
  #define IRQN_PIN_FLOW_SENSOR_B 17
  //#define IRQN_PIN_FLOW_SENSOR_C 18

// I2C addresses for the flow sensors
  #define I2C_ADDR_FLOW_SENSOR_A 0x0A
  #define I2C_ADDR_FLOW_SENSOR_B 0x0B
  //#define I2C_ADDR_FLOW_SENSOR_C 0x0C

// Define the flow sensor objects 
  SensirionI2cSf06Lf flowSensorA;
  SensirionI2cSf06Lf flowSensorB;
  //SensirionI2cSf06Lf flowSensorC;

// Error message for flow sensor
  static char errorMessage[64];
  static uint16_t error1;

// Define current sensor objects and I2C addresses 
  Adafruit_INA260 pumpINA260 = Adafruit_INA260();
  Adafruit_INA260 peltierINA260 = Adafruit_INA260();
  uint8_t pumpI2CAddress = 0x40;
  uint8_t peltierI2CAddress = 0x41;

// Pressure sensor specifications
  const float vSupply = 5.0;  // Supply voltage
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
  const int sampleSize = 10;   // Sample size
  const int NUMSAMPLES = 10;
  File myFile; //Initialization of SD card reader
  //PinName pin = digitalPinToPinName(D11); //This is for the water block controller. Can be any digital pin, make changes to correct pin here
  #define DACPIN A12
  int decimalPlaces = 3;

// WIFI related global variables, macros, and object
  #define SECRET_SSID "pigTrial"
  #define SECRET_PASS "123456789"
  char ssid[] = SECRET_SSID;  // your network SSID (name)
  char pass[] = SECRET_PASS;  // your network password (use for WPA, or use as key for WEP)
  int status = WL_IDLE_STATUS;
  WiFiServer server(80); //Keep 80 if connecting from a webpage

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
    double SetpointPeltier, InputPeltierPID, OutputPeltierPID; //Generating the input variables for the peltier plate cooling PID -- Setpoint is goal; input is what is detected; output is PID output 0-255
    double Kp_peltier=17, Ki_peltier=3, Kd_peltier=2; //Specify the initial tuning parameters. Ignoring Kd since not used often. -- watch youtube video: https://www.youtube.com/watch?v=IB1Ir4oCP5k
    PID myPID_peltier(&InputPeltierPID, &OutputPeltierPID, &SetpointPeltier, Kp_peltier, Ki_peltier, Kd_peltier, DIRECT, REVERSE); //By default, PID "warms" up, so we have to reverse it -- unknown what DIRECT is for

  //Water Block PID Loop
    double SetpointWB, InputWB, OutputWB; // setpoint is desired temp, input is current temp, output is 0-255
    double Kp_WB=15, Ki_WB=10, Kd_WB=0;
    PID myPID_WB(&InputWB, &OutputWB, &SetpointWB, Kp_WB, Ki_WB, Kd_WB, DIRECT, REVERSE);   
    int freq = 0;

  //Temperature controls for PID loops
     //These are parameters that can be adjusted for temperature cutoffs -- TempIdeals are equivalent to Setpoint
    int BrainTempIdeal = 10; //Ideal brain temperature, currently 12 for testing purposes (should be 25)
    int WBTempMax = 100; //The maximum peltier plate temperature, currently 25 for testing purposes (should be 40)
    int WBTempIdeal = 20; //The ideal temperature of the WB, currently 22 for testing purposes (should be 35) 

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


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Wire1.begin();
   while (!Serial) {};
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
    //Changing Flow Sensor Addresses to be correct and starting them
      flowSensorSetUp();
    //Starting up Current and Voltage Sensors
      currentSensorSetUp();

      

  //Setting up WiFi
    Serial.println("WIFI setup: "); 
    // set up the WiFi Access Point
      setupWiFiAP();
    // start the WiFi server
      // the server will listen for incoming connections on port 80 (the default port for HTTP)
    server.begin();

  //Setting up RTC for Data Time Saving. Adjust to date of implantation
    RTCset();

  //Initializing SD Card
    //Default CS pin is 4. Change to current pin if not working
    if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
    }
    Serial.println("initialization done.");

    //Opening the file we want to save in
      myFile = SD.open("Testing.txt", FILE_WRITE); //Change file name here
      //The line below establishes column headers for save file. Always seperate with comma when adding new ones
      myFile.println("Date,Time,Peltier Temp (C),Water Block Temp (C),Peltier Current (A),Peltier Voltage (V),Water Pump Current (A),Water Pump Voltage (V),Flow (mL),Pressure (psi),WiFI Client Status (0 = USB, 1 = WiFi");
      myFile.close(); 

  //Turning on Power to Pump and Peltier
    pinMode(peltierRelay, OUTPUT);
    pinMode(pumpRelay, OUTPUT);
    digitalWrite(peltierRelay, LOW); //For a normally open circuit and these relay specs, low means relay on and high means relay off.
    digitalWrite(pumpRelay, LOW); 

  //Initializing PWM pin
    /*mbed::PwmOut* pwm = new mbed::PwmOut(pin);
    pwm->period_ms(0); //1kHz
    pwm->pulsewidth_us(0);*/
  
  //Initializing DAC
    pinMode(DACPIN, OUTPUT);

    Serial.print("1");
}

void loop() {
  // put your main code here, to run repeatedly:

 // mbed::PwmOut* pwm = new mbed::PwmOut(pin);

  //Determining if Wifi is Connected or Not
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

  Serial.print("2");

  //Establishing Variable for printing depending on connection or not
    WiFiClient client = server.available();  
    int clientStatus;
    if (client) {
    // if you get a client, set clientStatus to 1, if not, set clientStatus to 0
    clientStatus = 1;
    } else{
      clientStatus = 0;
    }

Serial.print("3");
  //Getting all analog sensor data at once
    uint8_t i;
    waqt = millis()/1000;
    for (i=0; i < NUMSAMPLES; i++) { 
      //Intraarray Temperatures
        int intraArray1 = setMux(1,0); 
        samples1[i] = analogRead(intraArray1);
        int intraArray2 = setMux(1,1);
        samples2[i] = analogRead(intraArray2);
        int intraArray3 = setMux(1,2);
        samples3[i] = analogRead(intraArray3);
        int intraArray4 = setMux(1,3);
        samples4[i] = analogRead(intraArray4);

      //Extraarray Temperatures
        int extraArray1 = setMux(1,4);
        samples5[i] = analogRead(extraArray1);
        int extraArray2 = setMux(1,5);
        samples6[i] = analogRead(extraArray2);
        int extraArray3 = setMux(1,6);
        samples7[i] = analogRead(extraArray3);

      //Scalp Water Block (SWB) Temperatures
        int SWB1 = setMux(1,7);
        samples8[i] = analogRead(SWB1);
        int SWB2 = setMux(1,8);
        samples9[i] = analogRead(SWB2);
        int SWB3 = setMux(1,9);
        samples10[i] = analogRead(SWB3);

      //Scalp Water Block Entrance (entrSWB) Temperatures
        int entrSWB1 = setMux(1,10);
        samples11[i] = analogRead(entrSWB1);
        int entrSWB2 = setMux(1,11);
        samples12[i] = analogRead(entrSWB2);
        int entrSWB3 = setMux(1,12);
        samples13[i] = analogRead(entrSWB3);

      //Scalp Water Block Exit (exitSWB) Temperatures
        int exitSWB1 = setMux(1,13);
        samples14[i] = analogRead(exitSWB1);
        int exitSWB2 = setMux(1,14);
        samples15[i] = analogRead(exitSWB2); 
        int exitSWB3 = setMux(1,15);
        samples16[i] = analogRead(exitSWB3);   

      //Body Water Block Entrance (entrBWB) Temperatures
        int entrBWB1 = setMux(2,0);
        samples17[i] = analogRead(entrBWB1);
        int entrBWB2 = setMux(2,1);
        samples18[i] = analogRead(entrBWB2);
        int entrBWB3 = setMux(2,2);
        samples19[i] = analogRead(entrBWB3);
    
      //Body Water Block (BWB) Temperatures
        int BWB1 = setMux(2,3);
        samples20[i] = analogRead(BWB1);
        int BWB2 = setMux(2,4);
        samples21[i] = analogRead(BWB2);
        int BWB3 = setMux(2,5);
        samples22[i] = analogRead(BWB3);
        int BWB4 = setMux(2,6);
        samples23[i] = analogRead(BWB4);
        int BWB5 = setMux(2,7);
        samples24[i] = analogRead(BWB5);
        int BWB6 = setMux(2,8);
        samples25[i] = analogRead(BWB6);

      //Body Water Block Exit (exitBWB) Temperatures
        int exitBWB1 = setMux(2,9);
        samples26[i] = analogRead(exitBWB1);
        int exitBWB2 = setMux(2,10);
        samples27[i] = analogRead(exitBWB2);
        int exitBWB3 = setMux(2,11);
        samples28[i] = analogRead(exitBWB3);

      //Pressure Sensors
        int pressureSensor1 = setMux(2,12);
        samples29[i] = analogRead(pressureSensor1);
        int pressureSensor2 = setMux(2,13);
        samples30[i] = analogRead(pressureSensor2);
        int pressureSensor3 = setMux(2,14);
        samples31[i] = analogRead(pressureSensor3);
        int pressureSensor4 = setMux(2,15);
        samples32[i] = analogRead(pressureSensor4);

      delay(10);
    }
Serial.print(4);
  
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
    float avgPressureSensor3;
    float avgPressureSensor4;

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
    avgPressureSensor3 = 0;
    avgPressureSensor4 = 0;

    for (i=0; i< NUMSAMPLES; i++) {
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
      avgPressureSensor3 += samples31[i];
      avgPressureSensor4 += samples32[i];
    }

    avgIntraArray1 = avgIntraArray1/NUMSAMPLES;
    avgIntraArray2 = avgIntraArray2/NUMSAMPLES;
    avgIntraArray3 = avgIntraArray3/NUMSAMPLES;
    avgIntraArray4 = avgIntraArray4/NUMSAMPLES;
    avgExtraArray1 = avgExtraArray1/NUMSAMPLES;
    avgExtraArray2 = avgExtraArray2/NUMSAMPLES;
    avgExtraArray3 = avgExtraArray3/NUMSAMPLES;
    avgSWB1 = avgSWB1/NUMSAMPLES;
    avgSWB2 = avgSWB2/NUMSAMPLES;
    avgSWB3 = avgSWB3/NUMSAMPLES;
    avgEntrSWB1 = avgEntrSWB1/NUMSAMPLES;
    avgEntrSWB2 = avgEntrSWB2/NUMSAMPLES;
    avgEntrSWB3 = avgEntrSWB3/NUMSAMPLES;
    avgExitSWB1 = avgExitSWB1/NUMSAMPLES;
    avgExitSWB2 = avgExitSWB2/NUMSAMPLES;
    avgExitSWB3 = avgExitSWB3/NUMSAMPLES;
    avgEntrBWB1 = avgEntrBWB1/NUMSAMPLES;
    avgEntrBWB2 = avgEntrBWB2/NUMSAMPLES;
    avgEntrBWB3 = avgEntrBWB3/NUMSAMPLES;
    avgBWB1 = avgBWB1/NUMSAMPLES;
    avgBWB2 = avgBWB2/NUMSAMPLES;
    avgBWB3 = avgBWB3/NUMSAMPLES;
    avgBWB4 = avgBWB4/NUMSAMPLES;
    avgBWB5 = avgBWB5/NUMSAMPLES;
    avgBWB6 = avgBWB6/NUMSAMPLES;
    avgExitBWB1 = avgExitBWB1/NUMSAMPLES;
    avgExitBWB2 = avgExitBWB2/NUMSAMPLES;
    avgExitBWB3 = avgExitBWB3/NUMSAMPLES;
    avgPressureSensor1 = avgPressureSensor1/NUMSAMPLES;
    avgPressureSensor2 = avgPressureSensor2/NUMSAMPLES;
    avgPressureSensor3 = avgPressureSensor3/NUMSAMPLES;
    avgPressureSensor4 = avgPressureSensor4/NUMSAMPLES;
    
Serial.print("5");    

  //Converting temp sensors data into actual degrees (Celsius)
    //First convert to a resistance value
      float avgIntraArray1Resist = SERIESRESISTOR / (1023 /(avgIntraArray1) - 1);
      float avgIntraArray2Resist = SERIESRESISTOR / (1023 /(avgIntraArray2) - 1);
      float avgIntraArray3Resist = SERIESRESISTOR / (1023 /(avgIntraArray3) - 1);
      float avgIntraArray4Resist = SERIESRESISTOR / (1023 /(avgIntraArray4) - 1);
      float avgExtraArray1Resist = SERIESRESISTOR / (1023 /(avgExtraArray1) - 1);
      float avgExtraArray2Resist = SERIESRESISTOR / (1023 /(avgExtraArray2) - 1);
      float avgExtraArray3Resist = SERIESRESISTOR / (1023 /(avgExtraArray3) - 1);
      float avgSWB1Resist = SERIESRESISTOR / (1023 /(avgSWB1) - 1);
      float avgSWB2Resist = SERIESRESISTOR / (1023 /(avgSWB2) - 1);
      float avgSWB3Resist = SERIESRESISTOR / (1023 /(avgSWB3) - 1);
      float avgEntrSWB1Resist = SERIESRESISTOR / (1023 /(avgEntrSWB1) - 1);
      float avgEntrSWB2Resist = SERIESRESISTOR / (1023 /(avgEntrSWB2) - 1);
      float avgEntrSWB3Resist = SERIESRESISTOR / (1023 /(avgEntrSWB3) - 1);
      float avgExitSWB1Resist = SERIESRESISTOR / (1023 /(avgExitSWB1) - 1);
      float avgExitSWB2Resist = SERIESRESISTOR / (1023 /(avgExitSWB2) - 1);
      float avgExitSWB3Resist = SERIESRESISTOR / (1023 /(avgExitSWB3) - 1);
      float avgEntrBWB1Resist = SERIESRESISTOR / (1023 /(avgEntrBWB1) - 1);
      float avgEntrBWB2Resist = SERIESRESISTOR / (1023 /(avgEntrBWB2) - 1);
      float avgEntrBWB3Resist = SERIESRESISTOR / (1023 /(avgEntrBWB3) - 1);
      float avgBWB1Resist = SERIESRESISTOR / (1023 /(avgBWB1) - 1);
      float avgBWB2Resist = SERIESRESISTOR / (1023 /(avgBWB2) - 1);
      float avgBWB3Resist = SERIESRESISTOR / (1023 /(avgBWB3) - 1);
      float avgBWB4Resist = SERIESRESISTOR / (1023 /(avgBWB4) - 1);
      float avgBWB5Resist = SERIESRESISTOR / (1023 /(avgBWB5) - 1);
      float avgBWB6Resist = SERIESRESISTOR / (1023 /(avgBWB6) - 1);
      float avgExitBWB1Resist = SERIESRESISTOR / (1023 /(avgExitBWB1) - 1);
      float avgExitBWB2Resist = SERIESRESISTOR / (1023 /(avgExitBWB2) - 1);
      float avgExitBWB3Resist = SERIESRESISTOR / (1023 /(avgExitBWB3) - 1);
    //Convert Resistance to Temperature
      float steinhartIntraArray1 = 1/((log(avgIntraArray1Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartIntraArray2 = 1/((log(avgIntraArray2Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartIntraArray3 = 1/((log(avgIntraArray3Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartIntraArray4 = 1/((log(avgIntraArray4Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartExtraArray1 = 1/((log(avgExtraArray1Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartExtraArray2 = 1/((log(avgExtraArray2Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartExtraArray3 = 1/((log(avgExtraArray3Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartSWB1 = 1/((log(avgSWB1Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartSWB2 = 1/((log(avgSWB2Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartSWB3 = 1/((log(avgSWB3Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartEntrSWB1 = 1/((log(avgEntrSWB1Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartEntrSWB2 = 1/((log(avgEntrSWB2Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartEntrSWB3 = 1/((log(avgEntrSWB3Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartExitSWB1 = 1/((log(avgExitSWB1Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartExitSWB2 = 1/((log(avgExitSWB2Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartExitSWB3 = 1/((log(avgExitSWB3Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartEntrBWB1 = 1/((log(avgEntrBWB1Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartEntrBWB2 = 1/((log(avgEntrBWB2Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartEntrBWB3 = 1/((log(avgEntrBWB3Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartBWB1 = 1/((log(avgBWB1Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartBWB2 = 1/((log(avgBWB2Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartBWB3 = 1/((log(avgBWB3Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartBWB4 = 1/((log(avgBWB4Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartBWB5 = 1/((log(avgBWB5Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartBWB6 = 1/((log(avgBWB6Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartExitBWB1 = 1/((log(avgExitBWB1Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartExitBWB2 = 1/((log(avgExitBWB2Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
      float steinhartExitBWB3 = 1/((log(avgExitBWB3Resist / THERMISTORNOMINAL))/BCOEFFICIENT + 1.0 / (TEMPERATURENOMINAL + 273.15))-273.15;
Serial.print("6");
  //Average multiple temp sensors into 1 temperature
    float IntraArrayTemp = (steinhartIntraArray1 + steinhartIntraArray2 + steinhartIntraArray3 + steinhartIntraArray4) / 4;
    float ExtraArrayTemp = (steinhartExtraArray1 + steinhartExtraArray2 + steinhartExtraArray3) / 3;
    float SWBTemp = (steinhartSWB1 + steinhartSWB2 + steinhartSWB3) / 3;
    float EntrSWBTemp = (steinhartEntrSWB1 + steinhartEntrSWB2 + steinhartEntrSWB3) / 3;
    float ExitSWBTemp= (steinhartExitSWB1 + steinhartExitSWB2 + steinhartExitSWB3) / 3;
    float EntrBWBTemp = (steinhartEntrBWB1 + steinhartEntrBWB2 + steinhartEntrBWB3) / 3;
    float BWBTemp = (steinhartBWB1 + steinhartBWB2 + steinhartBWB3 + steinhartBWB4 + steinhartBWB5 + steinhartBWB6) / 6;
    float ExitBWBTemp = (steinhartExitBWB1 + steinhartExitBWB2 + steinhartExitBWB3) / 3;

  //Peltier Plate PID
    float braintemp_atm;
    braintemp_atm = (IntraArrayTemp + ExtraArrayTemp) / 2; //This is the current brain temperature
    InputPeltierPID = braintemp_atm;
    myPID_peltier.Compute();
    analogWrite(DACPIN, 255-OutputPeltierPID);

Serial.print("7");
  //Water Pump/Block PID
    float WBTemp_atm;
    WBTemp_atm = SWBTemp;
    if (WBTemp_atm >= WBTempIdeal) {
      InputWB = WBTemp_atm;
      myPID_WB.Compute();
      freq = (OutputWB * (60)) / 255;
      float period_millisec = 1/freq * 1000;
      //pwm->period_ms(period_millisec);
      //pwm->pulsewidth_us(period_millisec*1000/2);
    }
Serial.print("8");

  //Peltier Overcurrent and Overvoltage Protection
    float peltierCurrent;
    peltierCurrent = peltierINA260.readCurrent()/1000;
    float peltierVoltage;
    peltierVoltage = peltierINA260.readBusVoltage()/1000;
    //Turn on Power to peltier if too high
      if (peltierCurrent >= 2 || peltierVoltage >= 8) {
        analogWrite(DACPIN, 255);
        digitalWrite(peltierRelay, LOW);

        Serial.print("ERROR: Peltier Overcurrenting/volting");
        client.print("ERROR: Peltier Overcurrenting/volting");
      }
Serial.print("9");
  //Pump Overcurrent and Overvoltage Protection
    float pumpCurrent;
    pumpCurrent = pumpINA260.readCurrent()/1000;
    float pumpVoltage;
    pumpVoltage = pumpINA260.readBusVoltage()/1000;
    //Turn off Power to pump if too high
      if (pumpVoltage >= 5.3) {
        digitalWrite(pumpRelay, HIGH);
        freq = 0;
        float period_millisec = 1/freq * 1000;
        //pwm->period_ms(period_millisec);
        //pwm->pulsewidth_us(period_millisec*1000/2);
        analogWrite(DACPIN, 255);
        digitalWrite(peltierRelay, HIGH);

        Serial.println("ERROR: Water Pump Overvolting");
        client.println("ERROR: Water Pump Overvolting");
      }
Serial.print("10");
  //Flow Rate Sensor Detection
    float aFlow = 0.0;
    float aTemperature = 0.0;
    uint16_t aSignalingFlags = 0u;
    flowSensorA.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3S_4000B, aFlow, aTemperature, aSignalingFlags);
    float bFlow = 0.0;
    float bTemperature = 0.0;
    uint16_t bSignalingFlags = 0u;
    flowSensorB.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3S_4000B, bFlow, bTemperature, bSignalingFlags);
    //float cFlow = 0.0;
    //float cTemperature = 0.0;
    //uint16_t cSignalingFlags = 0u;
    //flowSensorC.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3S_4000B, cFlow, cTemperature, cSignalingFlags);
Serial.print("11");
  //Sudden Flow Rate Drop Detection
    if (aFlow = 0) {
        digitalWrite(pumpRelay, HIGH);
        freq = 0;
        float period_millisec = 1/freq * 1000;
        //pwm->period_ms(period_millisec);
        //pwm->pulsewidth_us(period_millisec*1000/2);
        analogWrite(DACPIN, 255);
        digitalWrite(peltierRelay, HIGH);

        Serial.println("ERROR: No Flow Detected");
        client.println("ERROR: No Flow Detected");
    }
Serial.print("12");
  //Detecting Pressure Values
    //Converting from input to voltage (conversion from spec sheet)
    float pressureSensor1Volt = avgPressureSensor1 * (3.2 / 1023.0);
    float pressureSensor2Volt = avgPressureSensor2 * (3.2 / 1023.0);
    float pressureSensor3Volt = avgPressureSensor3 * (3.2 / 1023.0);
    float pressureSensor4Volt = avgPressureSensor4 * (3.2 / 1023.0);

    //Converting Voltage to pressure (conversion from spec sheet)
    float pressureApplied1 = (15 / (0.8 * 5)) * (pressureSensor1Volt - 0.5) + 0;
    float pressureApplied2 = (15 / (0.8 * 5)) * (pressureSensor2Volt - 0.5) + 0;
    float pressureApplied3 = (15 / (0.8 * 5)) * (pressureSensor3Volt - 0.5) + 0;
    float pressureApplied4 = (15 / (0.8 * 5)) * (pressureSensor4Volt - 0.5) + 0;

  //Turn off system if abnormal pressure
    float lowPressure = 0.75;
    float highPressure = 2;
    if (pressureApplied1 < lowPressure || pressureApplied1 > highPressure ){
        analogWrite(DACPIN, 255); //DACPIN 0 means completely on, 255 means off
        freq = 0; //Turning off water pumps
        float period_millisec = 1/freq * 1000;
        //pwm->period_ms(period_millisec);
        //pwm->pulsewidth_us(period_millisec*1000/2);
        
        Serial.print("ERROR: Aberrant Pressure Levels");
        Serial.println(pressureApplied1);
        
        client.print("ERROR: Aberrant Pressure Levels");
        client.println(pressureApplied1);

    } 

  //Water block temp 6 degrees above baseline and max flow rate --> Countdown for 3 minutes activates
    if (WBTemp_atm >= WBTempMax && freq == 60) {
      startTime = startTime; //If maxxing out, keep the starttime of the max the same
    } else {
      int newTime = millis(); //If normal operations, update startime with current time
      startTime = newTime;
    }
      int TimeATM = millis();
    if (TimeATM - startTime >= 180000) { //1000 ms/s * 60 s/min * 3 min
      //If it is greater than 3 minutes, shutting everything down
      analogWrite(DACPIN, 255);
      freq = 0;
      float period_millisec = 1/freq * 1000;
      //pwm->period_ms(period_millisec);
      //pwm->pulsewidth_us(period_millisec*1000/2);
      digitalWrite(peltierRelay, HIGH);
      digitalWrite(pumpRelay, HIGH);
      
      Serial.println("CRITICAL ERROR: Cooling failed, Shutting system down");
      client.println("CRITICAL ERROR: Cooling failed, Shutting system down");
    }

  //Save Data to SD Card
    myFile = SD.open("Testing.txt", FILE_WRITE); //Opening the file here
    
    myFile.print(getLocaltime());
    myFile.print(",");

    myFile.print(braintemp_atm);
    myFile.print(",");
    myFile.print(WBTemp_atm); //Saving the data here. It is done like this so easy import to CSV format
    myFile.print(",");
    myFile.print(peltierCurrent);
    myFile.print(",");
    myFile.print(peltierINA260.readBusVoltage()/1000);
    myFile.print(",");
    myFile.print(pumpINA260.readCurrent()/1000);
    myFile.print(",");
    myFile.print(pumpINA260.readBusVoltage()/1000);
    myFile.print(",");
    myFile.print(aFlow);
    myFile.print(",");
    myFile.print(pressureApplied1);
    myFile.print(",");
    myFile.println(clientStatus);
    myFile.close();
 
  //Sending data to Serial
    //Printing Client Stats
      Serial.print("Client Status");
      Serial.print(clientStatus);
        Serial.print("\t");
    //Printing Time and Data
      Serial.print(getLocaltime());
        Serial.print("\t");
    //Printing Temperature Info
      Serial.print("Brain Temp:");
      Serial.print(braintemp_atm);
        Serial.print("\t");
      Serial.print("Water Block Temp:");
      Serial.print(WBTemp_atm);
        Serial.print("\t");
    //PID Strength
      Serial.print("P_OUT(div10):");
      Serial.print(OutputPeltierPID/10);
        Serial.print("\t");  
      Serial.print("WB_OUT(div10):");
      Serial.print(OutputWB/10);
        Serial.print("\t");
      Serial.print("FREQ:");
      Serial.print(freq);
        Serial.print("\t");
    //Current and Voltage Sensors
      Serial.print("Peltier Current:");
      Serial.print(peltierCurrent);
        Serial.print("\t");
      Serial.print("Peltier Voltage");
      Serial.print(peltierINA260.readBusVoltage()/1000);
        Serial.print("\t");
      Serial.print("Water Pump Current:");
      Serial.print(pumpINA260.readCurrent()/1000);
        Serial.print("\t");
      Serial.print("Water Pump Voltage:");
      Serial.print(pumpINA260.readBusVoltage()/1000);
        Serial.print("\t");
    //Flow Rate Sensors
      Serial.print("aFlow: ");
      Serial.print(aFlow);
        Serial.print("\t");
    //Pressure Sensors
      Serial.print(pressureApplied1, decimalPlaces);
      Serial.println("psi");
        Serial.print("\t");



  //Sending Data to Client
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
      client.print(peltierINA260.readBusVoltage()/1000);
        client.print("\t");
      client.print("Water Pump Current:");
      client.print(pumpINA260.readCurrent()/1000);
        client.print("\t");
      client.print("Water Pump Voltage:");
      client.print(pumpINA260.readBusVoltage()/1000);
        client.print("\t");
    //Flow Rate Sensors
      client.print("aFlow: ");
      client.print(aFlow);
        client.print("\t");
    //Pressure Sensors
      client.print(pressureApplied1, decimalPlaces);
      client.println("psi");
        client.print("\t");



}





/*
currentSensorSetUp() -  
Parameters - none
Return - none (function return type: void) 
*/
void currentSensorSetUp() {

  if (!pumpINA260.begin(pumpI2CAddress)) {
    Serial.println("Couldn't find pump current sensor");
    while (1);
  }
  Serial.println("Found pump current sensor");


  if (!peltierINA260.begin(peltierI2CAddress)) {
    Serial.println("Couldn't find peltier current sensor");
    while (1);
  }
  Serial.println("Found peltier current sensor");
  Serial.println();
}


/*
IMPORTANT - this function was copied from the program "exampleI2cAddressChange.ino" under the examples from the "Sensirion I2C SF06-LF" library   
flowSensorSetUp()
Parameters - none
Return - none (function return type: void) 
*/
void flowSensorSetUp() {
  error1 = NO_ERROR;

  Wire1.begin();

  // Make sure that sensors are in proper state to perform a address change by
  // doing a soft reset and not sending any other commands prior to the
  // address change procedure
  i2c_soft_reset();
  // SLF3x sensors need 25ms to start up after the reset
  delay(25);

  // Change address of the first sensor
  // Set IRQN_PIN_SENSOR_A to the GPIO pin number where you connected Pin 1
  // of your first sensor.
  error1 = changeSensorAddress(Wire1, I2C_ADDR_FLOW_SENSOR_A, IRQN_PIN_FLOW_SENSOR_A);
  if (error1 != NO_ERROR) {
    Serial.print("Error changing sensor address: ");
    errorToString(error1, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

  // Change address of the first sensor
  // Set IRQN_PIN_SENSOR_B to the GPIO pin number where you connected Pin 1
  // of your second sensor.
  error1 = changeSensorAddress(Wire1, I2C_ADDR_FLOW_SENSOR_B, IRQN_PIN_FLOW_SENSOR_B);
  if (error1 != NO_ERROR) {
    Serial.print("Error changing sensor address: ");
    errorToString(error1, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }


  // Initialize first sensor
  Serial.println("Initialising flow sensor A");
  flowSensorA.begin(Wire1, 0x0A);
  //readAndPrintSerial(sensorA);
  error1 = flowSensorA.startH2oContinuousMeasurement();
  if (error1 != NO_ERROR) {
    Serial.print("Error trying to execute startH2oContinuousMeasurement() for sensor A: ");
    errorToString(error1, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

  // Initialize second sensor
  Serial.println("Initialising flow sensor B");
  flowSensorB.begin(Wire1, 0x0B);
  //readAndPrintSerial(sensorB);
  error1 = flowSensorB.startH2oContinuousMeasurement();
  if (error1 != NO_ERROR) {
    Serial.print("Error trying to execute startH2oContinuousMeasurement() for sensor B: ");
    errorToString(error1, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    return;
  }

}


/*
IMPORTANT - this function was copied from the program "exampleI2cAddressChange.ino" under the examples from the "Sensirion I2C SF06-LF" library   
i2c_soft_reset()   
Parameters - none 
Return - none (function return type: void) 
*/
void i2c_soft_reset() {
  Wire1.beginTransmission(0x00);
  size_t writtenBytes = Wire1.write(0x06);
  uint8_t i2c_error = Wire1.endTransmission();
}


/*
IMPORTANT - this function was copied from the program "exampleI2cAddressChange.ino" under the examples from the "Sensirion I2C SF06-LF" library   
changeSensorAddress(TwoWire1& Wire1, uint16_t newI2cAddress, uint8_t sensorIrqPin)
Parameters - 
  TwoWire& Wire1
  uint16_t newI2cAddress 
  uint8_t sensorIrqPin
Return - NO_ERROR (function return type: int16_t) 
*/
int16_t changeSensorAddress(TwoWire& Wire1, uint16_t newI2cAddress, uint8_t sensorIrqPin) {
  uint8_t communication_buffer[5] = { 0 };
  int16_t localError = NO_ERROR;
  uint8_t* buffer_ptr = communication_buffer;

  // Send I2C address change command 0x3661 with the new I2C address as a
  // parameter (including CRC for the parameter)
  SensirionI2CTxFrame txFrame = SensirionI2CTxFrame::createWithUInt16Command(0x3661, buffer_ptr, 5);
  txFrame.addUInt16(newI2cAddress);
  // Note that the command is sent to the default address 0x08 of the sensor
  localError = SensirionI2CCommunication::sendFrame(SLF3C_1300F_I2C_ADDR_08, txFrame, Wire1);
  if (localError != NO_ERROR) {
    Serial.println("error sending address change command");
    errorToString(localError, errorMessage, sizeof errorMessage);
    Serial.println(errorMessage);
    Serial.println("As there are multiple sensors attached initially listening on the same I2C address \
        the acknowledge might overlap and cause an error which you can ignore if the subsequent communication is successful.");
  }

  // set IRQN pin of one sensor to high for at least 150μs to confirm address
  // change only after this pulse has been sent the sensor actually accepts
  // the new I2C address sent before
  pinMode(sensorIrqPin, OUTPUT);
  digitalWrite(sensorIrqPin, HIGH);
  delayMicroseconds(500);
  // reset IRQn pin back to low state
  digitalWrite(sensorIrqPin, LOW);

  // switch mode to input and listen to the pulse the sensor
  // sends 1500μs after the address change command to confirm the new I2C
  // address
  pinMode(sensorIrqPin, INPUT_PULLDOWN);
  delayMicroseconds(500);
  uint8_t success = 0;
  uint16_t cnt = 0;
  while (success == 0 && cnt < 100) {
    cnt++;
    success = digitalRead(sensorIrqPin);
    delayMicroseconds(10);
  }
  if (success == 0) {
    // return error as sensor did not acknowledge address change
    return -1;
  }

  Serial.print("Flow sensor address changed to: 0x");
  if (newI2cAddress < 16) {
    Serial.print("0");
  }
  Serial.println(newI2cAddress, HEX);
  return NO_ERROR;
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

  // wait 1 seconds for connection:
  delay(1000);
}

void RTCset()  // Set cpu RTC
{    
  tm t;
            t.tm_sec = (0);       // 0-59
            t.tm_min = (0);        // 0-59
            t.tm_hour = (0);         // 0-23
            t.tm_mday = (1);   // 1-31
            t.tm_mon = (0);       // 0-11  "0" = Jan, -1 
            t.tm_year = ((22)+100+1900);   // year since 1900,  current year + 100 + 1900 = correct year
            set_time(mktime(&t));       // set RTC clock       
}

String getLocaltime()
{
    char buffer[32];
    tm t;
    _rtc_localtime(time(NULL), &t, RTC_4_YEAR_LEAP_YEAR_SUPPORT);
    strftime(buffer, 32, "%Y-%m-%d %k:%M:%S", &t);
    return String(buffer);
}
