#include <Adafruit_INA260.h>
#include <SensirionI2cSf06Lf.h>
#include <Wire.h>
#include "i2c.h"

Adafruit_INA260 currentSensor1 = Adafruit_INA260();
Adafruit_INA260 currentSensor2 = Adafruit_INA260();

SensirionI2cSf06Lf flowSensor1;

uint8_t address1 = 0x40;
uint8_t address2 = 0x41;

void i2cSetup(){

  Serial.println("Initializing I2C devices");

  if (!currentSensor1.begin(address1, &Wire)) {
    Serial.println("Couldn't find current sensor 1");
  }
  else{
    Serial.println("Found current sensor 1");
  }

  if (!currentSensor2.begin(address2, &Wire)) {
    Serial.println("Couldn't find current sensor 2");
  }
  else{
    Serial.println("Found current sensor 2");
  }

  Wire.begin();
  flowSensor1.begin(Wire, SLF3C_1300F_I2C_ADDR_08);

  delay(100);
  flowSensor1.startH2oContinuousMeasurement();

  Serial.println();

}

void i2cPrint(){
  //sensor 1
  Serial.print("Sensor 1 Current: ");
  Serial.print(currentSensor1.readCurrent());
  Serial.println(" mA");

  Serial.print("Sensor 1 Bus Voltage: ");
  Serial.print(currentSensor1.readBusVoltage()/1000.0);
  Serial.println(" V");

  Serial.print("Sensor 1 Power: ");
  Serial.print(currentSensor1.readPower());
  Serial.println(" mW");

  Serial.println();

  //sensor 2
  Serial.print("Sensor 2 Current: ");
  Serial.print(currentSensor2.readCurrent());
  Serial.println(" mA");

  Serial.print("Sensor 2 Bus Voltage: ");
  Serial.print(currentSensor2.readBusVoltage()/1000.0);
  Serial.println(" V");

  Serial.print("Sensor 2 Power: ");
  Serial.print(currentSensor2.readPower());
  Serial.println(" mW");
  
  Serial.println();

  float aFlow = 0.0;
  float aTemperature = 0.0;    
  uint16_t aSignalingFlags = 0u;
  delay(20);
  flowSensor1.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3C_1300F, aFlow, aTemperature, aSignalingFlags);
   
  Serial.print("aFlow: ");
  Serial.print(aFlow);
  Serial.print("\t");
  Serial.print("aTemperature: ");
  Serial.print(aTemperature);
  Serial.print("\t");
  Serial.print("aSignalingFlags: ");
  Serial.print(aSignalingFlags);
  Serial.println();
}


