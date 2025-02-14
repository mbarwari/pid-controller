#include <Wire.h>
#include "i2c.h"

I2c::I2c(uint8_t addr1, uint8_t addr2) {
  address1 = addr1;
  address2 = addr2;
  for (int i = 0; i < 8; i++) {
    outputArray[i] = 0.0;
  }
  aFlow = 0.0;
  aTemperature = 0.0;
  aSignalingFlags = 0u;
}


void I2c::i2cSetup(){

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
}

void I2c::i2cPrint(){

  outputArray[0] = currentSensor1.readCurrent(); 
  outputArray[1] = currentSensor1.readBusVoltage()/1000.0; 
  outputArray[2] = currentSensor1.readPower(); 

  outputArray[3] = currentSensor2.readCurrent(); 
  outputArray[4] = currentSensor2.readBusVoltage()/1000.0; 
  outputArray[5] = currentSensor2.readPower(); 

  delay(20);
  flowSensor1.readMeasurementData(INV_FLOW_SCALE_FACTORS_SLF3C_1300F, aFlow, aTemperature, aSignalingFlags);
  outputArray[6] = aFlow; 
  outputArray[7] = aTemperature; 

  Serial.println("I2C Array Values:");
  for (int i = 0; i < 8; i++) {
    Serial.print("Index ");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(outputArray[i], 2); // Print with 2 decimal places
  }

}


