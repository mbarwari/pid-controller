#pragma once
#include <Adafruit_INA260.h>
#include <SensirionI2cSf06Lf.h>

class I2c{
  private:
    Adafruit_INA260 currentSensor1 = Adafruit_INA260();
    Adafruit_INA260 currentSensor2 = Adafruit_INA260();
    SensirionI2cSf06Lf flowSensor1;
    uint8_t address1;
    uint8_t address2;
    float outputArray[8];
    float aFlow;
    float aTemperature;    
    uint16_t aSignalingFlags;
  public: 
    I2c(uint8_t addr1, uint8_t addr2); 
    void i2cSetup();
    void i2cPrint();
}; 

