#include "i2c.h"
#include "Output.h"
#include "Multiplexer.h"


I2c i2c = I2c(0x40, 0x41);
Multiplexer mux; 

void setup() {

  // Initialize serial communication with a baud rate of 9600 bits per second
  Serial.begin(9600);
  // Wait until the Serial Monitor is connected (only works on native USB boards like ESP32-S2, Arduino GIGA, Leonardo, etc.)
  while (!Serial) { 
      delay(10); // Small delay to prevent excessive CPU usage while waiting for connection
  }
  // Once the Serial Monitor is detected, print a confirmation message
  Serial.println("Serial Monitor Connected!");

  i2c.i2cSetup();
  mux.muxSetup(); 

}
  

void loop() {

  i2c.i2cPrint();
  mux.muxPrint();
  delay(5000);
}

