#include "i2c.h"


void setup() {
  Serial.begin(9600);
  // Wait until serial port is opened
  while (!Serial) { delay(10); }

  i2cSetup();

}
  

void loop() {

  i2cPrint();
  delay(5000);

}

