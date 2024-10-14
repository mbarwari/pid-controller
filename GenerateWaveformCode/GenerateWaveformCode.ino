// This example generates different waveforms based on user input on A12/DAC1.

#include <Arduino_AdvancedAnalog.h>

#define N_SAMPLES 256
#define DEFAULT_FREQUENCY 10
AdvancedDAC dac0(A12);
uint8_t SAMPLES_BUFFER[N_SAMPLES];
float dac_frequency = DEFAULT_FREQUENCY;


void setup() {
    Serial.begin(115200);

    while (!Serial) {}
 
    generate_waveform(DEFAULT_FREQUENCY);
    Serial.print("Start");
    
}

void loop() {
  

      delay(3000);
      //String stringa = String(1);
      //stringa = Serial.readString();
      //int cmd = stringa.toInt();
      //cmd = 50;
      Serial.println("3");
      generate_waveform(20);
      Serial.println("4");
        

  
     /*if (Serial.available()>0){
      String stringa = String(1);
      stringa = Serial.readString();
      int cmd = stringa.toInt();
      //cmd = 50;
      Serial.println("3");
          generate_waveform(20);
        Serial.println("4");
     } */
     



    if (dac0.available()) {
        // Get a free buffer for writing.
        SampleBuffer buf = dac0.dequeue();

        // Write data to buffer.
        for (size_t i=0; i<buf.size(); i++) {
            buf[i] =  SAMPLES_BUFFER[i];
        }
        Serial.println("5");
        dac0.write(buf);
        Serial.println("6");
    }

    //Serial.println("7");

}


void generate_waveform(float freq)
{   

 // Square wave
  Serial.print("Waveform: Square ");
  for (int i=0; i<N_SAMPLES; i++){
  SAMPLES_BUFFER[i] = (i % 255) < 127 ? (255) : 0;
  }
   
  Serial.print("2");
  Serial.print("Current frequency: ");
  
  dac_frequency = freq;
  dac0.stop();
  if (!dac0.begin(AN_RESOLUTION_8, dac_frequency * N_SAMPLES, N_SAMPLES, 32)) {
    Serial.println("Failed to start DAC1 !");
    }

    
    Serial.print(dac_frequency);
    Serial.println("Hz");
    
}
