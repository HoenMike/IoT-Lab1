#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

// Create a MAX30100 object
MAX30100 sensor;

void setup() {
    Serial.begin(115200);
    Serial.print("Initializing MAX30100..");

    // Initialize sensor
    if (!sensor.begin()) {
        Serial.println("FAILED");
        for(;;);
    } else {
        Serial.println("SUCCESS");
    }

  sensor.setMode(MAX30100_MODE_SPO2_HR);
  sensor.setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_27_1MA);
  sensor.setLedsPulseWidth(MAX30100_SPC_PW_1600US_16BITS);
  sensor.setSamplingRate(MAX30100_SAMPRATE_100HZ);
  sensor.setHighresModeEnabled(true);
}


void loop() {
  uint16_t ir, red;
  sensor.update();
  while (sensor.getRawValues(&ir, &red)) {
    Serial.print(red);
    Serial.print(", ");
    Serial.println(ir);
  }
}
