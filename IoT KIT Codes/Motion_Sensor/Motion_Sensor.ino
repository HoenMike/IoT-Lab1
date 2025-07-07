int inputPin = 13; // for ESP8266 microcontroller
//int inputPin = 4; // for ESP32 microcontroller
 
void setup() {
  pinMode(inputPin, INPUT);
  Serial.begin(9600);
}
 
void loop(){
  int val = digitalRead(inputPin);
  if (val == HIGH) {
    Serial.println("Motion detected!");
    }
  else {
    Serial.println("No Motion detected!");
    }
    
delay(1000);
}
