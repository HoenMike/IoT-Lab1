const int ldrPin = 5;
int ldrValue = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  ldrValue = analogRead(ldrPin);
  Serial.println(ldrValue);
  delay(100);
}
