int sensorPin=5;
int sensorData;
void setup()
{  
  Serial.begin(9600);   
  pinMode(sensorPin,INPUT);                         
 }
void loop()
{
  sensorData = analogRead(sensorPin);       
  Serial.println("Sensor Data:");
  delay(1000);                                   
}
