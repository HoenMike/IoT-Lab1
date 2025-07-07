#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>

// Wi-Fi credentials
const char *ssid = "International University";
const char *password = "";
const char *mqttServer = "mikedaiot.cloud.shiftr.io";
const int mqttPort = 1883;
const char *mqttUser = "mikedaiot";
const char *mqttPassword = "MHL9Od957yL7o9tP";

// DHT Sensor
#define DHTPIN 4 // DHT sensor pin (GPIO4 = D2 on NodeMCU)
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// Relay Pins
#define RELAY1 14 // GPIO14 (D5 on NodeMCU)

// MQTT Client
WiFiClient espClient;
PubSubClient client(espClient);

void setup()
{
  Serial.begin(9600);
  dht.begin();
  setupWiFi();

  client.setServer(mqttServer, mqttPort);
  client.setCallback(callback);

  pinMode(RELAY1, OUTPUT);
  digitalWrite(RELAY1, LOW); // Ensure relay is off initially
}

void setupWiFi()
{
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

void reconnect()
{
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP8266Client", mqttUser, mqttPassword))
    {
      Serial.println("connected");
      client.subscribe("home/control");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void callback(char *topic, byte *payload, unsigned int length)
{
  String command = "";
  for (int i = 0; i < length; i++)
  {
    command += (char)payload[i];
  }
  handleCommand(command);
}

void handleCommand(String command)
{
  if (command == "light1_on")
  {
    digitalWrite(RELAY1, HIGH);
    Serial.println("Light 1 is ON");
  }
  else if (command == "light1_off")
  {
    digitalWrite(RELAY1, LOW);
    Serial.println("Light 1 is OFF");
  }
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  int lightLevel = analogRead(A0);

  String payload = String(humidity) + "," + String(lightLevel);
  Serial.println(payload);
  client.publish("home/sensors", payload.c_str());

  delay(5000);
}