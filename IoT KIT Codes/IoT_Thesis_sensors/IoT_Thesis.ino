/*----------LIBRARY----------*/
#include <PubSubClient.h>
#include <WiFi.h>
#include <ESP32Servo.h>
#include <DHT.h>
#include <AccelStepper.h>
#include <DS1307ESP.h>

/*----------STEP MOTOR DECLARATION----------*/
const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution
// ULN2003 Motor Driver Pins
#define IN1 18
#define IN2 19
#define IN3 12
#define IN4 14

AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);
int pos_step; // angle

/*----------SERVO DECLARATION----------*/
#define pin_servo 23 // GPIO 23
Servo myservo; 
int pos; // angle

/*----------DHT11 DECLARATION----------*/
#define DHTPIN 32 // GPIO 32
#define DHTTYPE DHT11 
DHT dht(DHTPIN, DHTTYPE);

/*----------MESSAGE DECLARATION----------*/
#define MSG_BUFFER_SIZE  (50)
char msg[MSG_BUFFER_SIZE];

/*----------RELAY DECLARATION----------*/
#define RelayPin1 25 // GPIO 25
#define RelayPin2 33 // GPIO 33

/*-----------PIR DECLARATION-----------*/
#define PIR_pin 26 // GPIO 26
int PIR_value;

/*----------LIGHT DECLARATION----------*/
#define photo_pin 27 // GPIO 27 = ADC 17
int light;

/*--------REAL-TIME DECLARATION--------*/
DS1307ESP rtc;
uint32_t lastTime;

/*---------DC MOTOR DECLARATION--------*/
// L298N Motor Driver Pins
#define L298N_ENA 5
#define L298N_IN1 17
#define L298N_IN2 16
//#define L298N_IN3 17
//#define L298N_IN4 16

int dc_speed = 200;
String mess = "";
//// Switches
//#define SwitchPin1 7  // D7
//#define SwitchPin2 3   //D3 

////WiFi Status LED
//#define wifiLed    0   //D0

int toggleState_1 = 1; //Define integer to remember the toggle state for relay 1
int toggleState_2 = 1; //Define integer to remember the toggle state for relay 2


/*----------NETWORK CONNECTION----------*/
//#define WIFI_NETWORK "Hung Dien"
//#define WIFI_PASSWORD "hungdien1972"

#define WIFI_NETWORK "Smurf"
#define WIFI_PASSWORD "Baolong301100"

/*----------MQTT BROKER CONNECTION----------*/
//const char* mqttServer = "hello22.cloud.shiftr.io";
//const char* mqttUserName = "hello22"; // MQTT username
//const char* mqttPwd = "B1Fh8ZPsBZd50X3p"; // MQTT password
//const char* clientID = "65564018-9b89-434a-92d1-5c0ec6d438a9"; // client id

//const char* mqttServer = "192.168.1.50";
const char* mqttServer = "192.168.8.237";
const char* mqttUserName = ""; // MQTT username
const char* mqttPwd = ""; // MQTT password
const char* clientID = ""; // client id

/*----------P/S TOPIC DECLARATION----------*/
#define sub1 "Led_1"
#define sub2 "Fan_1"
#define sub5 "Servo"
#define sub3 "temp"
#define sub4 "humi"
#define sub6 "StepMotor"
#define sub7 "PIR"
#define sub8 "light"
#define sub9 "realtime"
#define sub10 "DCmotor"
#define sub11 "Speed"

#define pub1 "Led_1"
#define pub2 "Fan_1"
#define pub3 "temp"
#define pub4 "humi"
#define pub5 "Servo"
#define pub6 "StepMotor"
#define pub7 "PIR"
#define pub8 "light"
#define pub9 "realtime"
#define sub10 "DCmotor"
#define pub11 "Speed"
#define pubgod "test"

WiFiClient espClient;
PubSubClient client(espClient);


/*----------FUNCTION----------*/
void setup_wifi() { // WiFi connection
 delay(10);
 WiFi.mode(WIFI_STA);
 WiFi.begin(WIFI_NETWORK, WIFI_PASSWORD);
 while (WiFi.status() != WL_CONNECTED) {
 delay(500);
 Serial.print(".");
 }
 Serial.println("");
 Serial.println("WiFi connected");
 Serial.println("IP address: ");
 Serial.println(WiFi.localIP());
}

void reconnect() { // MQTT broker connection
 while (!client.connected()) {
 if (client.connect(clientID, mqttUserName, mqttPwd)) {
 Serial.println("MQTT connected");
      // ... and resubscribe
      client.subscribe(sub1);
      client.subscribe(sub2);
      client.subscribe(sub5);
      client.subscribe(sub6);
      client.subscribe(sub10);
      client.subscribe(sub11);
    } 
    else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void step_motor_init(){ // Setup step motor's qualities
  stepper.setMaxSpeed(500);
  stepper.setAcceleration(100);
}

void run_degree(int input_degree){ // Treating step motor angle
  int degreeint;
  degreeint = (input_degree*4096)/360;
  Serial.println(degreeint);
  stepper.moveTo(degreeint); // set target forward 300 steps
  while (stepper.distanceToGo() != 0) 
  {  stepper.run(); 
  }
}

// Subscribe TOPIC and read MESSAGE
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  if (strstr(topic, sub1)) // TOPIC: Led_1
  {
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin1, LOW);   // Turn the LED on (Note that LOW is the voltage level
      Serial.print("LIGHT: OFF: ");
      Serial.println(digitalRead(RelayPin1));
    } else {
      digitalWrite(RelayPin1, HIGH);  // Turn the LED off by making the voltage HIGH
      Serial.print("LIGHT: ON: ");
      Serial.println(digitalRead(RelayPin1));
    }    
  }

  else if (strstr(topic, sub2)) // TOPIC: Fan_1
  {
    for (int i = 0; i < length; i++) {
      Serial.print((char)payload[i]);
    }
    Serial.println();
    // Switch on the LED if an 1 was received as first character
    if ((char)payload[0] == '0') {
      digitalWrite(RelayPin2, LOW);   // Turn the LED on (Note that LOW is the voltage level
      Serial.println("Fan: OFF");
    } else {
      digitalWrite(RelayPin2, HIGH);  // Turn the LED off by making the voltage HIGH
      Serial.println("Fan: ON");
    }
  }
  
  else if (strstr(topic, sub5)) // TOPIC: Servo
  {
    String response = "";
    for (int i = 0; i < length; i++) {
      response = response + (char)payload[i];
    }
    Serial.print("Response: [");
    Serial.print(response);
    Serial.println("] ");
    char Buf[50];
    response.toCharArray(Buf, 50);
    pos = atoi(Buf);
    if (pos == 0 || response == "OFF")
    {
      myservo.write(0);
      Serial.println("Servo angle: 0°");
    }
    myservo.write(pos);
    Serial.print("Servo angle: ");
    Serial.print(pos);
    Serial.println("°");
  }

  else if (strstr(topic, sub6)) // TOPIC: StepMotor
  {
    String response = "";
    for (int i = 0; i < length; i++) {
      response = response + (char)payload[i];
    }
    Serial.print("Response: [");
    Serial.print(response);
    Serial.println("] ");
    char Buf[50];
    response.toCharArray(Buf, 50);
    pos_step = atoi(Buf);
    if (pos_step == 0 || response == "OFF")
    {
      run_degree(0);
      Serial.println("Step motor angle: 0°");
    }
    run_degree(pos_step);
    Serial.print("Step motor angle: ");
    Serial.print(pos_step);
    Serial.println("°");
  }

  else if (strstr(topic, sub11)) // TOPIC: Speed
  {
    String response = "";
    for (int i = 0; i < length; i++) {
      response = response + (char)payload[i];
    }
    Serial.print("Response: [");
    Serial.print(response);
    Serial.println("] ");
    char Buf[50];
    response.toCharArray(Buf, 50);
    dc_speed = atoi(Buf);
  }

  else if (strstr(topic, sub10)) // TOPIC: DCMotor
  {
    String response = "";
    for (int i = 0; i < length; i++) {
      response = response + (char)payload[i];
    }
    Serial.print("Response: [");
    Serial.print(response);
    Serial.println("] ");
    char Buf[10];
    response.toCharArray(Buf, 10);
    int dc_num = atoi(Buf);
    if (dc_num == 2)
    {
      Serial.println("DC motor is going forward");
      analogWrite(L298N_ENA, dc_speed);
      digitalWrite(L298N_IN1, HIGH);
      digitalWrite(L298N_IN2, LOW);
    } 
    else if (dc_num == 1)
    {
      Serial.println("DC motor is going backward");
      digitalWrite(L298N_IN1, LOW);
      digitalWrite(L298N_IN2, HIGH);
    }
    else
    {
      Serial.println("DC motor stopped");
      digitalWrite(L298N_IN1, LOW);
      digitalWrite(L298N_IN2, LOW);
    }
  }
  else
  {
    Serial.println("unsubscribed topic");
  }
}

/*----------SETUP-SETUP-SETUP----------*/
void setup() {
  Serial.begin(115200);
  myservo.attach(pin_servo);
  pinMode(RelayPin1, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  dht.begin();
  rtc.begin();
  rtc.DSadjust(0, 13, 19, 2022, 6, 29); // 00:19:21 16 Mar 2022
//  pinMode(SwitchPin1, INPUT_PULLUP);
//  pinMode(SwitchPin2, INPUT_PULLUP);

//  pinMode(wifiLed, OUTPUT);

  //During Starting all Relays should TURN OFF
  digitalWrite(RelayPin1, LOW);
  digitalWrite(RelayPin2, LOW);

  
  //During Starting WiFi LED should TURN OFF
//  digitalWrite(wifiLed, HIGH);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  pinMode(L298N_ENA, OUTPUT);
  pinMode(L298N_IN1, OUTPUT);
  pinMode(L298N_IN2, OUTPUT);

  pinMode(PIR_pin, INPUT);
// initialize the stepper library
  step_motor_init();

  setup_wifi();
  client.setServer(mqttServer, 1883);
  client.setCallback(callback);
}

long unsigned timer1 = millis();

/*----------LOOP-LOOP-LOOP----------*/
void loop() {
  if (!client.connected()) 
  {
//    digitalWrite(wifiLed, HIGH);
    reconnect();
  }
  else
  {
    if (millis() - timer1 >1000) 
    { 
      float h = dht.readHumidity();
      float t = dht.readTemperature();
      int humi = (int) h;
      int temp = (int) t;
      char buf_h[10];
      char buf_t[10];
      itoa(humi, buf_h, 10);
      itoa(temp, buf_t, 10);
      Serial.print("Temperature: ");
      Serial.println(t);
      Serial.print("Humidity: ");
      Serial.println(h);
      Serial.println(buf_h);
      client.publish(pub3, buf_t);
      client.publish(pub4, buf_h);
      timer1 = millis();

      PIR_value = digitalRead(PIR_pin);
      if (PIR_value == 1)
      {
        Serial.println("Motion");
      }
      else
      {
        Serial.println("NO one");
      }
      char buf_PIR[3];
      itoa(PIR_value, buf_PIR, 10);
      client.publish(pub7, buf_PIR);

      light =  map(analogRead(photo_pin), 0, 1023, 0, 100);
      int light_intensity = 100 - light;
//      int light_intensity = 70;
      char buf_LIGHT[3];
      itoa(light_intensity, buf_LIGHT, 10);
      Serial.print("Light intensity: ");
      Serial.println(light);
      client.publish(pub8, buf_LIGHT);

      mess = "{\"temperature\":\"" + String(t) + "\"," +
         "\"humidity\":\"" + String(h) + "\","+
         "\"light\":\"" + String(buf_LIGHT) + "\","+
         "\"human\":\"" + String(buf_PIR) + "\"}";
       //  client.publish(pubgod,mess);
       Serial.println(mess);
      char buf_mess[100];
      mess.toCharArray(buf_mess, 100);
      client.publish(pubgod,buf_mess);
    }

    if(millis() - lastTime >= 1000) {
      updateTime();
      lastTime = millis();
    }
    
//    digitalWrite(wifiLed, LOW);
//    manual_control();
  }
  client.loop();
}

void updateTime()
{
      rtc.DSread(); 
      Serial.println(rtc.getDateTime(true));  //  (String) Monday, 14-March-2022 00:09:21
      char buf_REALTIME[50];
      String realtime = rtc.getDateTime(true);
      realtime.toCharArray(buf_REALTIME, 50);
      client.publish(pub9, buf_REALTIME);
} 
