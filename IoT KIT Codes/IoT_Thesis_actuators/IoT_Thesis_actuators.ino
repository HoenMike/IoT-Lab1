 /*************************************************** 
  NodeMCU
****************************************************/ 
#include <AccelStepper.h>
#include <ESP8266WiFi.h> 
#include <Servo.h>

#include "Adafruit_MQTT.h" 
#include "Adafruit_MQTT_Client.h"

const int stepsPerRevolution = 2048;  // change this to fit the number of steps per revolution

#define DHTTYPE DHT11

// ULN2003 Motor Driver Pins
#define IN1 5
#define IN2 4
#define IN3 14
#define IN4 12
#define IN3_DC 0
#define IN4_DC 2
#define ENB 16


/************************* WiFi Access Point *********************************/ 
#define WLAN_SSID       "Smurf" 
#define WLAN_PASS       "Baolong301100" 
#define MQTT_SERVER     "192.168.8.237" // static ip address
#define MQTT_PORT       1883                    
#define MQTT_USERNAME   "" 
#define MQTT_PASSWORD   "" 
#define LED_PIN1     10   // Pin connected to the LED. GPIO 5
#define FAN_PIN2     15  
  
#define servoPin 13 
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE  (80)
char msg[MSG_BUFFER_SIZE];
int value = 0;


// Timers auxiliar variables

char data[80];

int dc_motor_valspeed = 0;
int dc_motor_state = 2;
int servo_1_angle = 0;  // servo1 position in degrees
Servo servo_1;

AccelStepper stepper(AccelStepper::HALF4WIRE, IN1, IN3, IN2, IN4);
/************ Global State ******************/ 
// Create an ESP8266 WiFiClient class to connect to the MQTT server. 
WiFiClient client_1; 
// Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
Adafruit_MQTT_Client mqtt(&client_1, MQTT_SERVER, MQTT_PORT, MQTT_USERNAME, MQTT_PASSWORD); 
/****************************** Feeds ***************************************/ 
// Setup a feed called 'pi_led1' for publishing. 
// Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
Adafruit_MQTT_Publish pi_led1 = Adafruit_MQTT_Publish(&mqtt, MQTT_USERNAME "pi_led1");

// Setup a feed called 'esp8266_led1' for subscribing to changes. 
Adafruit_MQTT_Subscribe Led_1 = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "Led_1"); 
Adafruit_MQTT_Subscribe Fan_1 = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "Fan_1"); 
// Setup a feed called 'esp_step_motor1' for subscribing to changes. 
Adafruit_MQTT_Subscribe StepMotor = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "StepMotor"); 

// Setup a feed called 'esp_step_motor1' for subscribing to changes. 
Adafruit_MQTT_Subscribe Servo_ctl = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "Servo_ctl"); 

// Setup a feed called 'esp_dc_motor1' for subscribing to changes. 
Adafruit_MQTT_Subscribe Speed = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "Speed");

// Setup a feed called 'esp_dc_motor1' for subscribing to changes. 
Adafruit_MQTT_Subscribe DCmotor = Adafruit_MQTT_Subscribe(&mqtt, MQTT_USERNAME "DCmotor");


void step_motor_init(){
  stepper.setMaxSpeed(5000);
  stepper.setAcceleration(100);
}

void run_degree(int input_degree){
  int degreeint;
  degreeint = (input_degree*4096)/360;
  stepper.moveTo(degreeint); // set target forward 300 steps
  while (stepper.distanceToGo() != 0) 
  {  stepper.run(); 
  }
}

void servo_degree(int input_degree){
servo_1.write(input_degree);
}

void DC_motor_run(int speed,int mode) 
{
 // set speed 0 to 256
  analogWrite(ENB, speed);
  switch(mode)
  {
  case 2:
  // turn on motor forward
    digitalWrite(IN3_DC, LOW);
    digitalWrite(IN4_DC, HIGH);
    break;
  case 1:
  // turn on motor backward
    digitalWrite(IN3_DC, HIGH);
    digitalWrite(IN4_DC, LOW);
    break;
  case 0:
  // turn off motor forward
    digitalWrite(IN3_DC, LOW);
    digitalWrite(IN4_DC, LOW);
    break;
  default:
  // turn off motor forward
    digitalWrite(IN3_DC, LOW);
    digitalWrite(IN4_DC, LOW);
    break;
  }
}
/*************************** Sketch Code ************************************/ 
void MQTT_connect(); 
void setup() { 
 Serial.begin(115200); 
 delay(10); 
 pinMode(LED_PIN1, OUTPUT); 
 digitalWrite(LED_PIN1, LOW);
 pinMode(FAN_PIN2, OUTPUT); 
 digitalWrite(FAN_PIN2, LOW); 
  // Setup button as an input with internal pull-up.  
// initialize the stepper library
 pinMode(IN1, OUTPUT);
 pinMode(IN2, OUTPUT);
 pinMode(IN3, OUTPUT);
 pinMode(IN4, OUTPUT);
 pinMode(ENB, OUTPUT);
 pinMode(IN3_DC, OUTPUT);
 pinMode(IN4_DC, OUTPUT);
// initialize the stepper library
 step_motor_init();
 servo_1.attach(servoPin);
 
 Serial.println(F("RPi-ESP-MQTT")); 
 // Connect to WiFi access point. 
 Serial.println(); Serial.println(); 
 Serial.print("Connecting to "); 
 Serial.println(WLAN_SSID); 
 WiFi.begin(WLAN_SSID, WLAN_PASS); 
 while (WiFi.status() != WL_CONNECTED) { 
   delay(500); 
   Serial.print("."); 
 } 
 Serial.println(); 
 Serial.println("WiFi connected"); 
 Serial.println("IP address: "); Serial.println(WiFi.localIP()); 
 // Setup MQTT subscription for esp8266_led feed. 
 mqtt.subscribe(&Led_1);
 mqtt.subscribe(&Fan_1); 
 mqtt.subscribe(&StepMotor);
 mqtt.subscribe(&Servo_ctl);
 mqtt.subscribe(&Speed);
 mqtt.subscribe(&DCmotor);
} 
uint32_t x=0; 
void loop() { 
 // Check if the button has been pressed by looking for a change from high to 
 // low signal (with a small delay to debounce). 

 
 // Ensure the connection to the MQTT server is alive (this will make the first 
 // connection and automatically reconnect when disconnected).  See the MQTT_connect 
 MQTT_connect(); 
 // this is our 'wait for incoming subscription packets' busy subloop 
 // try to spend your time here 
 // Here its read the subscription 
 Adafruit_MQTT_Subscribe *subscription; 
 while ((subscription = mqtt.readSubscription())) { 
   if (subscription == &Led_1) { 
     char *message = (char *)Led_1.lastread; 
     Serial.print(F("Got_led1: ")); 
     Serial.println(message); 
     // Check if the message was ON, OFF, or TOGGLE. 
     if (strncmp(message, "1", 1) == 0) { 
       // Turn the LED on. 
       digitalWrite(LED_PIN1, HIGH); 
     } 
     else if (strncmp(message, "0", 1) == 0) { 
       // Turn the LED off. 
       digitalWrite(LED_PIN1, LOW); 
     } 
     else if (strncmp(message, "TOGGLE", 6) == 0) { 
       // Toggle the LED. 
       digitalWrite(LED_PIN1, !digitalRead(LED_PIN1)); 
      }  
   }
   if (subscription == &Fan_1) { 
     char *message = (char *)Fan_1.lastread; 
     Serial.print(F("Got_fan1: ")); 
     Serial.println(message); 
     // Check if the message was ON, OFF, or TOGGLE. 
     if (strncmp(message, "1", 1) == 0) { 
       // Turn the LED on. 
       digitalWrite(FAN_PIN2, HIGH); 
     } 
     else if (strncmp(message, "0", 1) == 0) { 
       // Turn the LED off. 
       digitalWrite(FAN_PIN2, LOW); 
     } 
     else if (strncmp(message, "TOGGLE", 6) == 0) { 
       // Toggle the LED. 
       digitalWrite(FAN_PIN2, !digitalRead(FAN_PIN2)); 
      }  
   }  
   if (subscription == &StepMotor) { 
     char *message = (char *)StepMotor.lastread; 
     Serial.print(F("Got_Step_motor Data: ")); 
     Serial.println(message); 
     if (strlen(message)>= 1) { 
       run_degree(atoi(message));
      }  
   } 
   if (subscription == &Servo_ctl) { 
     char *message = (char *)Servo_ctl.lastread; 
     Serial.print(F("Got_servo1 Data: ")); 
     Serial.println(message); 
     if (strlen(message)>= 1) { 
       servo_degree(atoi(message));
       Serial.println(servo_1.read()); 
      }  
   } 
    if (subscription == &Speed) { 
       char *message = (char *)Speed.lastread; 
       Serial.print(F("Got_dc_motor_speed Data: ")); 
       Serial.println(message); 
       if (strlen(message)>= 1) { 
         dc_motor_valspeed = atoi(message);
         DC_motor_run(dc_motor_valspeed,dc_motor_state);
        }  
     } 
     if (subscription == &DCmotor) { 
       char *message = (char *)DCmotor.lastread; 
       Serial.print(F("Got_dc_motor_mode Data: ")); 
       Serial.println(message); 
       if (strncmp(message, "1", 1) == 0) { 
       // Turn the dc motor backward
       DC_motor_run(dc_motor_valspeed,1);
       dc_motor_state = 1;
     } 
     else if (strncmp(message, "2", 1) == 0) { 
       // Turn the dc motor off. 
       DC_motor_run(dc_motor_valspeed,2);
       dc_motor_state = 2;
     } 
     else if (strncmp(message, "0", 1) == 0) { 
        
       DC_motor_run(dc_motor_valspeed,0);
       dc_motor_state = 0;
         
     } 
 }
 } 
 
 delay(20);
} 
// Function to connect and reconnect as necessary to the MQTT server. 
void MQTT_connect() { 
int8_t ret; 
 // Stop if already connected. 
 if (mqtt.connected()) { 
   return; 
 } 
 Serial.print("Connecting to MQTT... "); 
 uint8_t retries = 3; 
 while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected 
      Serial.println(mqtt.connectErrorString(ret)); 
      Serial.println("Retrying MQTT connection in 5 seconds..."); 
      mqtt.disconnect(); 
      delay(5000);  // wait 5 seconds 
      retries--; 
      if (retries == 0) { 
        // basically die and wait for WDT to reset me 
        while (1); 
      } 
} 
 Serial.println("MQTT Connected!"); 
} 
