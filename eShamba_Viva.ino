#include<WiFi.h>
#include<PubSubClient.h>

#include<AHT10.h>
#include<Wire.h>
#include "RTClib.h"

AHT10 aht;
RTC_DS3231 rtc;


float temp, humi;
float Time,Dist;

float light,lightIntensity;
float moisture, moisturePercentage;

int dryValue = 900;
int wetValue = 800;

int onTime,onHour,onMin;

//defining GPIO pins

#define Pump 2
#define lightPin 35
#define moisturePin 35

#define WIFI_SSID "Kori"
#define WIFI_PASS "123456789"

const char* mqtt_server = "13.51.175.213";
           

WiFiClient Client;
PubSubClient mqttClient(Client);

/*********************************************************************************/
void reconnect() {
  while (!mqttClient.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (mqttClient.connect("ESP32Client")) {
      Serial.println("Connected to MQTT broker");

      // Subscribing to a topic for the actuators
      mqttClient.subscribe("pump");
      mqttClient.subscribe("fan");
      mqttClient.subscribe("time");
      //subscribing to the topic that checks scheduled irrigation time
      mqttClient.subscribe("onHour");
      mqttClient.subscribe("onMin");
      
    } else {
      Serial.print("Failed to connect to MQTT broker, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" Retrying in 5 seconds...");
      delay(5000);
    }
  }
}

/*******************************************************************************/

// Creating a callback function to check incoming messages from the broker 
void callback(String topic, byte* message, unsigned int length) {
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;
  
  for (int i = 0; i < length; i++) {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  if(topic=="pump"){
      Serial.print("Switching the pump: ");
      if(messageTemp == "ON"){
        digitalWrite(Pump, HIGH);
        Serial.print("pump is turned On");
      }
      else if(messageTemp == "OFF"){
        digitalWrite(Pump, LOW);
        Serial.print("Pump is turned Off");
      }
  }else if(topic=="onHour"){
    onHour = messageTemp.toInt();
    }else if(topic=="onMin"){
      onMin = messageTemp.toInt();
      }else{
        Serial.println("Message arrived on unknown topic");
        }
}

/************************************************************************/

void setup() {

  Serial.begin(9600);
  aht.begin();

  pinMode(Pump,OUTPUT);
  pinMode(lightPin,INPUT);
  pinMode(moisturePin,INPUT);
  


  if (!rtc.begin()) {
    Serial.println("Could not find RTC! Check circuit.");
    while (1);
  }

  
  rtc.adjust(DateTime(__DATE__, __TIME__));
  delay(5000);
  
  WiFi.mode(WIFI_STA);
  
  Serial.print("Connecting...");
  WiFi.begin(WIFI_SSID,WIFI_PASS);

  while(WiFi.status() != WL_CONNECTED){
    Serial.print("...");
    delay(500);
    }

    Serial.println();
    Serial.println("Connected!");

    mqttClient.setServer(mqtt_server,1883);
    mqttClient.setCallback(callback);

}

********************************************************************************/


void loop() {

  readSensorData();
  
  publishData();
  
  CheckTimer();



   
  if (!mqttClient.connected()) {
    reconnect();
  }
  if(!mqttClient.loop())
    mqttClient.connect("ESP8266Client");
  
  
}
/*********************************************************************/

// data logging function  from the sensors->()

void readSensorData(){
  temp = aht.readTemperature();
  humi = aht.readHumidity();

  int dayValue = 360;
  int nightValue = 5;

  light= analogRead(lightPin);
  lightIntensity = map(light,nightValue,dayValue,0,100);
  lightIntensity = constrain(lightIntensity, 0,97);

  moisture= analogRead(moisturePin);
  moisturePercentage = map(moisture,dryValue,wetValue,0,100);
  moisturePercentage = constrain(moisturePercentage, 0,97);

  
  Serial.print("The moisture percentage: ");
  Serial.println(moisturePercentage);
  Serial.print("The Light intensity: ");
  Serial.println(lightIntensity);

  Serial.println("The temperature of the room is: " +String(temp) +" and humidity is: "+ String(humi));
  
  }

/************************************************************************************/

 // publishing data to the broker function->()
void publishData(){

  char Temp[8];
  dtostrf(temp, 4, 2, Temp);

  char Humi[8];
  dtostrf(humi, 4, 2, Humi);

  char dist[8];
  dtostrf(Dist, 4, 2, dist);

  char Light[8];
  dtostrf(lightIntensity, 4, 2, Light);

  char Moisture[8];
  dtostrf(moisturePercentage, 4, 2, Moisture);

  mqttClient.publish("temperature",Temp);
  delay(1000);
  mqttClient.publish("humidity",Humi);
  delay(1000);
  mqttClient.publish("light",Light);
  delay(1000);
  mqttClient.publish("moisture",Moisture);
  delay(1000);
  
  }
/*******************************************************************************/
 //function to check the rtc timer->()

void CheckTimer(){

   // Print the irrigation time
   Serial.print("The Irrigation is scheduled at: ");
   Serial.print(onHour);
   Serial.print(":");
   Serial.println(onMin);

   //checking the current time
   DateTime now = rtc.now();

   
   while(onHour == now.hour() && onMin==now.minute()){  

    Serial.println("Irrigation started: ");
    digitalWrite(Pump,HIGH);
    delay(1000*60);
    digitalWrite(Pump,LOW);
    
    DateTime now = rtc.now();
    
    }
   
  }

/*********************************************************************/  
