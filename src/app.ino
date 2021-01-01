
#include <ESP8266WiFi.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <PubSubClient.h>
#include "secrets.h"

#define hum_topic "livingroom/sensor/humidity"
#define motion_topic "livingroom/sensor/motion"
#define temp_topic "livingroom/sensor/temperature"
#define ldr_topic "livingroom/sensor/light"

#define DHTPIN D4     // Digital pin connected to the DHT sensor 
// Feather HUZZAH ESP8266 note: use pins 3, 4, 5, 12, 13 or 14 --
// Pin 15 can work but DHT must be disconnected during program upload.

#define DHTTYPE    DHT11     // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);

WiFiClient espClient;
PubSubClient client(espClient);

int ldr = A0; // LDR light sensor pin
int led = D2; // Define the LED as Pin 13
int motion_sensor = D8; // Define the Motion Sensor Connected to Pin 8
int lastMsgTime = 0;
int send_interval = 30000; //Send message interval in ms
uint32_t delayMS;
void setup() {
  pinMode(led, OUTPUT); // initialize the LED as the output
  pinMode(motion_sensor, INPUT); // initialize the sensor as the input
 
  Serial.begin(115200); // Define the serial communication

   dht.begin();
   // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
  Serial.println(F("------------------------------------"));
  Serial.println(F("Temperature Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("°C"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("°C"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("°C"));
  Serial.println(F("------------------------------------"));
  // Print humidity sensor details.
  dht.humidity().getSensor(&sensor);
  Serial.println(F("Humidity Sensor"));
  Serial.print  (F("Sensor Type: ")); Serial.println(sensor.name);
  Serial.print  (F("Driver Ver:  ")); Serial.println(sensor.version);
  Serial.print  (F("Unique ID:   ")); Serial.println(sensor.sensor_id);
  Serial.print  (F("Max Value:   ")); Serial.print(sensor.max_value); Serial.println(F("%"));
  Serial.print  (F("Min Value:   ")); Serial.print(sensor.min_value); Serial.println(F("%"));
  Serial.print  (F("Resolution:  ")); Serial.print(sensor.resolution); Serial.println(F("%"));
  Serial.println(F("------------------------------------"));
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  // Set delay between sensor readings based on sensor details.
  delayMS = sensor.min_delay / 1000;
  
}

void setup_wifi() {
  delay(10);
  WiFi.begin(wifi_ssid, wifi_password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (client.connect("nodemcu", mqtt_user, mqtt_pass)) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

float ldr_val = 0; // Store the value of ldr sensor
void checkLdr(){
  int sensorValue = analogRead(A0);   // read the input on analog pin 0

  float newVal = sensorValue * (5.0 / 1023.0);   // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 3V)

  if (newVal != ldr_val) {
    ldr_val = newVal;
    Serial.print("LDR:");
    Serial.println(String(ldr_val).c_str());
    client.publish(ldr_topic, String(ldr_val).c_str(), true);
  }
}


int state = LOW; // Motion Detection
int motion_val = 0; // Store the value of motion sensor

void checkMotion() {
  int newVal = digitalRead(motion_sensor); // Reading the sensor value

  if (newVal != motion_val) {
    motion_val = newVal;
    Serial.print("Motion:");
    Serial.println(String(motion_val).c_str());
    client.publish(motion_topic, String(motion_val).c_str(), true);
  }

  if (motion_val == HIGH) { // if sensor is high
    digitalWrite(led, HIGH); // switch on the LED
    delay(100); // 100 milliseconds delay

    if (state == LOW) {
      Serial.println("Motion was detected");
      state = HIGH; // Update the variable state in to HIGH
    }
  }
  else {
    digitalWrite(led, LOW); // Turning off the LED
    delay(200); // 200 milliseconds delay

    if (state == HIGH) {
      Serial.println("Motion stopped!");
      state = LOW; // update the variable state into LOW
    }
  }
}

float temp_val = 0; // Store the value of temp sensor
float hum_val = 0; // Store the value of humidity sensor

void checkDHT() {
   //delay(delayMS);
  // Get temperature event and print its value.
  sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    if (event.temperature != temp_val) {
      temp_val = event.temperature;
      Serial.print(F("Temperature: "));
      Serial.print(event.temperature);
      Serial.println(F("°C"));
      client.publish(temp_topic, String(event.temperature).c_str(), true);
    }
  }

  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    if (event.relative_humidity != hum_val) {
      hum_val = event.relative_humidity;
      Serial.print(F("Humidity: "));
      Serial.print(event.relative_humidity);
      Serial.println(F("%"));
      client.publish(hum_topic, String(event.relative_humidity).c_str(), true);
    }
  }



  if (motion_val == HIGH) { // if sensor is high
    digitalWrite(led, HIGH); // switch on the LED
    delay(100); // 100 milliseconds delay

    if (state == LOW) {
      Serial.println("Motion was detected");
      state = HIGH; // Update the variable state in to HIGH
    }
  }
  else {
    digitalWrite(led, LOW); // Turning off the LED
    delay(200); // 200 milliseconds delay

    if (state == HIGH) {
      Serial.println("Motion stopped!");
      state = LOW; // update the variable state into LOW
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  long now = millis();
  if (now - lastMsgTime < send_interval) {
    // Wait a few seconds between measurements
    lastMsgTime = now;

    //Check motion state
    checkMotion();
    //Check DHT sensor values
    checkDHT();
    //Check LDR sensor
    checkLdr();
  }
}