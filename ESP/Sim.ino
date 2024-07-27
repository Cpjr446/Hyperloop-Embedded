#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <MPU6050.h>

#define DHTPIN 4
#define DHTTYPE DHT22
#define RELAY_PIN 5

const char* ssid = "Chandraprakash's Galaxy M32 5G";
const char* password = "ziku6944";
const char* mqtt_server = "e9a94e816f1f4b5a9fb07635ed0ccfb9.s1.eu.hivemq.cloud";
const int mqtt_port = 8883;

DHT dht(DHTPIN, DHTTYPE);
MPU6050 mpu;
WiFiClient espClient;
PubSubClient client(espClient);

void setup_wifi() {
  delay(10);
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  if (message == "startCooling") {
    digitalWrite(RELAY_PIN, HIGH);
  } else if (message == "stopCooling") {
    digitalWrite(RELAY_PIN, LOW);
  }
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  Serial.begin(115200);
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  dht.begin();
  Wire.begin();
  mpu.initialize();
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      client.subscribe("pod/control");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  float temperature = dht.readTemperature();
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  char msg[100];
  snprintf(msg, 100, "{\"temperature\": %.2f, \"ax\": %d, \"ay\": %d, \"az\": %d}", temperature, ax, ay, az);
  client.publish("pod/sensors", msg);

  delay(2000);
}
