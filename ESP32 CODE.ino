#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <MQUnifiedsensor.h>
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include <PubSubClient.h>
#include <WiFi.h>

#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme; 

#define placa "ESP-32"
#define Voltage_Resolution 5
#define pin 33 
#define type "MQ-135"
#define ADC_Bit_Resolution 12 
#define RatioMQ135CleanAir 3.6
double CO;
MQUnifiedsensor MQ135(placa, Voltage_Resolution, ADC_Bit_Resolution, pin, type);

const int ldrPin = 32;  // Pin analog untuk sensor LDR
unsigned int AnalogValueLDR;
unsigned long delayTime;

const char* ssid = "Bosmuda88";
const char* password = "adipura1c";
const char* mqtt_server = "192.168.1.22"; 
const int mqtt_port = 1883;

int nodeNumber = 1; // Setel secara unik pada setiap node

WiFiClient espClient;
PubSubClient client(espClient);

AsyncWebServer server(80);

// Data kalibrasi LDR
const float resistances[] = {4095, 1646, 986, 806, 624, 470, 350, 336, 310, 270, 258, 240, 220};
const float luxValues[] = {0, 25, 50, 75, 100, 125, 150, 175, 200, 225, 250, 275, 300};
const int numCalibrationPoints = sizeof(resistances) / sizeof(resistances[0]);

// Fungsi untuk melakukan map nilai float (analogWrite untuk float)
float mapFloat(float x, const float xArray[], const float yArray[], int size) {
  int i = 0;
  while (i < size - 1 && x < xArray[i]) {
    i++;
  }
  float t = (x - xArray[i]) / (xArray[i + 1] - xArray[i]);
  float result = yArray[i] + t * (yArray[i + 1] - yArray[i]);

  return result;
}

void setup() {
  Serial.begin(115200);

  Serial.println("Connected to Wifi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Serial.println("\n-------------------------------\n");
  delay(1000);

  bool status;
  status = bme.begin(0x76);
  if (!status) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1);
  }

  Serial.println("-- Default Test --");
  delayTime = 5000;

  MQ135.setRegressionMethod(1); 
  MQ135.setA(110.47);
  MQ135.setB(-2.862);

  MQ135.init(); 
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);
  
  if(isinf(calcR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(calcR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}

  MQ135.serialDebug(true);

  Serial.println();

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is a sample response.");
  });

  AsyncElegantOTA.begin(&server);
  server.begin();

  Serial.println("HTTP server started");
  Serial.println("\n-------------------------------\n");
  delay(1000);

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);

  Serial.println("Attempting MQTT connection...connected");
  Serial.println("\n-------------------------------\n");
  delay(1000);
}

void loop() {
  Serial.println("-------------------------------\n");

  printValues();

  delay(delayTime);
  AsyncElegantOTA.loop();

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}

void callback(char* topic, byte* payload, unsigned int length) {
  // Callback untuk memproses pesan yang diterima dari broker MQTT
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ESP32Client")) {
      Serial.println("connected");
      // Set status saat terhubung ulang
      String nodeStatusTopic = "/node" + String(nodeNumber) + "/status";
      client.publish(nodeStatusTopic.c_str(), "Aktif");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void printValues() {
  Serial.print("Temperature (BME280) = ");
  Serial.print(bme.readTemperature());
  Serial.println(" *C");

  Serial.print("Pressure (BME280) = ");
  Serial.print(bme.readPressure() / 100.0F);
  Serial.println(" hPa");

  Serial.print("Humidity (BME280) = ");
  Serial.print(bme.readHumidity());
  Serial.println(" %");

  MQ135.update();
  MQ135.setA(110.47); MQ135.setB(-2.862);
  float CO = MQ135.readSensor();  // Menggunakan variabel lokal

  Serial.print("PPM (MQ135) = ");
  Serial.print(CO);
  Serial.println(" ppm");

  AnalogValueLDR = analogRead(ldrPin);
  // Konversi nilai sensor LDR menjadi Lux
  float lux = mapFloat(AnalogValueLDR, resistances, luxValues, numCalibrationPoints);
  
  // Pastikan nilai Lux tidak negatif
  if (lux < 0.0) {
    lux = 0.0;
  }

  Serial.print("LDR Value = ");
  Serial.println(lux);

  if (client.connected()) {
    String topicTemperature = "/node" + String(nodeNumber) + "/sensor/temperature";
    String topicPressure = "/node" + String(nodeNumber) + "/sensor/pressure";
    String topicHumidity = "/node" + String(nodeNumber) + "/sensor/humidity";
    String topicCO = "/node" + String(nodeNumber) + "/sensor/co";
    String topicLDR = "/node" + String(nodeNumber) + "/sensor/ldr";

    client.publish(topicTemperature.c_str(), String(bme.readTemperature()).c_str());
    client.publish(topicPressure.c_str(), String(bme.readPressure() / 100.0F).c_str());
    client.publish(topicHumidity.c_str(), String(bme.readHumidity()).c_str());
    client.publish(topicCO.c_str(), String(CO).c_str());
    client.publish(topicLDR.c_str(), String(lux).c_str());
  }
}
