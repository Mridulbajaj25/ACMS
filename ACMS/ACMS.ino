#define BLYNK_TEMPLATE_ID "TMPL3RMMbUoJ3"
#define BLYNK_TEMPLATE_NAME "ACMS"
#define BLYNK_AUTH_TOKEN "ZM3vlhQ-fVG4ytxR_Fngi8y1prH3hS7U"

#include <Wire.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <MPU6050.h>

MPU6050 mpu;
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "Nothing";
char pass[] = "123456789";

#define TH 5  // DHT11 sensor pin
#define MQ2 34 // Gas sensor pin

#define D0 32
#define D1 33


static const int RXPin = 17;
static const int TXPin = 16;
static const uint32_t GPSBaud = 9600;

int Vibration_Sensor = 25;
int Sensor_State = 0;
const int ledPin = 27;
TinyGPSPlus gps;
SoftwareSerial GPS(RXPin, TXPin);
DHT dht(TH, DHT11);
BlynkTimer timer;

void reconnectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("Reconnecting WiFi...");
    WiFi.disconnect();
    WiFi.reconnect();
    delay(5000);
  }
}

void setup() {
  Serial.begin(115200);
  // GPS.begin(GPSBaud);
  Wire.begin();

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Blynk.begin(auth, ssid, pass);

  dht.begin();
  pinMode(MQ2, INPUT);

  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(Vibration_Sensor, INPUT);
  pinMode(ledPin, OUTPUT);
  analogReadResolution(12);

  Serial.println("Initializing Sensors...");
  if (!mpu.testConnection()) {
      Serial.println("MPU6050 connection failed!");
      while (1);
  }
  Serial.println("All sensors connected!");
  mpu.initialize();

  timer.setInterval(2000L, DHT11sensor);
  timer.setInterval(2000L, gasSensor);
  timer.setInterval(2000L, MPU);
}

void DHT11sensor() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  if (isnan(h) || isnan(t)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }
  Serial.print("Temperature: "); Serial.print(t); Serial.println(" °C");
  Serial.print("Humidity: "); Serial.print(h); Serial.println(" %");
  Blynk.virtualWrite(V0, t);
  Blynk.virtualWrite(V1, h);
}

void gasSensor() {
  int gasValue = analogRead(MQ2);
  int mappedValue = map(gasValue, 0, 4095, 0, 100);
  Serial.print("C02 Value: "); Serial.println(mappedValue);
  Blynk.virtualWrite(V2, mappedValue);
}

void vibrationSensor(){
    Sensor_State = digitalRead(Vibration_Sensor);
  if (Sensor_State == HIGH) {
     
    Serial.println("CRASH Detected");
    sendLocation();
    Serial.println ("Lat = 28.670734 and Long = 77.094191");
    digitalWrite(ledPin, HIGH);
  } else {
    Serial.println("No Vibration");
    digitalWrite(ledPin, LOW);
  }
}

void sendLocation() {
  while (GPS.available() > 0) {
    gps.encode(GPS.read());
  }

  if (gps.location.isUpdated() && gps.location.isValid()) {
    Serial.print("Latitude= "); 
    Serial.print(gps.location.lat(), 6);
    Serial.print(" Longitude= "); 
    Serial.println(gps.location.lng(), 6);

    Serial.print("Sending Location: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(", ");
    Serial.println(gps.location.lng(), 6);
  } else {
    // Serial.println("No GPS Location");
  }
}


void MPU() {
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;
  float gyroX = gx / 131.0;
  float gyroY = gy / 131.0;
  float gyroZ = gz / 131.0;
  Serial.print("Accel X: "); Serial.print(accelX);
  Serial.print(" Y: "); Serial.print(accelY);
  Serial.print(" Z: "); Serial.println(accelZ);
  Serial.print("Gyro X: "); Serial.print(gyroX);
  Serial.print(" Y: "); Serial.print(gyroY);
  Serial.print(" Z: "); Serial.println(gyroZ);
  Serial.println("------------------------");
}

void loop() {
  
  reconnectWiFi();
  
  int A = digitalRead(D0); // Read D0 Pin
   
  if (A == HIGH) {
     Serial.println("System Turning ON");
     delay(1000);
    while (1){
          Blynk.run();
          timer.run();
          MPU();
          DHT11sensor();
          gasSensor();
          vibrationSensor();
          delay(200);
      int B = digitalRead(D1); // Read D0 Pin
      if (B == HIGH) {
        Serial.println("System Turning OFF");
        break;
      }
    }
  }
}
