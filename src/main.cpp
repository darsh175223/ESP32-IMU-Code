#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// WiFi credentials
const char* ssid     = "";
const char* password = "";

// UDP settings
const char* serverIP = "";
const int serverPort = 9000;
WiFiUDP udp;

// MPU6050 settings
#define MPU_ADDR 0x68

// Timing
unsigned long lastSend = 0;
const int sendIntervalMs = 10; // 100 Hz

void setup() {
  Serial.begin(115200);
  
  // Initialize I2C for MPU6050
  Wire.begin(21, 22);

  // Wake up MPU6050
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  delay(100);

  // Configure accelerometer (±2g)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1C);
  Wire.write(0x00);
  Wire.endTransmission();

  // Configure gyroscope (±250°/s)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Enable low-pass filter
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A);
  Wire.write(0x06); // DLPF = 5Hz
  Wire.endTransmission();

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.println("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("Connecting... WiFi status: ");
    Serial.println(WiFi.status());
  }

  WiFi.setSleep(false); // Important for low latency
  Serial.println("\nWiFi connected");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());

  udp.begin(12345); // Local UDP port
}

void loop() {
  if (millis() - lastSend >= sendIntervalMs) {
    lastSend = millis();

    // Read raw MPU6050 data
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)14, (uint8_t)1);

    int16_t ax = Wire.read() << 8 | Wire.read();
    int16_t ay = Wire.read() << 8 | Wire.read();
    int16_t az = Wire.read() << 8 | Wire.read();
    Wire.read(); Wire.read(); // Skip temperature
    int16_t gx = Wire.read() << 8 | Wire.read();
    int16_t gy = Wire.read() << 8 | Wire.read();
    int16_t gz = Wire.read() << 8 | Wire.read();

    // Format and send UDP packet with raw values
    char buf[128];
    snprintf(buf, sizeof(buf), 
             "ax=%d,ay=%d,az=%d,gx=%d,gy=%d,gz=%d",
             ax, ay, az, gx, gy, gz);

    udp.beginPacket(serverIP, serverPort);
    udp.write((uint8_t*)buf, strlen(buf));
    udp.endPacket();

    // Optional: Print to Serial for debugging
    Serial.println(buf);
  }
}