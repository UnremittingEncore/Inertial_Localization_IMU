/*********
  Rui Santos
  Complete project details at https://RandomNerdTutorials.com/esp32-mpu-6050-web-server/
  
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files.
  
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
*********/

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Arduino_JSON.h>
#include <KalmanFilter.h>
#include "SPIFFS.h"

//------ Kalman Filter ------------

KalmanFilter kalmanX(0.001, 0.003, 0.03);
KalmanFilter kalmanY(0.001, 0.003, 0.03);
KalmanFilter kalmanZ(0.001, 0.003, 0.03);

float accPitch = 0;
float accRoll = 0;
float accYaw = 0;

float kalPitch = 0;
float kalRoll = 0;
float kalYaw = 0;

float M_P1 = 3.141592; // Also defined in math.h 
const float alpha = 0.5;
const float multiplier = 0.0001;
//----------------------------------

// Replace with your network credentials
const char* ssid = "MyNetwork";
const char* password = "so8tbbdlelz6";

// Create AsyncWebServer object on port 80
AsyncWebServer server(80);

// Create an Event Source on /events
AsyncEventSource events("/events");

// Json Variable to Hold Sensor Readings
JSONVar readings;

// Timer variables
unsigned long lastTime = 0;  
unsigned long lastTimeKal = 0;
unsigned long lastTimeAcc = 0;
unsigned long gyroDelay = 50;
unsigned long kalmanDelay = 50;
unsigned long accelerometerDelay = 50;

// Create a sensor object
Adafruit_MPU6050 mpu;

sensors_event_t a, g, temp;

float gyroX, gyroY, gyroZ;
float accX, accY, accZ;
float temperature;

//Gyroscope sensor deviation
float gyroXerror = 0.07;
float gyroYerror = 0.03;
float gyroZerror = 0.01;

// Init MPU6050
void initMPU(){
  bool mpu_condition = mpu.begin(); 
  Serial.println(mpu_condition);
  if (!mpu_condition) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
}

void initSPIFFS() {
  if (!SPIFFS.begin()) {
    Serial.println("An error has occurred while mounting SPIFFS");
  }
  Serial.println("SPIFFS mounted successfully");
}

// Initialize WiFi
void initWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println("");
  Serial.print("Connecting to WiFi...");
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(1000);
  }
  Serial.println("");
  Serial.println(WiFi.localIP());
}

String getGyroReadings(){
  mpu.getEvent(&a, &g, &temp);

  float gyroX_temp = g.gyro.x;
  if(abs(gyroX_temp) > gyroXerror)  {
    gyroX += gyroX_temp/50.00;
  }
  
  float gyroY_temp = g.gyro.y;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp/70.00;
  }

  float gyroZ_temp = g.gyro.z;
  if(abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp/90.00;
  }
   
  readings["gyroX"] = String(gyroX);
  readings["gyroY"] = String(gyroY);
  readings["gyroZ"] = String(gyroZ);

  String jsonString = JSON.stringify(readings);
  return jsonString;
}

String getAccReadings() {
  mpu.getEvent(&a, &g, &temp);
  // Get current acceleration values in g-force units and removing fluctuations with a low pass filter  
  accX = (a.acceleration.x*alpha+(a.acceleration.x*(1.0-alpha)))/9.8;
  accY = (a.acceleration.y*alpha+(a.acceleration.y*(1.0-alpha)))/9.8;
  accZ = (a.acceleration.z*alpha+(a.acceleration.z*(1.0-alpha)))/9.8;
  readings["accX"] = String(accX);
  readings["accY"] = String(accY);
  readings["accZ"] = String(accZ);
  String accString = JSON.stringify (readings);
  return accString;
}
//------ Kalman Filter ------------

String getKalReadings() {
  
  mpu.getEvent(&a, &g, &temp);
  
  // Get current acceleration values and removing fluctuations with a low pass filter  
  accX = (a.acceleration.x*alpha+(a.acceleration.x*(1.0-alpha)))/9.8;
  accY = (a.acceleration.y*alpha+(a.acceleration.y*(1.0-alpha)))/9.8;
  accZ = (a.acceleration.z*alpha+(a.acceleration.z*(1.0-alpha)))/9.8;
  
  // Gyro readings
  float gyroX_temp = g.gyro.x;
  if(abs(gyroX_temp) > gyroXerror)  {
    gyroX += gyroX_temp/100.00;
  }
  
  float gyroY_temp = g.gyro.y;
  if(abs(gyroY_temp) > gyroYerror) {
    gyroY += gyroY_temp/70.00;
  }

  float gyroZ_temp = g.gyro.z;
  if(abs(gyroZ_temp) > gyroZerror) {
    gyroZ += gyroZ_temp/90.00;
  }
  
  // Calculate Pitch, Roll and Yaw from accelerometer (deg)
  accPitch = (atan2(accX, sqrt(accY*accY + accZ*accZ))*180.0)/M_P1;
  accRoll  = (atan2(accY, accZ)*180.0)/M_P1;
  accYaw  = (atan2(accZ, sqrt(accX*accX + accZ*accZ))*180.0)/M_P1;

  // Kalman filter
  kalPitch = (1-0.8)*gyroX-accPitch*multiplier;
  kalRoll = (1-multiplier)*gyroY-accRoll*multiplier;
  kalYaw = (1-multiplier)*gyroZ-accYaw*multiplier;
  
  readings["kalPitch"] = String(kalPitch);
  readings["kalRoll"] = String(kalRoll);
  readings["kalYaw"] = String(kalYaw);
  String kalString = JSON.stringify (readings);
  return kalString;
}

//----------------------------------
void setup() {
  Serial.begin(115200);
//  Wire.begin(21, 22, 100000);
  initWiFi();
  initSPIFFS();
  initMPU();

  // Handle Web Server
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send(SPIFFS, "/index.html", "text/html");
  });

  server.serveStatic("/", SPIFFS, "/");

  server.on("/reset", HTTP_GET, [](AsyncWebServerRequest *request){
    gyroX=0;
    gyroY=0;
    gyroZ=0;
    request->send(200, "text/plain", "OK");
  });

  // Handle Web Server Events
  events.onConnect([](AsyncEventSourceClient *client){
    if(client->lastId()){
      Serial.printf("Client reconnected! Last message ID that it got is: %u\n", client->lastId());
    }
    // send event with message "hello!", id current millis
    // and set reconnect delay to 1 second
    client->send("hello!", NULL, millis(), 10000);
  });
  server.addHandler(&events);

  server.begin();
}

void loop() {
  if ((millis() - lastTime) > gyroDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getGyroReadings().c_str(),"gyro_readings",millis());
    lastTime = millis();
  }
  if ((millis() - lastTimeAcc) > accelerometerDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getAccReadings().c_str(),"accelerometer_readings",millis());
    lastTimeAcc = millis();
  }
  if ((millis() - lastTimeKal) > kalmanDelay) {
    // Send Events to the Web Server with the Sensor Readings
    events.send(getKalReadings().c_str(),"kalman_readings",millis());
    lastTimeKal = millis();
  }
}
