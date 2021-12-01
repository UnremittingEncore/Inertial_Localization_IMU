# Accelerometer Based Approach for Indoor Localization

Complete MPU6050 filtering and calibration code https://github.com/UnremittingEncore/ESP_MPU6050_Calibration

## Project Overview
- We use Server-Sent Events to update all the readings.
- The 3D object is created using a JavaScript library called [three.js](https://threejs.org).

## Update Network Credentials (in the Arduino sketch)
```
// Replace with your network credentials
const char* ssid = "REPLACE_WITH_YOUR_SSID";
const char* password = "REPLACE_WITH_YOUR_PASSWORD";
```

## Useful Links
- Related Project https://RandomNerdTutorials.com/esp32-mpu-6050-web-server/
- Kalman Filter Link https://github.com/jarzebski/Arduino-KalmanFilter
 
 
