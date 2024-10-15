#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include "MPU6050_Angle.h"

// --- GPS Definitions ---
static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
MPU6050_Angle angleSensor;

