#include "TinyGPS++.h"
#include "SoftwareSerial.h"
#include <Wire.h>
#include "MPU6050_Angle.h"

// --- GPS Definitions ---
static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

// Référence latitude et longitude
float lat_ref = 0;
float lon_ref = 0;
bool is_ref_set = false; // Pour s'assurer que la référence est définie

// Fonction pour récupérer l'accélération (dans l'axe x)
float readAcceleration() {
  return angleSensor.readRowAcc(); // Remplace par la lecture réelle de l'accéléro
}

// Fonction pour convertir les lat/lon GPS en x, y (mètres)
// La première fois, définit la position initiale comme (x=0, y=0)
void convertGPStoXY(float lat, float lon, float &x, float &y) {
  if (!is_ref_set) {
    // Initialiser la position de référence pour que le premier point soit (0, 0)
    lat_ref = lat;
    lon_ref = lon;
    is_ref_set = true;
  }
  
  // Conversion des degrés en mètres par rapport à la position de référence
  x = (lat - lat_ref) * 111320;  // Latitude -> distance en mètres
  y = (lon - lon_ref) * 111320 * cos(lat_ref * PI / 180.0);  // Longitude -> distance en mètres
}

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
MPU6050_Angle angleSensor;

// --- Kalman Filter Definitions ---
float X[2] = {0, 0}; // State [x, y, vx, vy]
float P[2][2] = {
    {10^9, 0},
    {0, 10^9}
}; // Estimate error covariance

float Q[2][2] = {
    {0.1, 0},
    {0, 0.1}
}; // Process noise covariance

float R = 4; // Measurement noise covariance

float H[2] = {1,0}; // Measurement matrix

float F[2][2] = {
    {0,1},
    {0,0},
}; // State transition matrix

