#include "TinyGPS++.h"
#include "SoftwareSerial.h"

// --- GPS Definitions ---
static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
unsigned long previousMillis = 0;

// --- Kalman Filter Definitions ---
int Ts = 0.1;

float X[2] = {0, 0}; // State [x, y]

float P[2][2] = {
    {10^9, 0},
    {0, 10^9}
}; // Estimate error covariance

float Q[2][2] = {
    {(Ts^3)/3, (Ts^2)/2},
    {(Ts^2)/2, T},
}; // Process noise covariance

float R[1][1] = {
    {4} // Précision de 2m
}; // Measurement noise covariance

float H[1][2] = {
    {1, 0}
}; // Measurement matrix

float F[2][2] = {
    {0, 1},
    {0, 0}
}; // State transition matri
void setup() {
    Serial.begin(9600);
    ss.begin(GPSBaud);
    Serial.println(F("Latitude   Longitude"));
    Serial.println(F("(deg)      (deg)    "));
    Serial.println(F("--------------------"));
}

void loop() {

}

