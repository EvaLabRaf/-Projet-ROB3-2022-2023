#include "TinyGPS++.h"
#include "SoftwareSerial.h"

// --- Kalman Filter Definitions ---
class KalmanFilter {
public:
    float X[2] = {0, 0}; // State [latitude, longitude]
    float P[2][2] = {{1, 0}, {0, 1}}; // Estimate error covariance
    float Q[2][2] = {{0.0001, 0}, {0, 0.0001}}; // Process noise covariance
    float R[2][2] = {{0.0005, 0}, {0, 0.0005}}; // Measurement noise covariance
    float H[2][2] = {{1, 0}, {0, 1}}; // Measurement matrix
    
    void predict() {
        // Since A is the identity matrix, the state doesn't change during prediction
        // Update covariance P = A * P * A^T + Q
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                P[i][j] += Q[i][j];
            }
        }
    }

    void update(float Z[2]) {
        // Kalman Gain K = P * H^T * (H * P * H^T + R)^-1
        float S[2][2] = {
            {P[0][0] + R[0][0], P[0][1] + R[0][1]},
            {P[1][0] + R[1][0], P[1][1] + R[1][1]}
        };
        float K[2][2] = {
            {P[0][0] / S[0][0], P[0][1] / S[1][1]},
            {P[1][0] / S[0][0], P[1][1] / S[1][1]}
        };
        
        // Update state X = X + K * (Z - H * X)
        for (int i = 0; i < 2; ++i) {
            float y = Z[i] - X[i]; // Measurement residual
            X[i] = X[i] + K[i][i] * y;
        }

        // Update covariance P = (I - K * H) * P
        for (int i = 0; i < 2; ++i) {
            P[i][i] = (1 - K[i][i]) * P[i][i];
        }
    }
};

// --- GPS Definitions ---
static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
KalmanFilter kalmanFilter;

void setup() {
    Serial.begin(9600);
    ss.begin(GPSBaud);
    Serial.println(F("Latitude   Longitude"));
    Serial.println(F("(deg)      (deg)    "));
    Serial.println(F("--------------------"));
}

void loop() {
    while (ss.available()) {
        gps.encode(ss.read());
    }

    if (gps.location.isValid()) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        float Z[2] = {latitude, longitude};

        kalmanFilter.predict();
        kalmanFilter.update(Z);

        Serial.print("Filtered Latitude: ");
        Serial.print(kalmanFilter.X[0], 6);
        Serial.print(" Longitude: ");
        Serial.println(kalmanFilter.X[1], 6);
    } else {
        Serial.println(F("No valid GPS data"));
    }

    delay(1000);
}
