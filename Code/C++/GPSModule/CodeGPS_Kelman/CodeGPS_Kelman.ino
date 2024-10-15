#include "TinyGPS++.h"
#include "SoftwareSerial.h"

// --- GPS Definitions ---
static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;

TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
unsigned long previousMillis = 0;

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

void setup() {
    Serial.begin(9600);
    ss.begin(GPSBaud);
    Serial.println(F("Latitude   Longitude"));
    Serial.println(F("(deg)      (deg)    "));
    Serial.println(F("--------------------"));
}

void loop() {
    // Read data from GPS
    while (ss.available() > 0) {
        gps.encode(ss.read());
    }

    if (gps.location.isValid()) {
        // Get current GPS data
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        float Z[2] = {latitude, longitude};

        // Calculate time difference
        unsigned long currentMillis = millis();
        float dt = (currentMillis - previousMillis) / 1000.0; // Convert to seconds
        previousMillis = currentMillis;

        // Update the F matrix with the current dt
        F[0][2] = dt;
        F[1][3] = dt;

        // --- Prediction Step ---
        float X_pred[2] = {0};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                X_pred[i] += F[i][j] * X[j];
            }
        }
        for (int i = 0; i < 2; ++i) {
            X[i] = X_pred[i];
        }

        float P_pred[2][2] = {0};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                for (int k = 0; k < 2; ++k) {
                    P_pred[i][j] += F[i][k] * P[k][j];
                }
            }
            P_pred[i][i] += Q[i][i];
        }
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                P[i][j] = P_pred[i][j];
            }
        }

        // --- Update Step ---
        // Compute Kalman Gain
        float S[2][2] = {0};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                for (int k = 0; k < 2; ++k) {
                    S[i][j] += H[i] * P[k][j];
                }
                S[i][j] += R;
            }
        }

        // Invert S
        float det = S[0][0] * S[1][1] - S[0][1] * S[1][0];
        float S_inv[2][2] = {
            {S[1][1] / det, -S[0][1] / det},
            {-S[1][0] / det, S[0][0] / det}
        };

        float K[2][2] = {0};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                for (int k = 0; k < 2; ++k) {
                    K[i][j] += P[i][k] * S_inv[k][j];
                }
            }
        }

        // Update state
        float y[2] = {0};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                y[i] += H[i] * X[j];
            }
            y[i] = Z[i] - y[i];
        }
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                X[i] += K[i][j] * y[j];
            }
        }

        // Update covariance
        float I[2][2] = {
            {1, 0},
            {0, 1}
        };
        float KH[2][2] = {0};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                for (int k = 0; k < 2; ++k) {
                    KH[i][j] += K[i][k] * H[k];
                }
                KH[i][j] = I[i][j] - KH[i][j];
            }
        }
        float P_new[2][2] = {0};
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                for (int k = 0; k < 2; ++k) {
                    P_new[i][j] += KH[i][k] * P[k][j];
                }
            }
        }
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                P[i][j] = P_new[i][j];
            }
        }

        // Output filtered position
        Serial.print("Filtered Latitude: ");
        Serial.print(X[0], 6);
        Serial.print(" Longitude: ");
        Serial.println(X[1], 6);
    } else {
        Serial.println(F("No valid GPS data"));
    }

    delay(1000);
}
