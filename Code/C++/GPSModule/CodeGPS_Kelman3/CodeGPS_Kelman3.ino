#include "TinyGPS++.h"
#include "SoftwareSerial.h"

// Pins pour le GPS
static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// Variables du filtre de Kalman
float X[2] = {0, 0};     // Vecteur d'état [x, v]
float P[2][2] = {{1, 0}, {0, 1}}; // Matrice de covariance d'erreur
float F[2][2] = {{1, 0.1}, {0, 1}}; // Matrice de transition d'état (Δt=0.1)
float G[2] = {0.005, 0.1}; // Influence de l'accélération [Δt²/2, Δt]
float Q[2][2] = {{0.1, 0}, {0, 0.1}}; // Covariance du bruit du modèle
float H[2] = {1, 0};     // Matrice de mesure (seulement la position)
float R = 4;             // Covariance du bruit de mesure GPS (précision de 2m)

// Fonction pour récupérer l'accélération (dans l'axe x)
float readAcceleration() {
  return angleSensor.readRowAcc(); // Remplace par la lecture réelle de l'accéléro
}

// Fonction pour récupérer la position GPS (latitude, utilisée pour x)
float readGPSPosition() {
  return gps.location.lat(); // Remplace par la lecture réelle du GPS
}

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  
  // Initialisation du filtre de Kalman
  X[0] = 0;   // Position initiale
  X[1] = 0;   // Vitesse initiale
}

void loop() {
  // Étape 1 : Prédiction avec l'accéléromètre
  float a_x = readAcceleration();             // Lire l'accélération
  float X_pred[2];                            // Vecteur d'état prédit
  X_pred[0] = F[0][0] * X[0] + F[0][1] * X[1] + G[0] * a_x;  // Nouvelle position
  X_pred[1] = F[1][0] * X[0] + F[1][1] * X[1] + G[1] * a_x;  // Nouvelle vitesse
  
  // Mise à jour de la covariance d'erreur P
  float P_pred[2][2];
  P_pred[0][0] = F[0][0] * P[0][0] + F[0][1] * P[1][0] + Q[0][0];
  P_pred[0][1] = F[0][0] * P[0][1] + F[0][1] * P[1][1];
  P_pred[1][0] = F[1][0] * P[0][0] + F[1][1] * P[1][0];
  P_pred[1][1] = F[1][0] * P[0][1] + F[1][1] * P[1][1] + Q[1][1];
  
  // Étape 2 : Correction avec les données GPS
  if (gps.location.isValid()) {
    float z = readGPSPosition();             // Mesure GPS
    float y = z - (H[0] * X_pred[0] + H[1] * X_pred[1]); // Erreur de mesure
    
    // Calcul du gain de Kalman
    float S = H[0] * P_pred[0][0] * H[0] + R; // Innovation
    float K[2];                               // Gain de Kalman
    K[0] = P_pred[0][0] * H[0] / S;
    K[1] = P_pred[1][0] * H[0] / S;
    
    // Mise à jour de l'état
    X[0] = X_pred[0] + K[0] * y;
    X[1] = X_pred[1] + K[1] * y;
    
    // Mise à jour de la covariance d'erreur P
    P[0][0] = (1 - K[0] * H[0]) * P_pred[0][0];
    P[0][1] = (1 - K[0] * H[0]) * P_pred[0][1];
    P[1][0] = -K[1] * H[0] * P_pred[0][0];
    P[1][1] = -K[1] * H[0] * P_pred[1][1];
  }

  // Affichage des résultats
  Serial.print("Position estimée: ");
  Serial.println(X[0]);
  Serial.print("Vitesse estimée: ");
  Serial.println(X[1]);
  delay(1000); // Pause d'1 seconde entre chaque calcul
}
``
