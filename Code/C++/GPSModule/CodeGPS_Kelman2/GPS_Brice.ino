// Variables de l'état
float pos_est = 0;  // Position estimée
float vel_est = 0;  // Vitesse estimée
float pos_real, vel_real;  // Position et vitesse réelles
float accel;  // Accélération mesurée

// Matrices du filtre de Kalman
float F[2][2] = { {1, 0.1}, {0, 1} };  // Matrice de transition (assume dt = 0.1 s)
float H[2][2] = { {1, 0}, {0, 0} };    // Matrice d'observation
float Q[2][2] = { {0.1, 0.01}, {0.01, 0.1} };  // Covariance du bruit de processus
float R = 4.0;  // Variance du bruit GPS (2m de précision = 4 m²)

// Covariance d'erreur initiale
float P[2][2] = { {1, 0}, {0, 1} };

// Mesures GPS et accéléromètre
float z;  // Mesure de position (GPS)
float a;  // Mesure d'accélération (accéléromètre)

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Obtenir les données du GPS et de l'accéléromètre
  z = getGPSPosition();  // Remplacer par la fonction de récupération du GPS
  a = getAcceleration();  // Remplacer par la fonction de récupération de l'accéléromètre

  // Prédiction (state prediction)
  pos_est += vel_est * 0.1;  // Predict new position (dt = 0.1 s)
  vel_est += a * 0.1;        // Update velocity with acceleration

  // Mise à jour de la matrice de covariance d'erreur (P = F*P*F' + Q)
  float P_new[2][2];
  P_new[0][0] = P[0][0] + P[0][1] * 0.1 + P[1][0] * 0.1 + P[1][1] * 0.01 + Q[0][0];
  P_new[0][1] = P[0][1] + P[1][1] * 0.1 + Q[0][1];
  P_new[1][0] = P[1][0] + P[1][1] * 0.1 + Q[1][0];
  P_new[1][1] = P[1][1] + Q[1][1];

  // Gain de Kalman (K = P * H' * (H * P * H' + R)^-1)
  float S = P_new[0][0] + R;  // Innovation covariance
  float K[2];
  K[0] = P_new[0][0] / S;
  K[1] = P_new[1][0] / S;

  // Correction (Update step)
  float y = z - pos_est;  // Innovation
  pos_est += K[0] * y;
  vel_est += K[1] * y;

  // Mise à jour de la matrice de covariance d'erreur (P = (I - K * H) * P)
  P[0][0] = (1 - K[0]) * P_new[0][0];
  P[0][1] = (1 - K[0]) * P_new[0][1];
  P[1][0] = -K[1] * P_new[0][0] + P_new[1][0];
  P[1][1] = -K[1] * P_new[0][1] + P_new[1][1];

  // Afficher la position et la vitesse estimées
  Serial.print("Position estimée: "); Serial.println(pos_est);
  Serial.print("Vitesse estimée: "); Serial.println(vel_est);

  // Attendre avant la prochaine itération
  delay(100);
}

// Fonctions fictives pour récupérer les mesures (à remplacer par des fonctions réelles)
float getGPSPosition() {
  // Code pour récupérer la position GPS
  return 100.0;  // Exemple de valeur
}

float getAcceleration() {
  // Code pour récupérer l'accélération
  return -9.81;  // Exemple de valeur (gravité)
}
