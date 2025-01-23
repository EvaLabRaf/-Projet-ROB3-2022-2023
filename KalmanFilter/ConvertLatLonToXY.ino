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

void setup() {
  Serial.begin(9600);
  ss.begin(GPSBaud);
  
  // Initialisation du filtre de Kalman
  X[0] = 0;   // Position initiale
  X[1] = 0;   // Vitesse initiale
}

void loop() {
  // Vérifie si des données GPS sont valides
  if (gps.location.isValid()) {
    float lat = gps.location.lat();
    float lon = gps.location.lng();
    
    // Conversion lat/lon en mètres (x, y)
    float x_gps, y_gps;
    convertGPStoXY(lat, lon, x_gps, y_gps);

    // Affichage des résultats
    Serial.print("Position estimée (x en m): ");
    Serial.println(X[0]);
    Serial.print("Vitesse estimée: ");
    Serial.println(X[1]);
    delay(1000); // Pause d'1 seconde entre chaque calcul
  }
}