% KalmanFilter.m

% Charger les données de capteurs à partir de SensorData.m
[t, gps_measurements, acc_measurements, x] = SensorData();

% Paramètres
sigmaP = 200;        % Écart-type pour le GPS
sigmaA = 39;         % Écart-type pour l'accéléromètre
frequence = 10;      % Fréquence d'échantillonnage (Hz)
T = 1/frequence;     % Intervalle de temps
gamma_s = 100;         % Bruit de processus
n = length(t);       % Nombre d'échantillons

% Initialisation de la dynamique
x_est = zeros(2, n);  % Estimation initiale [position; vitesse]
P = [10^9 0; 0 10^9];  % Covariance d'erreur initiale
P_diag = zeros(2, n);  % Stockage des valeurs diagonales de la covariance

% Matrices du filtre de Kalman
F = [0 1; 0 0];          % Matrice de transition
H_gps = [1 0];           % Mesure de la position avec GPS
R_gps = sigmaP^2;        % Covariance du bruit de mesure (GPS)
Qk = gamma_s * [(T^3)/3 (T^2)/2; (T^2)/2 T];  % Bruit de processus

% Matrice de transition d'état discrète
phi_k = eye(2) + F*T;

% Boucle du filtre de Kalman
for k = 1:n-1
    % Étape de correction avec la mesure GPS
    K = P * H_gps' / (H_gps * P * H_gps' + R_gps);  % Gain de Kalman
    z = gps_measurements(k);  % Mesure GPS à l'instant k
    x_est(:, k) = x_est(:, k) + K * (z - H_gps * x_est(:, k));  % Mise à jour de l'état

    % Étape de prédiction/extrapolation avec l'accélération mesurée
    u = acc_measurements(k);  % Accélération mesurée (en cm/s²)
    x_est(2, k) = x_est(2, k) + u * T;  % Correction de la vitesse avec l'accélération

    % Prédiction de la position et de la vitesse pour l'étape suivante
    x_est(:, k+1) = phi_k * x_est(:, k);

    % Mise à jour de la covariance
    P = (eye(2) - K * H_gps) * P;  % Mise à jour après correction
    P = phi_k * P * phi_k' + Qk;   % Prédiction de la covariance
    
    % Stockage des diagonales de la covariance pour évaluer l'incertitude
    P_diag(:, k) = diag(P);
end
P_diag(:, n) = diag(P);  % Dernière mise à jour de la covariance

% Affichage des résultats
figure;
subplot(2, 1, 1);
plot(t, gps_measurements, 'g--'); hold on;
plot(t, x_est(1,:), 'b'); hold on;
plot(t, x(1,:), "r"); hold on;
xlabel('Temps (s)');
ylabel('Position (cm)');
title('Estimation de la position avec filtre de Kalman');
legend('Mesure GPS', 'Position estimée', 'Position réelle');
grid on;

subplot(2, 1, 2);
plot(t, x_est(2, :), 'b'); hold on;
plot(t, x(2,:), 'r'); hold on;
xlabel('Temps (s)');
ylabel('Vitesse (cm/s)');
title('Estimation de la vitesse avec filtre de Kalman');
grid on;

% % Erreurs d'estimation pour la position
% figure;
% plot(t, gps_measurements - x_est(1,:), 'r--'); hold on;
% plot(t, gps_measurements - x_est(1,:) + sqrt(P_diag(1,:)), 'g--', t, gps_measurements - x_est(1,:) - sqrt(P_diag(1,:)), 'g--');
% title('Erreur d''estimation pour la position avec intervalle de confiance');
% grid on;

% % Erreurs d'estimation pour la vitesse
% figure;
% plot(t, 0 - x_est(2,:), 'r--'); hold on;
% plot(t, 0 - x_est(2,:) + sqrt(P_diag(2,:)), 'g--', t, 0 - x_est(2,:) - sqrt(P_diag(2,:)), 'g--');
% title('Erreur d''estimation pour la vitesse avec intervalle de confiance');
% grid on;
