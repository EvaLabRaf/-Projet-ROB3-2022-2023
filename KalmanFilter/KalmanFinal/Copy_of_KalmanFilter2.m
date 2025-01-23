% KalmanFilter.m
clear;

% Charger les données de capteurs à partir de SensorData.m
[t, gps_measurements, acc_measurements, x] = SensorData();

% Paramètres
sigmaP = 200;        % Écart-type pour le GPS
sigmaA = 10;         % Écart-type pour l'accéléromètre
frequence = 10;      % Fréquence d'échantillonnage (Hz)
T = 1/frequence;     % Intervalle de temps
gamma_s = 100;         % Bruit de processus
n = length(t);       % Nombre d'échantillons

% Initialisation de la dynamique
x_est = zeros(2, n);  % Estimation initiale [position; vitesse]
x_est(1, 1) = 0.0101; x_est(2, 1) = 0.2005;
P = [10^6 0; 0 10^6];  % Covariance d'erreur initiale

% Matrices du filtre de Kalman
F = [0 1; 0 0];          % Matrice de transition
H_gps = [1 0];           % Mesure de la position avec GPS
R_gps = sigmaP^2;        % Covariance du bruit de mesure (GPS)
Qk = gamma_s * [(T^3)/3 (T^2)/2; (T^2)/2 T];  % Bruit de processus
G = [(T^2)/2; T];      % Discrete-time input matrix

% Matrice de transition d'état discrète
phi_k = eye(2) + F*T;

erreur_position = zeros(1, n);  % Estimation error (position)
borne_sup = zeros(1, n);        % Upper bound (± sqrt(P))
borne_inf = zeros(1, n);        % Lower bound (± sqrt(P))


% Boucle du filtre de Kalman
for k = 2:n-1
    % Étape de correction avec la mesure GPS
    K = P * H_gps' / (H_gps * P * H_gps' + R_gps);  % Compute the Kalman gain matrix
    z = gps_measurements(k);  % Mesure GPS à l'instant k
    x_est(:, k) = x_est(:, k-1) + K * (z - H_gps * x_est(:, k-1));  % Mise à jour de l'état

    % Étape de prédiction/extrapolation avec l'accélération mesurée
    u = acc_measurements(k);  % Accélération mesurée (en cm/s²)
    x_est(:, k+1) = phi_k * x_est(:, k) + G * u;  % Correction de la vitesse avec l'accélération

    % Mise à jour de la covariance
    P = (eye(2) - K * H_gps) * P;  % Mise à jour après correction

    % Calculate the estimation error (real - estimated)
    erreur_position(k) = x(1, k) - x_est(1, k);

    % Calculate the bounds for the error (based on the covariance)
    borne_sup(k) = sqrt(P(1,1));  % Upper bound (± sqrt(P))
    borne_inf(k) = -sqrt(P(1,1)); % Lower bound (± sqrt(P))

    % Prédiciton de la prochain covariance
    P = phi_k * P * phi_k' + Qk;   % Prédiction de la covariance

end

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


% Plot estimation error with bounds
subplot(2, 1, 2);
plot(t, erreur_position, 'LineWidth', 2);  % Estimation error
hold on;
plot(t, borne_sup, '--k', 'LineWidth', 1.5);  % Upper bound (in pointillé)
plot(t, borne_inf, '--k', 'LineWidth', 1.5);  % Lower bound (in pointillé)
xlabel('Time (s)');
ylabel('Position Error (cm)');
legend('Position Estimation Error', 'Upper Bound', 'Lower Bound');
title('Analysis of the Position Estimation Error');
grid on;
