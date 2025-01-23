% KalmanFilter.m

function KalmanFilter()
    % Appel de la fonction de données des capteurs
    [t, gps_measurements, acc_measurements] = SensorData();

    % Filtre de Kalman
    % Initialisation des variables
    x = zeros(2, length(t)); % État [position; vitesse]
                  
    % Matrices du filtre de Kalman
    F = [0, 1; 0, 0];        % Matrice de transition d'état
    H = [1, 0];              % Matrice de mesure
    Q = [0.1, 0; 0, 0.1];    % Bruit de processus
    R = 200^2;               % Bruit de mesure (variance du GPS)
    P = [10^9, 0; 0, 10^9];  % Matrice de covariance d'erreur

    % Filtre de Kalman
    x_estimated = zeros(2, length(t)); % Estimation de l'état

    for i = 1:length(t)
        % Prédiction
        x(:, i) = F * x(:, max(i-1, 1)); % Prédire l'état suivant
        P = F * P * F' + Q; % Mise à jour de la covariance

        % Mise à jour avec les mesures GPS
        if i > 1
            % Mesure
            z = gps_measurements(i); % Mesure GPS
            y = z - H * x(:, i); % Résidu de mesure (erreur)

            % Gain de Kalman
            S = H * P * H' + R; % Covariance de la mesure
            K = P * H' / S; % Gain

            % Mise à jour de l'état et de la covariance
            x(:, i) = x(:, i) + K * y; % Estimation mise à jour
            P = (eye(2) - K * H) * P; % Mise à jour de la covariance
        end

        % Enregistrer les estimations
        x_estimated(:, i) = x(:, i);
    end

    % Affichage des résultats
    figure;

    % Graphique de la position réelle et estimée
    subplot(2,1,1);
    plot(t, gps_measurements, '--r', 'LineWidth', 1.5); % Mesures GPS bruitées
    hold on;
    plot(t, x_estimated(1, :), 'b', 'LineWidth', 2); % Position estimée
    xlabel('Temps (s)');
    ylabel('Position (cm)');
    title('Position réelle et estimée par le filtre de Kalman');
    legend('Mesures GPS', 'Position estimée');
    grid on;

    % Graphique de la vitesse estimée
    subplot(2,1,2);
    plot(t, x_estimated(2, :), 'b', 'LineWidth', 2); % Vitesse estimée
    xlabel('Temps (s)');
    ylabel('Vitesse (cm/s)');
    title('Vitesse estimée par le filtre de Kalman');
    grid on;
end
