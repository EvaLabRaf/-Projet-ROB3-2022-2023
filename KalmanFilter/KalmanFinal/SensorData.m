% SensorData.m

function [t, gps_measurements, acc_measurements, x] = SensorData()
    % Simulation des données des capteurs

    % Paramètres de simulation
    frequence = 10; % Fréquence d'échantillonnage (Hz)
    Tf = 20; % Temps total de la simulation (s)
    t = linspace(0, Tf, Tf * frequence); % Vecteur de temps

    % Paramètres des capteurs
    gps_precision = 200; % Précision du GPS (en cm)
    acc_precision = 1;   % Bruit de l'accéléromètre (en cm/s²)
    acc = 2;            % Accélération constante (cm/s²)
    vf = 10;            % Vitesse finale (cm/s)
    t_vf = 5;          % Temps pour atteindre la vitesse finale (s)

    % Initialisation des variables
    x = zeros(3, length(t)); % [position; vitesse]
    gps_measurements = zeros(size(t)); % Mesures GPS
    acc_measurements = zeros(size(t)); % Mesures accéléromètre

    % Simuler la dynamique du mouvement
    for i = 1:length(t)
        if t(i) <= t_vf
            % Phase d'accélération
            x(1, i) = 0.5 * acc * t(i)^2; % Position
            x(2, i) = acc * t(i); % Vitesse
            x(3, i) = acc; % Acceleration
        else
            % Phase de mouvement uniforme
            x(1, i) = x(1, t_vf * frequence) + vf * (t(i) - t_vf); % Position
            x(2, i) = vf; % Vitesse constante
            x(3, i) = 0; % Acceleration
        end

        % Générer des mesures bruitées
        gps_measurements(i) = x(1, i) + gps_precision * (2 * rand() -1);  % Mesures GPS bruitées
        acc_measurements(i) = x(3, i) + acc_precision * (2 * rand() -1);  % Mesures accéléromètre bruitées
    end

    % % Affichage des résultats
    % figure;
    % 
    % % Graphique de la vitesse
    % subplot(3,1,1);
    % plot(t, x(2,:), 'LineWidth', 2);
    % xlabel('Temps (s)');
    % ylabel('Vitesse (cm/s)');
    % title('Vitesse en fonction du temps');
    % grid on;
    % 
    % % Graphique de la position réelle
    % subplot(3,1,2);
    % plot(t, x(1,:), 'LineWidth', 2);
    % hold on;
    % plot(t, gps_measurements, '--r'); % Mesures GPS bruitées
    % xlabel('Temps (s)');
    % ylabel('Position (cm)');
    % title('Position réelle et mesures GPS');
    % legend('Position réelle', 'Mesures GPS');
    % grid on;
    % 
    % % Graphique des mesures de l'accéléromètre
    % subplot(3,1,3);
    % plot(t, x(3,:), 'LineWidth', 2);
    % hold on;
    % plot(t, acc_measurements, '--r');
    % xlabel('Temps (s)');
    % ylabel('Accélération (cm/s²)');
    % title('Mesures de l''accéléromètre avec bruit');
    % legend('Acceleration réelle', 'Mesures MPU');
    % grid on;
end
