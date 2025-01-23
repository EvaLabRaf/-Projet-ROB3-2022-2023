% Simulation d'un véhicule et génération de mesures pour le filtre de Kalman

x0 = 0;  % Position initiale
v0 = 0;  % Vitesse initiale
acc = 2;   % Accélération constante (cm/s²)
vf = 10; % Vitesse finale (cm/s)
t_vf = 5; % Temps pour atteindre la vitesse finale (s)

frequence = 10; % Fréquence d'échantillonnage (Hz)
T = 1/frequence; % Période d'échantillonnage (s)
Tf = 20; % Temps total de la simulation (s)
t = linspace(0, Tf, 200); % Vecteur de temps

% Variables
a = zeros(size(t));  % Vecteur pour l'acceleration
v = zeros(size(t));  % Vecteur pour la vitesse
x = zeros(size(t));  % Vecteur pour la position

% Simuler la dynamique du mouvement
for i = 1:length(t)
    if t(i) <= 5
        % Phase 1 : Accélération (0 à 5 secondes)
        a(i) = acc;
        v(i) = acc * t(i);
        x(i) = 0.5 * acc * t(i)^2;
    else
        % Phase 2 : Mouvement uniforme (vitesse constante)
        a(i) = 0;
        v(i) = vf;
        x(i) = 0.5 * acc * t_vf^2 + vf * (t(i) - t_vf);
    end
end

% Génération des mesures GPS et accéléromètre

gps_precision = 200; % en cm
acc_precision = 1;  % en cm/s²

gps_measurements = x + gps_precision * randn(size(x));  % Simuler des mesures GPS avec bruit
acc_measurements = a + acc_precision * randn(size(t));  % Simuler des mesures accéléromètre avec bruit

% Affichage des résultats
figure;

% Graphique de la vitesse
subplot(3,1,1);
plot(t, v, 'LineWidth', 2);
xlabel('Temps (s)');
ylabel('Vitesse (cm/s)');
title('Vitesse en fonction du temps');
grid on;

% Graphique de la position réelle
subplot(3,1,2);
plot(t, x, 'LineWidth', 2);
hold on;
plot(t, gps_measurements, '--r'); % Mesures GPS bruitées
xlabel('Temps (s)');
ylabel('Position (cm)');
title('Position réelle et mesures GPS');
legend('Position réelle', 'Mesures GPS');
grid on;

% Graphique des mesures de l'accéléromètre
subplot(3,1,3);
plot(t, a, 'LineWidth', 2);
hold on;
plot(t, acc_measurements, '--r');
xlabel('Temps (s)');
ylabel('Accélération (cm/s²)');
title('Mesures de l''accéléromètre avec bruit');
legend('Acceleration réelle', 'Mesures MPU');
grid on;

